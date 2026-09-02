/*
 * mowercontrol - drives the mower from gps samples, status frames and tcp commands.
 */
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "mowercontrol.h"

#define GGA_HISTORY 20
#define WO_LINE_MAX 65536
#define WORKORDER_DIR "workorder"
#define WORKORDER_EXT ".wo"
#define WORKORDER_NAME_MAX 32
#define TCP_LINE_MAX 128
#define STATE_RC_MIN 32
#define STATE_RC_TURNING 34
#define STATE_RC_MAX 64
#define FIX_RTK_FIXED 4
#define FIX_RTK_FLOAT 5
#define MM_PER_DEGREE 111320000.0
#define AT_POINT_MM 500
#define DIRECTION_SPEED 50
#define DIRECTION_MEASURE_MS 3000
#define RUN_SPEED 50
#define TARGET_BEHIND_DEG 90.0
#define OFF_COURSE_DEG 10.0
#define TURN_START_MS 1000		/* the mower has this long to report the turn */

typedef enum {
	mctr_idle = 0,
	mctr_read_wo_line,
	mctr_wait,
	mctr_run_to_point_start,
	mctr_rtp_determine_direction,
	mctr_rtp_wait_for_turn,
	mctr_run_towards_point
}mctrstate_t;

static mctrstate_t mctrstate = mctr_idle;
static FILE *wo_file;
static char wo_line[WO_LINE_MAX];
static int  wo_cmd_id;
static int  wo_wait_event;
static point_t wo_target;
static bool direction_known;
static bool turn_started;		/* the mower has reported the ordered turn running */
static double direction;		/* degrees from north, clockwise */
static point_t state_start_point;	/* position when the current state was entered */
static long long state_deadline;	/* monotonic milliseconds */
static msg_mower_status_t status;
static char gga_log[GGA_HISTORY][GPS_LINE_MAX];
static int gga_log_pos = -1;		/* last inserted slot */

/* Not stepped by ntp, unlike time() */
static long long now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);

	return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static int valid_name(const char *s)
{
	for (; *s; s++) {
		if (!isalnum((unsigned char)*s) && *s != '_' && *s != '-') {
			return 0;
		}
	}
	return 1;
}

static void start_workorder(const char *name)
{
	char path[sizeof(WORKORDER_DIR) + WORKORDER_NAME_MAX + sizeof(WORKORDER_EXT)];

	if (wo_file) {
		fclose(wo_file);
	}

	snprintf(path, sizeof(path), "%s/%s%s", WORKORDER_DIR, name, WORKORDER_EXT);
	wo_file = fopen(path, "r");
	if (!wo_file) {
		printf("Failed to open workorder %s\n", path);
		return;
	}
	printf("workorder %s started\n", name);
	mctrstate = mctr_read_wo_line;
}

static void tcp_row(const char *row)
{
	char name[WORKORDER_NAME_MAX];
	int fieldtype, action;

	printf("tcp row: %s\n", row);
	if (sscanf(row, "%d,%d,%31[^,\r]", &fieldtype, &action, name) != 3) {
		printf("tcp row ignored, needs fieldtype,action,name\n");
		return;
	}
	if (fieldtype != 1 || action != 1 || !valid_name(name)) {
		printf("tcp row ignored, fieldtype %d action %d name %s\n", fieldtype, action, name);
		return;
	}

	start_workorder(name);
}

void mowercontrol_input_tcp(const uint8_t *data, size_t len)
{
	static char line[TCP_LINE_MAX];
	static size_t line_len;
	size_t i;

	for (i = 0; i < len; i++) {
		if (data[i] == '\n') {
			line[line_len] = 0;
			line_len = 0;
			tcp_row(line);
		} else if (line_len < sizeof(line) - 1) {
			line[line_len++] = (char)data[i];
		}
	}
}

void mowercontrol_input_mowercom(uint8_t msgid, const uint8_t *msgbuf)
{
	if (msgid == MSG_MOWER_STATUS) {
		memcpy(&status, msgbuf, sizeof(status));
	}
}

void mowercontrol_input_gps(gps_recordtype type, const char *line)
{
	if (type != gps_gga) {		/* rmc not used yet */
		return;
	}

	gga_log_pos = (gga_log_pos + 1) % GGA_HISTORY;
	strcpy(gga_log[gga_log_pos], line);
}

/* cmd_id 1: left_speed, right_speed, disc_speed, force_run */
static void wo_cmd_run(void)
{
	int8_t p[4];

	if (sscanf(wo_line, "%*d,%hhd,%hhd,%hhd,%hhd", &p[0], &p[1], &p[2], &p[3]) == 4) {
		printf("cmd run: left %d right %d disc %d force %d\n", p[0], p[1], p[2], p[3]);
		direction_known = false;	/* the mower moves without us tracking the heading */
		mowercom_send(MSG_REMOTE_CONTROL_RUN, p, sizeof(p));
	}
}

/* cmd_id 2: wheel_speed, turn_angle, turn_dir, disc_speed, force_run */
static void wo_cmd_turn(void)
{
	int speed, angle, dir, disc, force;
	uint8_t p[5];

	if (sscanf(wo_line, "%*d,%d,%d,%d,%d,%d", &speed, &angle, &dir, &disc, &force) != 5) {
		return;
	}

	p[0] = speed;
	p[1] = angle;
	p[2] = dir;
	p[3] = disc;
	p[4] = force;
	printf("cmd turn: %d deg %s speed %d disc %d force %d\n", angle, dir ? "right" : "left", speed, disc, force);
	direction_known = false;	/* the mower turns without us tracking the heading */
	mowercom_send(MSG_REMOTE_CONTROL_TURN, p, sizeof(p));
}

/* cmd_id 3: event, timeout */
static void wo_cmd_wait(void)
{
	int event, timeout;

	if (sscanf(wo_line, "%*d,%d,%d", &event, &timeout) != 2) {
		return;
	}

	printf("cmd wait: event %d timeout %d s\n", event, timeout);
	wo_wait_event = event;
	state_deadline = now_ms() + (long long)timeout * 1000;
	mctrstate = mctr_wait;
}

/* cmd_id 4: latitude_nmea, latitude_hemisphere, longitude_nmea, longitude_hemisphere */
static void wo_cmd_run_to_point(void)
{
	char lat[16], ns[2], lon[16], ew[2];

	if (sscanf(wo_line, "%*d,%15[^,],%1[^,],%15[^,],%1[^,\r\n]", lat, ns, lon, ew) != 4) {
		return;
	}

	wo_target = nmea_to_point(lat, ns, lon, ew);
	printf("cmd run to point: %.7f %.7f\n", wo_target.lat, wo_target.lon);
	mctrstate = mctr_run_to_point_start;
}

static int gga_quality(void)
{
	nmea_gga_t gga;

	if (!parse_gga(gga_log[gga_log_pos], &gga)) {
		return 0;
	}

	return atoi(gga.quality);
}

static point_t current_point(void)
{
	nmea_gga_t gga;

	parse_gga(gga_log[gga_log_pos], &gga);

	return nmea_to_point(gga.lat, gga.ns, gga.lon, gga.ew);
}

/* Degrees from north, clockwise */
static double bearing_between_points(const point_t *from, const point_t *to)
{
	double dlat, dlon;

	dlat = to->lat - from->lat;
	dlon = (to->lon - from->lon) * cos(from->lat * M_PI / 180.0);

	return atan2(dlon, dlat) * 180.0 / M_PI;
}

/* Millimetres between two points, equirectangular - the error is under a mm at mower range */
static int distance_between_points(const point_t *from, const point_t *to)
{
	double dlat, dlon;

	dlat = (to->lat - from->lat) * MM_PER_DEGREE;
	dlon = (to->lon - from->lon) * MM_PER_DEGREE * cos(from->lat * M_PI / 180.0);

	return (int)sqrt(dlat * dlat + dlon * dlon);
}

static bool is_wait_condition_done(void)
{
	int quality;

	switch(wo_wait_event) {
		case 1:
			return(status.state != STATE_RC_TURNING);
		case 2:
			quality = gga_quality();
			return quality == FIX_RTK_FIXED || quality == FIX_RTK_FLOAT;
		case 3:
			return gga_quality() == FIX_RTK_FIXED;
	}
	return false;
}

void mowercontrol_execute()
{
	point_t pos;
	double turn;

	if(status.state < STATE_RC_MIN || status.state > STATE_RC_MAX) {
		mctrstate = mctr_idle;
	}

	switch(mctrstate) {
		case mctr_idle:
			if(wo_file) {
				printf("Closing workorder file\n");
				fclose(wo_file);
				wo_file = NULL;
				direction_known = false;
			}
			break;
		case mctr_read_wo_line:
			if (!fgets(wo_line, sizeof(wo_line), wo_file)) {
				printf("workorder done\n");
				fclose(wo_file);
				wo_file = NULL;
				mctrstate = mctr_idle;
				break;
			}
			printf("wo row: %s", wo_line);
			sscanf(wo_line, "%d", &wo_cmd_id);	/* rows without a cmd_id are skipped */
			switch(wo_cmd_id) {
				case 1:
					wo_cmd_run();
					break;
				case 2:
					wo_cmd_turn();
					break;
				case 3:
					wo_cmd_wait();
					break;
				case 4:
					wo_cmd_run_to_point();
					break;
			}
			break;
		case mctr_wait:
			if (is_wait_condition_done() || now_ms() >= state_deadline) {
				printf("wait done\n");
				mctrstate = mctr_read_wo_line;
			}
			break;
		case mctr_run_to_point_start:
			pos = current_point();
			if (distance_between_points(&pos, &wo_target) < AT_POINT_MM) {
				printf("rtp: already at point\n");
				mctrstate = mctr_read_wo_line;
			} else if (!direction_known) {
				printf("rtp: %d mm to go, measuring direction\n", distance_between_points(&pos, &wo_target));
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ DIRECTION_SPEED, DIRECTION_SPEED, 0, 0 }, 4);
				state_start_point = pos;
				state_deadline = now_ms() + DIRECTION_MEASURE_MS;
				mctrstate = mctr_rtp_determine_direction;
			} else {	/* turn on the spot until we point at the target */
				turn = fmod(bearing_between_points(&pos, &wo_target) - direction + 540.0, 360.0) - 180.0;
				printf("rtp: %d mm to go, heading %.0f, turning %.0f deg %s\n", distance_between_points(&pos, &wo_target), direction, fabs(turn), turn < 0 ? "left" : "right");
				mowercom_send(MSG_REMOTE_CONTROL_TURN, (uint8_t[]){ 0, (uint8_t)fabs(turn), turn < 0 ? 0 : 1, 0, 0 }, 5);
				turn_started = false;
				state_deadline = now_ms() + TURN_START_MS;
				direction = bearing_between_points(&pos, &wo_target);
				mctrstate = mctr_rtp_wait_for_turn;
			}
			break;
		case mctr_rtp_determine_direction:
			pos = current_point();
			if (distance_between_points(&pos, &wo_target) < AT_POINT_MM) {
				printf("rtp: at point, %d mm to go\n", distance_between_points(&pos, &wo_target));
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4); // stop mower
				mctrstate = mctr_read_wo_line;
			} else if (now_ms() >= state_deadline) {
				direction = bearing_between_points(&state_start_point, &pos);
				direction_known = true;
				printf("rtp: direction measured over %d mm, heading %.0f\n", distance_between_points(&state_start_point, &pos), direction);
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4); // stop mower
				mctrstate = mctr_run_to_point_start;
			}
			break;
		case mctr_rtp_wait_for_turn:
			pos = current_point();
			if (distance_between_points(&pos, &wo_target) < AT_POINT_MM) {
				printf("rtp: at point while turning, %d mm to go\n", distance_between_points(&pos, &wo_target));
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4); // stop mower
				mctrstate = mctr_read_wo_line;
			} else if (status.state == STATE_RC_TURNING) {
				if (!turn_started) {
					printf("rtp: turn running\n");
				}
				turn_started = true;
			} else if (turn_started) {
				printf("rtp: turn complete, running at speed %d\n", RUN_SPEED);
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ RUN_SPEED, RUN_SPEED, 0, 0 }, 4);
				state_start_point = pos;
				state_deadline = now_ms() + DIRECTION_MEASURE_MS;
				mctrstate = mctr_run_towards_point;
			} else if (now_ms() >= state_deadline) {
				printf("rtp: turn never reported, carrying on\n");
				turn_started = true;
			}
			break;
		case mctr_run_towards_point:
			pos = current_point();
			if (distance_between_points(&pos, &wo_target) < AT_POINT_MM) {
				printf("rtp: at point, %d mm to go\n", distance_between_points(&pos, &wo_target));
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4); // stop mower
				mctrstate = mctr_read_wo_line;
			} else if (now_ms() >= state_deadline) {
				direction = bearing_between_points(&state_start_point, &pos);
				turn = fmod(bearing_between_points(&pos, &wo_target) - direction + 540.0, 360.0) - 180.0;
				if (fabs(turn) > TARGET_BEHIND_DEG) {
					printf("rtp: target behind, %d mm to go, heading %.0f, off course %.0f deg\n", distance_between_points(&pos, &wo_target), direction, turn);
					mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4); // stop mower
					mctrstate = mctr_read_wo_line;
				} else if (fabs(turn) > OFF_COURSE_DEG) {
					printf("rtp: %d mm to go, heading %.0f, off course %.0f deg, correcting\n", distance_between_points(&pos, &wo_target), direction, turn);
					mowercom_send(MSG_REMOTE_CONTROL_TURN, (uint8_t[]){ RUN_SPEED, (uint8_t)fabs(turn), turn < 0 ? 0 : 1, 0, 0 }, 5);
					turn_started = false;
					state_deadline = now_ms() + TURN_START_MS;
					direction = bearing_between_points(&pos, &wo_target);
					mctrstate = mctr_rtp_wait_for_turn;
				}
			}
			break;
		default:
			printf("Illegal mctrstate: %d", (int)mctrstate);
	}
}
