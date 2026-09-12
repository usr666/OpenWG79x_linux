/*
 * mowercontrol - drives the mower from gps samples, status frames and tcp commands.
 */
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "log.h"
#include "mowercontrol.h"

#define GGA_HISTORY 64
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
#define MIN_HEADING_MM 1000
#define RUN_SPEED 50
#define TARGET_BEHIND_DEG 90.0
#define OFF_COURSE_DEG 10.0
#define TURN_START_MS 1000
#define COLLISION_SPEED 30
#define COLLISION_REVERSE_MS 1000
#define COLLISION_FORWARD_MS 2000
#define COLLISION_TURN_DEG 90
#define COLLISION_RETRIES 5
#define POLYGON_MAX_POINTS 32
#define MOW_RESUME_MS 5000
#define LINE_MAX_POINTS 32
#define DIRECTION_PRINT_MS 1000
#define DISC_SPEED 50

typedef enum {
	mctr_idle = 0,
	mctr_read_wo_line,
	mctr_wait,
	mctr_run_to_point_start,
	mctr_rtp_start_determine_direction,
	mctr_rtp_determine_direction,
	mctr_rtp_wait_for_turn,
	mctr_run_towards_point,
	mctr_collision_backoff,
	mctr_collision_turn_away,
	mctr_collision_forward,
	mctr_collision_turn_back,
	mctr_mow,
	mctr_mow_polygon,
	mctr_mow_outside_run,
	mctr_mow_outside_turn,
	mctr_mow_check_wire
}mctrstate_t;

static mctrstate_t mctrstate = mctr_idle;
static mctrstate_t mctrstate_after_collision = mctr_idle;
static bool collision_turn_left;	/* which way the collision detour goes */
static int collision_retries;		/* detours since the last one that worked */
static point_t polygon[POLYGON_MAX_POINTS];
static int polygon_points;
static point_t mow_center;
static long long mow_deadline;
static FILE *wo_file;
static char wo_line[WO_LINE_MAX];
static int  wo_cmd_id;
static int  wo_wait_event;
static point_t wo_target;
static point_t line_points[LINE_MAX_POINTS];
static int line_count;
static int line_target;
static int line_step;
static int line_dir;
static int disc_speed;
static bool turn_started;
static double direction;
static long long direction_print_deadline;
static long long state_deadline;
static msg_mower_status_t status;
static point_t gga_log[GGA_HISTORY];
static int gga_log_pos;
static int gga_log_count;
static int gga_last_quality;

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
		logf(INFO, "Failed to open workorder %s\n", path);
		return;
	}
	logf(INFO, "workorder %s started\n", name);
	collision_retries = 0;
	line_count = 0;			/* a line left unfinished by the last workorder is not ours */
	mctrstate = mctr_read_wo_line;
}

static void tcp_row(const char *row)
{
	const char *field;
	char name[WORKORDER_NAME_MAX];
	int fieldtype, action;

	logf(INFO, "tcp row: %s\n", row);

	if (sscanf(row, "%d", &fieldtype) != 1 || fieldtype != 1) {
		logf(INFO, "tcp row ignored, not fieldtype 1\n");
		return;
	}

	field = strchr(row, ',');
	if (!field || sscanf(field + 1, "%d", &action) != 1) {
		logf(INFO, "tcp row ignored, no action\n");
		return;
	}

	switch(action) {
		case 1:
			field = strchr(field + 1, ',');
			if (!field || sscanf(field + 1, "%31[^,\r]", name) != 1 || !valid_name(name)) {
				logf(INFO, "tcp row ignored, bad workorder name\n");
				return;
			}
			start_workorder(name);
			break;
		case 2:
			logf(INFO, "tcp abort\n");
			mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4);
			mctrstate = mctr_idle;
			break;
		default:
			logf(INFO, "tcp row ignored, action %d\n", action);
			break;
	}
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
		if (status.left_wheel_speed != status.right_wheel_speed || status.left_wheel_speed < 0) {
			gga_log_count = 0;
		}
		logf(INFO, "status: state %u wheels %d/%d bump %u lift %u wire %u%u%u soc %u\n", status.state, status.left_wheel_speed, status.right_wheel_speed, (status.sensor_status >> 0) & 1, (status.sensor_status >> 1) & 1, (status.wire_sensor_status >> 0) & 1, (status.wire_sensor_status >> 1) & 1, (status.wire_sensor_status >> 2) & 1, status.battery_soc);
	}
}

void mowercontrol_input_gps(gps_recordtype type, const char *line)
{
	nmea_gga_t gga;

	if (type != gps_gga) {		/* rmc not used yet */
		return;
	}
	if (!parse_gga(line, &gga)) {
		return;
	}

	gga_last_quality = atoi(gga.quality);
	gga_log_pos = (gga_log_pos + 1) % GGA_HISTORY;
	gga_log[gga_log_pos] = nmea_to_point(gga.lat, gga.ns, gga.lon, gga.ew);
	if (gga_log_count < GGA_HISTORY) {
		gga_log_count++;
	}
}

/* cmd_id 1: left_speed, right_speed, disc_speed, force_run */
static void wo_cmd_run(void)
{
	int8_t p[4];

	if (sscanf(wo_line, "%*d,%hhd,%hhd,%hhd,%hhd", &p[0], &p[1], &p[2], &p[3]) == 4) {
		logf(INFO, "cmd run: left %d right %d disc %d force %d\n", p[0], p[1], p[2], p[3]);
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
	logf(INFO, "cmd turn: turn ordered %d deg %s at speed %d, disc %d force %d\n", angle, dir ? "right" : "left", speed, disc, force);
	turn_started = false;		/* so a wait for this turn can't resolve on a stale flag */
	mowercom_send(MSG_REMOTE_CONTROL_TURN, p, sizeof(p));
}

/* cmd_id 3: event, timeout */
static void wo_cmd_wait(void)
{
	int event, timeout;

	if (sscanf(wo_line, "%*d,%d,%d", &event, &timeout) != 2) {
		return;
	}

	logf(INFO, "cmd wait: event %d timeout %d s\n", event, timeout);
	wo_wait_event = event;
	state_deadline = now_ms() + (long long)timeout * 1000;
	mctrstate = mctr_wait;
}

static void wo_cmd_mow(void)
{
	int mowtime;

	if (sscanf(wo_line, "%*d,%d", &mowtime) != 1) {
		return;
	}

	mow_deadline = now_ms() + (long long)mowtime * 1000;
	logf(INFO, "cmd mow: %d s\n", mowtime);
	mowercom_send(MSG_REMOTE_CONTROL_MOW, NULL, 0);
	mctrstate = mctr_mow;
}

/* cmd_id 4: latitude_nmea, latitude_hemisphere, longitude_nmea, longitude_hemisphere */
static void wo_cmd_run_to_point(void)
{
	char lat[16], ns[2], lon[16], ew[2];

	if (sscanf(wo_line, "%*d,%15[^,],%1[^,],%15[^,],%1[^,\r\n]", lat, ns, lon, ew) != 4) {
		return;
	}

	wo_target = nmea_to_point(lat, ns, lon, ew);
	line_count = 0;
	disc_speed = 0;
	logf(INFO, "cmd run to point: %.7f %.7f\n", wo_target.lat, wo_target.lon);
	mctrstate = mctr_run_to_point_start;
}

static void wo_cmd_mow_polygon(void)
{
	const char *field = wo_line;
	char lat[16], ns[2], lon[16], ew[2];
	int mowtime, numpoints, i, j;

	if (sscanf(wo_line, "%*d,%d", &mowtime) != 1) {
		return;
	}

	for (i = 0; i < 2; i++) {		/* step over cmd_id and mowtime */
		field = field ? strchr(field, ',') : NULL;
		if (field) {
			field++;
		}
	}

	if (!field || sscanf(field, "%15[^,],%1[^,],%15[^,],%1[^,\r\n]", lat, ns, lon, ew) != 4) {
		logf(INFO, "cmd mow polygon: no centre point\n");
		return;
	}
	mow_center = nmea_to_point(lat, ns, lon, ew);

	for (i = 0; i < 4; i++) {		/* step over the centre point */
		field = field ? strchr(field, ',') : NULL;
		if (field) {
			field++;
		}
	}

	if (!field || sscanf(field, "%d", &numpoints) != 1) {
		logf(INFO, "cmd mow polygon: no corner count\n");
		return;
	}
	if (numpoints < 3 || numpoints > POLYGON_MAX_POINTS) {
		logf(INFO, "cmd mow polygon: %d corners, must be 3..%d\n", numpoints, POLYGON_MAX_POINTS);
		return;
	}

	field = strchr(field, ',');		/* step over numpoints */
	if (field) {
		field++;
	}

	for (i = 0; i < numpoints; i++) {
		if (!field || sscanf(field, "%15[^,],%1[^,],%15[^,],%1[^,\r\n]", lat, ns, lon, ew) != 4) {
			logf(INFO, "cmd mow polygon: corner %d missing\n", i + 1);
			return;
		}
		polygon[i] = nmea_to_point(lat, ns, lon, ew);
		for (j = 0; j < 4; j++) {
			field = field ? strchr(field, ',') : NULL;
			if (field) {
				field++;
			}
		}
	}

	polygon_points = numpoints;
	mow_deadline = now_ms() + (long long)mowtime * 1000;
	logf(INFO, "cmd mow polygon: %d s, centre %.7f %.7f, %d corners\n", mowtime, mow_center.lat, mow_center.lon, numpoints);
	mowercom_send(MSG_REMOTE_CONTROL_MOW, NULL, 0);
	state_deadline = now_ms() + MOW_RESUME_MS;
	mctrstate = mctr_mow_polygon;
}

static bool point_in_polygon(const point_t *p)
{
	bool inside = false;
	int i, j;

	for (i = 0, j = polygon_points - 1; i < polygon_points; j = i++) {
		if ((polygon[i].lat > p->lat) != (polygon[j].lat > p->lat) && p->lon < (polygon[j].lon - polygon[i].lon) * (p->lat - polygon[i].lat) / (polygon[j].lat - polygon[i].lat) + polygon[i].lon) {
			inside = !inside;
		}
	}

	return inside;
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

/* cmd_id 8: numpoints, targetpoint, then 4 fields per point */
static void wo_cmd_follow_line(void)
{
	const char *field = wo_line;
	char lat[16], ns[2], lon[16], ew[2];
	point_t pos;
	int numpoints, target, i, j, dist, bestdist;

	if (sscanf(wo_line, "%*d,%d,%d", &numpoints, &target) != 2) {
		logf(INFO, "cmd follow line: no point count\n");
		return;
	}
	if (numpoints < 2 || numpoints > LINE_MAX_POINTS) {
		logf(INFO, "cmd follow line: %d points, must be 2..%d\n", numpoints, LINE_MAX_POINTS);
		return;
	}
	if (target < 1 || target > numpoints) {
		logf(INFO, "cmd follow line: target %d, must be 1..%d\n", target, numpoints);
		return;
	}

	for (i = 0; i < 3; i++) {		/* step over cmd_id, numpoints and targetpoint */
		field = field ? strchr(field, ',') : NULL;
		if (field) {
			field++;
		}
	}

	for (i = 0; i < numpoints; i++) {
		if (!field || sscanf(field, "%15[^,],%1[^,],%15[^,],%1[^,\r\n]", lat, ns, lon, ew) != 4) {
			logf(INFO, "cmd follow line: point %d missing\n", i + 1);
			return;
		}
		line_points[i] = nmea_to_point(lat, ns, lon, ew);
		for (j = 0; j < 4; j++) {
			field = field ? strchr(field, ',') : NULL;
			if (field) {
				field++;
			}
		}
	}

	pos = gga_log[gga_log_pos];
	line_step = 0;
	bestdist = distance_between_points(&pos, &line_points[0]);
	for (i = 1; i < numpoints; i++) {
		dist = distance_between_points(&pos, &line_points[i]);
		if (dist < bestdist) {
			line_step = i;
			bestdist = dist;
		}
	}

	line_count = numpoints;
	line_target = target - 1;
	line_dir = (line_target < line_step) ? -1 : 1;
	disc_speed = DISC_SPEED;
	logf(INFO, "cmd follow line: %d points, nearest is %d at %d mm, target is %d\n", numpoints, line_step + 1, bestdist, target);
	line_step -= line_dir;
	mctrstate = mctr_read_wo_line;
}

/* The nearest point from index from onwards, counting towards the target only, so
   points left behind are never gone back for. -1 once the target is behind us. */
static int nearest_ahead(const point_t *pos, int from)
{
	int i, best = -1, dist, bestdist = 0;

	for (i = from; i >= 0 && i < line_count && i != line_target + line_dir; i += line_dir) {
		dist = distance_between_points(pos, &line_points[i]);
		if (best < 0 || dist < bestdist) {
			best = i;
			bestdist = dist;
		}
	}

	return best;
}

static bool calculate_direction(void)
{
	point_t now = gga_log[gga_log_pos];
	int i, pos, distance;

	for (i = 20; i < gga_log_count; i++) { // Require at least 20 points to calculate direction
		pos = (gga_log_pos - i + GGA_HISTORY) % GGA_HISTORY;
		distance=distance_between_points(&gga_log[pos], &now);
		if (distance >= MIN_HEADING_MM) {
			direction = bearing_between_points(&gga_log[pos], &now);
			if (now_ms() >= direction_print_deadline) {
				logf(INFO, "direction %.0f over %d mm, %d samples\n", direction, distance, i);
				direction_print_deadline = now_ms() + DIRECTION_PRINT_MS;
			}
			return true;
		}
	}

	return false;
}

static bool is_wait_condition_done(void)
{
	switch(wo_wait_event) {
		case 1:
			if (status.state == STATE_RC_TURNING) {
				turn_started = true;
				return false;
			}
			return turn_started;
		case 2:
			return gga_last_quality == FIX_RTK_FIXED || gga_last_quality == FIX_RTK_FLOAT;
		case 3:
			return gga_last_quality == FIX_RTK_FIXED;
	}
	return false;
}

static bool collision_handling(mctrstate_t resumestate) {
	bool direction_left = true;
	switch(status.wire_sensor_status&0x03) {
		case 3:
			switch(status.sensor_status) {
				case 0:
					return false;
				default:
					break;
			}
			break;
		case 1:
			direction_left = false;
			break;
		default:
			break;
	}
	if (++collision_retries > COLLISION_RETRIES) {
		logf(INFO, "collision: still stuck after %d detours, giving up\n", COLLISION_RETRIES);
		mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4);
		mctrstate = mctr_idle;
		return true;
	}

	logf(INFO, "collision: sensors %02X wire %02X, detour %d, reversing, will turn %s\n", status.sensor_status, status.wire_sensor_status, collision_retries, direction_left ? "left" : "right");
	mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ -COLLISION_SPEED, -COLLISION_SPEED, 0, 1 }, 4);
	collision_turn_left = direction_left;
	state_deadline = now_ms() + COLLISION_REVERSE_MS;
	mctrstate_after_collision = resumestate;
	mctrstate = mctr_collision_backoff;

	return true;
}

static void turn_towards_center(void)
{
	point_t pos;
	double turn;

	pos = gga_log[gga_log_pos];
	turn = fmod(bearing_between_points(&pos, &mow_center) - direction + 540.0, 360.0) - 180.0;
	logf(INFO, "mow polygon: heading %.0f, turn ordered %d deg %s towards the centre\n", direction, (int)fabs(turn), turn < 0 ? "left" : "right");
	mowercom_send(MSG_REMOTE_CONTROL_TURN, (uint8_t[]){ 0, (uint8_t)fabs(turn), turn < 0 ? 0 : 1, 0, 0 }, 5);
	turn_started = false;
	state_deadline = now_ms() + TURN_START_MS;
	direction = bearing_between_points(&pos, &mow_center);
	mctrstate = mctr_mow_outside_turn;
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
				logf(INFO, "Closing workorder file\n");
				fclose(wo_file);
				wo_file = NULL;
			}
			break;
		case mctr_read_wo_line:
			if (line_count > 0) {
				pos = gga_log[gga_log_pos];
				line_step = nearest_ahead(&pos, line_step + line_dir);
				if (line_step < 0) {
					logf(INFO, "follow line: target reached\n");
					line_count = 0;
					break;
				}
				wo_target = line_points[line_step];
				logf(INFO, "follow line: point %d of %d\n", line_step + 1, line_count);
				mctrstate = mctr_run_to_point_start;
				break;
			}
			if (!fgets(wo_line, sizeof(wo_line), wo_file)) {
				logf(INFO, "workorder done\n");
				fclose(wo_file);
				wo_file = NULL;
				mctrstate = mctr_idle;
				break;
			}
			logf(INFO, "wo row: %s", wo_line);
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
				case 5:
					wo_cmd_mow();
					break;
				case 6:
					logf(INFO, "cmd go to charge\n");
					mowercom_send(MSG_REMOTE_CONTROL_FIND_CHARGER, NULL, 0);
					break;
				case 7:
					wo_cmd_mow_polygon();
					break;
				case 8:
					wo_cmd_follow_line();
					break;
			}
			break;
		case mctr_wait:
			if (is_wait_condition_done() || now_ms() >= state_deadline) {
				logf(INFO, "wait done\n");
				mctrstate = mctr_read_wo_line;
			}
			break;
		case mctr_run_to_point_start:
			pos = gga_log[gga_log_pos];
			if (line_count > 0) {		/* a detour has moved us, so spend the stop turn on the point nearest now */
				line_step = nearest_ahead(&pos, line_step);
				wo_target = line_points[line_step];
			}
			if (distance_between_points(&pos, &wo_target) < AT_POINT_MM) {
				logf(INFO, "rtp: already at point\n");
				mctrstate = mctr_read_wo_line;
			} else if (!calculate_direction()) {
				mctrstate = mctr_rtp_start_determine_direction;
			} else {
				turn = fmod(bearing_between_points(&pos, &wo_target) - direction + 540.0, 360.0) - 180.0;
				logf(INFO, "rtp: %d mm to go, heading %.0f, turn ordered %d deg %s at speed 0\n", distance_between_points(&pos, &wo_target), direction, (int)fabs(turn), turn < 0 ? "left" : "right");
				mowercom_send(MSG_REMOTE_CONTROL_TURN, (uint8_t[]){ 0, (uint8_t)fabs(turn), turn < 0 ? 0 : 1, disc_speed, 0 }, 5);
				turn_started = false;
				state_deadline = now_ms() + TURN_START_MS;
				direction = bearing_between_points(&pos, &wo_target);
				mctrstate = mctr_rtp_wait_for_turn;
			}
			break;
		case mctr_rtp_start_determine_direction:
			pos = gga_log[gga_log_pos];
			logf(INFO, "rtp: %d mm to go, no heading, mowing to get one\n", distance_between_points(&pos, &wo_target));
			mowercom_send(MSG_REMOTE_CONTROL_MOW, NULL, 0);
			mctrstate = mctr_rtp_determine_direction;
			break;
		case mctr_rtp_determine_direction:
			if(collision_handling(mctr_rtp_start_determine_direction)) {
				break;
			}
			pos = gga_log[gga_log_pos];
			if (distance_between_points(&pos, &wo_target) < AT_POINT_MM) {
				logf(INFO, "rtp: at point, %d mm to go\n", distance_between_points(&pos, &wo_target));
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4);
				mctrstate = mctr_read_wo_line;
			} else if (calculate_direction()) {
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4);
				mctrstate = mctr_run_to_point_start;
			}
			break;
		case mctr_rtp_wait_for_turn:
			if(collision_handling(mctr_run_to_point_start)) {
				break;
			}
			pos = gga_log[gga_log_pos];
			if (distance_between_points(&pos, &wo_target) < AT_POINT_MM) {
				logf(INFO, "rtp: at point while turning, %d mm to go\n", distance_between_points(&pos, &wo_target));
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4);
				mctrstate = mctr_read_wo_line;
			} else if (status.state == STATE_RC_TURNING) {
				if (!turn_started) {
					logf(INFO, "rtp: turn running\n");
				}
				turn_started = true;
			} else if (turn_started) {
				logf(INFO, "rtp: turn complete, running at speed %d\n", RUN_SPEED);
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ RUN_SPEED, RUN_SPEED, disc_speed, 0 }, 4);


				mctrstate = mctr_run_towards_point;
			} else if (now_ms() >= state_deadline) {
				logf(INFO, "rtp: turn never reported, carrying on\n");
				turn_started = true;
			}
			break;
		case mctr_run_towards_point:
			if(collision_handling(mctr_run_to_point_start)) {
				break;
			}
			pos = gga_log[gga_log_pos];
			if (distance_between_points(&pos, &wo_target) < AT_POINT_MM) {
				logf(INFO, "rtp: at point, %d mm to go\n", distance_between_points(&pos, &wo_target));
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4);
				mctrstate = mctr_read_wo_line;
			} 
			else if (calculate_direction()) {
				turn = fmod(bearing_between_points(&pos, &wo_target) - direction + 540.0, 360.0) - 180.0;
				if (fabs(turn) > TARGET_BEHIND_DEG) {
					logf(INFO, "rtp: heading %.0f, %d mm to go, off course %.0f deg, target behind, giving up on the point\n", direction, distance_between_points(&pos, &wo_target), turn);
					mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4);
					mctrstate = mctr_read_wo_line;
				} else if (fabs(turn) > OFF_COURSE_DEG) {
					logf(INFO, "rtp: heading %.0f, %d mm to go, off course %.0f deg, turn ordered %d deg %s at speed %d\n", direction, distance_between_points(&pos, &wo_target), turn, (int)fabs(turn), turn < 0 ? "left" : "right", RUN_SPEED);
					mowercom_send(MSG_REMOTE_CONTROL_TURN, (uint8_t[]){ RUN_SPEED, (uint8_t)fabs(turn), turn < 0 ? 0 : 1, disc_speed, 0 }, 5);
					turn_started = false;
					state_deadline = now_ms() + TURN_START_MS;
					direction = bearing_between_points(&pos, &wo_target);
					mctrstate = mctr_rtp_wait_for_turn;
				}
			}
			break;
		case mctr_collision_backoff:
			if (now_ms() >= state_deadline) {	/* ordered while still reversing, a turn from rest is ignored */
				logf(INFO, "collision: turn ordered %d deg %s at speed 0\n", COLLISION_TURN_DEG, collision_turn_left ? "left" : "right");
				mowercom_send(MSG_REMOTE_CONTROL_TURN, (uint8_t[]){ 0, COLLISION_TURN_DEG, collision_turn_left ? 0 : 1, 0, 0 }, 5);
				turn_started = false;
				state_deadline = now_ms() + TURN_START_MS;
				mctrstate = mctr_collision_turn_away;
			}
			break;
		case mctr_collision_turn_away:
			if (collision_handling(mctrstate_after_collision)) {
				break;
			}
			if (status.state == STATE_RC_TURNING) {
				turn_started = true;
			} else if (turn_started) {
				logf(INFO, "collision: running forward at speed %d\n", COLLISION_SPEED);
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ COLLISION_SPEED, COLLISION_SPEED, 0, 0 }, 4);
				state_deadline = now_ms() + COLLISION_FORWARD_MS;
				mctrstate = mctr_collision_forward;
			} else if (now_ms() >= state_deadline) {
				logf(INFO, "collision: turn never reported, carrying on\n");
				turn_started = true;
			}
			break;
		case mctr_collision_forward:
			if (collision_handling(mctrstate_after_collision)) {
				break;
			}
			if (now_ms() >= state_deadline) {	/* ordered while still running forward */
				logf(INFO, "collision: turn ordered %d deg %s at speed 0\n", COLLISION_TURN_DEG, collision_turn_left ? "right" : "left");
				mowercom_send(MSG_REMOTE_CONTROL_TURN, (uint8_t[]){ 0, COLLISION_TURN_DEG, collision_turn_left ? 1 : 0, 0, 0 }, 5);
				turn_started = false;
				state_deadline = now_ms() + TURN_START_MS;
				mctrstate = mctr_collision_turn_back;
			}
			break;
		case mctr_collision_turn_back:
			if (collision_handling(mctrstate_after_collision)) {
				break;
			}
			if (status.state == STATE_RC_TURNING) {
				turn_started = true;
			} else if (turn_started) {
				logf(INFO, "collision: handled, resuming state %d\n", mctrstate_after_collision);
				collision_retries = 0;
				mctrstate = mctrstate_after_collision;
			} else if (now_ms() >= state_deadline) {
				logf(INFO, "collision: turn never reported, carrying on\n");
				turn_started = true;
			}
			break;
		case mctr_mow:
			if (now_ms() >= mow_deadline) {
				logf(INFO, "mow: time is up, stopping\n");
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4);
				mctrstate = mctr_read_wo_line;
			}
			break;
		case mctr_mow_polygon:
			if (now_ms() >= mow_deadline) {
				logf(INFO, "mow polygon: time is up, stopping\n");
				mowercom_send(MSG_REMOTE_CONTROL_RUN, (int8_t[]){ 0, 0, 0, 0 }, 4);
				mctrstate = mctr_read_wo_line;
				break;
			}
			if (now_ms() < state_deadline) {
				break;
			}
			pos = gga_log[gga_log_pos];
			if (!point_in_polygon(&pos)) {	/* the turn goes out while it still rolls, a turn from rest is ignored */
				if (calculate_direction()) {
					turn_towards_center();
				} else {
					logf(INFO, "mow polygon: outside, no direction, mowing to get one\n");
					mowercom_send(MSG_REMOTE_CONTROL_MOW, NULL, 0);
					mctrstate = mctr_mow_outside_run;
				}
			}
			break;
		case mctr_mow_outside_run:
			/* the detour ends stopped, so resume through the state that starts the mower again */
			if (collision_handling(mctr_mow_check_wire)) {
				break;
			}
			if (calculate_direction()) {
				turn_towards_center();
			}
			break;
		case mctr_mow_outside_turn:
			if (collision_handling(mctr_mow_check_wire)) {
				break;
			}
			if (status.state == STATE_RC_TURNING) {
				turn_started = true;
			} else if (turn_started) {
				mctrstate = mctr_mow_check_wire;
			} else if (now_ms() >= state_deadline) {
				logf(INFO, "mow polygon: turn never reported, carrying on\n");
				turn_started = true;
			}
			break;
		case mctr_mow_check_wire:
			if (collision_handling(mctr_mow_check_wire)) {
				break;
			}
			logf(INFO, "mow polygon: inside the wire, resuming mow\n");
			mowercom_send(MSG_REMOTE_CONTROL_MOW, NULL, 0);
			state_deadline = now_ms() + MOW_RESUME_MS;
			mctrstate = mctr_mow_polygon;
			break;
		default:
			logf(INFO, "Illegal mctrstate: %d", (int)mctrstate);
	}
}
