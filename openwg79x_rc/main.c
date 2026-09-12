/*
 * openwg79x_rc - Remote control for openwg79x
 * Usage: openwg79x_rc [uart_device]
 */
#define _DEFAULT_SOURCE

#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "gps.h"
#include "log.h"
#include "mowercom.h"
#include "mowercontrol.h"
#include "tcpcom.h"

#define DEFAULT_UART_DEVICE "/dev/ttyUSB0"

#define LOOP_MS 100

#define TCP_READ_MAX 256

#define SAMPLES_PER_FILE 50000

#define LOG_LINE_MAX 256

#define LOG_HEADER \
	"#version 3\n" \
	"#1,timestamp_utc,latitude_nmea,latitude_hemisphere," \
	"longitude_nmea,longitude_hemisphere,fix_quality," \
	"satellites_used,hdop,wifi_signal,mower_state,battery_soc_percent," \
	"front_bump,lift,wire_right_inside,wire_left_inside,near_wire\n"

static volatile sig_atomic_t stop;

static int   uart_fd = -1, gps_fd = -1, tcp_fd = -1;
static FILE *log_file;
static char  log_path[64];
static unsigned long file_samples;

static msg_mower_status_t status;
static nmea_gga_t gga;
static nmea_rmc_t rmc;
static char gga_line[GPS_LINE_MAX];
static char rmc_line[GPS_LINE_MAX];

static void log_name(char *out, size_t n)
{
	time_t now = time(NULL);
	struct tm tm;

	gmtime_r(&now, &tm);
	strftime(out, n, "mowerlog_%Y%m%d_%H_%M_%S.csv", &tm);
}

static FILE *log_open(const char *path)
{
	FILE *f = fopen(path, "w");

	if (!f) {
		logf(INFO, "open %s: %s\n", path, strerror(errno));
		return NULL;
	}
	fprintf(f, "%s", LOG_HEADER);
	fflush(f);
	return f;
}

/* Signal level of the first wireless interface in dBm, 0 when there is none. */
static int wifi_signal(void)
{
	char line[128];
	FILE *f = fopen("/proc/net/wireless", "r");
	int level = 0;

	if (!f) {
		return 0;
	}

	while (fgets(line, sizeof(line), f)) {
		char *p = strchr(line, ':');

		if (p && sscanf(p + 1, "%*s %*s %d", &level) == 1) {
			break;
		}
	}
	fclose(f);
	return level;
}

static void log_row(const char *row)
{
	fputs(row, log_file);
	fflush(log_file);
	tcpcom_broadcast(row);
}

static void log_write_sample(void)
{
	char line[LOG_LINE_MAX];
	char st[64];
	char time_utc[32];
	char wifi[8] = "";
	int level = wifi_signal();

	parse_gga(gga_line, &gga);
	parse_rmc(rmc_line, &rmc);
	gps_timestamp(&gga, &rmc, time_utc, sizeof(time_utc));
	if (level) {
		snprintf(wifi, sizeof(wifi), "%d", level);
	}

	/* The two status bytes are unpacked into one column per bit. */
	snprintf(st, sizeof(st), "%u,%u,%u,%u,%u,%u,%u", status.state, status.battery_soc, (status.sensor_status >> 0) & 1, (status.sensor_status >> 1) & 1, (status.wire_sensor_status >> 0) & 1, (status.wire_sensor_status >> 1) & 1, (status.wire_sensor_status >> 2) & 1);

	snprintf(line, sizeof(line), "1,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", time_utc, gga.lat, gga.ns, gga.lon, gga.ew, gga.quality, gga.sats, gga.hdop, wifi, st);

	log_row(line);
}

/* One sample logged, with a new file started when this one is full. */
static void log_sample(void)
{
	log_write_sample();
	file_samples++;

	if (file_samples < SAMPLES_PER_FILE) {
		return;
	}

	fclose(log_file);
	log_name(log_path, sizeof(log_path));
	log_file = log_open(log_path);
	if (!log_file) {
		stop = 1;
		return;
	}
	file_samples = 0;
	logf(INFO, "\ncontinuing in %s\n", log_path);
}

int main(int argc, char **argv)
{
	int i;
	uint8_t msgbuf[MAX_MSG_SIZE];
	uint8_t msgid;
	size_t len;
	char gpsline[GPS_LINE_MAX];
	uint8_t buf[TCP_READ_MAX];
	gps_recordtype gpstype;
	const char *uart_device = (argc > 1) ? argv[1] : DEFAULT_UART_DEVICE;
	struct pollfd fds[2];

	setvbuf(stdout, NULL, _IOLBF, 0);	/* systemd gives us a pipe, which is block buffered by default */

	logf(INFO, "openwg79x_rc built %s %s\n", __DATE__, __TIME__);

	log_name(log_path, sizeof(log_path));

	uart_fd = mowercom_open(uart_device);
	if (uart_fd < 0) {
		logf(INFO, "Failed to open mower uart %s\n", uart_device);
		return 1;
	}

	gps_fd = gps_open();
	if (gps_fd < 0) {
		logf(INFO, "Failed to open gpspipe\n");
		mowercom_close();
		return 1;
	}

	log_file = log_open(log_path);
	if (!log_file) {
		logf(INFO, "Failed to open log file %s\n", log_path);
		gps_close();
		mowercom_close();
		return 1;
	}

	tcp_fd = tcpcom_open();

	logf(INFO, "openwg79x_rc: uart=%s log=%s tcp=%d\n", uart_device, log_path, TCPCOM_PORT);

	fds[0].fd = uart_fd;
	fds[1].fd = gps_fd;
	fds[0].events = fds[1].events = POLLIN;

	while (!stop) {
		fds[1].fd = gps_fd;		/* -1 once gpspipe is gone, poll skips it */
		poll(fds, 2, LOOP_MS);

		gpstype = gps_read(gpsline);
		if(gpstype != gps_none) {
			mowercontrol_input_gps(gpstype, gpsline);
			if(gpstype == gps_gga) {
				strcpy(gga_line, gpsline);
			} else if(gpstype == gps_rmc) {
				strcpy(rmc_line, gpsline);
			}
		}

		i = mowercom_read(&msgid, msgbuf, sizeof(msgbuf), &len);
		if (i > 0) {
			mowercontrol_input_mowercom(msgid, msgbuf);
			if(msgid == MSG_MOWER_STATUS) {
				memcpy(&status, msgbuf, sizeof(status));
				log_sample();
			}
		} else if (i < 0) {
			logf(INFO, "mower uart lost\n");
			return 1;
		}

		tcpcom_accept();
		if((i = tcpcom_read(buf, sizeof(buf))) > 0) {
			mowercontrol_input_tcp(buf, i);
		}

		mowercontrol_execute();
	}

	if (log_file) {
		fclose(log_file);
	}
	tcpcom_close();
	gps_close();
	mowercom_close();

	logf(INFO, "exit: stopped\n");
	return 0;
}
