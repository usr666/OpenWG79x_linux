/*
 * openwg79x_rc - Remote control for openwg79x
 *
 * Reads MOWER_STATUS frames from the mower UART, reads NMEA from gpsd
 * (via gpspipe -r) and appends one line per second combining the two.
 * Positions are stored exactly as GGA reports them - see logformat.txt.
 *
 * Usage: openwg79x_rc [uart_device] [log_file]
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
#include "mowercom.h"
#include "tcpcom.h"

#define DEFAULT_UART_DEVICE "/dev/ttyUSB0"

#define SAMPLES_PER_FILE 50000

#define LOG_LINE_MAX 256

#define LOG_HEADER \
	"#version 2\n" \
	"#1,timestamp_utc,latitude_nmea,latitude_hemisphere," \
	"longitude_nmea,longitude_hemisphere,fix_quality," \
	"satellites_used,hdop,wifi_signal,mower_state,battery_soc_percent," \
	"front_bump,lift,wire_right_inside,wire_left_inside,near_wire\n"

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
		fprintf(stderr, "open %s: %s\n", path, strerror(errno));
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

	if (!f)
		return 0;

	while (fgets(line, sizeof(line), f)) {
		char *p = strchr(line, ':');

		if (p && sscanf(p + 1, "%*s %*s %d", &level) == 1)
			break;
	}
	fclose(f);
	return level;
}

static void log_write_sample(FILE *f, const nmea_gga_t *gga, const nmea_rmc_t *rmc, const msg_mower_status_t *st)
{
	char line[LOG_LINE_MAX];
	char status[64];
	char time_utc[32];
	char wifi[8] = "";
	int level = wifi_signal();

	gps_timestamp(gga, rmc, time_utc, sizeof(time_utc));
	if (level)
		snprintf(wifi, sizeof(wifi), "%d", level);

	/* The two status bytes are unpacked into one column per bit. */
	if (st)
		snprintf(status, sizeof(status), "%u,%u,%u,%u,%u,%u,%u", st->state, st->battery_soc, (st->sensor_status >> 0) & 1, (st->sensor_status >> 1) & 1, (st->wire_sensor_status >> 0) & 1, (st->wire_sensor_status >> 1) & 1, (st->wire_sensor_status >> 2) & 1);
	else
		snprintf(status, sizeof(status), ",,,,,,");	/* no status frame yet */

	snprintf(line, sizeof(line), "1,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n", time_utc, gga->lat, gga->ns, gga->lon, gga->ew, gga->quality, gga->sats, gga->hdop, wifi, status);

	fputs(line, f);
	fflush(f);
	tcpcom_broadcast(line);
}

static volatile sig_atomic_t stop;

static void on_signal(int sig)
{
	(void)sig;
	stop = 1;
}

int main(int argc, char **argv)
{
	const char *uart_device = (argc > 1) ? argv[1] : DEFAULT_UART_DEVICE;
	const char *log_path;
	char default_log[64];

	static uint8_t msgbuf[MAX_MSG_SIZE];
	msg_mower_status_t status = { 0 };
	nmea_gga_t gga = { 0 };
	nmea_rmc_t rmc = { 0 };
	unsigned long samples = 0, file_samples = 0;
	int uart_fd, gps_fd, tcp_fd;
	FILE *log;

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);

	if (argc > 2) {
		log_path = argv[2];
	} else {
		log_name(default_log, sizeof(default_log));
		log_path = default_log;
	}

	uart_fd = mowercom_open(uart_device);
	if (uart_fd < 0)
		return 1;

	gps_fd = gps_open();
	if (gps_fd < 0) {
		mowercom_close();
		return 1;
	}

	log = log_open(log_path);
	if (!log) {
		gps_close();
		mowercom_close();
		return 1;
	}

	tcp_fd = tcpcom_open();
	if (tcp_fd < 0)
		fprintf(stderr, "no live feed - logging to file only\n");

	printf("openwg79x_rc: uart=%s log=%s tcp=%d\n", uart_device, log_path, TCPCOM_PORT);

	while (!stop) {
		struct pollfd fds[3];
		int nfds = 0, uart_idx, gps_idx = -1, tcp_idx = -1;

		uart_idx = nfds;
		fds[nfds].fd = uart_fd;
		fds[nfds].events = POLLIN;
		nfds++;

		if (gps_fd >= 0) {
			gps_idx = nfds;
			fds[nfds].fd = gps_fd;
			fds[nfds].events = POLLIN;
			nfds++;
		}

		if (tcp_fd >= 0) {
			tcp_idx = nfds;
			fds[nfds].fd = tcp_fd;
			fds[nfds].events = POLLIN;
			nfds++;
		}

		if (poll(fds, nfds, -1) < 0) {
			if (errno == EINTR)
				continue;
			fprintf(stderr, "poll: %s\n", strerror(errno));
			break;
		}

		if (fds[uart_idx].revents & (POLLERR | POLLHUP | POLLNVAL)) {
			fprintf(stderr, "mower uart lost\n");
			break;
		}

		if (fds[uart_idx].revents & POLLIN) {
			uint8_t msgid;
			size_t len;
			int r;

			while ((r = mowercom_read(&msgid, msgbuf, sizeof(msgbuf), &len)) > 0) {
				if (msgid != MSG_MOWER_STATUS || len < sizeof(status))
					continue;

				memcpy(&status, msgbuf, sizeof(status));
				log_write_sample(log, &gga, &rmc, &status);
				samples++;
				file_samples++;
				printf("\r%lu samples  %s%s %s%s  %ssat  state=%u soc=%u%%  ", samples, gga.lat, gga.ns, gga.lon, gga.ew, gga.sats, status.state, status.battery_soc);
				fflush(stdout);

				if (file_samples >= SAMPLES_PER_FILE) {
					fclose(log);
					log_name(default_log, sizeof(default_log));
					log_path = default_log;
					log = log_open(log_path);
					if (!log)
						break;
					file_samples = 0;
					printf("\ncontinuing in %s\n", log_path);
				}
			}
			if (r < 0 || !log)
				break;
		}

		if (gps_idx >= 0 && (fds[gps_idx].revents & (POLLIN | POLLHUP))) {
			int r;

			while ((r = gps_read(&gga, &rmc)) > 0)
				;
			if (r < 0) {
				fprintf(stderr, "gpspipe closed - no more position data\n");
				gps_fd = -1;
			}
		}

		if (tcp_idx >= 0 && (fds[tcp_idx].revents & POLLIN))
			tcpcom_accept();
	}

	if (log)
		fclose(log);
	tcpcom_close();
	gps_close();
	mowercom_close();

	printf("\nwrote %lu samples to %s\n", samples, log_path);
	return 0;
}
