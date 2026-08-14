/*
 * gps - NMEA from gpsd via `gpspipe -r`. Sentences are stored as received.
 */
#define _DEFAULT_SOURCE

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "gps.h"

#define GPSPIPE_CMD "gpspipe -r"

static FILE  *gps_pipe;
static int    gps_fd = -1;
static char   gps_line[256];
static size_t gps_len;

static int nmea_checksum_ok(const char *s)
{
	uint8_t sum = 0;
	const char *p;

	if (*s != '$')
		return 0;
	for (p = s + 1; *p && *p != '*'; p++)
		sum ^= (uint8_t)*p;
	if (*p != '*' || !isxdigit((unsigned char)p[1]) ||
	    !isxdigit((unsigned char)p[2]))
		return 0;
	return (uint8_t)strtol(p + 1, NULL, 16) == sum;
}

/* Split in place on commas; the checksum suffix is cut off. */
static int nmea_split(char *s, char *field[], int max)
{
	int n = 0;

	field[n++] = s;
	for (char *p = s; *p && n < max; p++) {
		if (*p == ',') {
			*p = '\0';
			field[n++] = p + 1;
		} else if (*p == '*') {
			*p = '\0';
			break;
		}
	}
	return n;
}

/*
 * One raw NMEA sentence from `gpspipe -r`. Talker IDs vary (GP/GN/GL/GA),
 * so only the last three characters of the sentence name are matched.
 */
static int parse_nmea_line(const char *line, nmea_gga_t *gga, nmea_rmc_t *rmc)
{
	char buf[128];
	char *f[24];
	int nf;

	if (line[0] != '$' || strlen(line) < 9 || strlen(line) >= sizeof(buf))
		return 0;
	if (!nmea_checksum_ok(line))
		return 0;

	snprintf(buf, sizeof(buf), "%s", line);
	nf = nmea_split(buf, f, 24);

	if (!memcmp(buf + 3, "GGA", 3) && nf > 8) {
		snprintf(gga->time,    sizeof(gga->time),    "%s", f[1]);
		snprintf(gga->lat,     sizeof(gga->lat),     "%s", f[2]);
		snprintf(gga->ns,      sizeof(gga->ns),      "%s", f[3]);
		snprintf(gga->lon,     sizeof(gga->lon),     "%s", f[4]);
		snprintf(gga->ew,      sizeof(gga->ew),      "%s", f[5]);
		snprintf(gga->quality, sizeof(gga->quality), "%s", f[6]);
		snprintf(gga->sats,    sizeof(gga->sats),    "%s", f[7]);
		snprintf(gga->hdop,    sizeof(gga->hdop),    "%s", f[8]);
		gga->valid = 1;
		return 1;
	}

	if (!memcmp(buf + 3, "RMC", 3) && nf > 9) {
		snprintf(rmc->date, sizeof(rmc->date), "%s", f[9]);
		rmc->valid = 1;
		return 2;
	}

	return 0;
}

int gps_open(void)
{
	gps_pipe = popen(GPSPIPE_CMD, "r");
	if (!gps_pipe) {
		fprintf(stderr, "popen %s: %s\n", GPSPIPE_CMD, strerror(errno));
		return -1;
	}
	gps_fd = fileno(gps_pipe);
	fcntl(gps_fd, F_SETFL, O_NONBLOCK);
	return gps_fd;
}

/* Takes one complete sentence out of the buffer, if there is one. */
static int take_line(char *out, size_t n)
{
	size_t i;

	for (i = 0; i < gps_len; i++) {
		if (gps_line[i] != '\n' && gps_line[i] != '\r')
			continue;

		gps_line[i] = '\0';
		snprintf(out, n, "%s", gps_line);
		memmove(gps_line, gps_line + i + 1, gps_len - i - 1);
		gps_len -= i + 1;
		return 1;
	}
	return 0;
}

int gps_read(nmea_gga_t *gga, nmea_rmc_t *rmc)
{
	char line[sizeof(gps_line)];
	ssize_t n;

	for (;;) {
		while (take_line(line, sizeof(line))) {
			int type = parse_nmea_line(line, gga, rmc);

			if (type)
				return type;
		}

		if (gps_len == sizeof(gps_line))	/* no line break, drop it */
			gps_len = 0;

		n = read(gps_fd, gps_line + gps_len, sizeof(gps_line) - gps_len);
		if (n == 0)
			return -1;
		if (n < 0)
			return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
		gps_len += (size_t)n;
	}
}

void gps_close(void)
{
	if (gps_pipe) {
		pclose(gps_pipe);
		gps_pipe = NULL;
		gps_fd = -1;
	}
}

/* GGA "hhmmss.ss" + RMC "ddmmyy" -> "YYYY-MM-DDThh:mm:ss.sssZ" */
void gps_timestamp(const nmea_gga_t *gga, const nmea_rmc_t *rmc, char *out, size_t n)
{
	static int warned_no_date;
	const char *hhmmss = gga->time;
	const char *d = rmc->date;
	int year;
	unsigned char mon, day, hh, mi;
	double sec;

	if (strlen(d) == 6) {
		int v = (d[4] - '0') * 10 + (d[5] - '0');

		day  = (d[0] - '0') * 10 + (d[1] - '0');
		mon  = (d[2] - '0') * 10 + (d[3] - '0');
		year = (v < 80) ? 2000 + v : 1900 + v;
	} else {
		time_t now = time(NULL);
		struct tm tm;

		gmtime_r(&now, &tm);
		year = tm.tm_year + 1900;
		mon  = tm.tm_mon + 1;
		day  = tm.tm_mday;
		if (!warned_no_date) {
			fprintf(stderr, "no RMC yet - dating samples from the system clock\n");
			warned_no_date = 1;
		}
	}
	if (strlen(hhmmss) < 6) {
		out[0] = '\0';
		return;
	}
	hh  = (hhmmss[0] - '0') * 10 + (hhmmss[1] - '0');
	mi  = (hhmmss[2] - '0') * 10 + (hhmmss[3] - '0');
	sec = atof(hhmmss + 4);

	/* Always three decimals, whatever resolution the receiver sends. */
	snprintf(out, n, "%04d-%02d-%02dT%02d:%02d:%06.3fZ", year, mon, day, hh, mi, sec);
}
