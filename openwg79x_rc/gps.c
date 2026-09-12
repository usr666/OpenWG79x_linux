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
#include "log.h"

#define GPSPIPE_CMD "gpspipe -r"

static FILE  *gps_pipe;
static int    gps_fd = -1;
static char   gps_line[GPS_LINE_MAX];
static int    no_of_stored_chars=0;
static bool   at_eof;

static int nmea_checksum_ok(const char *s)
{
	uint8_t sum = 0;
	const char *p;

	if (*s != '$') {
		return 0;
	}
	for (p = s + 1; *p && *p != '*'; p++) {
		sum ^= (uint8_t)*p;
	}
	if (*p != '*' || !isxdigit((unsigned char)p[1]) || !isxdigit((unsigned char)p[2])) {
		return 0;
	}
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
 * Talker IDs vary (GP/GN/GL/GA), so only the last three characters of the
 * sentence name are matched.
 */
static gps_recordtype record_type(const char *line)
{
	if (line[0] != '$' || strlen(line) < 9) {
		return gps_none;
	}
	if (!memcmp(line + 3, "GGA", 3)) {
		return gps_gga;
	}
	if (!memcmp(line + 3, "RMC", 3)) {
		return gps_rmc;
	}
	return gps_none;
}

bool parse_gga(const char *line, nmea_gga_t *gga)
{
	char buf[GPS_LINE_MAX];
	char *f[24];

	if (!nmea_checksum_ok(line)) {
		return false;
	}

	snprintf(buf, sizeof(buf), "%s", line);
	if (nmea_split(buf, f, 24) <= 8) {
		return false;
	}

	snprintf(gga->time,    sizeof(gga->time),    "%s", f[1]);
	snprintf(gga->lat,     sizeof(gga->lat),     "%s", f[2]);
	snprintf(gga->ns,      sizeof(gga->ns),      "%s", f[3]);
	snprintf(gga->lon,     sizeof(gga->lon),     "%s", f[4]);
	snprintf(gga->ew,      sizeof(gga->ew),      "%s", f[5]);
	snprintf(gga->quality, sizeof(gga->quality), "%s", f[6]);
	snprintf(gga->sats,    sizeof(gga->sats),    "%s", f[7]);
	snprintf(gga->hdop,    sizeof(gga->hdop),    "%s", f[8]);
	gga->valid = 1;
	return true;
}

bool parse_rmc(const char *line, nmea_rmc_t *rmc)
{
	char buf[GPS_LINE_MAX];
	char *f[24];

	if (!nmea_checksum_ok(line)) {
		return false;
	}

	snprintf(buf, sizeof(buf), "%s", line);
	if (nmea_split(buf, f, 24) <= 9) {
		return false;
	}

	snprintf(rmc->date, sizeof(rmc->date), "%s", f[9]);
	rmc->valid = 1;
	return true;
}

/* NMEA ddmm.mmmmm plus hemisphere to decimal degrees */
static double nmea_to_degrees(const char *value, const char *hemisphere)
{
	double v, minutes;
	int deg;

	v = atof(value);
	deg = (int)(v / 100);
	minutes = v - deg * 100;
	if (*hemisphere == 'S' || *hemisphere == 'W') {
		return -(deg + minutes / 60.0);
	}

	return deg + minutes / 60.0;
}

point_t nmea_to_point(const char *lat, const char *ns, const char *lon, const char *ew)
{
	point_t p;

	p.lat = nmea_to_degrees(lat, ns);
	p.lon = nmea_to_degrees(lon, ew);

	return p;
}

int gps_open(void)
{
	gps_pipe = popen(GPSPIPE_CMD, "r");
	if (!gps_pipe) {
		logf(INFO, "popen %s: %s\n", GPSPIPE_CMD, strerror(errno));
		return -1;
	}
	gps_fd = fileno(gps_pipe);
	fcntl(gps_fd, F_SETFL, O_NONBLOCK);
	return gps_fd;
}

gps_recordtype gps_read(char *line)
{
	ssize_t n;
	int i, chars_left;

	if(no_of_stored_chars == (int)sizeof(gps_line)) {	/* no line break, drop it */
		no_of_stored_chars = 0;
	}

	n = read(gps_fd, &gps_line[no_of_stored_chars], sizeof(gps_line) - no_of_stored_chars);
	if(n > 0) {
		no_of_stored_chars += n;
	} else if(n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
		at_eof = true;
	}
	for(i=0;i<no_of_stored_chars;i++) {
		if(gps_line[i] == '\n') {
			memcpy(line, gps_line, i);
			line[i] = '\0';
			if(i > 0 && line[i-1] == '\r') {
				line[i-1] = '\0';
			}
			chars_left = no_of_stored_chars - i - 1;
			memmove(gps_line, &gps_line[i+1], chars_left);
			no_of_stored_chars = chars_left;
			if(nmea_checksum_ok(line) != 0) {
				return record_type(line);
			} else {
				return gps_none;
			}
		}
	}
	return gps_none;
}

bool gps_eof(void)
{
	return at_eof;
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
			logf(INFO, "no RMC yet - dating samples from the system clock\n");
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
