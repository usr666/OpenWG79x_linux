#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stddef.h>

#define GPS_LINE_MAX 256

typedef enum {
	gps_none = 0,
	gps_gga,
	gps_rmc
} gps_recordtype;

typedef struct {
	double lat;			/* decimal degrees, negative for S */
	double lon;			/* decimal degrees, negative for W */
} point_t;

typedef struct {
	char time[16];			/* hhmmss.ss   */
	char lat[16];			/* ddmm.mmmmm  */
	char ns[2];			/* N or S      */
	char lon[16];			/* dddmm.mmmmm */
	char ew[2];			/* E or W      */
	char quality[4];
	char sats[4];
	char hdop[8];
	int  valid;
} nmea_gga_t;

typedef struct {
	char date[8];			/* ddmmyy */
	int  valid;
} nmea_rmc_t;

int  gps_open(void);		/* starts gpspipe, returns fd to poll, -1 on error */
/* Stores one sentence in line, which must hold GPS_LINE_MAX bytes, and names its type.
   Reads at most one line per call and keeps an incomplete one internally. */
gps_recordtype gps_read(char *line);
bool parse_gga(const char *line, nmea_gga_t *gga);
point_t nmea_to_point(const char *lat, const char *ns, const char *lon, const char *ew);
bool parse_rmc(const char *line, nmea_rmc_t *rmc);
bool gps_eof(void);		/* gpspipe closed, no more position data */
void gps_close(void);
void gps_timestamp(const nmea_gga_t *gga, const nmea_rmc_t *rmc, char *out, size_t n);

#endif
