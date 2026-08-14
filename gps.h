#ifndef GPS_H
#define GPS_H

#include <stddef.h>

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
/* -1 closed, 0 nothing complete yet, 1 gga updated, 2 rmc updated */
int  gps_read(nmea_gga_t *gga, nmea_rmc_t *rmc);
void gps_close(void);
void gps_timestamp(const nmea_gga_t *gga, const nmea_rmc_t *rmc, char *out, size_t n);

#endif
