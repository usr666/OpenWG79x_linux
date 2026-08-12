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

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_UART_DEVICE "/dev/ttyUSB0"
#define GPSPIPE_CMD         "gpspipe -r"

#define SAMPLE_INTERVAL_MS  1000

#define LOG_HEADER \
	"#timestamp_utc,latitude_nmea,latitude_hemisphere," \
	"longitude_nmea,longitude_hemisphere,fix_quality,fix_mode," \
	"satellites_used,hdop,mower_state,battery_soc_percent," \
	"front_bump,lift,wire_right_inside,wire_left_inside,near_wire\n"

/* --- protocol ----------------------------------------------------------- */

#define FRAME_START      0x7E
#define FRAME_ESC        0x7D
#define ESC_XOR          0x20
#define MSG_MOWER_STATUS 0x00

/* MSG_ID + LEN + 255 payload + 2 CRC */
#define MAX_FRAME (2 + 255 + 2)

struct mower_status {
	uint8_t state;
	uint8_t sensor_status;
	uint8_t wire_sensor_status;
	int8_t  left_wheel_speed;
	int8_t  right_wheel_speed;
	uint8_t battery_soc;
	int     valid;
};

struct frame_parser {
	uint8_t buf[MAX_FRAME];
	size_t  len;
	int     in_frame;
	int     escaped;
};

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (int bit = 0; bit < 8; bit++)
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
					     : (uint16_t)(crc << 1);
	}
	return crc;
}

static void handle_frame(const uint8_t *buf, size_t len, struct mower_status *st)
{
	uint8_t  msg_id  = buf[0];
	uint8_t  payload = buf[1];
	uint16_t crc_rx, crc_calc;

	if (len != (size_t)payload + 4)		/* MSG_ID + LEN + payload + CRC */
		return;

	crc_rx   = (uint16_t)(buf[len - 2] << 8) | buf[len - 1];
	crc_calc = crc16_ccitt(buf, len - 2);
	if (crc_rx != crc_calc)
		return;

	if (msg_id == MSG_MOWER_STATUS && payload >= 6) {
		st->state              = buf[2];
		st->sensor_status      = buf[3];
		st->wire_sensor_status = buf[4];
		st->left_wheel_speed   = (int8_t)buf[5];
		st->right_wheel_speed  = (int8_t)buf[6];
		st->battery_soc        = buf[7];
		st->valid              = 1;
	}
}

static void parse_bytes(struct frame_parser *p, const uint8_t *data, size_t n,
			struct mower_status *st)
{
	for (size_t i = 0; i < n; i++) {
		uint8_t b = data[i];

		/* An unescaped START always resynchronizes. */
		if (b == FRAME_START) {
			p->len = 0;
			p->in_frame = 1;
			p->escaped = 0;
			continue;
		}
		if (!p->in_frame)
			continue;

		if (b == FRAME_ESC) {
			p->escaped = 1;
			continue;
		}
		if (p->escaped) {
			b ^= ESC_XOR;
			p->escaped = 0;
		}

		if (p->len >= MAX_FRAME) {	/* runaway frame, drop it */
			p->in_frame = 0;
			continue;
		}
		p->buf[p->len++] = b;

		/* Complete once LEN is known and all payload + CRC arrived. */
		if (p->len >= 2 && p->len == (size_t)p->buf[1] + 4) {
			handle_frame(p->buf, p->len, st);
			p->in_frame = 0;
		}
	}
}

/* --- uart --------------------------------------------------------------- */

static int uart_open(const char *device)
{
	struct termios tio;
	int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", device, strerror(errno));
		return -1;
	}
	if (tcgetattr(fd, &tio) < 0) {
		fprintf(stderr, "tcgetattr %s: %s\n", device, strerror(errno));
		close(fd);
		return -1;
	}

	cfmakeraw(&tio);			/* 8N1, no flow control */
	cfsetispeed(&tio, B38400);
	cfsetospeed(&tio, B38400);
	tio.c_cflag |= CLOCAL | CREAD;
	tio.c_cflag &= ~CRTSCTS;
	tio.c_cc[VMIN]  = 0;
	tio.c_cc[VTIME] = 0;

	if (tcsetattr(fd, TCSANOW, &tio) < 0) {
		fprintf(stderr, "tcsetattr %s: %s\n", device, strerror(errno));
		close(fd);
		return -1;
	}
	tcflush(fd, TCIFLUSH);
	return fd;
}

/* --- nmea --------------------------------------------------------------- */

/*
 * Coordinates are kept as the literal GGA field text - ddmm.mmmmm and the
 * hemisphere character - so nothing is lost to a conversion we would only
 * have to undo later.
 */
struct gps_fix {
	char lat[16];			/* ddmm.mmmmm  */
	char ns[2];			/* N or S      */
	char lon[16];			/* dddmm.mmmmm */
	char ew[2];			/* E or W      */
	char hdop[8];
	int  quality;			/* GGA fix quality, 0 = no fix */
	int  gsa_mode;			/* GSA: 1 none, 2 = 2D, 3 = 3D */
	int  sats;
	char time[32];			/* ISO8601 UTC, milliseconds */
	int  valid;
};

/*
 * Carried between sentences: the date lives in RMC and the 2D/3D distinction
 * in GSA, but the position itself arrives in GGA.
 */
struct nmea_state {
	int have_date;
	int year, mon, day;
	int gsa_mode;
	int warned_no_date;
};

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

/* Digits and at most one decimal point - enough to reject a garbled field. */
static int is_coord(const char *s)
{
	int digits = 0, dots = 0;

	for (; *s; s++) {
		if (isdigit((unsigned char)*s))
			digits++;
		else if (*s == '.' && ++dots == 1)
			continue;
		else
			return 0;
	}
	return digits >= 3;
}

/* "hhmmss.ss" + the date from RMC -> "YYYY-MM-DDThh:mm:ss.sssZ" */
static void nmea_timestamp(const char *hhmmss, struct nmea_state *ns,
			   char *out, size_t n)
{
	int year = ns->year, mon = ns->mon, day = ns->day;
	int hh, mi;
	double sec;

	if (!ns->have_date) {
		time_t now = time(NULL);
		struct tm tm;

		gmtime_r(&now, &tm);
		year = tm.tm_year + 1900;
		mon  = tm.tm_mon + 1;
		day  = tm.tm_mday;
		if (!ns->warned_no_date) {
			fprintf(stderr, "no RMC yet - dating samples from the "
					"system clock\n");
			ns->warned_no_date = 1;
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
	snprintf(out, n, "%04d-%02d-%02dT%02d:%02d:%06.3fZ",
		 year, mon, day, hh, mi, sec);
}

/*
 * One raw NMEA sentence from `gpspipe -r`. Talker IDs vary (GP/GN/GL/GA),
 * so only the last three characters of the sentence name are matched.
 */
static void parse_nmea_line(const char *line, struct nmea_state *ns,
			    struct gps_fix *fix)
{
	char buf[128];
	char *f[24];
	int nf;

	if (line[0] != '$' || strlen(line) < 9 || strlen(line) >= sizeof(buf))
		return;
	if (!nmea_checksum_ok(line))
		return;

	snprintf(buf, sizeof(buf), "%s", line);
	nf = nmea_split(buf, f, 24);

	if (!memcmp(buf + 3, "RMC", 3) && nf > 9) {
		/* field 9: ddmmyy */
		const char *d = f[9];
		int v;

		if (strlen(d) == 6) {
			ns->day  = (d[0] - '0') * 10 + (d[1] - '0');
			ns->mon  = (d[2] - '0') * 10 + (d[3] - '0');
			v        = (d[4] - '0') * 10 + (d[5] - '0');
			ns->year = (v < 80) ? 2000 + v : 1900 + v;
			ns->have_date = 1;
		}
		return;
	}

	/* field 2: 1 = no fix, 2 = 2D, 3 = 3D. Multi-constellation receivers
	 * send one GSA per talker; the last one in the cycle wins. */
	if (!memcmp(buf + 3, "GSA", 3) && nf > 2) {
		ns->gsa_mode = atoi(f[2]);
		return;
	}

	if (memcmp(buf + 3, "GGA", 3) || nf < 9)
		return;

	/* field 6: fix quality, 0 = no fix - keep the previous position */
	if (atoi(f[6]) == 0)
		return;
	if (!is_coord(f[2]) || !is_coord(f[4]))
		return;
	if ((*f[3] != 'N' && *f[3] != 'S') || (*f[5] != 'E' && *f[5] != 'W'))
		return;

	snprintf(fix->lat, sizeof(fix->lat), "%s", f[2]);
	snprintf(fix->ns,  sizeof(fix->ns),  "%s", f[3]);
	snprintf(fix->lon, sizeof(fix->lon), "%s", f[4]);
	snprintf(fix->ew,  sizeof(fix->ew),  "%s", f[5]);
	snprintf(fix->hdop, sizeof(fix->hdop), "%s", f[8]);

	fix->quality  = atoi(f[6]);
	fix->sats     = atoi(f[7]);
	fix->gsa_mode = ns->gsa_mode;

	nmea_timestamp(f[1], ns, fix->time, sizeof(fix->time));
	fix->valid = 1;
}

/* --- log ---------------------------------------------------------------- */

/*
 * Opened for append: a restart continues the same log instead of erasing it.
 * The column header is written only when the file is new.
 */
static FILE *log_open(const char *path)
{
	FILE *f = fopen(path, "a");

	if (!f) {
		fprintf(stderr, "open %s: %s\n", path, strerror(errno));
		return NULL;
	}
	if (ftell(f) == 0) {
		fprintf(f, "%s", LOG_HEADER);
		fflush(f);
	}
	return f;
}

/*
 * One line per sample. Append-only, so the file is never in a broken state:
 * losing power costs at most the line being written.
 */
static void log_write_sample(FILE *f, const struct gps_fix *fix,
			     const struct mower_status *st)
{
	fprintf(f, "%s,%s,%s,%s,%s,%d,%d,%d,%s,",
		fix->time, fix->lat, fix->ns, fix->lon, fix->ew,
		fix->quality, fix->gsa_mode, fix->sats, fix->hdop);

	/* The two status bytes are unpacked into one column per bit. */
	if (st->valid)
		fprintf(f, "%u,%u,%u,%u,%u,%u,%u\n",
			st->state, st->battery_soc,
			(st->sensor_status >> 0) & 1,
			(st->sensor_status >> 1) & 1,
			(st->wire_sensor_status >> 0) & 1,
			(st->wire_sensor_status >> 1) & 1,
			(st->wire_sensor_status >> 2) & 1);
	else
		fprintf(f, ",,,,,,\n");	/* no status frame yet */

	fflush(f);
}

/* --- main --------------------------------------------------------------- */

static volatile sig_atomic_t stop;

static void on_signal(int sig)
{
	(void)sig;
	stop = 1;
}

static long elapsed_ms(const struct timespec *from, const struct timespec *to)
{
	return (to->tv_sec - from->tv_sec) * 1000L +
	       (to->tv_nsec - from->tv_nsec) / 1000000L;
}

int main(int argc, char **argv)
{
	const char *uart_device = (argc > 1) ? argv[1] : DEFAULT_UART_DEVICE;
	const char *log_path;
	char default_log[64];

	struct frame_parser parser = { 0 };
	struct mower_status status = { 0 };
	struct nmea_state nmea = { 0 };
	struct gps_fix fix = { 0 };
	struct timespec last_sample;
	char gps_line[256];
	size_t gps_len = 0;
	unsigned long samples = 0;
	int uart_fd, gps_fd;
	FILE *gps, *log;

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	signal(SIGPIPE, SIG_IGN);

	if (argc > 2) {
		log_path = argv[2];
	} else {
		time_t now = time(NULL);
		struct tm tm;

		gmtime_r(&now, &tm);
		strftime(default_log, sizeof(default_log), "mowerlog_%Y%m%d_%H_%M_%S.csv", &tm);
		log_path = default_log;
	}

	uart_fd = uart_open(uart_device);
	if (uart_fd < 0)
		return 1;

	gps = popen(GPSPIPE_CMD, "r");
	if (!gps) {
		fprintf(stderr, "popen %s: %s\n", GPSPIPE_CMD, strerror(errno));
		close(uart_fd);
		return 1;
	}
	gps_fd = fileno(gps);
	fcntl(gps_fd, F_SETFL, O_NONBLOCK);

	log = log_open(log_path);
	if (!log) {
		pclose(gps);
		close(uart_fd);
		return 1;
	}

	printf("openwg79x_rc: uart=%s gps=\"%s\" log=%s\n",
	       uart_device, GPSPIPE_CMD, log_path);

	clock_gettime(CLOCK_MONOTONIC, &last_sample);

	while (!stop) {
		struct pollfd fds[2];
		struct timespec now;
		uint8_t buf[512];
		int nfds = 0, uart_idx, gps_idx = -1;
		long wait;
		ssize_t n;

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

		clock_gettime(CLOCK_MONOTONIC, &now);
		wait = SAMPLE_INTERVAL_MS - elapsed_ms(&last_sample, &now);
		if (wait < 0)
			wait = 0;

		if (poll(fds, nfds, (int)wait) < 0) {
			if (errno == EINTR)
				continue;
			fprintf(stderr, "poll: %s\n", strerror(errno));
			break;
		}

		if (fds[uart_idx].revents & POLLIN) {
			n = read(uart_fd, buf, sizeof(buf));
			if (n > 0)
				parse_bytes(&parser, buf, (size_t)n, &status);
		}

		if (gps_idx >= 0 && (fds[gps_idx].revents & (POLLIN | POLLHUP))) {
			n = read(gps_fd, buf, sizeof(buf));
			if (n > 0) {
				for (ssize_t i = 0; i < n; i++) {
					if (buf[i] == '\n' || buf[i] == '\r') {
						if (gps_len) {
							gps_line[gps_len] = '\0';
							parse_nmea_line(gps_line,
									&nmea,
									&fix);
							gps_len = 0;
						}
					} else if (gps_len < sizeof(gps_line) - 1) {
						gps_line[gps_len++] = (char)buf[i];
					}
				}
			} else if (n == 0 || (n < 0 && errno != EAGAIN &&
					      errno != EWOULDBLOCK)) {
				fprintf(stderr,
					"gpspipe closed - no more position data\n");
				gps_fd = -1;
			}
		}

		clock_gettime(CLOCK_MONOTONIC, &now);
		if (elapsed_ms(&last_sample, &now) >= SAMPLE_INTERVAL_MS) {
			last_sample = now;
			if (fix.valid) {
				log_write_sample(log, &fix, &status);
				samples++;
				printf("\r%lu samples  %s%s %s%s  %dsat  "
				       "state=%u soc=%u%%  ",
				       samples, fix.lat, fix.ns, fix.lon,
				       fix.ew, fix.sats, status.state,
				       status.battery_soc);
				fflush(stdout);
			}
		}
	}

	fclose(log);
	pclose(gps);
	close(uart_fd);

	printf("\nwrote %lu samples to %s\n", samples, log_path);
	return 0;
}
