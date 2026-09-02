/*
 * mowercom - Communication with lawnmower
 */
#define _DEFAULT_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "mowercom.h"

#define FRAME_START      0x7E
#define FRAME_ESC        0x7D
#define ESC_XOR          0x20

/* MSG_ID + LEN + 255 payload + 2 CRC */
#define MAX_FRAME (2 + 255 + 2)

typedef struct {
	uint8_t buf[MAX_FRAME];
	size_t  len;
	int     in_frame;
	int     escaped;
} frame_parser_t;

static int    uart_fd = -1;
static char   uart_name[64];
static frame_parser_t parser;
static uint8_t rx[512];
static size_t  rx_len, rx_pos;

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (int bit = 0; bit < 8; bit++) {
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static int handle_frame(const uint8_t *buf, size_t len, uint8_t *msgid, void *data, size_t size, size_t *msglen)
{
	uint8_t  payload = buf[1];
	uint16_t crc_rx, crc_calc;

	if (len != (size_t)payload + 4) {	/* MSG_ID + LEN + payload + CRC */
		return 0;
	}
	if (payload > size) {
		return 0;
	}

	crc_rx   = (uint16_t)(buf[len - 2] << 8) | buf[len - 1];
	crc_calc = crc16_ccitt(buf, len - 2);
	if (crc_rx != crc_calc) {
		return 0;
	}

	*msgid = buf[0];
	*msglen = payload;
	memcpy(data, buf + 2, payload);
	return 1;
}

static int parse_byte(uint8_t b, uint8_t *msgid, void *data, size_t size, size_t *msglen)
{
	/* An unescaped START always resynchronizes. */
	if (b == FRAME_START) {
		parser.len = 0;
		parser.in_frame = 1;
		parser.escaped = 0;
		return 0;
	}
	if (!parser.in_frame) {
		return 0;
	}

	if (b == FRAME_ESC) {
		parser.escaped = 1;
		return 0;
	}
	if (parser.escaped) {
		b ^= ESC_XOR;
		parser.escaped = 0;
	}

	if (parser.len >= MAX_FRAME) {		/* runaway frame, drop it */
		parser.in_frame = 0;
		return 0;
	}
	parser.buf[parser.len++] = b;

	/* Complete once LEN is known and all payload + CRC arrived. */
	if (parser.len >= 2 && parser.len == (size_t)parser.buf[1] + 4) {
		parser.in_frame = 0;
		return handle_frame(parser.buf, parser.len, msgid, data, size, msglen);
	}
	return 0;
}

int mowercom_open(const char *device)
{
	struct termios tio;
	int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		printf("open %s: %s\n", device, strerror(errno));
		return -1;
	}
	if (tcgetattr(fd, &tio) < 0) {
		printf("tcgetattr %s: %s\n", device, strerror(errno));
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
		printf("tcsetattr %s: %s\n", device, strerror(errno));
		close(fd);
		return -1;
	}
	tcflush(fd, TCIFLUSH);

	snprintf(uart_name, sizeof(uart_name), "%s", device);
	uart_fd = fd;
	return uart_fd;
}

int mowercom_read(uint8_t *msgid, void *data, size_t size, size_t *len)
{
	ssize_t n;

	for (;;) {
		while (rx_pos < rx_len) {
			if (parse_byte(rx[rx_pos++], msgid, data, size, len)) {
				return 1;
			}
		}
		rx_pos = rx_len = 0;

		n = read(uart_fd, rx, sizeof(rx));
		if (n == 0) {
			return 0;
		}
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				return 0;
			}
			printf("read %s: %s\n", uart_name, strerror(errno));
			return -1;
		}
		rx_len = (size_t)n;
	}
}

int mowercom_send(uint8_t msgid, const void *data, size_t len)
{
	uint8_t frame[MAX_FRAME_SIZE];
	uint8_t wire[2 * MAX_FRAME_SIZE];
	size_t f = 0, n = 0, i;
	uint16_t crc;

	if (uart_fd < 0 || len > MAX_MSG_SIZE) {
		return -1;
	}

	frame[f++] = msgid;
	frame[f++] = (uint8_t)len;
	if (len) {
		memcpy(frame + f, data, len);
	}
	f += len;
	crc = crc16_ccitt(frame, f);
	frame[f++] = (uint8_t)(crc >> 8);
	frame[f++] = (uint8_t)crc;

	wire[n++] = FRAME_START;
	for (i = 0; i < f; i++) {
		if (frame[i] == FRAME_START || frame[i] == FRAME_ESC) {
			wire[n++] = FRAME_ESC;
			wire[n++] = frame[i] ^ ESC_XOR;
		} else {
			wire[n++] = frame[i];
		}
	}

	if (write(uart_fd, wire, n) != (ssize_t)n) {
		printf("write %s: %s\n", uart_name, strerror(errno));
		return -1;
	}
	return 0;
}

void mowercom_close(void)
{
	if (uart_fd >= 0) {
		close(uart_fd);
		uart_fd = -1;
	}
}
