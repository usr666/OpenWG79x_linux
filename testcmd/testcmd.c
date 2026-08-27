/*
 * testcmd - sends one remote control command to a local openwg79x_rc over tcp.
 *
 * Frames go out unescaped, as the tcp side expects. Run without arguments for usage.
 */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define TCP_PORT 32000
#define FRAME_START 0x7E
#define MSG_REMOTE_CONTROL_RUN 0x01
#define MSG_REMOTE_CONTROL_TURN 0x02
#define MAX_TX 16

static const char usage[] =
	"usage: testcmd run <left> <right> <disc> <force>\n"
	"       testcmd turn <speed> <angle> <dir> <disc> <force>\n"
	"\n"
	"  left, right  wheel speed in percent, -100..100, negative runs backwards\n"
	"  speed        nominal wheel speed during the turn, -100..100\n"
	"  angle        degrees to turn, 0..255\n"
	"  dir          turn direction, 0 = left, 1 = right\n"
	"  disc         cutting disc speed in percent, -100..100, 0 stops the disc\n"
	"  force        1 keeps running when hitting obstacles, 0 stops on them\n";

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (int bit = 0; bit < 8; bit++)
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
	}
	return crc;
}

static uint8_t arg(char **argv, int i, long lo, long hi, const char *name)
{
	long v = strtol(argv[i], NULL, 0);

	if (v < lo || v > hi) {
		fprintf(stderr, "%s must be %ld..%ld\n", name, lo, hi);
		exit(1);
	}
	return (uint8_t)v;
}

int main(int argc, char **argv)
{
	uint8_t frame[MAX_TX];
	struct sockaddr_in addr;
	size_t n = 0;
	uint16_t crc;
	int fd;

	frame[n++] = FRAME_START;

	if (argc == 6 && !strcmp(argv[1], "run")) {
		frame[n++] = MSG_REMOTE_CONTROL_RUN;
		frame[n++] = 4;
		frame[n++] = arg(argv, 2, -100, 100, "left");
		frame[n++] = arg(argv, 3, -100, 100, "right");
		frame[n++] = arg(argv, 4, -100, 100, "disc");
		frame[n++] = arg(argv, 5, 0, 1, "force");
	} else if (argc == 7 && !strcmp(argv[1], "turn")) {
		frame[n++] = MSG_REMOTE_CONTROL_TURN;
		frame[n++] = 5;
		frame[n++] = arg(argv, 2, -100, 100, "speed");
		frame[n++] = arg(argv, 3, 0, 255, "angle");
		frame[n++] = arg(argv, 4, 0, 1, "dir");
		frame[n++] = arg(argv, 5, -100, 100, "disc");
		frame[n++] = arg(argv, 6, 0, 1, "force");
	} else {
		fprintf(stderr, "%s", usage);
		return 1;
	}

	crc = crc16_ccitt(frame + 1, n - 1);		/* msg_id + len + payload */
	frame[n++] = (uint8_t)(crc >> 8);
	frame[n++] = (uint8_t)crc;

	fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0) {
		perror("socket");
		return 1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = htons(TCP_PORT);

	if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("connect");
		close(fd);
		return 1;
	}

	if (write(fd, frame, n) != (ssize_t)n) {
		perror("write");
		close(fd);
		return 1;
	}

	printf("sent %zu bytes:", n);
	for (size_t i = 0; i < n; i++)
		printf(" %02X", frame[i]);
	printf("\n");

	close(fd);
	return 0;
}
