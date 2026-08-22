/*
 * tcpcom - sends a copy of every log line to connected TCP clients.
 */
#define _DEFAULT_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "tcpcom.h"

static int listen_fd = -1;
static int clients[TCPCOM_MAX_CLIENTS];

static void set_nonblock(int fd)
{
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
}

int tcpcom_open(void)
{
	struct sockaddr_in addr;
	int on = 1;
	int i;

	for (i = 0; i < TCPCOM_MAX_CLIENTS; i++)
		clients[i] = -1;

	listen_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (listen_fd < 0) {
		fprintf(stderr, "tcp socket: %s\n", strerror(errno));
		return -1;
	}

	setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
	set_nonblock(listen_fd);

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(TCPCOM_PORT);

	if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(listen_fd, TCPCOM_MAX_CLIENTS) < 0) {
		fprintf(stderr, "tcp port %d: %s\n", TCPCOM_PORT, strerror(errno));
		close(listen_fd);
		listen_fd = -1;
		return -1;
	}

	return listen_fd;
}

void tcpcom_accept(void)
{
	int fd = accept(listen_fd, NULL, NULL);
	int i;

	if (fd < 0)
		return;

	set_nonblock(fd);

	for (i = 0; i < TCPCOM_MAX_CLIENTS; i++) {
		if (clients[i] < 0) {
			clients[i] = fd;
			return;
		}
	}
	close(fd);	/* no free slot */
}

void tcpcom_broadcast(const char *text)
{
	size_t len = strlen(text);
	int i;

	if (listen_fd < 0)
		return;

	for (i = 0; i < TCPCOM_MAX_CLIENTS; i++) {
		if (clients[i] < 0)
			continue;
		if (write(clients[i], text, len) != (ssize_t)len) {
			close(clients[i]);
			clients[i] = -1;
		}
	}
}

void tcpcom_close(void)
{
	int i;

	for (i = 0; i < TCPCOM_MAX_CLIENTS; i++) {
		if (clients[i] >= 0) {
			close(clients[i]);
			clients[i] = -1;
		}
	}

	if (listen_fd >= 0) {
		close(listen_fd);
		listen_fd = -1;
	}
}
