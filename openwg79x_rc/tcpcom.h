#ifndef TCPCOM_H
#define TCPCOM_H

#include <stddef.h>
#include <stdint.h>

#define TCPCOM_PORT 32000
#define TCPCOM_MAX_CLIENTS 4
#define TCPCOM_MAX_FDS (TCPCOM_MAX_CLIENTS + 1)

int  tcpcom_open(void);			/* listening socket, returns fd to poll, -1 on error */
int  tcpcom_fds(int *fds);		/* listen fd + clients, returns count, fds[TCPCOM_MAX_FDS] */
void tcpcom_accept(void);		/* call when the listen fd is readable */
/* Bytes from one client, returns how many, 0 when none are waiting. Call until 0. */
int  tcpcom_read(uint8_t *data, size_t size);
void tcpcom_broadcast(const char *text);	/* copy of one log line to every client */
void tcpcom_close(void);

#endif
