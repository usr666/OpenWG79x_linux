#ifndef TCPCOM_H
#define TCPCOM_H

#define TCPCOM_PORT 32000
#define TCPCOM_MAX_CLIENTS 4

int  tcpcom_open(void);			/* listening socket, returns fd to poll, -1 on error */
void tcpcom_accept(void);		/* call when the listen fd is readable */
void tcpcom_broadcast(const char *text);	/* copy of one log line to every client */
void tcpcom_close(void);

#endif
