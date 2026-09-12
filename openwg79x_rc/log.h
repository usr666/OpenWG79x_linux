#ifndef LOG_H
#define LOG_H

#define LOG_ENABLED 1		/* 0 removes every logf call from the build */

typedef enum {
	ERROR = 0,
	WARN,
	INFO,
	DEBUG
} loglevel_t;

extern loglevel_t loglevel;	/* levels up to and including this are printed */

void log_printf(loglevel_t level, const char *fmt, ...);

#if LOG_ENABLED
#define logf(level, ...) log_printf(level, __VA_ARGS__)
#else
#define logf(level, ...)
#endif

#endif
