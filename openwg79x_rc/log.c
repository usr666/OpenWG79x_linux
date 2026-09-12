/*
 * log - printf with a level in front of the line; levels past loglevel are dropped.
 */
#include <stdarg.h>
#include <stdio.h>

#include "log.h"

static const char *const tags[] = { "E ", "W ", "I ", "D " };

loglevel_t loglevel = INFO;

void log_printf(loglevel_t level, const char *fmt, ...)
{
	va_list args;

	if (level > loglevel) {
		return;
	}

	fputs(tags[level], stdout);
	va_start(args, fmt);
	vprintf(fmt, args);
	va_end(args);
}
