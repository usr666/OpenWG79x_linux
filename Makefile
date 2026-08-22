CFLAGS ?= -O2
CFLAGS += -Wall -Wextra

openwg79x_rc: main.c gps.c gps.h mowercom.c mowercom.h tcpcom.c tcpcom.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ main.c gps.c mowercom.c tcpcom.c

clean:
	rm -f openwg79x_rc

.PHONY: clean
