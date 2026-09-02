CFLAGS ?= -O2
CFLAGS += -Wall -Wextra

openwg79x_rc: main.c gps.c gps.h mowercom.c mowercom.h mowercontrol.c mowercontrol.h tcpcom.c tcpcom.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ main.c gps.c mowercom.c mowercontrol.c tcpcom.c -lm

clean:
	rm -f openwg79x_rc

.PHONY: clean
