CFLAGS ?= -O2
CFLAGS += -Wall -Wextra

openwg79x_rc: main.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ main.c

clean:
	rm -f openwg79x_rc

.PHONY: clean
