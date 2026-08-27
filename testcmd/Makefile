CFLAGS ?= -O2
CFLAGS += -Wall -Wextra

testcmd: testcmd.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ testcmd.c

clean:
	rm -f testcmd

.PHONY: clean
