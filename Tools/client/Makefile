
APP = tofclient

# CC = clang
CFLAGS += -Wall -std=gnu11
LDFLAGS = -leditline -lpthread

SRCS = main.c \
	protocol.c \
	uart.c

OBJS = ${SRCS:c=o}

.PHONY: clean

all: $(APP)

$(APP): $(OBJS)
	$(CC) $^ $(LDFLAGS) -o $@

clean:
	$(RM) $(APP) $(OBJS) *~ 

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c $<
