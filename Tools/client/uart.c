#include <errno.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>

#include "uart.h"

#define DEFAULT_PORT_NAME     "/dev/ttyUSB0"
#define DEFAULT_UART_BAUDRATE 115200

static int set_attrs(int fd, int baudrate, int parity)
{
    struct termios2 tty;
    memset(&tty, 0, sizeof(tty));
    ioctl(fd, TCGETS2, &tty);

    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= BOTHER;
    tty.c_ispeed = baudrate;
    tty.c_ospeed = baudrate;

    // 8-bit chars
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    // disable break processing
    tty.c_iflag &= ~IGNBRK;
    // disable CR -> NL translation
    tty.c_iflag &= ~ICRNL;
    // no signaling chars, no echo, no canonical processing
    tty.c_lflag = 0;
    // no remapping, no delays
    tty.c_oflag = 0;
    // read doesn't block
    tty.c_cc[VMIN]  = 0;
    // No timeout
    tty.c_cc[VTIME] = 0;

    // shut off xon/xoff ctrl
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // ignore modem controls, enable reading
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    ioctl(fd, TCSETS2, &tty);

    return 0;
}

static void set_blocking(int fd, int should_block)
{
    struct termios2 tty;
    memset(&tty, 0, sizeof tty);
    ioctl(fd, TCGETS2, &tty);

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    // No timeout
    tty.c_cc[VTIME] = 0;

    ioctl(fd, TCSETS2, &tty);
}

int uart_open(const char *port, int baudrate)
{
    int fd = open(port ? port : DEFAULT_PORT_NAME,
                  O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        return -1;
    }

    set_attrs(fd, baudrate ? baudrate : DEFAULT_UART_BAUDRATE, 0);
    set_blocking(fd, 1);

    return fd;
}

int uart_close(int fd)
{
    if (fd < 0)
        return -1;

    close(fd);
    fd = -1;

    return 0;
}

int uart_read(int fd, unsigned char *buffer, unsigned int buffer_size)
{
    if (fd < 0)
        return -1;

    return read(fd, buffer, buffer_size);
}

int uart_write(int fd, unsigned char *buffer, unsigned int buffer_size)
{
    if (fd < 0)
        return -1;

    return write(fd, buffer, buffer_size);
}
