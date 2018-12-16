#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int uart_open(const char *port, int baudrate);
int uart_close(int fd);
int uart_read(int fd, unsigned char *buffer, unsigned int buffer_size);
int uart_write(int fd, unsigned char *buffer, unsigned int buffer_size);

#ifdef __cplusplus
}
#endif
