#include "uart.h"

int uart_init(uart_t *obj, UART_HandleTypeDef *handle, cbuffer_t *cbuf)
{
    return 1;
}

int uart_receive(uart_t *obj)
{
    return 0;
}

int uart_send(uart_t *obj, void *data, size_t datasize)
{
    return 0;
}
