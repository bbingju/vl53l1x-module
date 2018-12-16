#include "uart.h"
#include "debug.h"
#include "main.h"

int uart_init(uart_t *obj, UART_HandleTypeDef *handle, cbuffer_t *cbuf)
{
    obj->handle = handle;
    obj->cbuffer   = cbuf;

    return 0;
}

int uart_receive(uart_t *obj)
{
    return 0;
}

int uart_send(uart_t *obj, void *data, size_t datasize)
{
    HAL_StatusTypeDef status;
    status = HAL_UART_Transmit(obj->handle, data, datasize, 0xFFFF );
    if (status != HAL_OK) {
        DBG_LOG("%s:%L HAL_UART_Transmit error (%d)", __func__, __LINE__, status);
        return -1;
    }

    return datasize;
}
