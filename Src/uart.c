#include "uart.h"
#include "debug.h"
#include "main.h"

int uart_init(uart_t *obj, UART_HandleTypeDef *handle, cbuffer_t *cbuf)
{
    obj->handle = handle;
    obj->cbuffer   = cbuf;

    return 0;
}

int uart_receive(uart_t *obj, uint8_t *data, uint16_t size, uint32_t timeout)
{
    HAL_StatusTypeDef status;

    status = HAL_UART_Receive(obj->handle, data, size, timeout);
    if (status != HAL_OK) {
        DBG_LOG("%s:%L HAL_UART_Recevie (%d)", __func__, __LINE__, status);
        return -1;
    }

    return size;
}

int uart_send(uart_t *obj, void *data, size_t size)
{
    HAL_StatusTypeDef status;
    status = HAL_UART_Transmit(obj->handle, data, size, 0xFFFF );
    if (status != HAL_OK) {
        DBG_LOG("%s:%L HAL_UART_Transmit error (%d)", __func__, __LINE__, status);
        return -1;
    }

    return size;
}
