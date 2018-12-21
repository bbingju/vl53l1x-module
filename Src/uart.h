#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx_hal.h"
#include "cbuffer.h"
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256

typedef struct uart_s {
    UART_HandleTypeDef *handle;
    cbuffer_t *cbuffer;

    uint8_t tx_buffer[TX_BUFFER_SIZE];
} uart_t;

#ifdef __cplusplus
extern "C" {
#endif

int uart_init(uart_t *obj, UART_HandleTypeDef *handle, cbuffer_t *cbuf);

int uart_receive(uart_t *obj, uint8_t *data, uint16_t *size, uint32_t timeout);
int uart_send(uart_t *obj, void *data, size_t size);

#ifdef __cplusplus
}
#endif
