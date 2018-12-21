#include "uart.h"
#include "context.h"
#include "debug.h"
#include "main.h"
#include "protocol.h"

#include <string.h>

// lut table size 512B (256 * 16bit)
static const uint16_t crc_ccitt_lut[] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, \
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, \
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, \
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, \
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, \
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, \
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, \
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, \
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, \
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, \
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, \
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, \
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, \
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, \
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, \
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, \
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, \
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, \
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, \
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, \
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, \
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, \
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, \
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, \
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, \
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, \
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, \
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, \
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, \
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, \
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, \
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 \
};

__IO ITStatus UartReady = RESET;
__IO ITStatus UartTxCompleted = RESET;

/**
 * Compute the crc of a frame
 */
static uint16_t crc_from_buffer(uint8_t * buf,
                                uint32_t len)
{
    uint16_t crc = 0xffff;
    uint8_t index;
    uint32_t i;

    for (i = 0; i < len; i++)
    {
        index = buf[i] ^ (crc >> 8);
        crc = crc_ccitt_lut[index] ^ (crc << 8);
    }
    return crc;
}

static int encode(uint8_t *buffer_in, uint16_t len_in,
                  uint8_t *buffer_out, uint16_t len_out)
{
    int offset = 0;

    if (len_out < len_in + 5)
        return -1;

    uint16_t crc = crc_from_buffer(buffer_in, len_in);
    DBG_LOG("%s: encoded crc = 0x%04X\n", __func__, crc);

    *(buffer_out + offset) = PREAMBLE_OCTET;
    offset++;
    *(buffer_out + offset) = PREAMBLE_OCTET;
    offset++;

    memcpy(buffer_out + offset, buffer_in, len_in);
    offset += len_in;

    *((uint16_t *)(buffer_out + offset)) = crc;
    offset += 2;

    *(buffer_out + offset) = END_OCTET;
    offset++;

    return offset;
}

static int decode(uint8_t *buffer, uint16_t len)
{
    uint16_t crc, crc_from_frame;

    crc = crc_from_buffer(buffer, len);
    crc_from_frame = *((uint16_t *) (buffer + len - 2));

    if (crc != crc_from_frame) {
        DBG_LOG("%s: crc mismatch 0x%04X 0x%04X\r\n", __func__, crc, crc_from_frame);
        return -1;
    }

    return len - 2;
}

int uart_init(uart_t *obj, UART_HandleTypeDef *handle, cbuffer_t *cbuf)
{
    obj->handle = handle;
    obj->cbuffer = cbuf;

    return 0;
}

/* int uart_receive(uart_t *obj, uint8_t *data, uint16_t *size, uint32_t timeout) */
/* { */
/*     HAL_StatusTypeDef status; */

/*     /\* status = HAL_UART_Receive(obj->handle, rx_buffer, size, timeout); *\/ */
/*     /\* if (status != HAL_OK) { *\/ */
/*     /\*     DBG_LOG("%s HAL_UART_Recevie error (%d)\n", __func__, status); *\/ */
/*     /\*     return -1; *\/ */
/*     /\* } *\/ */
/*     HAL_UART_Receive_IT(obj->handle, uart_rx_buffer, 1); */

/*     /\* cbuffer_t *cbfr = obj->cbuffer; *\/ */
/*     /\* uint8_t c; *\/ */
/*     /\* if (cbuffer_len(cbfr) < 7) *\/ */
/*     /\*     return -1; *\/ */

/*     /\* /\\* while (!cbuffer_isempty(cbfr)) { *\\/ *\/ */
/*     /\* /\\*     cbuffer_peek(cbfr, &c); *\\/ *\/ */
      
/*     /\* /\\*     if (c != PREAMBLE_OCTET) *\\/ *\/ */
/*     /\* /\\*         cbuffer_pop(cbfr, &c); *\\/ *\/ */
/*     /\* /\\*     else *\\/ *\/ */
/*     /\* /\\*         break; *\\/ *\/ */
/*     /\* /\\* } *\\/ *\/ */

/*     /\* static uint8_t buffer[256] = { 0 }; *\/ */
/*     /\* static int count = 0; *\/ */
/*     /\* while (!cbuffer_isempty(cbfr)) { *\/ */
/*     /\*     cbuffer_pop(cbfr, &buffer[count++]); *\/ */
/*     /\* } *\/ */

/*     /\* DBG_LOG("%s: hexdump\r\n", __func__); *\/ */
/*     /\* for (int i = 0; i < count; i++) { *\/ */
/*     /\*     DBG_LOG("0x%02X ", buffer[i]); *\/ */
/*     /\*     if (i % 10 == 9) *\/ */
/*     /\*         DBG_LOG("\r\n"); *\/ */
/*     /\* } *\/ */
/*     /\* DBG_LOG("\r\n"); *\/ */

/*     /\* /\\* validate the byte stream *\\/ *\/ */
/*     /\* int offset = 0; *\/ */
/*     /\* uint8_t length = buffer[3]; *\/ */
/*     /\* if (buffer[4 + length + 2] != END_OCTET) { *\/ */
/*     /\*     DBG_LOG("Gatcha!!!\n"); *\/ */
/*     /\*     /\\* if (crc check error) *\\/ *\/ */
/*     /\*     return -1; *\/ */
/*     /\* } *\/ */

/*     /\* memcpy(data, &buffer[2], length + 2); *\/ */
/*     /\* if (rx_buffer[0] == PREAMBLE_OCTET && *\/ */
/*     /\*     rx_buffer[1] == PREAMBLE_OCTET) *\/ */
/*     /\* { *\/ */
/*     /\*     uint8_t length = rx_buffer[3]; *\/ */
/*     /\*     if (length < 128 && rx_buffer[4 + length + 2] == END_OCTET) { *\/ */
/*     /\*         memcpy(data, &rx_buffer[2], length + 2); *\/ */
/*     /\*         return length + 2; *\/ */
/*     /\*     } *\/ */
/*     /\*     /\\* int offset = 2; *\\/ *\/ */
/*     /\*     /\\* int ret; *\\/ *\/ */
/*     /\*     /\\* while (rx_buffer[offset] != END_OCTET) { *\\/ *\/ */
/*     /\*     /\\*     ret = cbuffer_push(obj->cbuffer, rx_buffer[offset]); *\\/ *\/ */
/*     /\*     /\\*     if (ret == -1) *\\/ *\/ */
/*     /\*     /\\*         break; *\\/ *\/ */
/*     /\*     /\\* } *\\/ *\/ */
/*     /\* } *\/ */

/*     return 0; */
/* } */

int uart_send(uart_t *obj, void *data, size_t size)
{
    int encoded_size = encode(data, size, obj->tx_buffer, 256);
    if (encoded_size == -1)
        return -1;

    /* DBG_LOG("%s: encoded size = %d\r\n", __func__, encoded_size); */
    /* for (int i = 0; i < encoded_size; i++) { */
    /*     DBG_LOG("0x%02X ", obj->tx_buffer[i]); */
    /*     if (i % 10 == 9) */
    /*         DBG_LOG("\r\n"); */
    /* } */
    /* DBG_LOG("\r\n"); */

    HAL_StatusTypeDef status;
    UartTxCompleted = RESET;
    status = HAL_UART_Transmit_DMA(obj->handle, obj->tx_buffer, encoded_size);
    if (status != HAL_OK) {
        DBG_LOG("%s:%L HAL_UART_Transmit_DMA error (%d)\n", __func__, __LINE__, status);
        return -1;
    }

    while (1) {
        if (UartTxCompleted == SET) {
            HAL_UART_DMAStop(obj->handle);
            break;
        }
    }
    UartTxCompleted = RESET;

    return encoded_size;
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Set transmission flag: trasfer complete*/
    /* if (huart->Instance == USART1) { */
    /*     uint8_t *ptr = &uart_rx_buffer[0]; */
    /*     while (ptr != huart->pRxBuffPtr) { */
    /*         cbuffer_push(pctx->uart.cbuffer, *ptr); */
    /*         ptr++; */
    /*     } */
    /*     __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST); */
    /*     /\* __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE); *\/ */
    /* } */
    UartReady = SET;
}

/**
  * @brief Tx Transfer completed callback.
  * @param huart UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* DBG_LOG("%s\n", __func__); */
    UartTxCompleted = SET;
}
