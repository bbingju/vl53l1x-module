#pragma once

#include <stdint.h>

typedef struct __packed {
    uint8_t id;
    uint8_t status;
    uint16_t range_mm;
} tof_result_t;

typedef struct __packed {
    
} tof_config_t;

typedef struct __packed {
    uint8_t type;
    uint8_t length;
    union {
        tof_result_t        tof_result_payload[12];
        tof_config_t        tof_config_payload;
    } payload;
} protocol_frame_t;

//Size of a given frame
#define FRAME_SIZE(__frame_ptr__)    ((__frame_ptr__)->length + 2)

//Max frame size (including crc)
#define MAX_FRAME_SIZE            (sizeof(protocol_frame_t) + 2)

#ifdef __cplusplus
extern "C" {
#endif

    /* int protocol_send_buffer(uint8_t *buffer, uint16_t len); */

#ifdef __cplusplus
}
#endif
