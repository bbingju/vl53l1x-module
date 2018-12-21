#pragma once

#include <stdint.h>

#define PREAMBLE_OCTET 0xFD
#define END_OCTET      0xFE

typedef struct __packed {
    uint8_t id;
    uint8_t status;
    uint16_t range_mm;
} tof_result_t;

typedef struct __packed {
    uint8_t top_left_x;
    uint8_t top_left_y;
    uint8_t bot_right_x;
    uint8_t bot_right_y;
} roi_config_t;

#define FRAME_TYPE_STOP       0
#define FRAME_TYPE_START      1
#define FRAME_TYPE_ROI_CONFIG 2
#define FRAME_TYPE_TOF_RESULT 3

typedef struct __packed {
    uint8_t type;
    uint8_t length;
    union {
        tof_result_t        tof_result_payload[12];
        roi_config_t        roi_config[12];
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
