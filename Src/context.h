#pragma once

#include "state.h"
#include "uart.h"

struct context_s {
    state_t state;
    uart_t uart;

    uint16_t measure_interval;
    uint8_t distance_mode;      // 0: short, 1: medium, 2: long, 3: unknown
};

extern struct context_s *pctx;
