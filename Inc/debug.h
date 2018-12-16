#pragma once

#ifdef USE_RTT_FOR_DEBUG
#include "SEGGER_RTT.h"
#endif

#ifdef USE_RTT_FOR_DEBUG
#define DBG_LOG(f_, ...)    SEGGER_RTT_printf(0, (f_), ##__VA_ARGS__)
#else
#define DBG_LOG(f_, ...)
#endif

