# Segger RTT sources
RTT_C_INCLUDES = \
-ILib/RTT

RTT_C_SOURCES = \
Lib/RTT/SEGGER_RTT.c \
Lib/RTT/SEGGER_RTT_printf.c \
Lib/Syscalls/SEGGER_RTT_Syscalls_GCC.c


# VL53L1X driver sources
VL53L1X_C_INCLUDES = \
-Ivl53l1x/core \
-Ivl53l1x/platform

VL53L1X_C_SOURCES = \
vl53l1x/core/vl53l1_api.c \
vl53l1x/core/vl53l1_api_calibration.c \
vl53l1x/core/vl53l1_api_core.c \
vl53l1x/core/vl53l1_api_debug.c \
vl53l1x/core/vl53l1_api_preset_modes.c \
vl53l1x/core/vl53l1_api_strings.c \
vl53l1x/core/vl53l1_core.c \
vl53l1x/core/vl53l1_core_support.c \
vl53l1x/core/vl53l1_error_strings.c \
vl53l1x/core/vl53l1_register_funcs.c \
vl53l1x/core/vl53l1_silicon_core.c \
vl53l1x/core/vl53l1_wait.c \
vl53l1x/platform/vl53l1_platform.c
