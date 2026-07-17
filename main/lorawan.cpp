// LoRaWAN uplink for BOARD_HELTEC_WIFI_LORA32_V4_R2 — the tree's ONLY C++
// translation unit, kept behind the plain-C API in lorawan.h. Whole file is
// gated on CONFIG_GEIGER_LORAWAN (set only in that board's sdkconfig): the
// CMake SRCS conditional keeps it out of other boards' builds, and this
// guard additionally yields an empty TU for the cppcheck gate's other-board
// define sets (_build.cmd scans all of main/ for every board).
#include "sdkconfig.h"
#if CONFIG_GEIGER_LORAWAN

// Implementation lands in Tasks 5-6.

#endif // CONFIG_GEIGER_LORAWAN
