/** @file
 *  @brief Host-test shim for <freertos/FreeRTOS.h> (V2.8.8).
 *
 *  Only the base types and constants the host-compiled firmware units use.
 *  The host test binary is single-threaded, so there is no scheduler here.
 *  See test/shim/README.md.
 */
#pragma once

#include <stdint.h>

typedef uint32_t TickType_t;
typedef int      BaseType_t;

#define pdTRUE         1
#define pdFALSE        0
#define portMAX_DELAY  ((TickType_t)0xFFFFFFFFu)
