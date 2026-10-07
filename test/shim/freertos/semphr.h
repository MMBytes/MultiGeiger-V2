/** @file
 *  @brief Host-test shim for <freertos/semphr.h> (V2.8.8).
 *
 *  A mutex in a single-threaded test binary needs no locking, so take/give
 *  always succeed. Create returns a dummy non-NULL handle, unless a test sets
 *  shim_mutex_create_fails to exercise the module's "mutex alloc failed"
 *  path (history.c then disables itself — that branch is part of the
 *  contract). The flag is defined once in test/test_main.c.
 *  See test/shim/README.md.
 */
#pragma once

#include "freertos/FreeRTOS.h"

typedef void *SemaphoreHandle_t;

/** @brief Set non-zero to make the next xSemaphoreCreateMutex() return NULL. */
extern int shim_mutex_create_fails;

static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    static int token;   // any stable non-NULL address
    return shim_mutex_create_fails ? (SemaphoreHandle_t)0 : (SemaphoreHandle_t)&token;
}

// Functions, not comma-expression macros: firmware calls these as statements
// and ignores the result, which a macro would turn into -Wunused-value.
static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t ticks) {
    (void)sem; (void)ticks;
    return pdTRUE;
}

static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t sem) {
    (void)sem;
    return pdTRUE;
}
