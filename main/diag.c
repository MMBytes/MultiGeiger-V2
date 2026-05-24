#include "diag.h"

#include <stdatomic.h>

static atomic_uint_least32_t g_i2c_errors = 0;

void diag_i2c_error_inc(void) {
    atomic_fetch_add(&g_i2c_errors, 1);
}

uint32_t diag_i2c_errors(void) {
    return (uint32_t)atomic_load(&g_i2c_errors);
}
