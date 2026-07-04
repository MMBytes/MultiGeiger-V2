#pragma once

/** @file
 *  @brief Shared Sensirion CRC-8 (V2.6.6).
 *
 *  SHT45, SPS30, and DNMS (Nettigo's I2C protocol is Sensirion-derived) each
 *  reimplemented the identical polynomial/init constants independently —
 *  consolidated here so the wire-level detail is written once. `static
 *  inline` so each translation unit gets its own copy, same pattern as
 *  i2c_bus.h's per-device helpers.
 */

#include <stddef.h>
#include <stdint.h>

/** @brief Sensirion CRC-8: polynomial 0x31, init 0xFF, no reflection, no
 *  XOR-out. Computed MSB-first over `n` bytes — callers feed one 16-bit
 *  word (2 bytes) at a time per the Sensirion I2C protocol convention. */
static inline uint8_t sensirion_crc8(const uint8_t *data, size_t n) {
    uint8_t c = 0xFF;
    for (size_t i = 0; i < n; i++) {
        c ^= data[i];
        for (int b = 0; b < 8; b++) {
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x31) : (uint8_t)(c << 1);
        }
    }
    return c;
}
