#pragma once

/** @file
 *  @brief Tiny utility helpers shared across modules.
 *
 *  Header-only — no util.c needed, no symbol bloat, no CMake change.
 */

#include <string.h>
#include <stddef.h>

/** @brief Bounded string copy with guaranteed null termination.
 *
 *  Drop-in replacement for the `strncpy(dst, src, n-1); dst[n-1] = 0;`
 *  pattern that was repeated ~8 times across config / main / log_ftp.
 *  Argument order matches BSD `strlcpy` (dst, src, dstsz) — NOT the
 *  snprintf-style (dst, dstsz, src) of the legacy `assign_str` helpers.
 *
 *  Truncates silently if src is longer than the destination buffer.
 *  No-op (and safe) when dstsz == 0.
 *
 *  V2.4.1 (C1): consolidated from the inline pattern; not using BSD's
 *  strlcpy directly because ESP-IDF's newlib doesn't ship it.
 */
static inline void safe_strcpy(char *dst, const char *src, size_t dstsz) {
    if (dstsz == 0) return;
    strncpy(dst, src, dstsz - 1);
    dst[dstsz - 1] = 0;
}
