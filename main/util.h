#pragma once

/** @file
 *  @brief Tiny utility helpers shared across modules.
 *
 *  Header-only — no util.c needed, no symbol bloat, no CMake change.
 *  Every function here is a self-contained pure helper with no IDF /
 *  FreeRTOS / hardware dependency, so the host-side test runner under
 *  `test/` can include this header directly.
 */

#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

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

/** @brief Constant-time byte compare for credential material.
 *
 *  Standard `strcmp` / `memcmp` short-circuit on the first differing
 *  byte — leaks position via response-time variance and in principle
 *  enables byte-at-a-time brute force. Returns 0 if the two buffers
 *  of length n are identical, non-zero otherwise.
 *
 *  V2.4.1+ (T1): moved from http_server.c so the host test runner can
 *  include it directly; originally added in V2.3.33 (B2 in the web
 *  security audit) for the basic-auth compare path.
 */
static inline int ct_memcmp(const void *a, const void *b, size_t n) {
    const uint8_t *pa = (const uint8_t *)a;
    const uint8_t *pb = (const uint8_t *)b;
    uint8_t diff = 0;
    for (size_t i = 0; i < n; i++) diff |= (uint8_t)(pa[i] ^ pb[i]);
    return diff;
}

/** @brief Parse one hex character to its 0..15 value, or -1 if not hex.
 *
 *  V2.4.1+ (T1): moved from http_server.c. Used internally by url_decode.
 */
static inline int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/** @brief URL-decode an application/x-www-form-urlencoded string in place.
 *
 *  Handles `+` → space and `%XY` → byte. Per RFC 3986 §2.1, a malformed
 *  `%XY` (XY not two hex digits, or `%` at end of string with insufficient
 *  bytes) is invalid encoding: the lone `%` is dropped and the trailing
 *  characters walk through as plain bytes (`%G5` → `G5`, `%2` at end → `2`).
 *
 *  V2.4.1+ (T1): moved from http_server.c. V2.4.1 (B6) introduced the
 *  RFC-strict behaviour; pre-V2.4.1 preserved the literal `%`.
 */
static inline void url_decode(char *s) {
    char *w = s;
    while (*s) {
        if (*s == '+') {
            *w++ = ' ';
            s++;
        } else if (*s == '%') {
            int hi = s[1] ? hex_nibble(s[1]) : -1;
            int lo = (s[1] && s[2]) ? hex_nibble(s[2]) : -1;
            if (hi >= 0 && lo >= 0) {
                *w++ = (char)((hi << 4) | lo);
                s += 3;
            } else {
                s++;   // drop lone/malformed `%`, keep walking
            }
        } else {
            *w++ = *s++;
        }
    }
    *w = 0;
}

/** @brief Percent-encode a string for safe use as a URL query-string VALUE.
 *
 *  RFC 3986 §2.3 unreserved characters (ALPHA / DIGIT / "-" / "." / "_" /
 *  "~") pass through; every other byte becomes %XX. Stops early (always
 *  NUL-terminated) if the destination fills. Caller should size `dstsz`
 *  for the worst-case 3× expansion of the longest input it cares about.
 *
 *  V2.5.20 (review R2): Radmon credentials were interpolated raw into the
 *  submit URL, so a password containing `&` / `=` / `+` / `%` / space broke
 *  or silently mangled the request. Generic helper so future query-string
 *  builders (tokens, IDs) can reuse it.
 */
static inline void url_encode_query_value(char *dst, size_t dstsz, const char *src) {
    static const char hex[] = "0123456789ABCDEF";
    size_t o = 0;
    if (dstsz == 0) return;
    while (*src) {
        unsigned char c = (unsigned char)*src++;
        bool unreserved = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                          (c >= '0' && c <= '9') ||
                          c == '-' || c == '.' || c == '_' || c == '~';
        if (unreserved) {
            if (o + 1 >= dstsz) break;
            dst[o++] = (char)c;
        } else {
            if (o + 3 >= dstsz) break;
            dst[o++] = '%';
            dst[o++] = hex[c >> 4];
            dst[o++] = hex[c & 0x0F];
        }
    }
    dst[o] = 0;
}

/** @brief Escape `&`, `"`, `<`, `>` for safe use inside an HTML
 *         `value="..."` attribute.
 *
 *  Writes up to `bufsz` bytes to `out` including the trailing NUL.
 *  Stops early if expansion would overflow the destination — output
 *  is always null-terminated. Caller should size `bufsz` to cover the
 *  worst-case 6× expansion (`&quot;` per char) for the longest input
 *  it cares about.
 *
 *  V2.4.1+ (T1): moved from http_server.c.
 */
static inline void html_esc(const char *in, char *out, size_t bufsz) {
    size_t o = 0;
    while (*in && o + 7 < bufsz) {
        switch (*in) {
            case '&':  memcpy(out + o, "&amp;",  5); o += 5; break;
            case '"':  memcpy(out + o, "&quot;", 6); o += 6; break;
            case '<':  memcpy(out + o, "&lt;",   4); o += 4; break;
            case '>':  memcpy(out + o, "&gt;",   4); o += 4; break;
            default:   out[o++] = (char)*in;
        }
        in++;
    }
    out[o] = 0;
}

/** @brief Format an uptime in seconds as a compact "Nd HHh MMm" / "HHh MMm SSs".
 *
 *  Below one day shows seconds for precision ("06h 12m 30s"); at or above a
 *  day, days lead and seconds drop ("2d 23h 59m"). Always null-terminated.
 *
 *  V2.5.33: lifted from http_server.c (was `static` there) so the heap-guard
 *  reboot log line in transmission.c can render uptime with the SAME formatting
 *  the /status page uses, instead of a second inline copy of the d/h/m maths.
 */
static inline void format_uptime(unsigned long s, char *out, size_t sz) {
    unsigned long m = s / 60;
    unsigned long h = m / 60;
    unsigned long d = h / 24;
    if (d > 0) {
        snprintf(out, sz, "%lud %02luh %02lum", d, h % 24, m % 60);
    } else {
        snprintf(out, sz, "%02luh %02lum %02lus", h, m % 60, s % 60);
    }
}
