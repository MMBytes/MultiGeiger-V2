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
#include <stdlib.h>   // strtol / strtof (parse_long_strict, parse_float_strict)

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
 *
 *  V2.7.3: dropped the strncpy call. GCC 16 raises -Wstringop-truncation on
 *  `strncpy(dst, src, dstsz - 1)` even though the very next line terminated
 *  the buffer, which under the host tests' -Werror is a hard failure waiting
 *  for CI's toolchain to catch up to the local one. The scan-then-copy below
 *  is byte-for-byte equivalent, including the tail zero-fill strncpy gave:
 *  dst[n .. dstsz-1] all end up zero, so a caller that relies on the unused
 *  remainder being blank (a struct handed to an IDF driver, a buffer about to
 *  be persisted) sees no change. Not strnlen(): it is POSIX-2008, glibc hides
 *  it behind __USE_XOPEN2K8, and the host tests compile with -std=c11 rather
 *  than gnu11, so it would not be declared. Not memchr() either — that is
 *  permitted to read all dstsz bytes, past the end of a short src.
 */
static inline void safe_strcpy(char *dst, const char *src, size_t dstsz) {
    if (dstsz == 0) return;
    size_t n = 0;
    while (n < dstsz - 1 && src[n] != '\0') n++;   // reads no further than the NUL
    memcpy(dst, src, n);
    memset(dst + n, 0, dstsz - n);
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

/** @brief Destination size that holds html_esc() of any `max`-char input.
 *
 *  Worst case is 6 bytes out per byte in (every char -> "&quot;"). Why +2,
 *  not +1: html_esc's loop guard (`o + 7 < bufsz`) reserves a full 7 bytes
 *  (6-byte escape + NUL) before emitting ANY char, with a STRICT inequality,
 *  so emitting the Nth char after N-1 six-byte expansions needs
 *  bufsz >= 6N+2. A 6N+1 buffer drops the final char of a max-length
 *  all-quotes value (V2.6.24 MAX-review finding). V2.8.8: moved here from
 *  http_server.c so test/test_main.c pins this exact macro.
 */
#define ESC_WORST(max) ((max) * 6 + 2)

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
 *  V2.8.5: `bufsz == 0` returns without writing — the terminator store
 *  below used to hit out[0] of a zero-length buffer. No caller passes 0
 *  today; the guard keeps the "always safe" promise true for the next one.
 */
static inline void html_esc(const char *in, char *out, size_t bufsz) {
    if (bufsz == 0) return;
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

/** @brief Same as format_uptime() but always omits seconds ("Nd HHh MMm" or
 *  "HHh MMm"). Used where seconds add no diagnostic value (e.g. per-cycle log
 *  lines) and keeping the field short matters for log readability. */
static inline void format_uptime_hm(unsigned long s, char *out, size_t sz) {
    unsigned long m = s / 60;
    unsigned long h = m / 60;
    unsigned long d = h / 24;
    if (d > 0) {
        // d>0 branch omits seconds — identical output to format_uptime; delegate
        // so any future change to the day/hour format stays in one place.
        format_uptime(s, out, sz);
    } else {
        snprintf(out, sz, "%02luh %02lum", h, m % 60);
    }
}

// --- V2.8.5 pure helpers (host-tested in test/test_main.c) -----------------

/** @brief Parse a whole string as a base-10 integer, rejecting anything else.
 *
 *  `strtol(val, NULL, 10)` — what the /config schema dispatch used before —
 *  returns 0 for "abc", 12 for "12abc" and 0 for "", so a typo silently
 *  became a valid-looking value (e.g. `tube_type=abc` stored Unknown and
 *  zeroed the uploaded dose). This accepts only an optional sign and digits
 *  with nothing left over; leading whitespace is tolerated because strtol
 *  skips it. Overflow saturates to LONG_MIN/LONG_MAX, which every caller's
 *  range check then rejects, so errno is not consulted.
 *
 *  @param s    NUL-terminated input. NULL or empty → false.
 *  @param out  Written only on success.
 *  @return true if the entire string was a number.
 */
static inline bool parse_long_strict(const char *s, long *out) {
    if (!s || !*s || !out) return false;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0') return false;
    *out = v;
    return true;
}

/** @brief Float counterpart of parse_long_strict(). "nan" parses but fails
 *  every caller's `>= lo && <= hi` test (all NaN comparisons are false);
 *  "inf" parses and fails the upper bound — so neither can be stored. */
static inline bool parse_float_strict(const char *s, float *out) {
    if (!s || !*s || !out) return false;
    char *end = NULL;
    float v = strtof(s, &end);
    if (end == s || *end != '\0') return false;
    *out = v;
    return true;
}

/** @brief Wrap-safe "has `interval` elapsed since `last`?" for 32-bit tick
 *  counters.
 *
 *  `now >= last + interval` breaks when the counter wraps (FreeRTOS ticks
 *  at 100 Hz wrap every ~497 days): `last + interval` overflows to a small
 *  number and the comparison is true on every tick. Unsigned subtraction
 *  gives the true elapsed count across the wrap as long as the real gap is
 *  below 2^32 ticks.
 */
static inline bool tick_interval_elapsed(uint32_t now, uint32_t last,
                                         uint32_t interval) {
    return (uint32_t)(now - last) >= interval;
}

/** @brief Ticks left until `interval` has elapsed since `last`, or 0 if it
 *  already has. Same wrap-safe arithmetic as tick_interval_elapsed(). */
static inline uint32_t tick_interval_remaining(uint32_t now, uint32_t last,
                                               uint32_t interval) {
    uint32_t elapsed = now - last;
    return (elapsed >= interval) ? 0u : (interval - elapsed);
}

/** @brief Format a non-negative count into exactly 4 characters for a
 *  fixed-width display cell.
 *
 *  - below 1000      : right-aligned integer  " 234" / "   9" / "1000"
 *  - 1000 .. 9949    : one decimal + k        "3.4k" / "9.9k"
 *  - 9950 and above  : integer + k + space    "10k " / "99k " (clamped)
 *
 *  The 9950 boundary is the point where `%3.1f` of v/1000 rounds up to
 *  "10.0", which made 9950..9999 render as the 5-character "10.0k" and
 *  overflow the 128 px OLED row at 2x font (display.c PM number screen).
 *  `outsz` must be at least 5; smaller buffers get a truncated string.
 */
static inline void fmt_kilo4(char *out, size_t outsz, float v) {
    if (v < 1000.0f) {
        snprintf(out, outsz, "%4d", (int)(v + 0.5f));
    } else if (v < 9950.0f) {
        snprintf(out, outsz, "%3.1fk", (double)(v / 1000.0f));
    } else {
        int k = (int)(v / 1000.0f + 0.5f);
        // v >= 9950 makes k >= 10 already; the lower clamp tells the compiler
        // so, which keeps -Wformat-truncation quiet on 2-digit output.
        if (k < 10) k = 10;
        if (k > 99) k = 99;
        snprintf(out, outsz, "%2dk ", k);
    }
}

/** @brief Turn strftime's "%z" offset ("+1000") at the end of `buf` into
 *  the RFC 3339 form ("+10:00") in place.
 *
 *  @param buf    String of length `n` ending in a 5-char ±HHMM offset.
 *  @param n      strlen(buf).
 *  @param bufsz  Capacity of buf; the splice needs n + 2 bytes.
 *  @return The new length (n + 1), or n unchanged when the tail is not a
 *          ±HHMM offset or the buffer has no room for the colon.
 */
static inline size_t tz_offset_add_colon(char *buf, size_t n, size_t bufsz) {
    if (n < 5 || n + 2 > bufsz) return n;
    if (buf[n - 5] != '+' && buf[n - 5] != '-') return n;
    buf[n + 1] = '\0';
    buf[n]     = buf[n - 1];   // shift offset minutes right by one
    buf[n - 1] = buf[n - 2];
    buf[n - 2] = ':';          // colon between offset hours and minutes
    return n + 1;
}
