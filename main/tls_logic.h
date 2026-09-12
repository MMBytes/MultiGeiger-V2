#pragma once

/** @file
 *  @brief Pure helpers for the HTTPS web server (V2.8.0).
 *
 *  Header-only and free of IDF / FreeRTOS / socket / libc-time dependencies
 *  so the host test runner (`test/test_main.c`) can cover them. The
 *  IDF-dependent side (key generation, NVS, httpd wiring) lives in
 *  tls_cert.c / http_server.c.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** @brief Render a digest as upper-case colon-separated hex ("AA:BB:...").
 *
 *  This is the format `openssl x509 -fingerprint -sha256` prints, so a user
 *  can compare the /status line against the certificate their browser shows
 *  byte for byte.
 *
 *  @param digest  Raw bytes (32 for SHA-256).
 *  @param n       Byte count; must be > 0.
 *  @param out     Destination; needs `n * 3` bytes (2 hex + separator per
 *                 byte, the last separator slot holds the NUL).
 *  @param outsz   Size of @p out.
 *  @return true on success. On failure @p out is set to "" (when outsz > 0)
 *          so a caller that prints it unconditionally never shows garbage.
 */
static inline bool fingerprint_hex(const uint8_t *digest, size_t n,
                                   char *out, size_t outsz) {
    if (!out || outsz == 0) return false;
    if (!digest || n == 0 || outsz < n * 3) {
        out[0] = 0;
        return false;
    }
    static const char hex[] = "0123456789ABCDEF";
    char *p = out;
    for (size_t i = 0; i < n; i++) {
        if (i) *p++ = ':';
        *p++ = hex[digest[i] >> 4];
        *p++ = hex[digest[i] & 0x0F];
    }
    *p = 0;
    return true;
}

/** @brief Build the Location value for the :80 -> :443 redirect.
 *
 *  Produces `https://<host><uri>` where <host> is the request's Host header
 *  with any `:port` suffix removed (a browser that typed `http://x/` sends
 *  `Host: x` or `Host: x:80`; either must land on `https://x/`). Host is
 *  untrusted request text, so the builder fails closed instead of emitting
 *  a partial URL:
 *    - empty host, or nothing before the first ':'  -> false
 *    - host starting with '[' (IPv6 literal; the ':' split would mangle it
 *      and the firmware has no IPv6 listener anyway)   -> false
 *    - uri not starting with '/'                        -> false
 *    - result would not fit (snprintf truncation)       -> false
 *  On every failure @p out is "" when outsz > 0.
 *
 *  @param host   Value of the Host request header (may carry ":port").
 *  @param uri    Request-target incl. query string, e.g. "/log?tail=1".
 *  @param out    Destination buffer.
 *  @param outsz  Size of @p out.
 *  @return true when @p out holds the complete URL.
 */
static inline bool https_redirect_location(const char *host, const char *uri,
                                           char *out, size_t outsz) {
    if (!out || outsz == 0) return false;
    out[0] = 0;
    if (!host || !uri) return false;
    if (host[0] == 0 || host[0] == '[' || uri[0] != '/') return false;
    size_t hlen = strcspn(host, ":");
    if (hlen == 0) return false;
    int n = snprintf(out, outsz, "https://%.*s%s", (int)hlen, host, uri);
    if (n < 0 || (size_t)n >= outsz) {
        out[0] = 0;
        return false;
    }
    return true;
}

// --- Calendar (TZ-free) --------------------------------------------------------
// Howard Hinnant's days_from_civil / civil_from_days: exact for the proleptic
// Gregorian calendar, no tables, no libc. Used because the device's TZ is a
// user setting (ntp.c setenv("TZ")), which makes mktime()/localtime() local
// time, and X.509 validity must be UTC.

/** @brief Days since 1970-01-01 for a civil UTC date (month 1..12, day 1..31). */
static inline int64_t civil_to_epoch_days(int y, unsigned m, unsigned d) {
    y -= (m <= 2);
    const int      era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);                       // [0, 399]
    const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1; // [0, 365]
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;           // [0, 146096]
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

/** @brief UTC epoch seconds for a civil date-time. No range checks — callers
 *         pass either constants or fields already validated by mbedTLS. */
static inline int64_t civil_to_epoch(int y, unsigned m, unsigned d,
                                     unsigned hh, unsigned mm, unsigned ss) {
    return civil_to_epoch_days(y, m, d) * 86400 + (int64_t)hh * 3600 + (int64_t)mm * 60 + ss;
}

/** @brief Format UTC epoch seconds as X.509 GeneralizedTime "YYYYMMDDhhmmss".
 *
 *  @param t      Seconds since 1970-01-01T00:00:00Z; must be >= 0 and < year 10000.
 *  @param out    Needs 15 bytes (14 digits + NUL).
 *  @param outsz  Size of @p out.
 *  @return true on success; on failure @p out is "" when outsz > 0.
 */
static inline bool epoch_to_generalized_time(int64_t t, char *out, size_t outsz) {
    if (!out || outsz == 0) return false;
    out[0] = 0;
    if (outsz < 15 || t < 0) return false;
    int64_t z = t / 86400 + 719468;
    const int64_t  era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = (unsigned)(z - era * 146097);                                    // [0, 146096]
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;           // [0, 399]
    int64_t        y   = (int64_t)yoe + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);                         // [0, 365]
    const unsigned mp  = (5 * doy + 2) / 153;                                             // [0, 11]
    const unsigned d   = doy - (153 * mp + 2) / 5 + 1;                                    // [1, 31]
    const unsigned m   = mp < 10 ? mp + 3 : mp - 9;                                       // [1, 12]
    y += (m <= 2);
    if (y > 9999) return false;
    const unsigned secs = (unsigned)(t % 86400);
    snprintf(out, outsz, "%04lld%02u%02u%02u%02u%02u", (long long)y, m, d,
             secs / 3600, (secs / 60) % 60, secs % 60);
    return true;
}
