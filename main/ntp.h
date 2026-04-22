#pragma once

/** @file
 *  @brief SNTP client with Sydney local-time conversion.
 *
 *  Uses the ESP-IDF SNTP layer; exposes a simple "time is usable" predicate
 *  so transmitters can gate uploads and avoid 1970-era timestamps.
 */

#include <stdbool.h>
#include <time.h>

/** @brief Start SNTP with up to three servers and set the local timezone.
 *
 *  Empty server strings are skipped (no slot registered). Non-empty pointers
 *  must outlive SNTP — esp_sntp_setservername does not copy. @p tz_posix is
 *  a POSIX TZ string (e.g. "AEST-10AEDT,M10.1.0,M4.1.0/3"); NULL or empty
 *  leaves TZ unchanged. Safe to call once after GOT_IP.
 */
void ntp_setup(const char *s1, const char *s2, const char *s3, const char *tz_posix);

/** @brief True once the clock has ticked past 2025-01-01 (NTP has synced). */
bool ntp_time_valid(void);

/** @brief Drain the deferred sync-complete log message from the main task.
 *
 *  The SNTP callback runs on the tcpip task with a small stack, so logging
 *  is deferred here instead of called from the callback directly.
 */
void ntp_poll(void);

/** @brief Current local time as "YYYY-MM-DDTHH:MM:SS". Static buffer — not reentrant. */
const char *ntp_localtime_str(void);
