#pragma once

/** @file
 *  @brief SNTP client with Sydney local-time conversion.
 *
 *  Uses the ESP-IDF SNTP layer; exposes a simple "time is usable" predicate
 *  so transmitters can gate uploads and avoid 1970-era timestamps.
 */

#include <stdbool.h>
#include <time.h>

/** @brief Set the process TZ environment variable + tzset(). Independent of
 *  SNTP/WiFi — safe to call at boot before either is up. @p tz_posix is a
 *  POSIX TZ string (e.g. "AEST-10AEDT,M10.1.0,M4.1.0/3"); NULL or empty
 *  leaves TZ unchanged.
 *
 *  V2.6.21: split out of ntp_setup() so standalone-SD boards (which never
 *  reach GOT_IP, so ntp_setup() itself never ran) still get local-time
 *  timestamps everywhere localtime_r() is used — ESP-IDF's own SYSTEM_FULL
 *  log timestamp formatter included, so this alone makes every /log line
 *  TZ-aware. main.c calls this once at boot, right after config_load();
 *  ntp_setup() also calls it (below), so a later STA GOT_IP just re-applies
 *  the same value — harmless.
 */
void ntp_set_timezone(const char *tz_posix);

/** @brief Start SNTP with up to three servers and set the local timezone.
 *
 *  Empty server strings are skipped (no slot registered). Non-empty pointers
 *  must outlive SNTP — esp_sntp_setservername does not copy. @p tz_posix is
 *  a POSIX TZ string (e.g. "AEST-10AEDT,M10.1.0,M4.1.0/3"); NULL or empty
 *  leaves TZ unchanged. Safe to call once after GOT_IP.
 */
void ntp_setup(const char *s1, const char *s2, const char *s3, const char *tz_posix);

/** @brief True once the clock has ticked past 2026-01-01 (NTP has synced). */
bool ntp_time_valid(void);

/** @brief Drain the deferred sync-complete log message from the main task.
 *
 *  The SNTP callback runs on the tcpip task with a small stack, so logging
 *  is deferred here instead of called from the callback directly.
 */
void ntp_poll(void);

/** @brief Current local time as RFC 3339 with numeric UTC offset, e.g.
 *  "2026-06-13T20:45:50+10:00" (DST-aware via the configured TZ).
 *  Static buffer — not reentrant. */
const char *ntp_localtime_str(void);

/** @brief Boot epoch (wall_time - uptime), refreshed on every SNTP sync.
 *  Returns 0 if no sync has occurred yet this session.
 *  Updating each hourly sync self-corrects a bad first-sync timestamp and
 *  keeps crystal drift < 1 s between syncs. */
time_t ntp_boot_epoch(void);

/** @brief NTP-accurate uptime in seconds, clamped to 0.
 *  Uses ntp_boot_epoch() when available; falls back to raw esp_timer when
 *  no sync has occurred. Clamped so an NTP step-back never wraps to ~136yr
 *  when the caller casts to unsigned. */
unsigned long ntp_uptime_s(void);
