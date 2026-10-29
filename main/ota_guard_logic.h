#pragma once

/** @file
 *  @brief OTA rollback probation rule (V2.8.20).
 *  Pure functions, free of ESP-IDF, so the host tests can cover them;
 *  ota_guard.c maps the IDF image states onto ota_guard_state_t.
 *
 *  A freshly installed image is "on probation" until it has run long enough
 *  to count as working: the boot AP window has closed (or would have — no AP
 *  client holding it) plus OTA_GUARD_PROBATION_AFTER_AP_US. Only then is it
 *  confirmed, so a crash, a watchdog reset or a power cut before that point
 *  lets a rollback bootloader return to the previous image. Same rule for
 *  every mode; WiFi STA, uploads, NTP etc. are deliberately not criteria.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Image state of the running app, as far as the probation rule cares. */
typedef enum {
    OTA_GUARD_ST_OTHER,       ///< anything else (invalid, aborted, unreadable)
    OTA_GUARD_ST_UNDEFINED,   ///< installed by a firmware without rollback tracking
    OTA_GUARD_ST_NEW,         ///< installed by V2.8.20+, old bootloader (stays NEW)
    OTA_GUARD_ST_PENDING,     ///< rollback bootloader: first boot, not yet confirmed
    OTA_GUARD_ST_VALID,       ///< already confirmed
} ota_guard_state_t;

/** @brief How long after the boot AP window the image is confirmed. */
#define OTA_GUARD_PROBATION_AFTER_AP_US (120 * 1000000LL)

/** @brief True if the running image still has to be confirmed: it runs from
 *         an OTA slot (never factory) and is NEW or PENDING. */
bool ota_guard_needs_confirm(bool running_is_ota_slot, ota_guard_state_t st);

/** @brief True once the probation period is over.
 *  @param now_us            current uptime (µs)
 *  @param ap_window_over_us uptime at which the boot AP window closed (or would
 *                           have); 0 = not yet */
bool ota_guard_probation_over(int64_t now_us, int64_t ap_window_over_us);

/** @brief Spacing between probation confirm attempts after one failed. */
#define OTA_GUARD_RETRY_US (60 * 1000000LL)

/** @brief True if a probation confirm may be attempted now.
 *  @param next_try_us earliest uptime for the next attempt; 0 = none failed yet */
bool ota_guard_retry_due(int64_t now_us, int64_t next_try_us);

/** @brief Earliest uptime for the next attempt after one failed at @p failed_us. */
int64_t ota_guard_next_retry_us(int64_t failed_us);

#ifdef __cplusplus
}
#endif
