/** @file
 *  @brief OTA rollback probation rule (V2.8.20) — see ota_guard_logic.h.
 */

#include "ota_guard_logic.h"

bool ota_guard_needs_confirm(bool running_is_ota_slot, ota_guard_state_t st) {
    return running_is_ota_slot && (st == OTA_GUARD_ST_NEW || st == OTA_GUARD_ST_PENDING);
}

bool ota_guard_probation_over(int64_t now_us, int64_t ap_window_over_us) {
    return ap_window_over_us > 0 && now_us - ap_window_over_us >= OTA_GUARD_PROBATION_AFTER_AP_US;
}

bool ota_guard_retry_due(int64_t now_us, int64_t next_try_us) { return now_us >= next_try_us; }

int64_t ota_guard_next_retry_us(int64_t failed_us) { return failed_us + OTA_GUARD_RETRY_US; }
