/** @file
 *  @brief OTA rollback, app side (V2.8.20) — see ota_guard.h.
 */

#include "ota_guard.h"

#include <stdlib.h>   // abort() — OTA_TEST_FAULT images only

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "http_server.h"
#include "ota_guard_logic.h"

static const char *TAG = "ota";

// Local OTA-rollback TEST images only (see the top-level CMakeLists.txt):
// 1 = abort() at uptime 60 s, inside the probation period. Never set in CI.
#ifndef OTA_TEST_FAULT
#define OTA_TEST_FAULT 0
#endif
#define OTA_TEST_FAULT_AT_US (60 * 1000000LL)

// Written once by ota_guard_boot_report() (app_main, before any other task
// can call in); afterwards s_needs only goes true -> false, under s_lock.
static volatile bool s_needs = false;        // running image still to be confirmed
static bool s_no_rollback_bl = false;        // state was still NEW at boot: the bootloader does not roll back
static SemaphoreHandle_t s_lock = NULL;      // one confirm at a time (httpd vs main)
static int64_t s_window_over_us = 0;         // main task only
static bool s_warned_no_httpd = false;       // main task only
static int64_t s_next_try_us = 0;            // main task only; after a failed probation confirm

// --- Boot report ------------------------------------------------------------

/** @brief IDF image state -> the host-testable ota_guard_state_t. */
static ota_guard_state_t map_state(esp_ota_img_states_t st) {
    switch (st) {
    case ESP_OTA_IMG_NEW:            return OTA_GUARD_ST_NEW;
    case ESP_OTA_IMG_PENDING_VERIFY: return OTA_GUARD_ST_PENDING;
    case ESP_OTA_IMG_VALID:          return OTA_GUARD_ST_VALID;
    case ESP_OTA_IMG_UNDEFINED:      return OTA_GUARD_ST_UNDEFINED;
    default:                         return OTA_GUARD_ST_OTHER;
    }
}

void ota_guard_boot_report(void) {
    s_lock = xSemaphoreCreateMutex();
#if OTA_TEST_FAULT
    ESP_LOGE(TAG, "TEST FAULT IMAGE %d: this build crashes on purpose at uptime %lld s (OTA rollback test) "
                  "— not for normal use", OTA_TEST_FAULT, OTA_TEST_FAULT_AT_US / 1000000LL);
#endif
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_app_desc_t *app = esp_app_get_description();
    const bool slot = run && run->subtype >= ESP_PARTITION_SUBTYPE_APP_OTA_MIN &&
                      run->subtype <= ESP_PARTITION_SUBTYPE_APP_OTA_MAX;
    ota_guard_state_t st = OTA_GUARD_ST_OTHER;
    esp_ota_img_states_t raw;
    if (slot && esp_ota_get_state_partition(run, &raw) == ESP_OK) st = map_state(raw);
    s_needs = ota_guard_needs_confirm(slot, st);

    const char *label = run ? run->label : "?";
    if (!slot) {
        ESP_LOGI(TAG, "running %s (%s): factory image — nothing to confirm", label, app->version);
    } else if (st == OTA_GUARD_ST_PENDING) {
        ESP_LOGI(TAG, "running %s (%s) state=pending-verify — will confirm 2 min after the boot AP window",
                 label, app->version);
    } else if (st == OTA_GUARD_ST_NEW) {
        // A rollback bootloader turns NEW into PENDING_VERIFY before the app starts.
        s_no_rollback_bl = true;
        ESP_LOGI(TAG, "running %s (%s) state=new — this node's bootloader does not roll back; will "
                      "confirm 2 min after the boot AP window all the same", label, app->version);
    } else if (st == OTA_GUARD_ST_VALID) {
        ESP_LOGI(TAG, "running %s (%s) state=valid", label, app->version);
    } else if (st == OTA_GUARD_ST_UNDEFINED) {
        ESP_LOGI(TAG, "running %s (%s) state=undefined — installed by a firmware without rollback "
                      "tracking, nothing to confirm", label, app->version);
    } else {
        ESP_LOGW(TAG, "running %s (%s): image state unreadable — nothing to confirm", label, app->version);
    }

    const esp_partition_t *bad = esp_ota_get_last_invalid_partition();
    if (bad) {
        esp_app_desc_t d;
        const char *ver = esp_ota_get_partition_description(bad, &d) == ESP_OK ? d.version : "unknown version";
        ESP_LOGW(TAG, "firmware %s in %s was ROLLED BACK (it did not confirm — crash, watchdog or power "
                      "loss during its first minutes) — running the previous image; this note repeats "
                      "until the next update", ver, bad->label);
    }
}

// --- Confirm ----------------------------------------------------------------

/** @brief Confirm the running image if it still needs it. Caller holds s_lock.
 *  @param reason restart / update that triggered it, or NULL for the probation end
 *  @return true when nothing is left to confirm */
static bool confirm_locked(const char *reason) {
    if (!s_needs) return true;
    if (reason) {
        ESP_LOGI(TAG, "%s requested during the probation period — confirming the running firmware first",
                 reason);
    } else {
        ESP_LOGI(TAG, "probation period over (boot AP window + 2 min) — confirming the running firmware");
    }
    const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "confirm FAILED: %s — %s", esp_err_to_name(err),
                 s_no_rollback_bl ? "this node's bootloader has no rollback, so nothing is at risk"
                                  : "the firmware may be rolled back at the next reboot");
        return false;
    }
    s_needs = false;
    ESP_LOGI(TAG, "firmware confirmed (valid) — %s",
             s_no_rollback_bl ? "this node's bootloader has no rollback, so nothing was at risk"
                              : "rollback disarmed");
    return true;
}

bool ota_guard_confirm_now(const char *reason) {
    if (!s_needs) return true;
    if (!s_lock) return confirm_locked(reason);   // mutex alloc failed at boot: confirm unserialised
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool ok = confirm_locked(reason);
    xSemaphoreGive(s_lock);
    return ok;
}

bool ota_guard_settled(void) { return !s_needs; }

// --- Main-loop tick ---------------------------------------------------------

void ota_guard_tick(int64_t now_us, bool ap_window_over) {
#if OTA_TEST_FAULT == 1
    if (now_us >= OTA_TEST_FAULT_AT_US) {
        ESP_LOGE(TAG, "TEST FAULT IMAGE 1: crashing now (abort) — a rollback bootloader must return to "
                      "the previous image");
        abort();
    }
#endif
    if (ap_window_over && s_window_over_us == 0) s_window_over_us = now_us;
    if (!s_needs || !ota_guard_probation_over(now_us, s_window_over_us)) return;
    if (!ota_guard_retry_due(now_us, s_next_try_us)) return;
    if (http_server_get_handle() == NULL) {   // httpd_start() failed (HTTP fallback still counts)
        if (!s_warned_no_httpd) {
            s_warned_no_httpd = true;
            if (s_no_rollback_bl) {
                ESP_LOGW(TAG, "web server not running — firmware NOT confirmed; this node's bootloader "
                              "has no rollback, so nothing is at risk");
            } else {
                ESP_LOGE(TAG, "web server not running — firmware NOT confirmed; it will be rolled back "
                              "at the next reboot");
            }
        }
        return;
    }
    if (!ota_guard_confirm_now(NULL)) s_next_try_us = ota_guard_next_retry_us(now_us);   // ERROR once a minute
}
