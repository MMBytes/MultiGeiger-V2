#pragma once

/** @file
 *  @brief OTA rollback, app side (V2.8.20): confirm a freshly installed image
 *  once it has proven itself, and report the image state and any rollback.
 *
 *  With the rollback bootloader (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE) a new
 *  image boots as PENDING_VERIFY; if it resets before it is confirmed, the
 *  bootloader returns to the previous image. This module confirms it when the
 *  probation period is over (ota_guard_logic.h: boot AP window + 120 s, web
 *  server running), or earlier when the user starts an update or a restart
 *  from the web UI — reaching the UI and saving something is proof enough,
 *  and a deliberate restart must never undo a good update. On old bootloaders
 *  the image stays NEW and the confirm is harmless. Images installed by a
 *  firmware without rollback tracking (UNDEFINED) and the factory image need
 *  nothing.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Read the running image's state once and log it (plus a rollback, if
 *  one happened). Call early in app_main, before other tasks start: the
 *  rollback check SHA-validates the rolled-back image (~1 s). */
void ota_guard_boot_report(void);

/** @brief Main-loop hook, every iteration.
 *  @param now_us         esp_timer uptime
 *  @param ap_window_over true once the boot AP window has closed (or would
 *                        have: AP_WINDOW_US elapsed and no AP client holding it) */
void ota_guard_tick(int64_t now_us, bool ap_window_over);

/** @brief Confirm the running image now if it still needs it (any task).
 *  @param reason what the user asked for, e.g. "reboot" — named in the log
 *  @return false only if the confirm itself failed */
bool ota_guard_confirm_now(const char *reason);

/** @brief True once the image is confirmed or never needed confirming. */
bool ota_guard_settled(void);

#ifdef __cplusplus
}
#endif
