#pragma once

/** @file
 *  @brief Minimal on-device HTTP server (esp_http_server).
 *
 *  Endpoints:
 *    - `GET  /`        — status page (no auth)
 *    - `GET  /config`  — config form + reboot button (basic auth, user "admin")
 *    - `POST /config`  — save config to NVS, flag restart (basic auth)
 *    - `POST /reboot`  — flag restart without saving (basic auth)
 *    - `GET  /update`  — OTA upload form with XHR progress (basic auth)
 *    - `POST /update`  — raw firmware.bin body → esp_ota_write, flag restart (basic auth)
 *    - `GET  /log`     — rolling log buffer snapshot (basic auth)
 *
 *  Basic-auth password is the AP password from config; username is "admin".
 */

#include <stdbool.h>
#include "config.h"

/** @brief Start the server on port 80.
 *
 *  @p cfg is captured by pointer: GET reads it, POST updates in place and
 *  calls config_save() + sets the restart flag. @p chip_id is captured by
 *  pointer too — it's hardware-derived, owned by main.c, shown on the
 *  status and config pages.
 */
void http_server_start(config_t *cfg, const char *chip_id);

/** @brief Polled by the main task; true if a handler requested a restart.
 *
 *  main flushes pending log and calls esp_restart() so new settings
 *  (WiFi, NTP, targets) take effect on a clean boot.
 */
bool http_server_restart_requested(void);
