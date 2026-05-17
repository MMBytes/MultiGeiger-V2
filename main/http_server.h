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
 *  calls config_save() + main_request_restart() (V2.4.1 A9 — was a polled
 *  flag pre-V2.4.1). @p chip_id is captured by pointer too — hardware-
 *  derived, owned by main.c, shown on the status and config pages.
 */
void http_server_start(config_t *cfg, const char *chip_id);
