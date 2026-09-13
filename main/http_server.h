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
 *    - `GET  /log`     — rolling log buffer snapshot (no auth — since V2.3.33)
 *    - `GET  /api/env` — environment JSON for peer nodes (no auth)
 *    - `GET  /favicon.ico`    — inline icon (no auth)
 *    - `GET  /coredump.elf`   — stored crash dump, if any (basic auth)
 *    - `POST /coredump_erase` — clear the stored crash dump (basic auth)
 *    - `POST /lorawan_reset`  — wipe LoRaWAN session/nonces (basic auth; LoRaWAN boards only)
 *    - `GET  /cert.pem`       — this device's self-signed certificate PEM (no auth; HTTPS boards)
 *
 *  Basic-auth password is the AP password from config; username is "admin".
 *
 *  V2.8.0: on HAL_HAS_HTTPS boards (all PSRAM boards) the server is TLS on
 *  port 443 with a per-device certificate from tls_cert.h. Port 80 stays up
 *  as a second, minimal instance: /log, /api/env and /cert.pem are served
 *  plain there (all three unauthenticated and audited free of shared static
 *  buffers, so scripts and peer nodes need no TLS client); every other
 *  plain-HTTP request, any method, receives a 301 to the same path on
 *  https://. The two Heltec V2 builds stay plain HTTP on :80.
 *  HTTPS is opt-in per node via the `https_enable` config setting (default
 *  off, applied at the next boot); with it off the board serves plain HTTP
 *  on :80 exactly as V2.7.7 did.
 *  Design: docs/superpowers/specs/2026-09-12-https-web-server-design.md
 */

#include <stdbool.h>
#include "esp_http_server.h"
#include "config.h"

/** @brief Start the web server (:443 TLS on HAL_HAS_HTTPS boards, else :80 plain).
 *
 *  @p cfg is captured by pointer: GET reads it, POST updates in place and
 *  calls config_save() + main_request_restart() (V2.4.1 A9 — was a polled
 *  flag pre-V2.4.1). @p chip_id is captured by pointer too — hardware-
 *  derived, owned by main.c, shown on the status and config pages.
 */
void http_server_start(config_t *cfg, const char *chip_id);

/** @brief Handle from httpd_start(), or NULL before http_server_start()
 *  runs / if it failed. Lets other modules serialize onto the httpd task
 *  via httpd_queue_work() — see sd_card.c's mount/unmount for why. */
httpd_handle_t http_server_get_handle(void);
