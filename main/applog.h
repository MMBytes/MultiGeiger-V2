#pragma once

/** @file
 *  @brief In-memory rolling log buffer backing the /log web endpoint.
 *
 *  Installs an esp_log_set_vprintf() hook so every ESP_LOGx line — from our
 *  code, from WiFi, HTTP, bme280, anything — is echoed to UART unchanged and
 *  also appended to a 60 KB ring buffer. A small exclusion list suppresses
 *  known-noisy lines so useful context isn't pushed out.
 *
 *  Call applog_init() as the first step in app_main() to capture boot lines.
 */

#include <stddef.h>

/** @brief Install the log hook and create the mutex. Idempotent. */
void applog_init(void);

/** @brief Snapshot the ring in chronological order.
 *
 *  Returns a malloc'd, NUL-terminated buffer. *out_len is set to the length
 *  (excluding NUL). Caller must free. Returns NULL only on OOM; an empty ring
 *  returns a valid zero-length buffer. Safe to call from any task — briefly
 *  holds the log mutex while copying.
 */
char *applog_snapshot(size_t *out_len);
