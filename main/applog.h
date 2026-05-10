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
#include <stdbool.h>

/** @brief Install the log hook and create the mutex. Idempotent. */
void applog_init(void);

/** @brief Snapshot the ring in chronological order.
 *
 *  Returns a malloc'd, NUL-terminated buffer. *out_len is set to the length
 *  (excluding NUL). Caller must free. Returns NULL only on OOM; an empty ring
 *  returns a valid zero-length buffer. Safe to call from any task — briefly
 *  holds the log mutex while copying.
 *
 *  Used by the /log HTTP endpoint where a single Content-Length-friendly
 *  contiguous response is needed. The FTPS upload path uses the streaming
 *  API below instead, to avoid allocating a body buffer the size of the ring
 *  (especially important for the 1 MB PSRAM ring on FeatherS3-D and to keep
 *  the Heltec's tight internal-DRAM budget healthy at upload peak).
 */
char *applog_snapshot(size_t *out_len);

/** @brief Streaming snapshot — captures segment pointers into the ring with
 *  no allocation.
 *
 *  V2.3.16: zero-copy alternative to applog_snapshot() for the FTPS upload
 *  path. Returns up to two segment descriptors (seg_a + optional seg_b for
 *  the wrapped case) that point DIRECTLY into the ring memory. The caller
 *  reads from these pointers and sends the data via TLS in chunks of its
 *  own choosing. No body buffer allocated; total transient heap demand
 *  during upload drops by `ring_size` (60 KB on Heltec, 1 MB on FeatherS3-D).
 *
 *  Lifetime: the returned pointers are valid until applog_stream_end() is
 *  called. The mutex is held only briefly during applog_stream_begin() (to
 *  capture the segments); the writer (applog_vprintf) continues to advance
 *  the ring during the stream window. This means the LAST few bytes of the
 *  stream may include data the writer added after begin() returned (small
 *  inconsistency at the wrap boundary). Acceptable trade-off vs the cost
 *  of pausing all log writes for the upload duration.
 *
 *  Returns false only if applog has not been initialised; otherwise true,
 *  with seg_a/seg_b NULL+0 if the ring is empty.
 *
 *  Pair every begin() with exactly one end(). Currently end() is a no-op;
 *  reserved for future write-pause coordination if the in-flight inconsistency
 *  becomes a problem in practice.
 */
typedef struct {
    const char *seg_a;   /**< first contiguous segment, NULL if empty stream */
    size_t      len_a;
    const char *seg_b;   /**< second segment after wrap (NULL if not wrapped) */
    size_t      len_b;
} applog_stream_t;

bool applog_stream_begin(applog_stream_t *out);
void applog_stream_end(void);
