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
#include <stdint.h>

/** @brief Install the log hook and create the mutex. Idempotent. */
void applog_init(void);

// V2.5.20 (review R11): applog_snapshot() removed — dead since V2.3.17 moved
// /log onto the streaming API; it malloc'd the entire ring (up to 4 MB on
// PSRAM boards) as one contiguous buffer. Use applog_stream_begin() instead.

/** @brief Streaming snapshot — returns up to three segment descriptors the
 *  caller reads in order (seg_a → seg_b → seg_c) to consume the ring in
 *  chronological order.
 *
 *  V2.3.16 introduced this as a zero-copy alternative to applog_snapshot,
 *  pointing all segments directly at ring memory and releasing the mutex
 *  before the upload completed. V2.3.24 fixes a wrap-specific corruption
 *  bug that made the V2.3.16 design unsafe: in the wrapped case the
 *  oldest-data segment started exactly at `s_ring + s_pos + skip`, which is
 *  also where the writer's next ring_append() lands. Any log line emitted
 *  during the upload window (FTPS handshake messages, concurrent TX worker
 *  output, WiFi events) overwrote the start of that segment in-place,
 *  producing torn lines at the head of every uploaded file once the ring
 *  had wrapped at least once.
 *
 *  V2.3.24 fix: in the wrapped case, copy the first APPLOG_SNAP_SCRATCH_BYTES
 *  of the oldest-half tail into a pre-allocated scratch buffer at begin().
 *  The reader streams [scratch] → [tail remainder still in ring] → [newer
 *  pre-wrap half]. Writes during the snapshot land in the scratch's
 *  original ring address, leaving the scratch copy untouched. Provided
 *  cumulative writes during the snapshot stay below
 *  APPLOG_SNAP_SCRATCH_BYTES + skip, the snapshot's first segment is
 *  bit-perfect. If they exceed, corruption resumes past the scratch
 *  boundary — never worse than the V2.3.16 behaviour.
 *
 *  Mapping of segments per ring state:
 *    not wrapped:  seg_a = ring[0 .. s_valid_end];   seg_b/seg_c = NULL/0
 *    wrapped:      seg_a = scratch copy of oldest danger zone;
 *                  seg_b = ring tail remainder past the danger zone;
 *                  seg_c = ring[0 .. s_pos]   (newer pre-wrap half)
 *
 *  Returns false if applog has not been initialised, OR if another snapshot
 *  is already in flight (V2.8.0); otherwise true, with all-NULL segments if
 *  the ring is empty. Only one snapshot may be live at a time because the
 *  scratch copy is a single shared buffer that the caller streams after the
 *  mutex is released — a second begin() would overwrite it under the first
 *  reader. Callers must handle false: /log answers 503 + Retry-After, the
 *  FTPS uploader skips that upload.
 *
 *  Pair every begin() with exactly one end(). end() is no longer a no-op:
 *  it releases the in-flight slot and MUST be called on EVERY exit path
 *  after a successful begin(), or all later snapshots are locked out for
 *  the life of the boot. Scratch is still reused across snapshots, not
 *  freed.
 */
typedef struct {
    const char *seg_a;   /**< first segment (scratch copy when wrapped, ring otherwise); NULL if empty */
    size_t      len_a;
    const char *seg_b;   /**< second segment — ring tail past the scratch copy; NULL if not used */
    size_t      len_b;
    const char *seg_c;   /**< third segment — newer pre-wrap half; NULL if not wrapped */
    size_t      len_c;
} applog_stream_t;

bool applog_stream_begin(applog_stream_t *out);
void applog_stream_end(void);

/** @brief V2.8.17: run @p publish (syslog going live) under the applog mutex and
 *         return the ring's logical write offset at that moment.
 *
 *  Bytes before the offset were appended while syslog was down and never
 *  left the device; everything after it was forwarded live. Before the ring
 *  first wraps, the offset is also the byte index into a snapshot's seg_a,
 *  so syslog_init() can replay exactly the pre-syslog part of the boot.
 *  Returns 0 (and still calls @p publish) when applog isn't initialised.
 */
uint64_t applog_publish_mark(void (*publish)(const void *arg), const void *arg);

/** @brief V2.8.17: size of the ring in bytes (HAL_LOG_RING_BYTES), so a
 *         reader can tell how much room is left before the ring wraps. */
size_t applog_ring_size(void);
