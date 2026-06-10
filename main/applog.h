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
 *  Returns false only if applog has not been initialised; otherwise true,
 *  with all-NULL segments if the ring is empty.
 *
 *  Pair every begin() with exactly one end(). end() remains a no-op —
 *  scratch is reused across snapshots, not freed.
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
