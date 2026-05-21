#include "applog.h"
#include "hal.h"
#include "syslog.h"   // V2.4.15: per-line UDP forward in applog_vprintf

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"   // V2.3.15: also used for boot-time ring-region log

static const char *TAG = "applog";

// Rolling log buffer. The ESP-IDF vprintf hook captures every ESP_LOGx line
// (our code + WiFi + HTTP + bme280 + ...) without touching call sites. The
// hook runs on whichever task emitted the line, so the ring and its
// bookkeeping are protected by a FreeRTOS mutex.
//
// Buffer size is chosen per-board in hal.h via HAL_LOG_RING_BYTES — small
// boards stay at 60 KB internal DRAM, PSRAM boards size to a fraction of
// available external RAM (4 MB on FeatherS3-D 8 MB, 1 MB on QT Py 2 MB).
// Streaming /log + FTPS (V2.3.16/17) means ring size no longer creates
// transient internal-DRAM peaks, so the only practical cost is FTPS upload
// time scaling with body length (~5 s per MB on the LAN).

#define LOG_RING_SIZE   HAL_LOG_RING_BYTES
#define LOG_LINE_MAX    1024

// V2.3.24: snapshot scratch buffer size — sourced from hal.h per board so
// internal-DRAM-only boards (Heltec) can run a tight 6 KB and PSRAM boards
// (FeatherS3-D, QT Py) get a more generous 16 KB. At applog_stream_begin()
// we copy the first SNAP_SCRATCH_BYTES of the oldest-half tail (the segment
// most exposed to writer corruption — see applog.h header). Sized to cover
// realistic cumulative log writes during a 3-30 s FTPS upload: ~1 KB from
// the FTPS handshake itself, plus ~1-3 KB of concurrent TX worker / WiFi /
// status-page activity. Pre-allocated at init so there's no per-snapshot
// heap activity and no malloc-failure path.
#define SNAP_SCRATCH_BYTES HAL_LOG_SNAP_SCRATCH_BYTES

// Heap-allocated rather than static so the PSRAM path can request the
// allocation come from external SPIRAM. Allocated once in applog_init();
// applog_vprintf only runs after init succeeds.
static char             *s_ring        = NULL;
static char             *s_snap_scratch = NULL; // V2.3.24: snapshot scratch (see SNAP_SCRATCH_BYTES)
static size_t            s_pos         = 0;     // next write offset
static bool              s_wrapped     = false; // has the ring wrapped at least once
static size_t            s_valid_end   = 0;     // high-water when not wrapped; == size when wrapped
static SemaphoreHandle_t s_mtx         = NULL;
static vprintf_like_t    s_prev_hook   = NULL;
static bool              s_initialized = false;

// Exclusion list — lines containing any of these substrings are echoed to UART
// but NOT added to the ring buffer, so known-noisy lines don't push useful
// context out. Grow as new offenders appear.
static const char *const s_exclusions[] = {
    "esp-x509-crt-bundle: Certificate validated", // 3x per TX cycle
    NULL,
};

static bool is_excluded(const char *line) {
    for (const char *const *e = s_exclusions; *e; e++) {
        if (strstr(line, *e)) return true;
    }
    return false;
}

// The WiFi driver bypasses ESP_LOGx and prints its own lines via raw printf
// with a boot-relative tick (e.g. "I (82885037) wifi:Set ps type: 0...").
// CONFIG_LOG_TIMESTAMP_SOURCE_SYSTEM_FULL only formats lines that go through
// esp_log_writev, so those driver lines come out uglier than ours. Detect the
// pattern after vsnprintf and rewrite the (digits) segment to wall-clock form
// matching the rest of the log: "(YY-MM-DD HH:MM:SS.mmm)". UART echo is
// untouched (still raw) — only the ring buffer / /log / FTP'd files benefit.
static void rewrite_boot_ts(char *line, size_t bufsz) {
    char level = line[0];
    if (level != 'I' && level != 'W' && level != 'E' && level != 'D' && level != 'V') return;
    if (line[1] != ' ' || line[2] != '(') return;

    size_t i = 3;
    while (line[i] >= '0' && line[i] <= '9') i++;
    if (i == 3 || line[i] != ')' || line[i + 1] != ' ') return;

    // Restrict to known-noisy native modules so we never clobber a legitimate
    // numeric (...) field in someone else's message body. Add tags here as
    // they crop up in logs.
    if (strncmp(line + i + 2, "wifi:", 5) != 0) return;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    if (tv.tv_sec < 1700000000) return;  // pre-2023 — NTP not synced; leave raw

    struct tm tm;
    localtime_r(&tv.tv_sec, &tm);
    char ts[32];
    int ts_len = snprintf(ts, sizeof(ts),
                          "%02d-%02d-%02d %02d:%02d:%02d.%03ld",
                          (tm.tm_year + 1900) % 100, tm.tm_mon + 1, tm.tm_mday,
                          tm.tm_hour, tm.tm_min, tm.tm_sec,
                          (long)(tv.tv_usec / 1000));
    if (ts_len <= 0) return;

    size_t old_paren = (i + 1) - 2;        // length of "(<digits>)"
    size_t new_paren = (size_t)ts_len + 2; // length of "(<ts>)"
    size_t total     = strlen(line);
    if (new_paren > old_paren && total + (new_paren - old_paren) >= bufsz) return;

    // Shift the tail (everything after the closing paren, including the null).
    memmove(line + 2 + new_paren,
            line + 2 + old_paren,
            total - (2 + old_paren) + 1);
    line[2] = '(';
    memcpy(line + 3, ts, ts_len);
    line[3 + ts_len] = ')';
}

// Strip ANSI CSI escape sequences (e.g. "\033[0;32m"). ESP-IDF colorizes log
// levels when CONFIG_LOG_COLORS=y — keep them on UART but drop them from the
// ring so the /log output is plain text. Writes result back into buf.
static void strip_ansi(char *buf) {
    char *r = buf;
    char *w = buf;
    while (*r) {
        if (*r == '\033' && r[1] == '[') {
            r += 2;
            while (*r && *r != 'm' && *r != 'K' && *r != 'H' && *r != 'J') r++;
            if (*r) r++;
        } else {
            *w++ = *r++;
        }
    }
    *w = 0;
}

// Append raw bytes to the ring, maintaining wrap + valid_end.
static void ring_append(const char *data, size_t len) {
    while (len > 0) {
        size_t space = LOG_RING_SIZE - s_pos;
        size_t n = (len < space) ? len : space;
        memcpy(s_ring + s_pos, data, n);
        s_pos += n;
        data  += n;
        len   -= n;
        if (s_pos >= LOG_RING_SIZE) {
            s_pos = 0;
            s_wrapped = true;
            s_valid_end = LOG_RING_SIZE;
        } else if (!s_wrapped && s_pos > s_valid_end) {
            s_valid_end = s_pos;
        }
    }
}

static int applog_vprintf(const char *fmt, va_list args) {
    // Without the mutex in place yet (very early boot), fall back to a
    // direct vprintf so the first few log lines still make it to UART.
    if (!s_mtx) return vprintf(fmt, args);

    // Serialise UART echo *and* ring insertion under one mutex. Without this
    // two tasks calling ESP_LOGx concurrently (e.g. main-task FTP + CPU1 TX
    // worker) would interleave character-by-character in the UART stream —
    // observed as mashed-together lines with missing newlines.
    xSemaphoreTake(s_mtx, portMAX_DELAY);

    // va_copy: vprintf-family consumes the va_list, so we need a fresh copy
    // for the subsequent vsnprintf into the ring buffer.
    va_list args_echo;
    va_copy(args_echo, args);
    int rc = vprintf(fmt, args_echo);
    va_end(args_echo);

    // V2.4.20: line[] moved from stack to BSS. Same V2.4.16 fix pattern
    // as syslog_emit's buf[600]: a kilobyte of stack inside a logger that
    // any task can call from any code path is fragile, and the httpd
    // task's 8 KB stack is the tightest budget in the system. Concrete
    // failure case: 2026-05-21 V2.4.19 GET /config from an unauth client
    // → check_auth's locals (~250 B) + the vprintf/bufio/console_write/
    // usb_serial_jtag_write chain stacked on top of this 1 KB buffer ran
    // the httpd stack past its limit; corruption landed on the TCB and
    // FreeRTOS's `xTaskPriorityDisinherit` assertion (uxMutexesHeld == 0)
    // fired. Safe to move to BSS because s_mtx (taken above, released
    // below at the end of this function) serialises all callers — only
    // one task ever touches line at a time.
    static char line[LOG_LINE_MAX];
    int n = vsnprintf(line, sizeof(line), fmt, args);
    if (n > 0) {
        if (n >= (int)sizeof(line)) n = sizeof(line) - 1;
        line[n] = 0;
        strip_ansi(line);
        rewrite_boot_ts(line, sizeof(line));
        if (!is_excluded(line)) {
            size_t len = strlen(line);
            if (len > 0) {
                ring_append(line, len);
                // V2.4.15: also ship to UDP syslog (no-op if disabled or
                // pre-init). Inside applog's mutex but harmless — sendto
                // is non-blocking (MSG_DONTWAIT) and syslog_emit's own
                // s_in_emit guard plus its NEVER-calling-ESP_LOG rule
                // prevent any re-entry into vprintf.
                syslog_emit(line, len);
            }
        }
    }

    xSemaphoreGive(s_mtx);
    return rc;
}

void applog_init(void) {
    if (s_initialized) return;
#if HAL_HAS_PSRAM
    // Pull from external SPIRAM so the 1 MB ring doesn't eat internal DRAM.
    s_ring = heap_caps_malloc(LOG_RING_SIZE, MALLOC_CAP_SPIRAM);
#else
    s_ring = malloc(LOG_RING_SIZE);
#endif
    if (!s_ring) return;  // caller continues; hook just won't install

    // V2.3.24: pre-allocate snapshot scratch. Same memory class as the ring
    // (PSRAM where available, internal DRAM otherwise). If allocation fails
    // here the streaming snapshot falls back to the V2.3.16-compatible
    // zero-copy path with the known torn-line corruption — degraded but
    // still functional, so don't bail.
#if HAL_HAS_PSRAM
    s_snap_scratch = heap_caps_malloc(SNAP_SCRATCH_BYTES, MALLOC_CAP_SPIRAM);
#else
    s_snap_scratch = malloc(SNAP_SCRATCH_BYTES);
#endif

    s_mtx = xSemaphoreCreateMutex();
    if (!s_mtx) {
        free(s_ring);
        s_ring = NULL;
        if (s_snap_scratch) { free(s_snap_scratch); s_snap_scratch = NULL; }
        return;
    }
    s_prev_hook   = esp_log_set_vprintf(applog_vprintf);
    s_initialized = true;

    // V2.3.15: one-shot diagnostic so the boot log explicitly says where the
    // ring landed and how much headroom remains in that region. Lets future-me
    // confirm at a glance (per board) that PSRAM detection worked on FeatherS3-D
    // (or didn't, in which case we'd have silently fallen back to the 60 KB
    // SRAM path with no clue why /log only holds an hour of context).
#if HAL_HAS_PSRAM
    ESP_LOGI(TAG, "ring %u B in PSRAM (free SPIRAM after alloc: %u B)",
             (unsigned)LOG_RING_SIZE,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
#else
    ESP_LOGI(TAG, "ring %u B in internal DRAM (free heap after alloc: %u B)",
             (unsigned)LOG_RING_SIZE,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
#endif
}

char *applog_snapshot(size_t *out_len) {
    if (!s_mtx) {
        char *empty = malloc(1);
        if (!empty) { if (out_len) *out_len = 0; return NULL; }
        empty[0] = 0;
        if (out_len) *out_len = 0;
        return empty;
    }

    xSemaphoreTake(s_mtx, portMAX_DELAY);

    size_t total;
    char  *out;
    if (s_wrapped) {
        total = LOG_RING_SIZE;
        out = malloc(total + 1);
        if (!out) { xSemaphoreGive(s_mtx); if (out_len) *out_len = 0; return NULL; }
        // Oldest half = [s_pos .. end). Skip to first newline so we don't
        // start on a partial line cut by the wrap. Then the newer half =
        // [0 .. s_pos).
        size_t tail_len = LOG_RING_SIZE - s_pos;
        const char *tail = s_ring + s_pos;
        size_t skip = 0;
        while (skip < tail_len && tail[skip] != '\n') skip++;
        if (skip < tail_len) skip++;  // consume the newline itself
        size_t w = 0;
        memcpy(out + w, tail + skip, tail_len - skip);
        w += tail_len - skip;
        memcpy(out + w, s_ring, s_pos);
        w += s_pos;
        out[w] = 0;
        total = w;
    } else {
        total = s_valid_end;
        out = malloc(total + 1);
        if (!out) { xSemaphoreGive(s_mtx); if (out_len) *out_len = 0; return NULL; }
        memcpy(out, s_ring, total);
        out[total] = 0;
    }

    xSemaphoreGive(s_mtx);
    if (out_len) *out_len = total;
    return out;
}

// V2.3.24: snapshot returns up to three segments — see applog.h for the full
// failure-mode write-up. Summary: in the wrapped case, the oldest-half tail
// starts at exactly s_ring + s_pos + skip, which is also where the writer's
// next ring_append() lands. Releasing the mutex with that pointer exposes the
// segment's start to in-flight overwrites. V2.3.24 fixes this by copying the
// first SNAP_SCRATCH_BYTES of the tail into a pre-allocated scratch buffer
// under the mutex; the caller streams scratch + ring-tail-remainder + newer
// pre-wrap half. The first segment is now bit-stable for the entire snapshot
// lifetime (provided cumulative writes during the upload stay below the
// scratch size + skip — ~8 KB, comfortably above realistic worst case).
//
// Degradation if s_snap_scratch is NULL (allocation failed at init): falls
// back to the V2.3.16 behaviour (zero-copy seg_a pointing into the ring,
// torn-line corruption returns) rather than failing the snapshot outright.
bool applog_stream_begin(applog_stream_t *out) {
    out->seg_a = NULL;
    out->len_a = 0;
    out->seg_b = NULL;
    out->len_b = 0;
    out->seg_c = NULL;
    out->len_c = 0;

    if (!s_mtx) return false;

    xSemaphoreTake(s_mtx, portMAX_DELAY);

    if (s_wrapped) {
        // Skip to the first newline after s_pos so the snapshot starts on a
        // clean line boundary (the byte at s_pos itself is mid-line: it's
        // where the writer's last ring_append() ended, partway through some
        // log entry that was wrapped over).
        size_t tail_len = LOG_RING_SIZE - s_pos;
        const char *tail = s_ring + s_pos;
        size_t skip = 0;
        while (skip < tail_len && tail[skip] != '\n') skip++;
        if (skip < tail_len) skip++;  // consume the newline itself

        const char *tail_start = tail + skip;
        size_t      tail_avail = tail_len - skip;

        if (s_snap_scratch && tail_avail > 0) {
            // Copy the danger zone (first SNAP_SCRATCH_BYTES of the tail) into
            // scratch. memcpy under the mutex — the writer can't be running
            // ring_append concurrently, so the source bytes are stable for
            // the duration of the copy.
            size_t copy_len = (tail_avail < SNAP_SCRATCH_BYTES) ? tail_avail : SNAP_SCRATCH_BYTES;
            memcpy(s_snap_scratch, tail_start, copy_len);

            out->seg_a = s_snap_scratch;
            out->len_a = copy_len;
            // seg_b is the tail remainder past the danger zone — still in
            // ring memory but safe from writer corruption since writes from
            // s_pos forward have to fill skip + SNAP_SCRATCH_BYTES bytes
            // before reaching this region.
            if (tail_avail > copy_len) {
                out->seg_b = tail_start + copy_len;
                out->len_b = tail_avail - copy_len;
            }
            out->seg_c = s_ring;
            out->len_c = s_pos;
        } else {
            // V2.3.16 fallback: scratch unavailable (init OOM). Zero-copy
            // tail pointer; corruption resumes for files past the first wrap.
            out->seg_a = tail_start;
            out->len_a = tail_avail;
            out->seg_b = s_ring;
            out->len_b = s_pos;
            // seg_c stays NULL/0
        }
    } else {
        // Pre-wrap: writer has only ever appended forward, so the segment
        // is naturally line-aligned at byte 0 and writes during the snapshot
        // go to s_pos which is past s_valid_end. No corruption possible —
        // serve the ring directly with no copy.
        out->seg_a = s_ring;
        out->len_a = s_valid_end;
        // seg_b / seg_c stay NULL/0
    }

    xSemaphoreGive(s_mtx);
    return true;
}

void applog_stream_end(void) {
    // V2.3.24: still a no-op. Scratch is reused across snapshots, not freed.
    // Kept as a public API so callers always pair begin/end (forward-compat
    // for any future per-snapshot bookkeeping).
}
