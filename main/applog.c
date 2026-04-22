#include "applog.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

// 60 KB rolling log buffer. The ESP-IDF vprintf hook captures every ESP_LOGx
// line (our code + WiFi + HTTP + bme280 + ...) without touching call sites.
// The hook runs on whichever task emitted the line, so the ring and its
// bookkeeping are protected by a FreeRTOS mutex.

#define LOG_RING_SIZE   (60 * 1024)
#define LOG_LINE_MAX    1024

static char              s_ring[LOG_RING_SIZE];
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

    char line[LOG_LINE_MAX];
    int n = vsnprintf(line, sizeof(line), fmt, args);
    if (n > 0) {
        if (n >= (int)sizeof(line)) n = sizeof(line) - 1;
        line[n] = 0;
        strip_ansi(line);
        if (!is_excluded(line)) {
            size_t len = strlen(line);
            if (len > 0) ring_append(line, len);
        }
    }

    xSemaphoreGive(s_mtx);
    return rc;
}

void applog_init(void) {
    if (s_initialized) return;
    s_mtx = xSemaphoreCreateMutex();
    if (!s_mtx) return;  // caller continues; hook just won't install
    s_prev_hook   = esp_log_set_vprintf(applog_vprintf);
    s_initialized = true;
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
