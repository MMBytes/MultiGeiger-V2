#include "history.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "tube.h"   // tube_get_total_counts(), tube_is_enabled()

static const char *TAG = "history";

#define HIST_SAMPLE_MS   60000u   // 60 s between minute samples
#define HIST_MIN_PER_HR  60u      // minute samples that roll up into one hour

// --- Ring state (guarded by s_mux for cross-task reads) ---------------------
static uint16_t s_min[HIST_MIN_DEPTH];
static uint8_t  s_min_head;        // index of next write
static uint8_t  s_min_count;       // valid entries (caps at HIST_MIN_DEPTH)

static uint16_t s_hour[HIST_HOUR_DEPTH];
static uint8_t  s_hour_head;
static uint8_t  s_hour_count;

static uint16_t s_cpm_now;
static uint16_t s_cpm5;
static uint16_t s_cpm15;

// --- Sampler bookkeeping (writer-only; not under the mutex) -----------------
static uint32_t s_last_total;      // tube total at the previous sample
static uint32_t s_last_ms;         // ms at the previous sample
static bool     s_primed;          // first tick establishes the baseline
static uint32_t s_hour_sum;        // running sum of minute samples this hour
static uint8_t  s_hour_n;          // minute samples accumulated this hour

static SemaphoreHandle_t s_mux;

// Push v into a circular ring (head = next write, count caps at depth).
static void ring_push(uint16_t *ring, uint8_t depth,
                      uint8_t *head, uint8_t *count, uint16_t v) {
    ring[*head] = v;
    *head = (uint8_t)((*head + 1) % depth);
    if (*count < depth) (*count)++;
}

// Mean of the newest n samples (n clamped to count). 0 if empty.
static uint16_t ring_mean_last(const uint16_t *ring, uint8_t depth,
                               uint8_t head, uint8_t count, uint8_t n) {
    if (count == 0) return 0;
    if (n > count) n = count;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t idx = (uint8_t)((head + depth - 1 - i) % depth);   // newest backwards
        sum += ring[idx];
    }
    return (uint16_t)(sum / n);
}

void history_init(void) {
    memset(s_min,  0, sizeof(s_min));
    memset(s_hour, 0, sizeof(s_hour));
    s_min_head = s_min_count = 0;
    s_hour_head = s_hour_count = 0;
    s_cpm_now = s_cpm5 = s_cpm15 = 0;
    s_last_total = s_last_ms = 0;
    s_primed = false;
    s_hour_sum = 0;
    s_hour_n = 0;
    s_mux = xSemaphoreCreateMutex();
    if (!s_mux) ESP_LOGE(TAG, "mutex alloc failed — history disabled");
}

void history_tick(uint32_t now_ms) {
    if (!s_mux) return;
    if (!tube_is_enabled()) return;   // radiation-only; dust node collects nothing

    // First tick establishes the count/time baseline (now_ms is unknown at init).
    if (!s_primed) {
        s_last_total = tube_get_total_counts();
        s_last_ms    = now_ms;
        s_primed     = true;
        return;
    }
    if ((uint32_t)(now_ms - s_last_ms) < HIST_SAMPLE_MS) return;
    s_last_ms = now_ms;

    uint32_t total = tube_get_total_counts();
    uint32_t d = total - s_last_total;        // unsigned, wrap-safe
    s_last_total = total;
    uint16_t cpm = (d > 0xFFFEu) ? 0xFFFEu : (uint16_t)d;   // 60 s delta == CPM

    xSemaphoreTake(s_mux, portMAX_DELAY);
    ring_push(s_min, HIST_MIN_DEPTH, &s_min_head, &s_min_count, cpm);
    s_cpm_now = cpm;
    s_cpm5  = ring_mean_last(s_min, HIST_MIN_DEPTH, s_min_head, s_min_count, 5);
    s_cpm15 = ring_mean_last(s_min, HIST_MIN_DEPTH, s_min_head, s_min_count, 15);

    // Hourly rollup by SAMPLE COUNT (drift-proof): 60 minute samples → 1 hour.
    s_hour_sum += cpm;
    if (++s_hour_n >= HIST_MIN_PER_HR) {
        ring_push(s_hour, HIST_HOUR_DEPTH, &s_hour_head, &s_hour_count,
                  (uint16_t)(s_hour_sum / HIST_MIN_PER_HR));
        s_hour_sum = 0;
        s_hour_n = 0;
    }
    xSemaphoreGive(s_mux);
}

void history_get(history_snapshot_t *out) {
    // Empty by default — sentinel-fill the arrays so unfilled slots read clean.
    memset(out->cpm_min,  0xFF, sizeof(out->cpm_min));    // → HIST_EMPTY
    memset(out->cpm_hour, 0xFF, sizeof(out->cpm_hour));
    out->min_count = out->hour_count = 0;
    out->cpm_now = out->cpm5 = out->cpm15 = 0;
    if (!s_mux) return;

    xSemaphoreTake(s_mux, portMAX_DELAY);
    for (uint8_t i = 0; i < s_min_count; i++) {
        uint8_t idx = (uint8_t)((s_min_head + HIST_MIN_DEPTH - s_min_count + i) % HIST_MIN_DEPTH);
        out->cpm_min[i] = s_min[idx];
    }
    for (uint8_t i = 0; i < s_hour_count; i++) {
        uint8_t idx = (uint8_t)((s_hour_head + HIST_HOUR_DEPTH - s_hour_count + i) % HIST_HOUR_DEPTH);
        out->cpm_hour[i] = s_hour[idx];
    }
    out->min_count  = s_min_count;
    out->hour_count = s_hour_count;
    out->cpm_now = s_cpm_now;
    out->cpm5    = s_cpm5;
    out->cpm15   = s_cpm15;
    xSemaphoreGive(s_mux);
}
