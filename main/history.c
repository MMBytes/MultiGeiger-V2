#include "history.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "tube.h"        // tube_get_total_counts(), tube_is_enabled()
#include "tube_pcnt.h"   // V2.5.16: tube_pcnt_filtered_total() (filtered source)

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
static bool     s_filtered_src;    // V2.5.16: which total feeds the last sample
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
    s_filtered_src = false;
    s_hour_sum = 0;
    s_hour_n = 0;
    s_mux = xSemaphoreCreateMutex();
    if (!s_mux) ESP_LOGE(TAG, "mutex alloc failed — history disabled");
}

void history_tick(uint32_t now_ms, bool use_filtered) {
    if (!s_mux) return;
    if (!tube_is_enabled()) return;   // radiation-only; dust node collects nothing

    // V2.5.16: count source = PCNT width-filtered monotonic total when the
    // filter is on, else the raw ISR total — so cpm5/cpm15 stay consistent with
    // the filtered per-cycle CPM. Both totals are monotonic since boot; this is
    // the only place that reads the filtered one for the rolling averages.
    // V2.6.29 (PCNT subtract mode): the filtered source additionally subtracts
    // the monotonic blanked total, mirroring do_tx_cycle's per-cycle
    // subtraction — phantoms wide enough to pass the width filter are inside
    // the PCNT count, and hv_blanked is exactly their tally. Both are
    // monotonic, so the difference's per-minute DELTA is the corrected count;
    // with blanking off the blanked total is frozen (or 0) and the delta
    // contribution is zero, and the UNFILTERED branch is untouched — plain
    // counting mode sees no change. A blank on/off toggle shifts only future
    // increments, never the level, so no source re-prime is needed.
    uint32_t total = use_filtered ? (tube_pcnt_filtered_total() - tube_get_blanked_total())
                                  : tube_get_total_counts();

    // (Re)prime on the first tick (now_ms unknown at init) OR on a source
    // switch (a runtime pcnt_filter toggle) — the two totals have different
    // magnitudes, so carrying a delta across the switch would inject one
    // garbage minute. Re-priming skips a single sample instead.
    if (!s_primed || use_filtered != s_filtered_src) {
        s_last_total   = total;
        s_last_ms      = now_ms;
        s_primed       = true;
        s_filtered_src = use_filtered;
        return;
    }
    if ((uint32_t)(now_ms - s_last_ms) < HIST_SAMPLE_MS) return;
    s_last_ms = now_ms;

    uint32_t d = total - s_last_total;        // unsigned, wrap-safe
    s_last_total = total;
    // V2.6.29: a mis-configured subtract mode (phantoms NARROWER than the
    // width filter → blanked pulses were never in the PCNT count → double
    // subtraction) can make the composed filtered total DECREASE within a
    // minute; the unsigned delta then wraps huge and the clamp below would
    // paint a 65534 spike on the graph. A top-bit delta is that wrap (a real
    // 2^31-count minute is physically impossible) — read it as 0.
    if (d & 0x80000000u) d = 0;
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
