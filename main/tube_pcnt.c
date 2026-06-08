#include "tube_pcnt.h"

#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal.h"   // PIN_GMC_COUNT_INPUT

static const char *TAG = "tube_pcnt";

// Default width comb (ns). Index 0 = 0 = unfiltered reference. The widest tooth
// (index N-1) is the FILTER width and is overridden at init from config
// (pcnt_filter_width_ns); the lower teeth are fixed diagnostic references. All
// under the ~12.8 µs APB-filter ceiling.
const uint32_t TUBE_PCNT_WIDTHS_NS[TUBE_PCNT_NWIDTHS] = { 0, 250, 1000, 4000 };

// Actual widths in use this session (defaults, with the widest tooth replaced
// by the configured filter width). Logged + reported via tube_pcnt_width_ns().
static uint32_t s_widths[TUBE_PCNT_NWIDTHS];

// Counter only ever INCREASES (falling-edge count). Each unit runs as a
// MONOTONIC accumulator: with flags.accum_count + a watch point at high_limit,
// the driver adds high_limit to an internal accumulator each time the 16-bit
// counter overflows and resets, so pcnt_unit_get_count() returns a running
// total since boot. ~1.5 cps → one 32767 overflow ≈ every 6 h, far longer than
// any read gap, so even a watch-ISR delayed by a flash write can't lose counts.
// We NEVER clear after init; per-cycle values are software deltas (total minus
// last-read) — which also removes the old read-then-clear lost-edge window.
// The widest unit's total feeds history.c (filtered cpm5/cpm15) when the filter
// is on; the IDF driver requires low_limit < 0, so -1 (never reached).
#define TUBE_PCNT_HIGH_LIMIT  32767
#define TUBE_PCNT_LOW_LIMIT   (-1)

static pcnt_unit_handle_t    s_units[TUBE_PCNT_NWIDTHS] = { 0 };
static pcnt_channel_handle_t s_chans[TUBE_PCNT_NWIDTHS] = { 0 };
static uint32_t              s_last_read[TUBE_PCNT_NWIDTHS] = { 0 };  // per-cycle delta base
static bool s_active = false;

bool tube_pcnt_active(void) { return s_active; }

// Best-effort teardown of every unit/channel created so far. Used only on the
// init failure path; returns are intentionally ignored (a half-built unit may
// not be in a stoppable/disable-able state, which is fine — we just want the
// peripheral and its GPIO-matrix tap released).
static void tube_pcnt_teardown(void) {
    for (int i = 0; i < TUBE_PCNT_NWIDTHS; i++) {
        if (s_units[i]) {
            pcnt_unit_stop(s_units[i]);
            pcnt_unit_disable(s_units[i]);
        }
        if (s_chans[i]) {
            pcnt_del_channel(s_chans[i]);
            s_chans[i] = NULL;
        }
        if (s_units[i]) {
            pcnt_del_unit(s_units[i]);
            s_units[i] = NULL;
        }
    }
}

uint32_t tube_pcnt_width_ns(int i) {
    if (!s_active || i < 0 || i >= TUBE_PCNT_NWIDTHS) return 0u;
    return s_widths[i];
}

bool tube_pcnt_init(uint32_t filter_width_ns) {
    if (s_active) return true;   // idempotent

    // Widest tooth = the configured filter width; lower teeth = fixed defaults.
    for (int i = 0; i < TUBE_PCNT_NWIDTHS; i++) {
        s_widths[i] = (i == TUBE_PCNT_NWIDTHS - 1) ? filter_width_ns
                                                   : TUBE_PCNT_WIDTHS_NS[i];
    }

    esp_err_t err;
    for (int i = 0; i < TUBE_PCNT_NWIDTHS; i++) {
        s_last_read[i] = 0;
        pcnt_unit_config_t ucfg = {
            .high_limit = TUBE_PCNT_HIGH_LIMIT,
            .low_limit  = TUBE_PCNT_LOW_LIMIT,
            .flags.accum_count = true,   // accumulate across 16-bit overflow
        };
        err = pcnt_new_unit(&ucfg, &s_units[i]);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "unit %d new failed: %s", i, esp_err_to_name(err));
            goto fail;
        }

        // Glitch filter must be set while the unit is still in the init state
        // (before enable). Skip unit 0 — it's the unfiltered reference.
        if (s_widths[i] > 0) {
            pcnt_glitch_filter_config_t fcfg = {
                .max_glitch_ns = s_widths[i],
            };
            err = pcnt_unit_set_glitch_filter(s_units[i], &fcfg);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "unit %d filter %luns failed: %s",
                         i, (unsigned long)s_widths[i],
                         esp_err_to_name(err));
                goto fail;
            }
        }

        // Tap the SAME pad as the GMC ISR. The GPIO matrix fans the input out
        // to this PCNT unit without disturbing the CPU-interrupt path.
        pcnt_chan_config_t ccfg = {
            .edge_gpio_num  = PIN_GMC_COUNT_INPUT,
            .level_gpio_num = -1,
        };
        err = pcnt_new_channel(s_units[i], &ccfg, &s_chans[i]);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "unit %d channel failed: %s", i, esp_err_to_name(err));
            goto fail;
        }

        // Count FALLING edges (negative-edge → increase), matching
        // gmc_count_isr's GPIO_INTR_NEGEDGE convention.
        err = pcnt_channel_set_edge_action(
                  s_chans[i],
                  PCNT_CHANNEL_EDGE_ACTION_HOLD,       // rising: ignore
                  PCNT_CHANNEL_EDGE_ACTION_INCREASE);  // falling: count
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "unit %d edge action failed: %s", i, esp_err_to_name(err));
            goto fail;
        }

        // Watch point at high_limit drives the accum_count overflow handler
        // (must be added in the init state, before enable). Only the up-limit
        // is reachable since we never decrement.
        err = pcnt_unit_add_watch_point(s_units[i], TUBE_PCNT_HIGH_LIMIT);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "unit %d watch point failed: %s", i, esp_err_to_name(err));
            goto fail;
        }

        err = pcnt_unit_enable(s_units[i]);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "unit %d enable failed: %s", i, esp_err_to_name(err));
            goto fail;
        }
        err = pcnt_unit_clear_count(s_units[i]);
        if (err != ESP_OK) { goto fail; }
        err = pcnt_unit_start(s_units[i]);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "unit %d start failed: %s", i, esp_err_to_name(err));
            goto fail;
        }
    }

    s_active = true;
    ESP_LOGI(TAG, "PCNT width-comb up on GMC pin %d: widths_ns=%lu/%lu/%lu/%lu "
             "(widest = filter width; parallel to ISR)",
             PIN_GMC_COUNT_INPUT,
             (unsigned long)s_widths[0], (unsigned long)s_widths[1],
             (unsigned long)s_widths[2], (unsigned long)s_widths[3]);
    return true;

fail:
    tube_pcnt_teardown();
    s_active = false;
    ESP_LOGW(TAG, "PCNT width-comb DISABLED (init error) — ISR counting unaffected");
    return false;
}

void tube_pcnt_stop(void) {
    if (!s_active) return;   // idempotent — nothing up
    tube_pcnt_teardown();    // releases the units/channels + their internal DRAM
    s_active = false;
    ESP_LOGI(TAG, "PCNT width-comb stopped (released for OTA/teardown)");
}

void tube_pcnt_read(uint32_t out[TUBE_PCNT_NWIDTHS]) {
    for (int i = 0; i < TUBE_PCNT_NWIDTHS; i++) {
        if (s_active && s_units[i]) {
            // Per-cycle value = delta of the monotonic accumulator since the
            // last read (no clear → no lost-edge window). Unsigned subtraction
            // is wrap-safe. This consumer's last-read base is independent of
            // history.c's, so both can read the same total without interfering.
            int total = 0;
            pcnt_unit_get_count(s_units[i], &total);
            uint32_t t = (total < 0) ? 0u : (uint32_t)total;
            out[i] = t - s_last_read[i];
            s_last_read[i] = t;
        } else {
            out[i] = 0u;
        }
    }
}

uint32_t tube_pcnt_filtered_total(void) {
    if (!s_active) return 0u;
    int total = 0;
    pcnt_unit_get_count(s_units[TUBE_PCNT_NWIDTHS - 1], &total);  // widest = filter width
    return (total < 0) ? 0u : (uint32_t)total;
}
