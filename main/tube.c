#include "tube.h"
#include "tube_logic.h"

#include <string.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "tube";

// V2.5.31: clamp_u32() (saturating uint64->uint32 cast for µs deltas) and the
// gmc_classify() count/guard decision moved to the pure, host-testable
// tube_logic.h. Both are always_inline so they still fold into the IRAM count
// ISR at zero call cost and stay in IRAM (the ISR runs with the flash cache
// disabled, so it must not call into flash).

// Runtime enable flag. Set once at boot from tube_setup(enabled). When false,
// no GPIO interrupts or recharge gptimer are installed, and tube_read() short-
// circuits to zeros. We intentionally store this here rather than threading
// it through every caller so the disabled state is invisible to consumers.
static bool s_enabled = false;

// --- Pulse counter state ---
static volatile uint32_t isr_gmc_counts     = 0;
// V2.5.6: monotonic pulse total since boot — NEVER reset by tube_read(). The
// CPM-history sampler (history.c) reads this every 60 s and takes its own
// delta, so its 1-min resolution is fully decoupled from the destructive
// tube_read() window (which the TX cycle drains on its own ~150-180 s cadence).
static volatile uint32_t isr_gmc_total      = 0;
static volatile uint64_t isr_last_pulse_us  = 0;
static volatile uint32_t isr_min_us_between = UINT32_MAX;
static volatile uint32_t isr_max_us_between = 0;
static portMUX_TYPE mux_gmc = portMUX_INITIALIZER_UNLOCKED;

// --- V2.5.12: raw-edge count profiler ---
// isr_raw_edges increments on EVERY interrupt (every edge), before the dead-time
// gate, so (raw_edges - counts) isolates edges rejected as ringing/double-counts.
// isr_dt_hist bins consecutive edge-to-edge spacing (incl. sub-dead-time edges)
// to show WHERE excess edges sit in time: low bins = ringing/noise, top bins =
// real ~1.2 cps pulses. Snapshot/reset via tube_get_diag().
static volatile uint32_t isr_raw_edges    = 0;
static volatile uint64_t isr_last_edge_us = 0;
static volatile uint32_t isr_dt_hist[TUBE_DIAG_NBUCKETS] = {0};

// --- V2.5.30: opt-in retriggerable dead-time guard / burst-collapse ---
// isr_guard_us (0 = OFF) is a refractory window LAYERED on top of the fixed
// GMC_DEAD_TIME_US gate. An edge can start a new count only if it follows a
// quiet gap > isr_guard_us; edges inside the window (each within guard of the
// PRIOR edge) extend the dead zone and are dropped, collapsing an afterpulse /
// re-trigger train to one count. Set once at boot via tube_set_guard_us() (read
// from config), so a plain volatile read in the ISR is all the hot path costs.
// isr_guard_removed tallies the suppressed edges, snapshot+reset per cycle in
// tube_get_diag() and surfaced on the DIAG log line. See config_fields.def.
static volatile uint32_t isr_guard_us      = 0;
static volatile uint32_t isr_guard_removed = 0;

// --- HV charge state ---
static volatile uint32_t isr_hv_pulses    = 0;
static volatile bool     isr_hv_error     = false;
static volatile bool     isr_gmc_cap_full = false;
static portMUX_TYPE mux_hv  = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE mux_cap = portMUX_INITIALIZER_UNLOCKED;

// --- V2.6.9: HV-recharge coincidence diagnostic ---
// isr_last_hv_pulse_us is stamped at the START of every HV charge pulse (the
// S_PULSE_H edge in recharge_tick below — the gptimer ISR, NOT the GPIO count
// ISR). gmc_count_isr reads it under mux_hv (its own writer's lock, since a
// 64-bit value isn't atomic on this 32-bit core and the two ISRs run on
// independent interrupt sources that can preempt each other) and, for edges
// it actually counts, checks whether the gap since that stamp falls inside
// [HV_COINCIDENT_MIN_US, HV_COINCIDENT_MAX_US] (tube.h). isr_hv_coincident
// tallies the hits; snapshot+reset via tube_get_diag() alongside the other
// permanent diagnostics. See radiation_overcounting_independent_review.md
// for the field evidence this is built to test.
static volatile uint64_t isr_last_hv_pulse_us = 0;
static volatile uint32_t isr_hv_coincident    = 0;

// Last read timestamp for dt_ms window.
static int64_t last_read_us = 0;

// Per-pulse callback (speaker/LED tick). Set via tube_set_pulse_callback.
// V2.4.1 (B4): write guarded by the same critical section style as the
// other ISR-shared state in this file. An aligned 32-bit pointer store
// is atomic per ESP32 TRM, but the bare write was the only ISR-touched
// state in tube.c NOT going through portENTER_CRITICAL — fix the
// inconsistency so future callers (post-boot reassignment) don't need
// to re-discover the memory-barrier reasoning. Cost: ~10ns at boot,
// nothing in the ISR hot path (read in ISR is still a single load).
static volatile tube_pulse_cb_t s_pulse_cb = NULL;
static portMUX_TYPE mux_pulse_cb = portMUX_INITIALIZER_UNLOCKED;

void tube_set_pulse_callback(tube_pulse_cb_t cb) {
    portENTER_CRITICAL(&mux_pulse_cb);
    s_pulse_cb = cb;
    portEXIT_CRITICAL(&mux_pulse_cb);
}

// --- Recharge timer — 100 µs tick ---
// State machine: pulse the HV FET high/low, let the cap charge, check the
// cap-full comparator, repeat. On S_FULL the next-charge interval is scaled
// by (1 / pulses-just-used) so idle time tracks the leakage rate — more leaky
// = shorter idle. MAX_CHARGE_PULSES hits indicate a failed tube; back off
// 10 minutes before retrying.
#define PERIOD_DURATION_US 100
#define PERIODS(us) ((us) / PERIOD_DURATION_US)
#define MAX_CHARGE_PULSES 3333

static gptimer_handle_t recharge_timer = NULL;

static bool IRAM_ATTR recharge_tick(gptimer_handle_t timer,
                                    const gptimer_alarm_event_data_t *edata,
                                    void *user_ctx) {
    static uint32_t current = 0;
    static uint32_t next_state = 0;
    static uint32_t next_charge = PERIODS(1000000);  // initial 1 s between recharges
    if (++current < next_state) return false;
    current = 0;

    enum { S_INIT, S_PULSE_H, S_PULSE_L, S_CHECK, S_FULL, S_FAIL } ;
    static int state = S_INIT;
    static int charge_pulses = 0;

    if (state == S_INIT) {
        charge_pulses = 0;
        portENTER_CRITICAL_ISR(&mux_cap);
        isr_gmc_cap_full = false;
        portEXIT_CRITICAL_ISR(&mux_cap);
        state = S_PULSE_H;
    }
    while (state < S_FULL) {
        if (state == S_PULSE_H) {
            gpio_set_level(PIN_HV_FET_OUTPUT, 1);
            // V2.6.9: stamp the coincidence-diagnostic clock at the START of
            // the pulse (not S_FULL/S_CHECK) — that's the earliest moment any
            // electrical coupling into the count node could occur, so it's
            // the correct zero-point for HV_COINCIDENT_MIN/MAX_US in tube.h.
            portENTER_CRITICAL_ISR(&mux_hv);
            isr_last_hv_pulse_us = (uint64_t)esp_timer_get_time();
            portEXIT_CRITICAL_ISR(&mux_hv);
            state = S_PULSE_L;
            next_state = PERIODS(1500);
            return false;
        }
        if (state == S_PULSE_L) {
            gpio_set_level(PIN_HV_FET_OUTPUT, 0);
            state = S_CHECK;
            next_state = PERIODS(1000);
            return false;
        }
        if (state == S_CHECK) {
            charge_pulses++;
            if (isr_gmc_cap_full)            state = S_FULL;
            else if (charge_pulses < MAX_CHARGE_PULSES) state = S_PULSE_H;
            else                             state = S_FAIL;
        }
    }
    if (state == S_FULL) {
        portENTER_CRITICAL_ISR(&mux_hv);
        isr_hv_error  = false;
        isr_hv_pulses += charge_pulses;
        portEXIT_CRITICAL_ISR(&mux_hv);
        state = S_INIT;
        if (charge_pulses <= 1)  next_charge = next_charge * 5 / 4;
        else                     next_charge = next_charge * 2 / charge_pulses;
        if (next_charge < PERIODS(1000))          next_charge = PERIODS(1000);
        else if (next_charge > PERIODS(10000000)) next_charge = PERIODS(10000000);
        next_state = next_charge;
        return false;
    }
    if (state == S_FAIL) {
        portENTER_CRITICAL_ISR(&mux_hv);
        isr_hv_error = true;
        isr_hv_pulses += charge_pulses;
        portEXIT_CRITICAL_ISR(&mux_hv);
        state = S_INIT;
        next_charge = PERIODS(1000000);
        next_state  = PERIODS(10 * 60 * 1000000);   // retry after 10 min
        return false;
    }
    return false;
}

static void IRAM_ATTR gmc_count_isr(void *arg) {
    uint64_t now = (uint64_t)esp_timer_get_time();
    bool counted = false;

    // V2.6.9: snapshot the last HV-pulse stamp under its OWN lock, before
    // taking mux_gmc below — the two critical sections are sequential, not
    // nested, so this never risks a lock-order deadlock against recharge_tick
    // (which only ever takes mux_hv/mux_cap, never mux_gmc).
    uint64_t hv_last;
    portENTER_CRITICAL_ISR(&mux_hv);
    hv_last = isr_last_hv_pulse_us;
    portEXIT_CRITICAL_ISR(&mux_hv);

    portENTER_CRITICAL_ISR(&mux_gmc);

    // V2.5.12: profile every raw edge BEFORE the dead-time gate below.
    // V2.5.30: edt = gap since the previous edge, clamped (clamp_u32) so a >71.6
    // min quiet gap can't wrap the uint32 and mis-bin / spuriously guard-block.
    // Defaults to UINT32_MAX so the first-ever edge (le==0) reads as "maximally
    // separated" — it can never satisfy edt <= isr_guard_us, so the guard below
    // needs no special first-edge term. Reused by the guard (no recompute).
    isr_raw_edges++;
    uint64_t le = isr_last_edge_us;
    uint32_t edt = UINT32_MAX;
    if (le != 0) {
        edt = clamp_u32(now - le);
        uint32_t b;
        if      (edt <     50) b = 0;
        else if (edt <    190) b = 1;
        else if (edt <    500) b = 2;
        else if (edt <   1000) b = 3;
        else if (edt <   5000) b = 4;
        else if (edt <  50000) b = 5;
        else if (edt < 500000) b = 6;
        else                   b = 7;
        isr_dt_hist[b]++;
    }
    isr_last_edge_us = now;

    // dt = gap since the last COUNTED pulse (clamp_u32, same wrap guard as edt).
    // past_gate = would this edge clear the fixed 190µs dead-time gate? The
    // first-ever edge (last==0) has no prior pulse and always passes.
    uint64_t last = isr_last_pulse_us;
    uint32_t dt = 0;
    bool past_gate;
    if (last == 0) {
        past_gate = true;
    } else {
        dt = clamp_u32(now - last);
        past_gate = (dt > GMC_DEAD_TIME_US);
    }

    // V2.5.31: the count / guard / reject decision lives in the pure, host-tested
    // gmc_classify() (tube_logic.h). The opt-in retriggerable guard (isr_guard_us,
    // 0 = OFF) keys on `edt` (gap since the PREVIOUS edge), so every edge in a
    // burst — counted or not — keeps the channel dead and the whole afterpulse /
    // re-trigger train collapses to its single leading count; the first-ever edge
    // has edt == UINT32_MAX (above) and is never blocked. See tube_logic.h for the
    // full truth table.
    switch (gmc_classify(edt, past_gate, isr_guard_us)) {
        case GMC_GUARD_REMOVED:
            // Tally ONLY the edges the guard actually cost us a count — those that
            // would have cleared the 190µs gate. Sub-190µs edges are already owned
            // by that gate (they show up in `rejected`), so this keeps
            // guard_removed the guard's TRUE marginal effect, a clean subset of
            // rejected: counts_without_guard = counts + guard_removed. (Review #1.)
            isr_guard_removed++;
            break;
        case GMC_COUNT:
            isr_gmc_counts++;
            isr_gmc_total++;            // V2.5.6: monotonic history counter
            if (last != 0) {           // inter-pulse stats only between two counted pulses
                if (dt < isr_min_us_between) isr_min_us_between = dt;
                if (dt > isr_max_us_between) isr_max_us_between = dt;
            }
            isr_last_pulse_us = now;
            counted = true;
            // V2.6.9: coincidence check — see tube.h HV_COINCIDENT_MIN/MAX_US
            // and the block comment above isr_last_hv_pulse_us. hv_last==0
            // means no HV pulse has fired yet since boot (tube disabled node
            // never reaches here at all, since the ISR isn't installed).
            if (hv_last != 0) {
                uint32_t hv_gap = clamp_u32(now - hv_last);
                if (hv_gap >= HV_COINCIDENT_MIN_US && hv_gap <= HV_COINCIDENT_MAX_US) {
                    isr_hv_coincident++;
                }
            }
            break;
        case GMC_REJECT:
            break;
    }
    portEXIT_CRITICAL_ISR(&mux_gmc);
    if (counted) {
        tube_pulse_cb_t cb = s_pulse_cb;
        if (cb) cb();
    }
}

static void IRAM_ATTR cap_full_isr(void *arg) {
    portENTER_CRITICAL_ISR(&mux_cap);
    isr_gmc_cap_full = true;
    portEXIT_CRITICAL_ISR(&mux_cap);
}

void tube_setup(bool enabled) {
    s_enabled = enabled;

    // Always configure HV_FET as a static LOW output, even when disabled.
    // The boost MOSFET gate must be held at a known potential — leaving it
    // floating risks parasitic switching from coupled noise. With the FET
    // gate forced LOW the inductor doesn't cycle and HV stays at 0 V.
    gpio_config_t out_cfg = {
        .pin_bit_mask = 1ULL << PIN_HV_FET_OUTPUT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));
    gpio_set_level(PIN_HV_FET_OUTPUT, 0);

    if (!enabled) {
        last_read_us = esp_timer_get_time();
        ESP_LOGI(TAG, "tube setup: DISABLED — HV_FET=%d held LOW, ISRs and HV gptimer skipped",
                 PIN_HV_FET_OUTPUT);
        return;
    }

    // Cap-full: RISING edge
    gpio_config_t cap_cfg = {
        .pin_bit_mask = 1ULL << PIN_HV_CAP_FULL_INPUT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&cap_cfg));

    // GMC count: FALLING edge
    gpio_config_t count_cfg = {
        .pin_bit_mask = 1ULL << PIN_GMC_COUNT_INPUT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&count_cfg));

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_HV_CAP_FULL_INPUT, cap_full_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_GMC_COUNT_INPUT,   gmc_count_isr, NULL));

    // 100 µs periodic timer for HV recharge state machine.
    gptimer_config_t tcfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 tick = 1 µs
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&tcfg, &recharge_timer));
    gptimer_event_callbacks_t cbs = { .on_alarm = recharge_tick };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(recharge_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(recharge_timer));
    gptimer_alarm_config_t acfg = {
        .alarm_count = PERIOD_DURATION_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(recharge_timer, &acfg));
    ESP_ERROR_CHECK(gptimer_start(recharge_timer));

    last_read_us = esp_timer_get_time();
    ESP_LOGI(TAG, "tube setup: GMC pin %d (neg), HV FET %d, cap %d (pos)",
             PIN_GMC_COUNT_INPUT, PIN_HV_FET_OUTPUT, PIN_HV_CAP_FULL_INPUT);
}

void tube_read(uint32_t *counts_delta, uint32_t *dt_ms,
               uint32_t *min_us, uint32_t *max_us,
               uint32_t *hv_pulses, bool *hv_error) {
    int64_t now = esp_timer_get_time();
    *dt_ms = (uint32_t)((now - last_read_us) / 1000);
    last_read_us = now;

    if (!s_enabled) {
        // Window timer (dt_ms) still advances so other sensors can rate-divide
        // against a known interval. Everything radiation-related is zeroed.
        *counts_delta = 0;
        *min_us       = 0;
        *max_us       = 0;
        *hv_pulses    = 0;
        *hv_error     = false;
        return;
    }

    portENTER_CRITICAL(&mux_gmc);
    *counts_delta = isr_gmc_counts;
    *min_us       = isr_min_us_between;
    *max_us       = isr_max_us_between;
    isr_gmc_counts     = 0;
    isr_min_us_between = UINT32_MAX;
    isr_max_us_between = 0;
    portEXIT_CRITICAL(&mux_gmc);

    portENTER_CRITICAL(&mux_hv);
    *hv_pulses = isr_hv_pulses;
    *hv_error  = isr_hv_error;
    portEXIT_CRITICAL(&mux_hv);
}

bool tube_is_enabled(void) {
    return s_enabled;
}

uint32_t tube_get_total_counts(void) {
    // Non-destructive read of the monotonic pulse total. 32-bit aligned read is
    // atomic on Xtensa, but take mux_gmc anyway for consistency with the ISR
    // writer. Callers take their own deltas (unsigned subtraction is wrap-safe).
    uint32_t v;
    portENTER_CRITICAL(&mux_gmc);
    v = isr_gmc_total;
    portEXIT_CRITICAL(&mux_gmc);
    return v;
}

void tube_set_guard_us(uint32_t guard_us) {
    // V2.5.30: set the retriggerable dead-time-guard window (effective µs from
    // config_effective_guard_us; 0 disables). Single volatile uint32_t the ISR
    // reads on the hot path; take mux_gmc for consistency with the other ISR-shared
    // writers. Called at boot AND live from config_post on /config Save — the ISR
    // re-reads the volatile each edge, so it applies WITHOUT a reboot (unlike the
    // hardware-latched PCNT filter).
    portENTER_CRITICAL(&mux_gmc);
    isr_guard_us = guard_us;
    portEXIT_CRITICAL(&mux_gmc);
}

void tube_get_diag(uint32_t *raw_edges, uint32_t *guard_removed,
                   uint32_t *hv_coincident, uint32_t hist[TUBE_DIAG_NBUCKETS]) {
    // V2.5.12: snapshot + reset under the same lock as the ISR writer,
    // mirroring tube_read()'s window-reset discipline so the diag window tiles
    // cleanly cycle-to-cycle. V2.5.30: guard_removed rides the same snapshot.
    // V2.6.9: hv_coincident too — it's written under mux_gmc (see gmc_count_isr),
    // not mux_hv, so it belongs in this critical section, not a separate one.
    portENTER_CRITICAL(&mux_gmc);
    *raw_edges     = isr_raw_edges;
    *guard_removed = isr_guard_removed;
    *hv_coincident = isr_hv_coincident;
    for (int i = 0; i < TUBE_DIAG_NBUCKETS; i++) {
        hist[i] = isr_dt_hist[i];
        isr_dt_hist[i] = 0;
    }
    isr_raw_edges     = 0;
    isr_guard_removed = 0;
    isr_hv_coincident  = 0;
    portEXIT_CRITICAL(&mux_gmc);
}
