#include "tube.h"

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

// --- Pulse counter state ---
static volatile uint32_t isr_gmc_counts     = 0;
static volatile uint64_t isr_last_pulse_us  = 0;
static volatile uint32_t isr_min_us_between = UINT32_MAX;
static volatile uint32_t isr_max_us_between = 0;
static portMUX_TYPE mux_gmc = portMUX_INITIALIZER_UNLOCKED;

// --- HV charge state ---
static volatile uint32_t isr_hv_pulses    = 0;
static volatile bool     isr_hv_error     = false;
static volatile bool     isr_gmc_cap_full = false;
static portMUX_TYPE mux_hv  = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE mux_cap = portMUX_INITIALIZER_UNLOCKED;

// Last read timestamp for dt_ms window.
static int64_t last_read_us = 0;

// Per-pulse callback (speaker/LED tick). Set via tube_set_pulse_callback.
static volatile tube_pulse_cb_t s_pulse_cb = NULL;

void tube_set_pulse_callback(tube_pulse_cb_t cb) {
    s_pulse_cb = cb;
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
    portENTER_CRITICAL_ISR(&mux_gmc);
    uint64_t last = isr_last_pulse_us;
    if (last == 0) {
        isr_gmc_counts++;
        isr_last_pulse_us = now;
        counted = true;
    } else {
        uint32_t dt = (uint32_t)(now - last);
        if (dt > GMC_DEAD_TIME_US) {
            isr_gmc_counts++;
            if (dt < isr_min_us_between) isr_min_us_between = dt;
            if (dt > isr_max_us_between) isr_max_us_between = dt;
            isr_last_pulse_us = now;
            counted = true;
        }
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

void tube_setup(void) {
    // HV output
    gpio_config_t out_cfg = {
        .pin_bit_mask = 1ULL << PIN_HV_FET_OUTPUT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));
    gpio_set_level(PIN_HV_FET_OUTPUT, 0);

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
