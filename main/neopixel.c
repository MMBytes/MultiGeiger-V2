#include "neopixel.h"

#include "hal.h"

#if HAL_HAS_NEOPIXEL

#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tube.h"   // tube_set_pulse_callback

static const char *TAG = "neopixel";

// ---------------------------------------------------------------------------
// WS2812B timing — encoded as RMT symbols at 10 MHz resolution (100 ns/tick).
// Datasheet (Worldsemi WS2812B):
//   T0H = 0.4 µs ± 0.15  →  4 ticks
//   T0L = 0.85 µs ± 0.15 →  8 ticks
//   T1H = 0.8 µs ± 0.15  →  8 ticks
//   T1L = 0.45 µs ± 0.15 →  5 ticks
//   Reset = >50 µs LOW   →  end-of-frame gap from RMT idle
// 10 MHz resolution sits comfortably inside the ±150 ns tolerance band.
// ---------------------------------------------------------------------------
#define RMT_RESOLUTION_HZ   10000000   // 10 MHz → 100 ns per tick

static const rmt_symbol_word_t s_bit0 = {
    .level0 = 1, .duration0 = 4,    // 0.4 us HIGH
    .level1 = 0, .duration1 = 8,    // 0.8 us LOW
};
static const rmt_symbol_word_t s_bit1 = {
    .level0 = 1, .duration0 = 8,    // 0.8 us HIGH
    .level1 = 0, .duration1 = 5,    // 0.5 us LOW
};
// No explicit reset symbol needed — duration field is only 15 bits (max
// 32767 ticks = 3.2 ms at our 10 MHz resolution). The natural inter-frame
// gap between rmt_transmit() calls is multi-ms (we wait at least 40 ms
// between pulses), which is far more than the WS2812 datasheet's >50 us
// reset requirement. RMT idle level defaults to LOW between transmissions.

static rmt_channel_handle_t s_chan      = NULL;
static rmt_encoder_handle_t s_encoder   = NULL;
static TaskHandle_t          s_pulse_task = NULL;

// ---------------------------------------------------------------------------
// Synchronous one-pixel write. WS2812 expects byte order GRB (not RGB!).
// ---------------------------------------------------------------------------
void neopixel_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    if (!s_chan || !s_encoder) return;
    uint8_t grb[3] = { g, r, b };   // WS2812 wire order
    rmt_transmit_config_t cfg = { .loop_count = 0 };
    rmt_transmit(s_chan, s_encoder, grb, sizeof(grb), &cfg);
    rmt_tx_wait_all_done(s_chan, 100);  // ~30 µs in practice; 100 ms is paranoid
}

// ---------------------------------------------------------------------------
// Pulse worker — blocks on task notification, drives a red flash on each.
// Brightness intentionally low (R=20, G=B=0) to avoid being annoyingly bright
// in dark rooms while still being visible at arm's length.
// ---------------------------------------------------------------------------
#define PULSE_RED_BRIGHTNESS  20
#define PULSE_VISIBLE_MS      40

static void pulse_task(void *arg) {
    (void)arg;
    for (;;) {
        // Wait indefinitely for an ISR notification. ulTaskNotifyTake
        // resets the count to 0 on read so multiple coincident pulses
        // collapse to one visible flash — fine, the human eye couldn't
        // distinguish them anyway.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        neopixel_set_rgb(PULSE_RED_BRIGHTNESS, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(PULSE_VISIBLE_MS));
        neopixel_set_rgb(0, 0, 0);
    }
}

// IRAM-resident ISR handler — keep it minimal. Just notifies the worker.
static void IRAM_ATTR on_tube_pulse(void) {
    if (!s_pulse_task) return;
    BaseType_t hp_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_pulse_task, &hp_task_woken);
    portYIELD_FROM_ISR(hp_task_woken);
}

// ---------------------------------------------------------------------------
// Init: power gate HIGH, RMT TX channel, bytes encoder, worker task.
// ---------------------------------------------------------------------------
esp_err_t neopixel_init(void) {
    esp_err_t err;

#ifdef PIN_NEOPIXEL_POWER
    // Power gate — drive HIGH to enable the NeoPixel's 3V3 rail. Only some
    // boards need this (e.g. QT Py's switched rail); others power the LED
    // from an always-on rail with no dedicated gate GPIO — same "intentionally
    // undefined" idiom used for PIN_OLED_RESET.
    gpio_config_t pwr_cfg = {
        .pin_bit_mask = 1ULL << PIN_NEOPIXEL_POWER,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&pwr_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "power-gate gpio_config: %s", esp_err_to_name(err));
        return err;
    }
    gpio_set_level(PIN_NEOPIXEL_POWER, 1);
#endif

    // RMT TX channel on the data pin.
    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num          = PIN_NEOPIXEL_DATA,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,    // single pixel = 24 bits + reset; 64 is plenty
        .trans_queue_depth = 4,
    };
    err = rmt_new_tx_channel(&chan_cfg, &s_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel: %s", esp_err_to_name(err));
        return err;
    }

    // Bytes encoder turns each input byte (8 bits, MSB-first) into 8 RMT
    // symbols using the bit0/bit1 templates above. Trailing reset symbol
    // gives the WS2812 a ≥50 µs LOW to latch the new colour.
    rmt_bytes_encoder_config_t enc_cfg = {
        .bit0      = s_bit0,
        .bit1      = s_bit1,
        .flags.msb_first = 1,
    };
    err = rmt_new_bytes_encoder(&enc_cfg, &s_encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_bytes_encoder: %s", esp_err_to_name(err));
        return err;
    }

    err = rmt_enable(s_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable: %s", esp_err_to_name(err));
        return err;
    }

    // Pixel starts black so the boot sequence doesn't leave a stale colour
    // from any previous firmware run.
    neopixel_set_rgb(0, 0, 0);

#ifdef PIN_NEOPIXEL_POWER
    ESP_LOGI(TAG, "NeoPixel ready: data=GPIO%d power=GPIO%d (1 px)",
             PIN_NEOPIXEL_DATA, PIN_NEOPIXEL_POWER);
#else
    ESP_LOGI(TAG, "NeoPixel ready: data=GPIO%d, no power gate (always-on rail) (1 px)",
             PIN_NEOPIXEL_DATA);
#endif
    return ESP_OK;
}

void neopixel_register_pulse_tick(void) {
    if (s_pulse_task) return;   // idempotent
    BaseType_t ok = xTaskCreate(pulse_task, "neopixel", 2048, NULL,
                                tskIDLE_PRIORITY + 2, &s_pulse_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "pulse worker task creation failed");
        return;
    }
#if HAL_HAS_SPEAKER
    // speaker.c already owns the single tube-pulse callback slot on boards
    // with both a piezo and a NeoPixel (it needs the callback for its own
    // audio tick) — it calls neopixel_notify_pulse() directly instead of
    // this module re-registering (which would silently steal the slot back
    // and break the speaker's audio tick, per the design-spec gap found
    // while porting sparkfun_thing_plus_esp32s3). Just the worker task above
    // is needed here; the callback itself stays with speaker.c.
    ESP_LOGI(TAG, "pulse worker ready (red flash, %d ms) — driven via speaker.c", PULSE_VISIBLE_MS);
#else
    tube_set_pulse_callback(on_tube_pulse);
    ESP_LOGI(TAG, "tube-pulse hook registered (red flash, %d ms)", PULSE_VISIBLE_MS);
#endif
}

void IRAM_ATTR neopixel_notify_pulse(void) {
    on_tube_pulse();
}

#else   // HAL_HAS_NEOPIXEL == 0 → no-op stubs

esp_err_t neopixel_init(void)                  { return ESP_OK; }
void      neopixel_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    (void)r; (void)g; (void)b;
}
void      neopixel_register_pulse_tick(void)   { /* no-op */ }
void      neopixel_notify_pulse(void)          { /* no-op */ }

#endif  // HAL_HAS_NEOPIXEL
