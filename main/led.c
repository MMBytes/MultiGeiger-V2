// V2.5.19 — plain onboard user-LED pulse-tick driver. See led.h for the
// design rationale and the board-gating condition.

#include "led.h"
#include "hal.h"

// Active only on a board with a plain user LED that neither speaker.c
// (HAL_HAS_SPEAKER) nor neopixel.c (HAL_HAS_NEOPIXEL) already drives. The
// XIAO ESP32-S3 is the sole such board today.
#if defined(PIN_LED_BUILTIN) && !HAL_HAS_SPEAKER && !HAL_HAS_NEOPIXEL

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tube.h"   // tube_set_pulse_callback

static const char *TAG = "led";

// Polarity: active-low boards (XIAO ESP32-S3, GPIO21) light the LED with a LOW
// level. Default to active-high if a board ever defines PIN_LED_BUILTIN here
// without declaring the flag.
#ifndef HAL_LED_ACTIVE_LOW
#define HAL_LED_ACTIVE_LOW 0
#endif
#if HAL_LED_ACTIVE_LOW
#define LED_ON_LEVEL   0
#define LED_OFF_LEVEL  1
#else
#define LED_ON_LEVEL   1
#define LED_OFF_LEVEL  0
#endif

// Match neopixel.c's visible-flash duration so the two boards' pulse feedback
// looks the same to the eye.
#define PULSE_VISIBLE_MS  40

static TaskHandle_t s_pulse_task = NULL;

// Flash worker — blocks on an ISR notification, then drives one brief blink.
// ulTaskNotifyTake(pdTRUE) clears the count on read so a burst of coincident
// pulses collapses to a single visible flash (the eye can't resolve them).
static void pulse_task(void *arg) {
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        gpio_set_level(PIN_LED_BUILTIN, LED_ON_LEVEL);
        vTaskDelay(pdMS_TO_TICKS(PULSE_VISIBLE_MS));
        gpio_set_level(PIN_LED_BUILTIN, LED_OFF_LEVEL);
    }
}

// IRAM-resident GM-pulse handler — keep it minimal, just notify the worker.
static void IRAM_ATTR on_tube_pulse(void) {
    if (!s_pulse_task) return;
    BaseType_t hp_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_pulse_task, &hp_task_woken);
    portYIELD_FROM_ISR(hp_task_woken);
}

esp_err_t led_init(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_LED_BUILTIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(GPIO%d): %s", PIN_LED_BUILTIN, esp_err_to_name(err));
        return err;
    }
    gpio_set_level(PIN_LED_BUILTIN, LED_OFF_LEVEL);   // start dark
    ESP_LOGI(TAG, "user LED ready: GPIO%d (active-%s)",
             PIN_LED_BUILTIN, HAL_LED_ACTIVE_LOW ? "low" : "high");
    return ESP_OK;
}

void led_register_pulse_tick(void) {
    if (s_pulse_task) return;   // idempotent
    BaseType_t ok = xTaskCreate(pulse_task, "led", 2048, NULL,
                                tskIDLE_PRIORITY + 2, &s_pulse_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "pulse worker task creation failed");
        return;
    }
    tube_set_pulse_callback(on_tube_pulse);
    ESP_LOGI(TAG, "tube-pulse hook registered (LED flash, %d ms)", PULSE_VISIBLE_MS);
}

#else   // no led.c-owned user LED on this board → no-op stubs

esp_err_t led_init(void)              { return ESP_OK; }
void      led_register_pulse_tick(void) { /* no-op */ }

#endif
