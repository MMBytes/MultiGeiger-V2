#include "noise_sensor.h"
#include "dnms.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "noise";

// Most-recent successful sample — cached so HTTP handlers can read without
// going to the bus (the worker task owns the bus during a cycle).
static noise_sample_t    s_last_sample;
static bool              s_last_sample_valid = false;
static SemaphoreHandle_t s_cache_mux = NULL;

// Poll budget for noise_sensor_read's data-ready wait. The DNMS Teensy
// finalises the LAeq on CALCULATE_LEQ and makes it available within ~30 ms
// in normal operation, but a brand-new integration window after init may
// take longer — Teensy needs to fill at least one block of audio samples
// from the I²S microphone (~10 ms at 44.1 kHz × 256 samples). 12 s is the
// canonical airrohr/dusty timeout; we use the same.
#define READY_POLL_INTERVAL_MS  30
#define READY_POLL_MAX_TRIES    400   // 30 ms × 400 = 12 000 ms

static void cache_lock_init(void) {
    if (!s_cache_mux) s_cache_mux = xSemaphoreCreateMutex();
}

esp_err_t noise_sensor_init(i2c_master_bus_handle_t bus) {
    cache_lock_init();
    dnms_init(bus);   // logs internally, ESP_OK / ERR_NOT_FOUND / FAIL all fine

    // Trigger the first integration window if the sensor is alive — first
    // noise_sensor_read after the next TX cycle will return its result. If
    // we don't trigger here, the first read would have nothing to report
    // and would either block for the full 12 s timeout or read stale state.
    if (dnms_present()) {
        if (dnms_trigger() != ESP_OK) {
            ESP_LOGW(TAG, "first DNMS trigger failed — first cycle may have no data");
        }
    }
    ESP_LOGI(TAG, "noise sensor: %s", noise_sensor_name());
    return ESP_OK;
}

bool noise_sensor_present(void) {
    return dnms_present();
}

const char *noise_sensor_name(void) {
    return dnms_present() ? "DNMS" : "none";
}

const char *noise_sensor_version(void) {
    return dnms_present() ? dnms_get_version() : "none";
}

esp_err_t noise_sensor_trigger(void) {
    if (!dnms_present()) return ESP_FAIL;
    return dnms_trigger();
}

esp_err_t noise_sensor_read(noise_sample_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!dnms_present()) return ESP_FAIL;

    // Poll for data-ready before reading. In steady-state operation (cycles
    // 2..N) the result is normally available on the first or second poll
    // because we triggered CALCULATE_LEQ at the end of the previous cycle.
    // First cycle after boot may need most of the 12 s budget.
    bool ready = false;
    int  tries = 0;
    while (tries < READY_POLL_MAX_TRIES) {
        esp_err_t err = dnms_data_ready(&ready);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "DNMS data_ready poll failed: %s", esp_err_to_name(err));
            return err;
        }
        if (ready) break;
        vTaskDelay(pdMS_TO_TICKS(READY_POLL_INTERVAL_MS));
        tries++;
    }
    if (!ready) {
        ESP_LOGW(TAG, "DNMS data not ready after %d ms",
                 READY_POLL_INTERVAL_MS * READY_POLL_MAX_TRIES);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = dnms_read_leq(out);
    if (err == ESP_OK && s_cache_mux) {
        xSemaphoreTake(s_cache_mux, portMAX_DELAY);
        s_last_sample       = *out;
        s_last_sample_valid = true;
        xSemaphoreGive(s_cache_mux);
    }
    return err;
}

esp_err_t noise_sensor_get_last_sample(noise_sample_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!s_cache_mux || !s_last_sample_valid) return ESP_FAIL;
    xSemaphoreTake(s_cache_mux, portMAX_DELAY);
    bool valid = s_last_sample_valid;
    if (valid) *out = s_last_sample;
    xSemaphoreGive(s_cache_mux);
    return valid ? ESP_OK : ESP_FAIL;
}
