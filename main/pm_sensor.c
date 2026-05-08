#include "pm_sensor.h"
#include "sps30.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "pm";

// Most-recent successful sample + status — cached so HTTP handlers can read
// without going to the bus (the worker task owns the bus during a cycle, so
// concurrent reads would race). Mutex protects struct copies; updates are
// atomic from the producer's perspective.
static pm_sample_t        s_last_sample;
static bool               s_last_sample_valid = false;
static pm_sensor_status_t s_last_status;
static bool               s_last_status_valid = false;
static SemaphoreHandle_t  s_cache_mux = NULL;

static void cache_lock_init(void) {
    if (!s_cache_mux) s_cache_mux = xSemaphoreCreateMutex();
}

esp_err_t pm_sensor_init(i2c_master_bus_handle_t bus) {
    cache_lock_init();
    sps30_init(bus);   // logs internally, ESP_OK / ERR_NOT_FOUND both fine
    ESP_LOGI(TAG, "pm sensor: %s", pm_sensor_name());
    return ESP_OK;
}

bool pm_sensor_present(void) {
    return sps30_present();
}

const char *pm_sensor_name(void) {
    return sps30_present() ? "SPS30" : "none";
}

esp_err_t pm_sensor_read(pm_sample_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!sps30_present()) return ESP_FAIL;
    esp_err_t err = sps30_read(out);
    if (err == ESP_OK && s_cache_mux) {
        xSemaphoreTake(s_cache_mux, portMAX_DELAY);
        s_last_sample       = *out;
        s_last_sample_valid = true;
        xSemaphoreGive(s_cache_mux);
    }
    return err;
}

esp_err_t pm_sensor_read_status(pm_sensor_status_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!sps30_present()) return ESP_FAIL;
    uint32_t raw = 0;
    esp_err_t err = sps30_read_device_status(&raw);
    if (err != ESP_OK) return err;
    out->raw            = raw;
    out->fan_fail       = (raw & (1u << 4))  != 0;
    out->laser_fail     = (raw & (1u << 5))  != 0;
    out->fan_speed_warn = (raw & (1u << 21)) != 0;
    if (s_cache_mux) {
        xSemaphoreTake(s_cache_mux, portMAX_DELAY);
        s_last_status       = *out;
        s_last_status_valid = true;
        xSemaphoreGive(s_cache_mux);
    }
    return ESP_OK;
}

esp_err_t pm_sensor_get_last_sample(pm_sample_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!s_cache_mux || !s_last_sample_valid) return ESP_FAIL;
    xSemaphoreTake(s_cache_mux, portMAX_DELAY);
    bool valid = s_last_sample_valid;
    if (valid) *out = s_last_sample;
    xSemaphoreGive(s_cache_mux);
    return valid ? ESP_OK : ESP_FAIL;
}

esp_err_t pm_sensor_get_last_status(pm_sensor_status_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!s_cache_mux || !s_last_status_valid) return ESP_FAIL;
    xSemaphoreTake(s_cache_mux, portMAX_DELAY);
    bool valid = s_last_status_valid;
    if (valid) *out = s_last_status;
    xSemaphoreGive(s_cache_mux);
    return valid ? ESP_OK : ESP_FAIL;
}
