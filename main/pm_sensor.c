#include "pm_sensor.h"
#include "sps30.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "telemetry.h"

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

// V2.6.19: telemetry freshness gate for the CSV read callbacks below.
// s_last_sample_valid (above) latches true forever after the FIRST
// successful read and is never cleared on a later failure — it answers
// "has a sample ever arrived", which is right for pm_sensor_get_last_sample()'s
// existing HTTP-handler contract but wrong for the CSV: a sensor that drops
// off the bus mid-session would otherwise have the last-good sample logged
// on every subsequent row forever. s_tm_fresh instead reflects only THIS
// cycle's pm_sensor_read() outcome, set on both the success and every
// failure path.
static bool s_tm_fresh = false;

static void cache_lock_init(void) {
    if (!s_cache_mux) s_cache_mux = xSemaphoreCreateMutex();
}

static bool tm_read_pm1_0(char *cell, size_t cap, void *arg) {
    (void)arg;
    pm_sample_t sample;
    if (!s_tm_fresh || pm_sensor_get_last_sample(&sample) != ESP_OK) return false;
    snprintf(cell, cap, "%.1f", (double)sample.pm1_0);
    return true;
}

static bool tm_read_pm2_5(char *cell, size_t cap, void *arg) {
    (void)arg;
    pm_sample_t sample;
    if (!s_tm_fresh || pm_sensor_get_last_sample(&sample) != ESP_OK) return false;
    snprintf(cell, cap, "%.1f", (double)sample.pm2_5);
    return true;
}

static bool tm_read_pm4_0(char *cell, size_t cap, void *arg) {
    (void)arg;
    pm_sample_t sample;
    if (!s_tm_fresh || pm_sensor_get_last_sample(&sample) != ESP_OK) return false;
    snprintf(cell, cap, "%.1f", (double)sample.pm4_0);
    return true;
}

static bool tm_read_pm10(char *cell, size_t cap, void *arg) {
    (void)arg;
    pm_sample_t sample;
    if (!s_tm_fresh || pm_sensor_get_last_sample(&sample) != ESP_OK) return false;
    snprintf(cell, cap, "%.1f", (double)sample.pm10);
    return true;
}

esp_err_t pm_sensor_init(i2c_master_bus_handle_t bus) {
    cache_lock_init();
    sps30_init(bus);   // logs internally, ESP_OK / ERR_NOT_FOUND both fine
    ESP_LOGI(TAG, "pm sensor: %s", pm_sensor_name());

    // V2.6.19: register our CSV columns once, only when a PM sensor actually
    // ACKed. env_sensor's dual-bus probe can call pm_sensor_init() a second
    // time; the s_tm_registered guard (mirrors sht45.c) is what actually
    // prevents a double registration in that case.
    if (pm_sensor_present()) {
        static bool s_tm_registered = false;
        if (!s_tm_registered) {
            s_tm_registered = true;
            telemetry_register("SPS30 PM1.0 [ug/m3]", tm_read_pm1_0, NULL);
            telemetry_register("SPS30 PM2.5 [ug/m3]", tm_read_pm2_5, NULL);
            telemetry_register("SPS30 PM4.0 [ug/m3]", tm_read_pm4_0, NULL);
            telemetry_register("SPS30 PM10 [ug/m3]",  tm_read_pm10,  NULL);
        }
    }
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
    if (!sps30_present()) {
        s_tm_fresh = false;
        return ESP_FAIL;
    }
    esp_err_t err = sps30_read(out);
    if (err == ESP_OK && s_cache_mux) {
        xSemaphoreTake(s_cache_mux, portMAX_DELAY);
        s_last_sample       = *out;
        s_last_sample_valid = true;
        xSemaphoreGive(s_cache_mux);
    }
    s_tm_fresh = (err == ESP_OK);
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
