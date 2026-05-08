#include "pm_sensor.h"
#include "sps30.h"

#include "esp_log.h"

static const char *TAG = "pm";

esp_err_t pm_sensor_init(i2c_master_bus_handle_t bus) {
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
    if (sps30_present()) {
        return sps30_read(out);
    }
    return ESP_FAIL;
}
