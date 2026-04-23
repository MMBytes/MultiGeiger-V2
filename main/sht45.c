#include "sht45.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sht45";

#define SHT45_ADDR          0x44

// Commands (datasheet table 8)
#define CMD_MEASURE_HIGH    0xFD   // high-precision T+RH, ~8.2 ms
#define CMD_HEATER_200MW    0x39   // 200 mW for 0.1 s, then measure
#define CMD_SOFT_RESET      0x94

// Heater interval: only activate when humidity > this and 10 min have elapsed.
#define HEATER_HUMIDITY_THR  80.0f
#define HEATER_INTERVAL_MS   (10 * 60 * 1000UL)

static i2c_master_dev_handle_t s_dev  = NULL;
static bool                    s_ready = false;
static uint32_t                s_last_heat_ms = 0;

// CRC-8: polynomial 0x31, init 0xFF (Sensirion standard).
static bool crc_ok(uint8_t a, uint8_t b, uint8_t crc) {
    uint8_t v = 0xFF;
    v ^= a;
    for (int i = 0; i < 8; i++) v = (v & 0x80) ? (v << 1) ^ 0x31 : v << 1;
    v ^= b;
    for (int i = 0; i < 8; i++) v = (v & 0x80) ? (v << 1) ^ 0x31 : v << 1;
    return v == crc;
}

static esp_err_t send_cmd(uint8_t cmd) {
    return i2c_master_transmit(s_dev, &cmd, 1, 50);
}

esp_err_t sht45_init(i2c_master_bus_handle_t bus) {
    if (s_ready) return ESP_OK;

    // Quick probe before adding the device — avoids a lingering handle if
    // the sensor is absent.
    if (i2c_master_probe(bus, SHT45_ADDR, 50) != ESP_OK) {
        ESP_LOGW(TAG, "SHT45 not found at 0x%02X", SHT45_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SHT45_ADDR,
        .scl_speed_hz    = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &devcfg, &s_dev);
    if (err != ESP_OK) return err;

    // Soft-reset clears any incomplete measurement state. The datasheet
    // requires a 1 ms guard before the first command after reset.
    send_cmd(CMD_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(2));

    // Verify the sensor responds to a measurement command before declaring
    // it ready — guards against a stray ACK from another device at 0x44.
    float t, h;
    err = sht45_read(&t, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SHT45 probe read failed");
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_FAIL;
    }

    s_ready = true;
    ESP_LOGI(TAG, "SHT45 ready at 0x%02X (high-precision mode)", SHT45_ADDR);
    return ESP_OK;
}

bool sht45_present(void) {
    return s_ready;
}

esp_err_t sht45_read(float *temperature_c, float *humidity_pct) {
    if (!s_dev) return ESP_FAIL;

    esp_err_t err = send_cmd(CMD_MEASURE_HIGH);
    if (err != ESP_OK) return err;

    // High-precision conversion: 8.2 ms typical, 9.4 ms max. 10 ms is safe.
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t buf[6];
    err = i2c_master_receive(s_dev, buf, sizeof(buf), 50);
    if (err != ESP_OK) return err;

    if (!crc_ok(buf[0], buf[1], buf[2]) || !crc_ok(buf[3], buf[4], buf[5])) {
        ESP_LOGW(TAG, "SHT45 CRC mismatch");
        return ESP_FAIL;
    }

    uint16_t raw_t = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t raw_h = ((uint16_t)buf[3] << 8) | buf[4];

    // Sensirion datasheet section 4.6 transfer functions.
    float t = -45.0f + 175.0f * ((float)raw_t / 65535.0f);
    float h = -6.0f  + 125.0f * ((float)raw_h / 65535.0f);
    if (h < 0.0f)   h = 0.0f;
    if (h > 100.0f) h = 100.0f;

    if (temperature_c)  *temperature_c  = t;
    if (humidity_pct)   *humidity_pct   = h;
    return ESP_OK;
}

void sht45_heat_periodic(uint32_t now_ms, float humidity_pct) {
    if (!s_ready) return;
    if (humidity_pct < HEATER_HUMIDITY_THR) return;
    if (s_last_heat_ms != 0 &&
        (uint32_t)(now_ms - s_last_heat_ms) < HEATER_INTERVAL_MS) return;

    // CMD_HEATER_200MW triggers a 200 mW / 100 ms heat pulse and then takes
    // a measurement. The result is discarded — we just want the heat effect.
    // The next normal sht45_read() call will get fresh post-heat readings.
    ESP_LOGI(TAG, "SHT45 heater activated (RH=%.1f%%)", humidity_pct);
    send_cmd(CMD_HEATER_200MW);
    vTaskDelay(pdMS_TO_TICKS(120));   // 100 ms pulse + 20 ms settle
    uint8_t discard[6];
    i2c_master_receive(s_dev, discard, sizeof(discard), 50);
    s_last_heat_ms = now_ms;
}
