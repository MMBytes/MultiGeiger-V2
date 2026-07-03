// V2.6.6 — MAX17048 battery fuel-gauge driver. See fuel_gauge.h for design.

#include "fuel_gauge.h"

#include "hal.h"

#if HAL_HAS_FUEL_GAUGE

#include "esp_log.h"

static const char *TAG = "fuel_gauge";

#define MAX17048_ADDR   0x36

// Register addresses (per Maxim/Analog Devices MAX17048/49 datasheet).
#define REG_VCELL       0x02   // RO: 16-bit cell voltage, 78.125 uV/LSB
#define REG_SOC         0x04   // RO: 16-bit state of charge, 1/256 %/LSB
#define REG_CRATE       0x16   // RO: 16-bit SIGNED charge rate, 0.208 %/hr/LSB

// Presence hysteresis (mV). See fuel_gauge.h file header for the full
// derivation — this dead zone sits between the empirical ~0V no-battery
// reading and the ~2.4V floor a real cell can reach before its protection
// IC disconnects it.
#define BATT_PRESENT_MV  2000
#define BATT_ABSENT_MV   1500

static i2c_master_dev_handle_t s_dev          = NULL;
static bool                    s_ready        = false;
static bool                    s_batt_present = false;

// MAX17048 registers are BIG-endian (MSB first) — the opposite byte order
// from VEML7700's little-endian registers (see veml7700.c). Get this
// backwards and every reading is off by a factor of 256.
static esp_err_t reg_read16(uint8_t reg, uint16_t *out) {
    uint8_t in[2];
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, in, sizeof(in), 100);
    if (err != ESP_OK) return err;
    *out = ((uint16_t)in[0] << 8) | (uint16_t)in[1];
    return ESP_OK;
}

esp_err_t fuel_gauge_init(i2c_master_bus_handle_t bus) {
    if (s_ready) return ESP_OK;
    if (!bus) return ESP_ERR_INVALID_ARG;

    if (i2c_master_probe(bus, MAX17048_ADDR, 50) != ESP_OK) {
        ESP_LOGW(TAG, "MAX17048 not found at 0x%02X", MAX17048_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MAX17048_ADDR,
        .scl_speed_hz    = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &devcfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_bus_add_device: %s", esp_err_to_name(err));
        s_dev = NULL;
        return err;
    }

    // No init register writes — the chip free-runs once powered, and it's
    // on the always-on 3.3V rail (independent of the battery), so it ACKs
    // here regardless of whether a LiPo is attached.
    s_ready = true;
    ESP_LOGI(TAG, "MAX17048 ready at 0x%02X", MAX17048_ADDR);
    return ESP_OK;
}

bool fuel_gauge_present(void) {
    if (!s_ready) return false;

    uint16_t raw = 0;
    if (reg_read16(REG_VCELL, &raw) != ESP_OK) return false;

    // raw * 78.125 uV/LSB, converted to mV. Widened to uint64_t for the
    // multiply — raw (up to 65535) * 78125 overflows uint32_t.
    uint32_t mv = (uint32_t)(((uint64_t)raw * 78125u) / 1000000u);

    if (mv > BATT_PRESENT_MV)     s_batt_present = true;
    else if (mv < BATT_ABSENT_MV) s_batt_present = false;
    // else: inside the hysteresis band — hold the previous state.

    return s_batt_present;
}

esp_err_t fuel_gauge_read(float *volts, float *soc_pct, float *rate_pct_per_hr) {
    if (!s_ready) return ESP_FAIL;

    if (volts) {
        uint16_t raw = 0;
        esp_err_t err = reg_read16(REG_VCELL, &raw);
        if (err != ESP_OK) return err;
        *volts = (float)raw * 0.000078125f;   // 78.125 uV/LSB
    }

    if (soc_pct) {
        uint16_t raw = 0;
        esp_err_t err = reg_read16(REG_SOC, &raw);
        if (err != ESP_OK) return err;
        *soc_pct = (float)raw / 256.0f;        // 1/256 %/LSB
    }

    if (rate_pct_per_hr) {
        uint16_t raw = 0;
        esp_err_t err = reg_read16(REG_CRATE, &raw);
        if (err != ESP_OK) return err;
        // CRATE is signed — reinterpret the raw bit pattern before scaling.
        *rate_pct_per_hr = (float)(int16_t)raw * 0.208f;  // 0.208 %/hr/LSB
    }

    return ESP_OK;
}

#else   // HAL_HAS_FUEL_GAUGE == 0 → no-op stubs

esp_err_t fuel_gauge_init(i2c_master_bus_handle_t bus) { (void)bus; return ESP_OK; }
bool      fuel_gauge_present(void)                     { return false; }
esp_err_t fuel_gauge_read(float *volts, float *soc_pct, float *rate_pct_per_hr) {
    (void)volts; (void)soc_pct; (void)rate_pct_per_hr;
    return ESP_FAIL;
}

#endif  // HAL_HAS_FUEL_GAUGE
