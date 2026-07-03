// V2.6.6 — MAX17048 battery fuel-gauge driver. See fuel_gauge.h for design.

#include "fuel_gauge.h"

#include "hal.h"

#if HAL_HAS_FUEL_GAUGE

#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "fuel_gauge";

#define MAX17048_ADDR   0x36

// Register addresses (per Maxim/Analog Devices MAX17048/49 datasheet).
#define REG_VCELL       0x02   // RO: 16-bit cell voltage, 78.125 uV/LSB
#define REG_SOC         0x04   // RO: 16-bit state of charge, 1/256 %/LSB
#define REG_VERSION     0x08   // RO: 16-bit chip/firmware version
#define REG_HIBRT       0x0A   // R/W: 16-bit HibThr(hi)/ActThr(lo) hibernate thresholds
#define REG_CONFIG      0x0C   // R/W: 16-bit RCOMP/sleep/alert config
#define REG_VALERT      0x14   // R/W: 16-bit VALRT min(hi)/max(lo) alert thresholds
#define REG_CRATE       0x16   // RO: 16-bit SIGNED charge rate, 0.208 %/hr/LSB
#define REG_VRESET      0x18   // R/W: 8-bit reset-voltage threshold + comparator-disable
#define REG_CHIPID      0x19   // RO: 8-bit chip ID
#define REG_STATUS      0x1A   // R/W: 8-bit alert flags (RI/VH/VL/VR/HD/SC)

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

static esp_err_t reg_read8(uint8_t reg, uint8_t *out) {
    uint8_t in = 0;
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, &in, 1, 100);
    if (err != ESP_OK) return err;
    *out = in;
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

    // Diagnostic snapshot at startup — VERSION is the register Adafruit's
    // own MAX1704x libraries use as their sole "battery attached" sentinel
    // (0xFFFF if no response); logging it here lets us check whether that
    // pattern ever shows up on our always-on-rail wiring. HIBRT/CONFIG/
    // VALERT/VRESET are pure config registers we never write, so they stay
    // at their power-on-reset defaults for the whole session — logged once
    // here rather than per-TX. 0xFFFF/0xFF sentinels below mean "read
    // failed", not "chip reported this value".
    uint16_t version = 0xFFFF;
    uint16_t hibrt   = 0xFFFF;
    uint16_t config  = 0xFFFF;
    uint16_t valert  = 0xFFFF;
    uint8_t  vreset  = 0xFF;
    uint8_t  chipid  = 0xFF;
    uint8_t  status  = 0xFF;
    reg_read16(REG_VERSION, &version);
    reg_read16(REG_HIBRT, &hibrt);
    reg_read16(REG_CONFIG, &config);
    reg_read16(REG_VALERT, &valert);
    reg_read8(REG_VRESET, &vreset);
    reg_read8(REG_CHIPID, &chipid);
    reg_read8(REG_STATUS, &status);
    ESP_LOGI(TAG, "MAX17048 ready at 0x%02X (version=0x%04X hibrt=0x%04X config=0x%04X "
                  "valert=0x%04X vreset=0x%02X chip_id=0x%02X status=0x%02X)",
             MAX17048_ADDR, version, hibrt, config, valert, vreset, chipid, status);

    // VBUS-present detect — plain digital input, driven by dedicated board
    // circuitry (not a strap, no pull needed).
    gpio_set_direction(PIN_VBUS_DETECT, GPIO_MODE_INPUT);

    return ESP_OK;
}

bool fuel_gauge_ready(void) {
    return s_ready;
}

bool fuel_gauge_vbus_present(void) {
    if (!s_ready) return false;
    return gpio_get_level(PIN_VBUS_DETECT) != 0;
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

esp_err_t fuel_gauge_read_diag(uint16_t *version, uint8_t *status) {
    if (!s_ready) return ESP_FAIL;

    if (version) {
        esp_err_t err = reg_read16(REG_VERSION, version);
        if (err != ESP_OK) return err;
    }

    if (status) {
        esp_err_t err = reg_read8(REG_STATUS, status);
        if (err != ESP_OK) return err;
    }

    return ESP_OK;
}

#else   // HAL_HAS_FUEL_GAUGE == 0 → no-op stubs

esp_err_t fuel_gauge_init(i2c_master_bus_handle_t bus) { (void)bus; return ESP_OK; }
bool      fuel_gauge_ready(void)                       { return false; }
bool      fuel_gauge_vbus_present(void)                { return false; }
bool      fuel_gauge_present(void)                     { return false; }
esp_err_t fuel_gauge_read(float *volts, float *soc_pct, float *rate_pct_per_hr) {
    (void)volts; (void)soc_pct; (void)rate_pct_per_hr;
    return ESP_FAIL;
}
esp_err_t fuel_gauge_read_diag(uint16_t *version, uint8_t *status) {
    (void)version; (void)status;
    return ESP_FAIL;
}

#endif  // HAL_HAS_FUEL_GAUGE
