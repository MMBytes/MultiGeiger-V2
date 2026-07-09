// V2.6.6 — MAX17048 battery fuel-gauge driver. See fuel_gauge.h for design.

#include "fuel_gauge.h"

#include "hal.h"

#if HAL_HAS_FUEL_GAUGE

#include "esp_log.h"
#include "driver/gpio.h"
#include "i2c_bus.h"

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

static i2c_master_dev_handle_t s_dev            = NULL;
static volatile bool           s_ready          = false;  // written by init task, read by TX/HTTP/MQTT tasks
static volatile bool           s_user_present   = false;  // mirrors cfg->batt_present; written by init+HTTP tasks

// MAX17048 registers are BIG-endian (MSB first) — the opposite byte order
// from VEML7700's little-endian registers (see veml7700.c). Get this
// backwards and every reading is off by a factor of 256. i2c_dev_read_u16_be()
// (i2c_bus.h) makes that byte-order choice explicit at every call site.

esp_err_t fuel_gauge_init(i2c_master_bus_handle_t bus) {
    if (s_ready) return ESP_OK;
    if (!bus) return ESP_ERR_INVALID_ARG;

    esp_err_t err = i2c_probe_and_add(bus, MAX17048_ADDR, 400000, 50, &s_dev);
    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "MAX17048 not found at 0x%02X", MAX17048_ADDR);
        return err;
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_bus_add_device: %s", esp_err_to_name(err));
        return err;
    }

    // No init register writes — the chip free-runs once powered, and it's
    // on the always-on 3.3V rail (independent of the battery), so it ACKs
    // here regardless of whether a LiPo is attached.

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
    i2c_dev_read_u16_be(s_dev, REG_VERSION, &version);
    i2c_dev_read_u16_be(s_dev, REG_HIBRT, &hibrt);
    i2c_dev_read_u16_be(s_dev, REG_CONFIG, &config);
    i2c_dev_read_u16_be(s_dev, REG_VALERT, &valert);
    i2c_dev_read_regs(s_dev, REG_VRESET, &vreset, 1);
    i2c_dev_read_regs(s_dev, REG_CHIPID, &chipid, 1);
    i2c_dev_read_regs(s_dev, REG_STATUS, &status, 1);
    ESP_LOGI(TAG, "MAX17048 ready at 0x%02X (version=0x%04X hibrt=0x%04X config=0x%04X "
                  "valert=0x%04X vreset=0x%02X chip_id=0x%02X status=0x%02X)",
             MAX17048_ADDR, version, hibrt, config, valert, vreset, chipid, status);

#ifdef PIN_VBUS_DETECT
    // VBUS-present detect — plain digital input, driven by dedicated board
    // circuitry (not a strap, no pull needed). Only some boards wire a
    // dedicated VBUS-sense GPIO (e.g. FeatherS3-D); others have no such pin
    // (e.g. the WRL-24408's MCP73831 STAT output drives only an onboard LED,
    // not a GPIO) — same "intentionally undefined optional pin" idiom used
    // for PIN_OLED_RESET and PIN_NEOPIXEL_POWER.
    gpio_set_direction(PIN_VBUS_DETECT, GPIO_MODE_INPUT);
#endif

    // Set only after the GPIO is actually configured, so no reader can ever
    // observe s_ready==true with PIN_VBUS_DETECT still in its power-on state.
    s_ready = true;

    return ESP_OK;
}

bool fuel_gauge_vbus_present(void) {
#ifdef PIN_VBUS_DETECT
    if (!s_ready) return false;
    return gpio_get_level(PIN_VBUS_DETECT) != 0;
#else
    return false;   // No VBUS-sense GPIO on this board — always unknown/false.
#endif
}

bool fuel_gauge_present(void) {
    return s_ready && s_user_present;
}

void fuel_gauge_set_user_present(bool present) {
    s_user_present = present;
}

esp_err_t fuel_gauge_read(float *volts, float *soc_pct, float *rate_pct_per_hr) {
    if (!s_ready) return ESP_FAIL;

    // All requested registers are read into locals first, and the output
    // pointers are only written once every read has succeeded — so a
    // mid-sequence I2C failure never leaves the caller with a partially
    // populated result (matches veml7700_read()'s all-or-nothing contract).
    float v = 0.0f, s = 0.0f, r = 0.0f;

    if (volts) {
        uint16_t raw = 0;
        esp_err_t err = i2c_dev_read_u16_be(s_dev, REG_VCELL, &raw);
        if (err != ESP_OK) return err;
        v = (float)raw * 0.000078125f;   // 78.125 uV/LSB
    }

    if (soc_pct) {
        uint16_t raw = 0;
        esp_err_t err = i2c_dev_read_u16_be(s_dev, REG_SOC, &raw);
        if (err != ESP_OK) return err;
        s = (float)raw / 256.0f;        // 1/256 %/LSB
    }

    if (rate_pct_per_hr) {
        uint16_t raw = 0;
        esp_err_t err = i2c_dev_read_u16_be(s_dev, REG_CRATE, &raw);
        if (err != ESP_OK) return err;
        // CRATE is signed — reinterpret the raw bit pattern before scaling.
        r = (float)(int16_t)raw * 0.208f;  // 0.208 %/hr/LSB
    }

    if (volts)          *volts           = v;
    if (soc_pct)         *soc_pct         = s;
    if (rate_pct_per_hr) *rate_pct_per_hr = r;

    return ESP_OK;
}

esp_err_t fuel_gauge_read_diag(uint16_t *version, uint8_t *status) {
    if (!s_ready) return ESP_FAIL;

    // Same all-or-nothing contract as fuel_gauge_read(): buffer into locals
    // first, write outputs only once every requested read has succeeded.
    uint16_t v = 0;
    uint8_t  s = 0;

    if (version) {
        esp_err_t err = i2c_dev_read_u16_be(s_dev, REG_VERSION, &v);
        if (err != ESP_OK) return err;
    }

    if (status) {
        esp_err_t err = i2c_dev_read_regs(s_dev, REG_STATUS, &s, 1);
        if (err != ESP_OK) return err;
    }

    if (version) *version = v;
    if (status)  *status  = s;

    return ESP_OK;
}

#else   // HAL_HAS_FUEL_GAUGE == 0 → no-op stubs

esp_err_t fuel_gauge_init(i2c_master_bus_handle_t bus) { (void)bus; return ESP_OK; }
bool      fuel_gauge_vbus_present(void)                { return false; }
bool      fuel_gauge_present(void)                     { return false; }
void      fuel_gauge_set_user_present(bool present)    { (void)present; }
esp_err_t fuel_gauge_read(float *volts, float *soc_pct, float *rate_pct_per_hr) {
    (void)volts; (void)soc_pct; (void)rate_pct_per_hr;
    return ESP_FAIL;
}
esp_err_t fuel_gauge_read_diag(uint16_t *version, uint8_t *status) {
    (void)version; (void)status;
    return ESP_FAIL;
}

#endif  // HAL_HAS_FUEL_GAUGE
