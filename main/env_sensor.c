#include "env_sensor.h"
#include "hal.h"   // PIN_I2C_SDA, PIN_I2C_SCL, HAL_HAS_VEXT_GATE, PIN_VEXT

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sht45.h"
#include "bmp581.h"
#include "bmp390.h"
#include "bme688.h"
#include "bme280.h"

static const char *TAG = "env";

// I2C bus assignment is shared across boards (always I2C_NUM_0); pin numbers
// vary by board and come from hal.h.
#define I2C_PORT       I2C_NUM_0
#define I2C_FREQ_HZ    100000

static i2c_master_bus_handle_t s_bus = NULL;

// --- Init --------------------------------------------------------------------

esp_err_t env_sensor_init(void) {
#if HAL_HAS_VEXT_GATE
    // Heltec Vext rail (GPIO 21, active-LOW). On older Heltec module revisions
    // Vext was tied to GND on the carrier PCB so the OLED + I²C pull-up rail
    // was always powered. Newer WiFi Kit 32 V2 modules route Vext through a
    // P-channel MOSFET driven by GPIO 21 — without this drive the OLED, all
    // I²C pull-ups, and any external sensor breakouts on the OLED power rail
    // are unpowered. Symptom: every I²C probe times out and the IDF I²C driver
    // logs the misleading "GPIO X is not usable" warning. Driving GPIO 21 LOW
    // is harmless on the older modules where it had no effect, and required
    // on the newer ones — so always drive it.
    gpio_reset_pin(PIN_VEXT);
    gpio_set_direction(PIN_VEXT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_VEXT, 0);                // 0 = Vext ON
    vTaskDelay(pdMS_TO_TICKS(50));              // rail settle + OLED charge-pump warm-up
#endif

    // Create the shared I2C master bus. All sensors and (when present) the
    // OLED SSD1306 share this handle. Internal pull-ups enabled as
    // belt-and-braces — Heltec carrier PCB has 4.7 k externals on J_I2C; the
    // FeatherS3 Qwiic breakouts have their own pull-ups.
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port             = I2C_PORT,
        .sda_io_num           = PIN_I2C_SDA,
        .scl_io_num           = PIN_I2C_SCL,
        .clk_source           = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt    = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus: %s", esp_err_to_name(err));
        return err;
    }

    // BMP390, BME688 and BME280 all use I2C addresses 0x76/0x77 — only ONE of
    // the three can physically be on the bus at a time. We probe in priority
    // order (highest accuracy first): BMP581 (6th-gen Bosch, 0x46/0x47 —
    // independent of the 0x77 family, so it can coexist with any of them) →
    // BMP390 (dedicated barometer, fixed 0x77 on the Adafruit breakout) →
    // BME688 (newer Bosch T/P/H, gas off) → BME280 (legacy fallback). Each
    // later probe in the 0x77 family skips its address if a prior driver
    // claimed it, defending against an accidental dual-population on different
    // addresses (e.g. one chip jumpered to 0x76). Chip-ID is also verified
    // inside each driver — but BMP390 and BME280 both report 0x60, so chip-ID
    // alone is not enough; the address-skip ordering is the real defence.

    sht45_init(s_bus);   // 0x44 — separate I2C address, no conflict possible

    bool bmp581_ok = (bmp581_init(s_bus) == ESP_OK);  // 0x46/0x47 — independent of 0x77 family
    (void)bmp581_ok;
    bool bmp390_ok = (bmp390_init(s_bus) == ESP_OK);
    bool bme688_ok = (bme688_init(s_bus, bmp390_ok) == ESP_OK);
    bool bme_addr_77_busy = bmp390_ok || bme688_ok;
    bme280_init(s_bus, bme_addr_77_busy);

    ESP_LOGI(TAG, "env sensor: %s", env_sensor_name());
    return ESP_OK;
}

// --- Public API --------------------------------------------------------------

bool env_sensor_present(void) {
    return sht45_present() || bmp581_present() || bmp390_present() ||
           bme688_present() || bme280_present();
}

esp_err_t env_sensor_read(float *temperature_c, float *humidity_pct,
                          float *pressure_pa) {
    float t = 0, h = 0, p = 0;
    bool  have_t = false, have_h = false, have_p = false;

    // Temperature + humidity: SHT45 is primary (best accuracy and lowest
    // self-heating of all our T/H options).
    if (sht45_present()) {
        float st, sh;
        if (sht45_read(&st, &sh) == ESP_OK) {
            t = st; h = sh;
            have_t = have_h = true;
        }
    }

    // Pressure (and temperature fallback): BMP581 is the highest-priority
    // pressure source — 6th-gen Bosch barometer with factory-trimmed digital
    // output, ±0.4 Pa relative accuracy, 0.21 PaRMS noise at our OSR setting.
    // Lives at 0x46/0x47 so it can coexist with any 0x77-family chip.
    if (bmp581_present()) {
        float bt, bp;
        if (bmp581_read(&bt, &bp) == ESP_OK) {
            p = bp;
            have_p = true;
            if (!have_t) { t = bt; have_t = true; }
        }
    }

    // BMP390 — second-priority pressure source, used when BMP581 isn't
    // populated. Dedicated barometric chip (no humidity), 0x77 on the
    // Adafruit breakout.
    if (bmp390_present() && !have_p) {
        float bt, bp;
        if (bmp390_read(&bt, &bp) == ESP_OK) {
            p = bp;
            have_p = true;
            if (!have_t) { t = bt; have_t = true; }
        }
    }

    // BME688 — third-priority T/P/H source, only present when neither BMP581
    // (different address) nor BMP390 (same address) claimed the slot. Fills
    // any remaining gap.
    if (bme688_present() && (!have_t || !have_h || !have_p)) {
        float bt, bh, bp;
        if (bme688_read(&bt, &bh, &bp) == ESP_OK) {
            if (!have_t) { t = bt; have_t = true; }
            if (!have_h) { h = bh; have_h = true; }
            if (!have_p) { p = bp; have_p = true; }
        }
    }

    // BME280 — legacy fallback, only present when BMP390 and BME688 aren't
    // (all three share I2C 0x77).
    if (bme280_present() && (!have_t || !have_h || !have_p)) {
        float bt, bh, bp;
        if (bme280_read(&bt, &bh, &bp) == ESP_OK) {
            if (!have_t) { t = bt; have_t = true; }
            if (!have_h) { h = bh; have_h = true; }
            if (!have_p) { p = bp; have_p = true; }
        }
    }

    if (!have_t && !have_h && !have_p) return ESP_FAIL;

    if (temperature_c) *temperature_c = t;
    if (humidity_pct)  *humidity_pct  = h;
    if (pressure_pa)   *pressure_pa   = p;
    return ESP_OK;
}

// Returns the active-sensor name in priority order. The "+" separator marks
// data fusion across two chips (e.g. "SHT45+BMP581" = SHT45 supplies T/H,
// BMP581 supplies P). BMP581 lives at 0x46/0x47 so it can coexist with any
// of the 0x77-family chips (BMP390/BME688/BME280) — but BMP581 is always
// preferred for pressure when present, so the 0x77 chip is only useful as
// T/H backstop in that case (and we only get to BME688/BME280 if BMP581 is
// absent and they happen to also be populated). Listing every theoretical
// combination would explode the table; instead we cover the realistic
// populates: SHT45+BMP581, SHT45+BMP390, SHT45+BME688, SHT45+BME280, plus
// the single-chip degenerate cases.
const char *env_sensor_name(void) {
    bool has_sht = sht45_present();
    bool has_581 = bmp581_present();
    bool has_390 = bmp390_present();
    bool has_b68 = bme688_present();
    bool has_b28 = bme280_present();

    if (has_sht && has_581) return "SHT45+BMP581";
    if (has_sht && has_390) return "SHT45+BMP390";
    if (has_sht && has_b68) return "SHT45+BME688";
    if (has_sht && has_b28) return "SHT45+BME280";
    if (has_sht)            return "SHT45";
    if (has_581)            return "BMP581";
    if (has_390)            return "BMP390";
    if (has_b68)            return "BME688";
    if (has_b28)            return "BME280";
    return "none";
}

void env_sensor_heat_periodic(uint32_t now_ms, float humidity_pct) {
    sht45_heat_periodic(now_ms, humidity_pct);
}

i2c_master_bus_handle_t env_sensor_get_i2c_bus(void) {
    return s_bus;
}
