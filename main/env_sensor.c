#include "env_sensor.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sht45.h"
#include "bmp390.h"
#include "bme688.h"
#include "bme280.h"

static const char *TAG = "env";

// --- Board wiring (Heltec Wireless Stick V2) --------------------------------
#define PIN_SDA        4
#define PIN_SCL       15
#define PIN_VEXT      21    // Heltec Vext power switch (active-LOW MOSFET gate)
#define I2C_PORT       I2C_NUM_0
#define I2C_FREQ_HZ    100000

static i2c_master_bus_handle_t s_bus = NULL;

// --- Init --------------------------------------------------------------------

esp_err_t env_sensor_init(void) {
    // Heltec Vext rail (GPIO 21, active-LOW). On older Heltec module revisions
    // Vext was tied to GND on the carrier PCB so the OLED + I²C pull-up rail
    // was always powered. Newer Wireless Stick V2 modules (PICO-D4 / PICO-V3-02)
    // route Vext through a P-channel MOSFET driven by GPIO 21 — without this
    // drive the OLED, all I²C pull-ups, and any external sensor breakouts on
    // the OLED power rail are unpowered. Symptom: every I²C probe times out
    // and the IDF I²C driver logs the misleading "GPIO X is not usable"
    // warning. Driving GPIO 21 LOW is harmless on the older modules where it
    // had no effect, and required on the newer ones — so always drive it.
    gpio_reset_pin(PIN_VEXT);
    gpio_set_direction(PIN_VEXT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_VEXT, 0);                // 0 = Vext ON
    vTaskDelay(pdMS_TO_TICKS(50));              // rail settle + OLED charge-pump warm-up

    // Create the shared I2C master bus. All sensors and the OLED SSD1306 share
    // this handle. Internal pull-ups enabled as belt-and-braces — the PCB has
    // 4.7 k externals on J_I2C (now actually powered, thanks to Vext above).
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port             = I2C_PORT,
        .sda_io_num           = PIN_SDA,
        .scl_io_num           = PIN_SCL,
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
    // order (highest accuracy first): BMP390 (dedicated barometer, fixed 0x77
    // on the Adafruit breakout) → BME688 (newer Bosch T/P/H, gas off) → BME280
    // (legacy fallback). Each later probe skips 0x77 if a prior driver claimed
    // it, defending against an accidental dual-population on different
    // addresses (e.g. one chip jumpered to 0x76). Chip-ID is also verified
    // inside each driver — but BMP390 and BME280 both report 0x60, so chip-ID
    // alone is not enough; the address-skip ordering is the real defence.

    sht45_init(s_bus);   // 0x44 — separate I2C address, no conflict possible

    bool bmp390_ok = (bmp390_init(s_bus) == ESP_OK);
    bool bme688_ok = (bme688_init(s_bus, bmp390_ok) == ESP_OK);
    bool bme_addr_77_busy = bmp390_ok || bme688_ok;
    bme280_init(s_bus, bme_addr_77_busy);

    ESP_LOGI(TAG, "env sensor: %s", env_sensor_name());
    return ESP_OK;
}

// --- Public API --------------------------------------------------------------

bool env_sensor_present(void) {
    return sht45_present() || bmp390_present() ||
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

    // Pressure (and temperature fallback): BMP390 is the highest-priority
    // pressure source — dedicated barometric chip, most accurate of the three
    // 0x77-family devices. Also yields its temperature if SHT45 is absent.
    if (bmp390_present()) {
        float bt, bp;
        if (bmp390_read(&bt, &bp) == ESP_OK) {
            p = bp;
            have_p = true;
            if (!have_t) { t = bt; have_t = true; }
        }
    }

    // BME688 — second-priority T/P/H source, only present when BMP390 isn't
    // (they share I2C 0x77). Fills any remaining gap.
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
// data fusion across two chips (e.g. "SHT45+BMP390" = SHT45 supplies T/H,
// BMP390 supplies P). Only SHT45 + one Bosch chip is a real combination —
// BMP390 / BME688 / BME280 all share I2C 0x77 so at most one of them can be
// on the bus at any given time (see the probe-order comment in
// env_sensor_init). If two ARE somehow detected, the higher-priority chip
// wins by falling through into the single-name cases below.
const char *env_sensor_name(void) {
    bool has_sht = sht45_present();
    bool has_bmp = bmp390_present();
    bool has_b68 = bme688_present();
    bool has_b28 = bme280_present();

    if (has_sht && has_bmp) return "SHT45+BMP390";
    if (has_sht && has_b68) return "SHT45+BME688";
    if (has_sht && has_b28) return "SHT45+BME280";
    if (has_sht)            return "SHT45";
    if (has_bmp)            return "BMP390";
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
