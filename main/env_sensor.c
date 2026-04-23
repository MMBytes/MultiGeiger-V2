#include "env_sensor.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

#include "sht45.h"
#include "bmp390.h"
#include "bme280.h"

static const char *TAG = "env";

// --- Board wiring (Heltec Wireless Stick V2) --------------------------------
#define PIN_SDA        4
#define PIN_SCL       15
#define I2C_PORT       I2C_NUM_0
#define I2C_FREQ_HZ    100000

static i2c_master_bus_handle_t s_bus = NULL;

// --- Init --------------------------------------------------------------------

esp_err_t env_sensor_init(void) {
    // Create the shared I2C master bus. All sensors and the OLED SSD1306 share
    // this handle. Internal pull-ups enabled as belt-and-braces — the PCB has
    // 4.7 k externals on J_I2C.
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

    // Probe in priority order. BMP390 and BME280 share address 0x77, so
    // BMP390 is tried first. If BMP390 occupies 0x77, BME280 is only tried
    // at 0x76 to avoid a false detection against BMP390's register space.

    sht45_init(s_bus);   // 0x44 — no conflict possible

    bool bmp390_ok = (bmp390_init(s_bus) == ESP_OK);

    // Pass the bus and whether 0x77 is already occupied to the BME280 driver.
    bme280_init(s_bus, bmp390_ok);

    ESP_LOGI(TAG, "env sensor: %s", env_sensor_name());
    return ESP_OK;
}

// --- Public API --------------------------------------------------------------

bool env_sensor_present(void) {
    return sht45_present() || bmp390_present() || bme280_present();
}

esp_err_t env_sensor_read(float *temperature_c, float *humidity_pct,
                          float *pressure_pa) {
    float t = 0, h = 0, p = 0;
    bool  have_t = false, have_h = false, have_p = false;

    // Temperature + humidity: SHT45 is primary.
    if (sht45_present()) {
        float st, sh;
        if (sht45_read(&st, &sh) == ESP_OK) {
            t = st; h = sh;
            have_t = have_h = true;
        }
    }

    // Pressure: BMP390 is primary. Also grabs its temperature if SHT45 is
    // absent (BMP390 temperature is secondary but better than BME280).
    if (bmp390_present()) {
        float bt, bp;
        if (bmp390_read(&bt, &bp) == ESP_OK) {
            p = bp;
            have_p = true;
            if (!have_t) { t = bt; have_t = true; }
        }
    }

    // BME280 fills any remaining gap.
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

const char *env_sensor_name(void) {
    bool has_sht = sht45_present();
    bool has_bmp = bmp390_present();
    bool has_bme = bme280_present();

    if (has_sht && has_bmp) return "SHT45+BMP390";
    if (has_sht && has_bme) return "SHT45+BME280";
    if (has_sht)            return "SHT45";
    if (has_bmp && has_bme) return "BMP390+BME280";  // unusual but handled
    if (has_bmp)            return "BMP390";
    if (has_bme)            return "BME280";
    return "none";
}

void env_sensor_heat_periodic(uint32_t now_ms, float humidity_pct) {
    sht45_heat_periodic(now_ms, humidity_pct);
}

i2c_master_bus_handle_t env_sensor_get_i2c_bus(void) {
    return s_bus;
}
