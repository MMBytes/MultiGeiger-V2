// V2.3.30 — Vishay VEML7700 ambient-light sensor driver. See veml7700.h
// for design + reference_veml7700.md memory for full background.

#include "veml7700.h"

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "telemetry.h"

static const char *TAG = "veml7700";

#define VEML7700_ADDR        0x10

// Register addresses (per Vishay datasheet).
#define REG_ALS_CONFIG       0x00   // RW: gain[12:11], IT[9:6], persistence[5:4], int_en[1], shutdown[0]
#define REG_ALS_HIGH_THRESH  0x01   // RW
#define REG_ALS_LOW_THRESH   0x02   // RW
#define REG_POWER_SAVE       0x03   // RW
#define REG_ALS_DATA         0x04   // RO: 16-bit ALS counts
#define REG_WHITE_DATA       0x05   // RO: 16-bit white-channel counts
#define REG_INTERRUPT_STATUS 0x06   // RO: bit[14]=high tripped, bit[15]=low tripped

// Gain encoding (config bits 12:11). 1/8× = lowest sensitivity, widest range.
#define GAIN_1X              0x00
#define GAIN_2X              0x01
#define GAIN_1_8X            0x02
#define GAIN_1_4X            0x03

// Integration time encoding (config bits 9:6).
#define IT_100MS             0x00
#define IT_200MS             0x01
#define IT_400MS             0x02
#define IT_800MS             0x03
#define IT_50MS              0x08
#define IT_25MS              0x0C

// Default config: gain 1/8×, IT 100 ms, persistence 1, interrupts off,
// powered on. Resolution = 0.0042 × (800/100) × (2/0.125) = 0.5376 lux/count.
// Range = 0 to ~35 klux at full ALS scale (65535 counts).
#define DEFAULT_GAIN         GAIN_1_8X
#define DEFAULT_IT           IT_100MS
#define DEFAULT_IT_MS        100      // matches DEFAULT_IT — used for post-config wait
#define DEFAULT_RESOLUTION   0.5376f  // lux per count for the gain/IT above

// Build the 16-bit ALS_CONFIG word from gain + IT + flags. Persistence and
// interrupt-enable left at 0; shutdown bit 0 means powered ON (yes — bit 0
// is "shutdown" in the Vishay datasheet, so 0 = on).
static inline uint16_t build_config(uint8_t gain, uint8_t it) {
    return (uint16_t)(((uint16_t)gain << 11) | ((uint16_t)it << 6));
}

static i2c_master_dev_handle_t s_dev   = NULL;
static bool                    s_ready = false;

// V2.6.19: CSV read callback — deliberate exception to the "cache, don't
// re-read" pattern used by the other drivers' telemetry columns. Unlike
// SHT45/SGP41/pm/noise (each has an existing per-cycle caller whose cached
// result the CSV reuses), nothing calls veml7700_read() once per TX cycle
// today, so there is no cache to read. A live read here is cheap (two I2C
// register reads, same main service task, no mutex contention) and avoids
// inventing a cache with no other consumer.
static bool tm_read_lux(char *cell, size_t cap, void *arg) {
    (void)arg;
    float lux;
    if (veml7700_read(NULL, NULL, &lux) != ESP_OK) return false;
    snprintf(cell, cap, "%.1f", (double)lux);
    return true;
}

esp_err_t veml7700_init(i2c_master_bus_handle_t bus) {
    if (s_ready) return ESP_OK;
    if (!bus) return ESP_ERR_INVALID_ARG;

    // VEML7700 supports up to 400 kHz I²C.
    esp_err_t err = i2c_probe_and_add(bus, VEML7700_ADDR, 400000, 50, &s_dev);
    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "VEML7700 not found at 0x%02X", VEML7700_ADDR);
        return err;
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_bus_add_device: %s", esp_err_to_name(err));
        return err;
    }

    // 1. Wake from shutdown by writing 0x0000 first (per Adafruit lib pattern).
    //    Datasheet says tWAKE = 2.5 ms — wait conservatively.
    err = i2c_dev_write_u16_le(s_dev, REG_ALS_CONFIG, 0x0000);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "wake-write: %s", esp_err_to_name(err));
        i2c_dev_teardown(&s_dev);
        return err;
    }
    // V2.3.31: precise busy-wait. vTaskDelay(pdMS_TO_TICKS(5)) at 100 Hz tick
    // = 1 tick = 0..10 ms actual, sometimes shorter than tWAKE = 2.5 ms but
    // also unpredictable; busy-wait is deterministic and only runs once at init.
    esp_rom_delay_us(5000);

    // 2. Apply our default operating mode (gain + IT).
    err = i2c_dev_write_u16_le(s_dev, REG_ALS_CONFIG, build_config(DEFAULT_GAIN, DEFAULT_IT));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "config-write: %s", esp_err_to_name(err));
        i2c_dev_teardown(&s_dev);
        return err;
    }

    // 3. First reading after a config change is invalid. Wait at least one
    //    integration time before any veml7700_read() returns trustworthy
    //    data. We don't block here — caller's first read is naturally late
    //    enough (boot continues for hundreds of ms before TX cycles begin).
    //    Just log and return.
    s_ready = true;
    ESP_LOGI(TAG, "VEML7700 ready at 0x%02X (gain=1/8x, IT=%dms, %.4f lux/count)",
             VEML7700_ADDR, DEFAULT_IT_MS, (double)DEFAULT_RESOLUTION);

    // V2.6.19: register our CSV column once. env_sensor's dual-bus probe can
    // call veml7700_init() a second time; the s_tm_registered guard (mirrors
    // sht45.c) is what actually prevents a double registration in that case.
    static bool s_tm_registered = false;
    if (!s_tm_registered) {
        s_tm_registered = true;
        telemetry_register("VEML7700 Lux", tm_read_lux, NULL);
    }
    return ESP_OK;
}

bool veml7700_present(void) {
    return s_ready;
}

esp_err_t veml7700_read(uint16_t *raw_als, uint16_t *raw_white, float *lux) {
    if (!s_ready) return ESP_FAIL;

    uint16_t als_count = 0;
    esp_err_t err = i2c_dev_read_u16_le(s_dev, REG_ALS_DATA, &als_count);
    if (err != ESP_OK) return err;

    uint16_t white_count = 0;
    if (raw_white || true) {
        err = i2c_dev_read_u16_le(s_dev, REG_WHITE_DATA, &white_count);
        if (err != ESP_OK) return err;
    }

    if (raw_als)   *raw_als   = als_count;
    if (raw_white) *raw_white = white_count;

    if (lux) {
        // Linear conversion at the configured resolution.
        float L = (float)als_count * DEFAULT_RESOLUTION;

        // Polynomial non-linearity correction (Adafruit / Vishay app-note).
        // Essentially a no-op below ~500 lux (correction factor ≈ 1.002),
        // significant above ~5000 lux. Source: Adafruit_CircuitPython_VEML7700.
        // Applied always — safer than gating on raw count threshold and
        // mathematically equivalent at low values.
        L = (((6.0135e-13f * L - 9.3924e-9f) * L + 8.1488e-5f) * L + 1.0023f) * L;

        *lux = L;
    }
    return ESP_OK;
}
