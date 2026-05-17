// V2.3.29 — I²C bus lifecycle owner. See i2c_bus.h for design doc.

#include "i2c_bus.h"
#include "hal.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "i2c_bus";

static i2c_master_bus_handle_t s_bus_primary    = NULL;
#if defined(BOARD_FEATHERS3_D)
// Only the FeatherS3-D ever brings the second STEMMA QT bus online; on the
// other boards the declaration would trip -Wunused-variable. Keep the
// declaration scoped to the boards that actually read/write it.
static i2c_master_bus_handle_t s_bus_secondary  = NULL;
#endif
static bool                    s_secondary_kept = false;

i2c_master_bus_handle_t i2c_bus_get_primary(void) {
    if (s_bus_primary) return s_bus_primary;

#if HAL_HAS_VEXT_GATE
    // Heltec Vext rail (active-LOW MOSFET on PIN_VEXT). Older modules
    // had Vext tied to GND on the carrier PCB so this drive was a no-op;
    // newer modules route Vext through a P-MOSFET driven by GPIO 21 —
    // without this drive the OLED + I²C pull-ups are unpowered and
    // every probe times out with the misleading "GPIO X is not usable"
    // warning. Driving LOW is harmless on the older modules and required
    // on the newer ones, so always drive.
    gpio_reset_pin(PIN_VEXT);
    gpio_set_direction(PIN_VEXT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_VEXT, 0);                       // 0 = Vext ON
    vTaskDelay(pdMS_TO_TICKS(50));                     // rail settle + OLED charge-pump warm-up
#endif

    i2c_master_bus_config_t cfg = {
        .i2c_port             = I2C_NUM_0,
        .sda_io_num           = PIN_I2C_SDA,
        .scl_io_num           = PIN_I2C_SCL,
        .clk_source           = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt    = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus_primary);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "primary bus init failed: %s", esp_err_to_name(err));
        s_bus_primary = NULL;
        return NULL;
    }
    ESP_LOGI(TAG, "primary bus up (I2C_NUM_0, SDA=%d SCL=%d)",
             PIN_I2C_SDA, PIN_I2C_SCL);
    return s_bus_primary;
}

i2c_master_bus_handle_t i2c_bus_get_secondary(void) {
#if defined(BOARD_FEATHERS3_D)
    if (s_bus_secondary) return s_bus_secondary;

    // Enable LDO2 (powers 3V3.2 → STEMMA2 + onboard NeoPixel rail).
    // Default state of IO39 is hi-Z + internal pull-down → LDO2 OFF →
    // STEMMA2 V+ pin dead, no device can ACK there. Drive HIGH to wake.
    gpio_reset_pin(GPIO_NUM_39);
    gpio_set_direction(GPIO_NUM_39, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_39, 1);
    vTaskDelay(pdMS_TO_TICKS(10));                     // 3V3.2 rail settle (LDO2 turn-on ~1 ms)

    i2c_master_bus_config_t cfg = {
        .i2c_port             = I2C_NUM_1,
        .sda_io_num           = 16,                    // FeatherS3-D STEMMA2 SDA
        .scl_io_num           = 15,                    // FeatherS3-D STEMMA2 SCL
        .clk_source           = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt    = 7,
        .flags.enable_internal_pullup = true,          // STEMMA2 has no PCB pull-ups
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus_secondary);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "secondary bus init failed: %s", esp_err_to_name(err));
        s_bus_secondary = NULL;
        gpio_set_level(GPIO_NUM_39, 0);                // back off — don't burn LDO2 for nothing
        return NULL;
    }
    ESP_LOGI(TAG, "secondary bus up (I2C_NUM_1, SDA=16 SCL=15) — LDO2 enabled");
    return s_bus_secondary;
#else
    return NULL;   // Heltec / QT Py: no second bus on this board
#endif
}

void i2c_bus_secondary_keep_alive(void) {
    s_secondary_kept = true;
}

void i2c_bus_finalize(void) {
#if defined(BOARD_FEATHERS3_D)
    if (s_bus_secondary && !s_secondary_kept) {
        i2c_del_master_bus(s_bus_secondary);
        s_bus_secondary = NULL;
        gpio_set_level(GPIO_NUM_39, 0);                // LDO2 off — saves ~5–10 mA quiescent
        ESP_LOGI(TAG, "secondary bus torn down — no consumer (LDO2 off)");
    } else if (s_bus_secondary) {
        ESP_LOGI(TAG, "secondary bus kept alive — at least one consumer is using it");
    }
    // s_bus_secondary == NULL && !kept: secondary was never requested, nothing to do.
#endif
}
