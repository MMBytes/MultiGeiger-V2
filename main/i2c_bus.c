// V2.3.29 — I²C bus lifecycle owner. See i2c_bus.h for design doc.

#include "i2c_bus.h"
#include "hal.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "i2c_bus";

static i2c_master_bus_handle_t s_bus_primary    = NULL;
#if defined(BOARD_FEATHERS3_D) || defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
// Only boards with a real second I²C controller ever bring this online; on
// the other boards the declaration would trip -Wunused-variable. Keep the
// declaration scoped to the boards that actually read/write it.
static i2c_master_bus_handle_t s_bus_secondary  = NULL;
#endif
static bool                    s_secondary_kept = false;

// V2.5.19: primary-bus pin route selector. Honoured only where the board
// defines HAL_HAS_I2C_PINOUT_SWITCH (QT Py); set from g_cfg.i2c_pinout by
// main.c before the first i2c_bus_get_primary(). Guarded to the switch boards
// so the static doesn't trip -Wunused-variable where it's never read (same
// reason s_bus_secondary is BOARD_FEATHERS3_D-scoped above).
#if HAL_HAS_I2C_PINOUT_SWITCH
static bool                    s_primary_pinout = false;
#endif

void i2c_bus_set_primary_pinout(bool use_pinout) {
#if HAL_HAS_I2C_PINOUT_SWITCH
    s_primary_pinout = use_pinout;
#else
    (void)use_pinout;   // no alternate route on this board — selector inert
#endif
}

i2c_master_bus_handle_t i2c_bus_get_primary(void) {
    if (s_bus_primary) return s_bus_primary;

#if HAL_HAS_I2C_PINOUT_SWITCH
    // Pick the broken-out SDA/SCL pads when the user opted into pin-out mode,
    // else the onboard/STEMMA route. The switch only exists on boards that
    // define a PIN_I2C_*_ALT pair (guarded by the HAL flag above).
    const int sda = s_primary_pinout ? PIN_I2C_SDA_ALT : PIN_I2C_SDA;
    const int scl = s_primary_pinout ? PIN_I2C_SCL_ALT : PIN_I2C_SCL;
#else
    const int sda = PIN_I2C_SDA;
    const int scl = PIN_I2C_SCL;
#endif

#if HAL_HAS_VEXT_GATE
    // Heltec Vext rail (active-LOW MOSFET on PIN_VEXT). Older modules
    // had Vext tied to GND on the carrier PCB so this drive was a no-op;
    // newer modules route Vext through a P-MOSFET driven by GPIO 21 —
    // without this drive the OLED + I²C pull-ups are unpowered and
    // every probe times out with the misleading "GPIO X is not usable"
    // warning. Driving LOW is harmless on the older modules and required
    // on the newer ones, so always drive.
    // V2.6.22: ESP_ERROR_CHECK on this and every rail-gate triple below —
    // these can only fail on an invalid or input-only pin constant, i.e. a
    // board-port mistake (classic ESP32's GPIO34-39 are input-only and
    // gpio_set_direction silently no-ops on them). Aborting at first boot
    // with the offending line beats the alternative: an unpowered rail
    // where every I2C probe times out — the exact V2.6.7 dead-OLED
    // signature that took a bench session to root-cause.
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_VEXT));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_VEXT, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(PIN_VEXT, 0));      // 0 = Vext ON
    vTaskDelay(pdMS_TO_TICKS(50));                     // rail settle + OLED charge-pump warm-up
#endif

#if defined(BOARD_ADAFRUIT_ESP32S3_TFT_FEATHER) || defined(BOARD_ADAFRUIT_ESP32S3_FEATHER_4MB_2MBPSRAM)
    // V2.6.11 review fix: this board's shared TFT/I2C peripheral rail
    // (Adafruit's own "TFT_I2C_POWER" net, GPIO21) is active-HIGH direct
    // GPIO — opposite polarity from Heltec's active-LOW P-MOSFET Vext gate
    // above, so it can't reuse HAL_HAS_VEXT_GATE. Originally driven inside
    // display_tft_init(), which runs too late (after fuel_gauge_init() and
    // every env/PM/noise/GNSS/VEML I2C probe in main.c) — every one of
    // those NACK'd on a dead bus. i2c_bus_get_primary() is the first I2C
    // consumer main.c calls, so gating here (mirroring HAL_HAS_VEXT_GATE's
    // pattern) guarantees the rail is up before any probe, including the
    // TFT's own SPI bring-up which runs later via display_setup().
    //
    // BOARD_ADAFRUIT_ESP32S3_FEATHER_4MB_2MBPSRAM (#5477) shares this same
    // active-HIGH gate pattern on its own PIN_I2C_POWER_GATE (GPIO7,
    // Adafruit's "PIN_I2C_POWER" net) — no TFT on that board, but the same
    // "gate the only I2C bus before any probe" requirement applies.
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_I2C_POWER_GATE));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_I2C_POWER_GATE, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(PIN_I2C_POWER_GATE, 1));   // 1 = rail ON
    vTaskDelay(pdMS_TO_TICKS(50));                      // rail settle
#endif

    i2c_master_bus_config_t cfg = {
        .i2c_port             = I2C_NUM_0,
        .sda_io_num           = sda,
        .scl_io_num           = scl,
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
#if HAL_HAS_I2C_PINOUT_SWITCH
    ESP_LOGI(TAG, "primary bus up (I2C_NUM_0, SDA=%d SCL=%d, route=%s)",
             sda, scl, s_primary_pinout ? "pinout-pads" : "onboard/STEMMA");
#else
    ESP_LOGI(TAG, "primary bus up (I2C_NUM_0, SDA=%d SCL=%d)", sda, scl);
#endif
    return s_bus_primary;
}

i2c_master_bus_handle_t i2c_bus_get_secondary(void) {
#if defined(BOARD_FEATHERS3_D)
    if (s_bus_secondary) return s_bus_secondary;

    // Enable LDO2 (powers 3V3.2 → STEMMA2 + onboard NeoPixel rail).
    // Default state of IO39 is hi-Z + internal pull-down → LDO2 OFF →
    // STEMMA2 V+ pin dead, no device can ACK there. Drive HIGH to wake.
    ESP_ERROR_CHECK(gpio_reset_pin(GPIO_NUM_39));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_39, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_39, 1));
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
#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    if (s_bus_secondary) return s_bus_secondary;

    // V2.6.20 (see hal.h's HAL_HAS_VEXT_GATE comment on this board):
    // first real hardware showed every transaction on this bus timing
    // out, the classic signature of an unpowered I2C target — the
    // datasheet's claim that Vext_Ctrl doesn't gate the OLED was wrong.
    // Drive Vext_Ctrl LOW here (mirroring Heltec V2's PIN_VEXT pattern in
    // i2c_bus_get_primary() above) before bringing the controller up;
    // bench-confirmed to clear the timeouts and light up the OLED.
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_VEXT));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_VEXT, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(PIN_VEXT, 0));      // 0 = Vext ON
    vTaskDelay(pdMS_TO_TICKS(50));                     // rail settle + OLED charge-pump warm-up

    i2c_master_bus_config_t cfg = {
        .i2c_port             = I2C_NUM_1,
        .sda_io_num           = PIN_OLED_SDA,
        .scl_io_num           = PIN_OLED_SCL,
        .clk_source           = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt    = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus_secondary);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OLED bus init failed: %s", esp_err_to_name(err));
        s_bus_secondary = NULL;
        return NULL;
    }
    ESP_LOGI(TAG, "secondary bus up (I2C_NUM_1, SDA=%d SCL=%d) — onboard OLED, Vext-gated",
             PIN_OLED_SDA, PIN_OLED_SCL);
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
#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    // Vext (GPIO36, driven in i2c_bus_get_secondary() since V2.6.20) is
    // deliberately left ON even when no consumer claimed the bus — unlike
    // FeatherS3-D above, which sheds LDO2 by design when STEMMA2 is
    // unpopulated. This bus is dedicated permanently to the onboard OLED,
    // so a missing consumer means the probe failed (anomalous), not that
    // the rail is unneeded; and the same MOSFET also feeds the external Ve
    // header. This branch exists only so an OLED probe failure leaves a
    // log trace instead of silently leaking the I2C_NUM_1 handle with zero
    // diagnostic trail.
    if (s_bus_secondary && !s_secondary_kept) {
        ESP_LOGW(TAG, "OLED bus is up but no consumer claimed it — "
                      "OLED probe likely failed this boot");
    }
#endif
}
