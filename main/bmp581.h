#pragma once

/** @file
 *  @brief Thin native ESP-IDF driver for the Bosch BMP581 6th-generation
 *         barometric pressure and temperature sensor.
 *
 *  Uses the shared I2C master bus already created by env_sensor_init().
 *  Default address on the Adafruit #6407 breakout is 0x46 (SDO tied LOW on
 *  the PCB); SparkFun's breakout defaults to 0x47. The driver probes 0x46
 *  first and falls back to 0x47, so either board works without configuration.
 *
 *  Operating profile: pressure x16 oversampling, temperature x1, IIR BYPASS,
 *  forced mode (single conversion on demand). This matches Bosch's Table 9
 *  "high resolution" preset and ESPHome's default — at our 150 s polling
 *  interval, atmospheric noise dominates the sensor noise floor (0.21 PaRMS),
 *  so higher OSR is wasted power; IIR latency is wasted time since
 *  oversampling already does multi-sample averaging within each conversion.
 *
 *  No NVM compensation coefficients to fetch — BMP581 silicon ships with
 *  factory-trimmed digital output, the headline simplification of the BMP5xx
 *  generation. Pressure conversion is `raw / 64.0` Pa; temperature is
 *  `signed24 / 65536.0` °C (datasheet §4.5).
 */

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** @brief Probe the bus for a BMP581 (chip ID 0x50) and apply the operating
 *         profile.
 *
 *  Tries 0x46 (Adafruit default) then 0x47 (SparkFun default). Returns
 *  ESP_OK if the chip is found and ready. Chip ID 0x51 (= BMP585, a different
 *  Bosch product) is rejected with a warning to catch wrong-part populates.
 *  The bus must already be up.
 */
esp_err_t bmp581_init(i2c_master_bus_handle_t bus);

/** @brief True if init succeeded. */
bool bmp581_present(void);

/** @brief Trigger a forced-mode measurement and return the result.
 *
 *  Blocks ~12 ms while the sensor converts (10.4 ms tconv_p @16x + 1.0 ms
 *  tconv_t @1x, ESPHome's `ceil(1.05 × (tconv_p + tconv_t))` formula). Either
 *  output pointer may be NULL. Returns ESP_OK on success; ESP_FAIL if not
 *  present or I2C failed.
 */
esp_err_t bmp581_read(float *temperature_c, float *pressure_pa);
