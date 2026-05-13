#pragma once

/** @file
 *  @brief Unified environmental sensor interface — SHT45 plus a Bosch
 *         pressure/THP chip (BMP581 / BMP390 / BME688 / BME280).
 *
 *  Detection order on init:
 *    1. SHT45  at 0x44       — temperature + humidity (±0.1 °C, ±1 % RH)
 *    2. BMP581 at 0x46/0x47  — pressure + temperature (±0.4 Pa relative,
 *                              0.21 PaRMS noise) — independent of 0x77 family
 *    3. BMP390 at 0x77       — pressure + temperature (±0.5 hPa)
 *    4. BME688 at 0x77       — temperature + humidity + pressure (only if
 *                              BMP390 didn't claim 0x77)
 *    5. BME280 at 0x76 or 0x77 — legacy fallback (only 0x76 if a previous
 *                              0x77-family chip claimed 0x77)
 *
 *  This module owns the I2C bus. All devices on that bus (OLED SSD1306,
 *  pressure chip, SHT45) share the same handle, obtained via
 *  env_sensor_get_i2c_bus().
 *
 *  main.c replaces all bme280_*() calls with env_sensor_*() calls.
 *  display.c replaces bme280_get_i2c_bus() with env_sensor_get_i2c_bus().
 *  transmission.c is unchanged — the tx_context_t fields are identical.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** @brief Create the I2C bus and probe for sensors in priority order.
 *
 *  Always returns ESP_OK — sensor absence is non-fatal. Check
 *  env_sensor_present() to know whether any THP data is available.
 */
esp_err_t env_sensor_init(void);

/** @brief True if at least one sensor capable of temperature reading is present. */
bool env_sensor_present(void);

/** @brief Read one set of compensated measurements.
 *
 *  Any pointer may be NULL. Fills from the highest-accuracy source available:
 *    temperature — SHT45 > BMP390 > BME280
 *    humidity    — SHT45 > BME280 (BMP390 has no humidity channel)
 *    pressure    — BMP390 > BME280 (SHT45 has no pressure channel)
 *
 *  Units: temperature °C, humidity %RH, pressure Pa.
 *  Returns ESP_OK on success; ESP_FAIL if no sensor is ready or read failed.
 *
 *  V2.3.26: optional `raw_log` buffer (NULL to skip) is filled with one comma-
 *  separated segment per present-and-called sensor, e.g.
 *    "SHT45: T=18.86°C  H=0.00%, BMP390: T=18.86°C P=1026.60hPa"
 *  Failed reads emit "<NAME>: read failed". Trailing ", " is stripped so the
 *  caller can append " <fused>" with one space. Suggested cap: 160 bytes.
 */
esp_err_t env_sensor_read(float *temperature_c, float *humidity_pct,
                          float *pressure_pa,
                          char *raw_log, size_t raw_log_cap);

/** @brief Short human-readable label describing the active sensor combination.
 *
 *  Examples: "SHT45+BMP390", "SHT45", "BMP390", "BME280". Never NULL.
 */
const char *env_sensor_name(void);

/** @brief Call once per measurement cycle to conditionally fire the SHT45
 *         built-in heater (see sht45.h for policy details).
 */
void env_sensor_heat_periodic(uint32_t now_ms, float humidity_pct);

/** @brief Shared I2C master bus for the OLED and all on-board sensors.
 *
 *  Returns NULL if env_sensor_init() has not been called yet.
 */
i2c_master_bus_handle_t env_sensor_get_i2c_bus(void);
