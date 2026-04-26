#pragma once

/** @file
 *  @brief Native ESP-IDF driver for the Bosch BME680 / BME688 temperature,
 *         humidity and pressure sensor. Gas channel is intentionally NOT used
 *         (heater stays off — saves power and avoids self-heating skewing T/H).
 *
 *  Uses the shared I2C master bus (GPIO 4 SDA / GPIO 15 SCL) already created
 *  by env_sensor_init(). Probes 0x76 then 0x77 (configurable skip when another
 *  driver already owns 0x77). Verifies chip ID 0x61 — distinguishes from the
 *  BME280 (0x60) and BMP390 (0x60) at the same addresses.
 *
 *  Operating profile: oversampling T x8 / P x4 / H x2, IIR filter coef 3,
 *  forced mode (single-shot per read). Compensation uses double-precision
 *  floating point per the Bosch BME68x reference (datasheet section 5.3.2).
 */

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** @brief Probe the bus, verify chip ID, read calibration, configure the
 *         operating profile, and leave the chip in sleep mode (forced-mode
 *         conversions are triggered per-read).
 *
 *  @param bus          Shared I2C master bus handle (created by env_sensor_init).
 *  @param skip_addr_77 When true, only probes 0x76 — used when a BMP390 already
 *                      occupies 0x77.
 *
 *  Returns ESP_OK on success. On failure, bme688_read() will keep returning
 *  ESP_FAIL — caller should log once and continue.
 */
esp_err_t bme688_init(i2c_master_bus_handle_t bus, bool skip_addr_77);

/** @brief True if init succeeded and the sensor is ready to read. */
bool bme688_present(void);

/** @brief Trigger a forced-mode conversion and return compensated results.
 *
 *  Blocks ~50 ms while the sensor converts (T x8 + P x4 + H x2 worst-case
 *  ~38 ms; 50 ms adds margin). Units: temperature °C, humidity %RH, pressure
 *  Pa. Any out-pointer may be NULL. Returns ESP_OK on success; ESP_FAIL if
 *  not present or I2C failed.
 */
esp_err_t bme688_read(float *temperature_c, float *humidity_pct, float *pressure_pa);
