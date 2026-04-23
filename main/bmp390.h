#pragma once

/** @file
 *  @brief Thin native ESP-IDF driver for the Bosch BMP390 barometric pressure
 *         and temperature sensor.
 *
 *  Uses the shared I2C master bus (GPIO 4 SDA / GPIO 15 SCL) already created
 *  by env_sensor_init(). Default I2C address 0x77 (SDO tied to GND on the
 *  Adafruit breakout). Operating profile: pressure x8 oversampling, temperature
 *  x2 oversampling, IIR filter coefficient 3, forced mode (single conversion
 *  on demand).
 *
 *  Compensation algorithm uses double-precision floating point as recommended
 *  by Bosch (datasheet section 8.5) — the ESP32 FPU handles this natively.
 */

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** @brief Probe the bus for a BMP390, read calibration, and apply the
 *         operating profile.
 *
 *  Returns ESP_OK if the chip is found and ready. The bus must already be up.
 */
esp_err_t bmp390_init(i2c_master_bus_handle_t bus);

/** @brief True if init succeeded. */
bool bmp390_present(void);

/** @brief Trigger a forced-mode measurement and return compensated results.
 *
 *  Blocks ~20 ms while the sensor converts (P x8 + T x2 worst-case ~11 ms;
 *  20 ms adds comfortable margin). Either output pointer may be NULL.
 *  Returns ESP_OK on success; ESP_FAIL if not present or I2C failed.
 */
esp_err_t bmp390_read(float *temperature_c, float *pressure_pa);
