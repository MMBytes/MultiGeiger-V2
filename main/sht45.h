#pragma once

/** @file
 *  @brief Thin native ESP-IDF driver for the Sensirion SHT45 temperature and
 *         humidity sensor.
 *
 *  Uses the shared I2C master bus (GPIO 4 SDA / GPIO 15 SCL) already created
 *  by env_sensor_init(). Fixed I2C address 0x44. High-precision measurement
 *  mode (~8.2 ms per reading).
 *
 *  Includes an optional periodic heater activation: when humidity exceeds 80 %
 *  and at least 10 minutes have elapsed since the last activation, a 200 mW /
 *  100 ms heat pulse is applied. This burns off condensation that would
 *  otherwise saturate the capacitive element — relevant for outdoor deployment
 *  in humid climates.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** @brief Probe the bus for an SHT45 and configure it for high-precision mode.
 *
 *  Returns ESP_OK if the sensor acknowledged. The bus must already be up.
 */
esp_err_t sht45_init(i2c_master_bus_handle_t bus);

/** @brief True if init succeeded. */
bool sht45_present(void);

/** @brief Trigger a high-precision measurement and return the result.
 *
 *  Blocks ~10 ms while the sensor converts. Either output pointer may be NULL.
 *  Returns ESP_OK on success; ESP_FAIL if not present, I2C failed, or CRC
 *  mismatch.
 */
esp_err_t sht45_read(float *temperature_c, float *humidity_pct);

/** @brief Call once per sensor-read cycle with the current uptime and the
 *         last humidity reading.
 *
 *  Activates the built-in heater (200 mW / 100 ms) when humidity > 80 % and
 *  at least 10 minutes have elapsed since the previous activation. No-op if
 *  the sensor is not present.
 */
void sht45_heat_periodic(uint32_t now_ms, float humidity_pct);
