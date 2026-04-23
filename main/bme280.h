#pragma once

/** @file
 *  @brief Native ESP-IDF driver for the Bosch BME280 temperature/humidity/
 *         pressure sensor on the Heltec Wireless Stick V2.
 *
 *  Owns I2C bus 0 on SDA=GPIO4 / SCL=GPIO15 at 100 kHz. Auto-probes addresses
 *  0x76 then 0x77. Operating profile: oversampling T x8 / P x4 / H x2,
 *  IIR filter coef 4, standby 1000 ms, continuous normal mode.
 *
 *  The I2C bus is exposed so other on-board I2C devices (e.g. the SSD1306
 *  OLED on the same bus) can share it.
 */

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** @brief One-shot init: probe the supplied I2C bus, reset, read calibration,
 *         configure the operating profile, and enter normal mode.
 *
 *  @param bus          Shared I2C master bus handle (created by env_sensor_init).
 *  @param skip_addr_77 When true, only probes 0x76 — used when a BMP390 already
 *                      occupies 0x77 to avoid a false detection.
 *
 *  Returns ESP_OK on success. On failure, bme280_read() will keep returning
 *  ESP_FAIL — caller should log once and continue.
 */
esp_err_t bme280_init(i2c_master_bus_handle_t bus, bool skip_addr_77);

/** @brief True if init succeeded and the sensor is ready to read. */
bool bme280_present(void);

/** @brief Read one compensated sample.
 *
 *  Units: temperature °C, humidity %RH, pressure Pa. Any out-pointer may be
 *  NULL. Returns ESP_OK on success; ESP_FAIL if not present or I2C failed.
 */
esp_err_t bme280_read(float *temperature_c, float *humidity_pct, float *pressure_pa);

/** @brief Shared I2C master bus (for the OLED and other on-board devices).
 *
 *  Returns NULL if bme280_init() was never called or the bus couldn't be
 *  created. Safe to call even if the BME280 chip wasn't detected — the bus
 *  is brought up before the probe.
 */
i2c_master_bus_handle_t bme280_get_i2c_bus(void);
