#pragma once

/** @file
 *  @brief Sensirion SPS30 particulate-matter sensor driver (pure ESP-IDF I²C).
 *
 *  Hand-rolled implementation of the Sensirion SPS30 wire protocol — does not
 *  depend on Sensirion's Arduino library or their `embedded-i2c-sps30`
 *  component. Uses the IDF `i2c_master_*` API directly so it integrates with
 *  env_sensor's shared bus on I2C_NUM_0. Continuous-measurement mode (fan
 *  always running, ~60 mA) — simplest state machine, instantaneous reads,
 *  factory-default 7-day auto-clean trigger fires automatically without any
 *  wake-up dance.
 *
 *  Reads return all 10 channels exposed by float-format mode (4 mass conc.,
 *  5 number conc., typical particle size). Caller-facing struct lives in
 *  pm_sensor.h so the facade can be extended later with other PM sensors.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#include "pm_sensor.h"  // pm_sample_t

/** @brief Probe for SPS30 at 0x69 and start continuous-measurement mode.
 *
 *  Sequence:
 *    1. i2c_master_probe at 0x69 — bail with ESP_ERR_NOT_FOUND if absent.
 *    2. Soft reset (CMD 0xD304), wait 100 ms.
 *    3. Start measurement (CMD 0x0010 + parameter 0x03 0x00 = float mode +
 *       CRC), wait 30 ms for first conversion.
 *    4. Mark driver ready.
 *
 *  Returns ESP_OK on success, ESP_ERR_NOT_FOUND if no device responds at
 *  0x69, or another esp_err_t on bus failure.
 */
esp_err_t sps30_init(i2c_master_bus_handle_t bus);

/** @brief True if init succeeded and continuous measurement is running. */
bool sps30_present(void);

/** @brief Read one measurement record from the sensor.
 *
 *  Polls the data-ready flag for up to 1 s (100 ms intervals) before
 *  reading the 60-byte float-mode payload. CRC8 is verified on every
 *  word — any mismatch returns ESP_FAIL without populating *out.
 */
esp_err_t sps30_read(pm_sample_t *out);

/** @brief Manually trigger the fan auto-clean cycle.
 *
 *  Continuous mode runs auto-clean automatically at the factory-default
 *  7-day interval — this command is exposed for diagnostic / forced-clean
 *  use only. Fan ramps to max for ~10 s; measurements during that window
 *  are unreliable.
 */
esp_err_t sps30_start_fan_cleaning(void);

/** @brief Read the SPS30 device-status register (CMD 0xD206).
 *
 *  Returns the raw 32-bit status word in *status_out. Caller decodes the
 *  bit fields:
 *    bit 4  — fan failure (RPM out of target for >10 s, fan blocked)
 *    bit 5  — laser current out of regulation
 *    bit 21 — fan speed warning (drift, not yet failed)
 *  Other bits are reserved by Sensirion.
 *
 *  Returns ESP_OK on success, ESP_FAIL on I²C / CRC error.
 */
esp_err_t sps30_read_device_status(uint32_t *status_out);
