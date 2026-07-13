#pragma once

/** @file
 *  @brief V2.6.15: thin native ESP-IDF driver for the Sensirion SGP41
 *         multi-pixel VOC+NOx gas sensor (Adafruit breakout, SKU 6455).
 *
 *  Fixed I2C address 0x59 — no fleet-wide conflicts, auto-detected on
 *  whichever board it's physically wired to (no HAL_HAS_* gating needed).
 *
 *  We only surface the NOx index — the project's TX targets have no field
 *  for either VOC or NOx, so this is log/status/MQTT-HA only. The wire
 *  protocol always returns VOC and NOx raw signals together in one
 *  response; the VOC half is read (it has to be) and discarded.
 *
 *  Unlike every other sensor in this codebase, SGP41 needs a dedicated
 *  background task rather than a per-TX-cycle read: Sensirion's Gas Index
 *  Algorithm (vendored in sensirion_gas_index_algorithm.c/h) must be fed a
 *  raw sample at a fixed ~1 Hz cadence to produce a correctly-calibrated
 *  index, wildly faster than our ~150 s TX cycle. sgp41_init() starts this
 *  task; main.c and the TX/MQTT/status-page paths just read the latest
 *  cached result via sgp41_get_nox_index().
 *
 *  On first boot the sensor requires a 10 s x 1 Hz conditioning sequence
 *  before its first real measurement — handled internally by the
 *  background task, transparent to callers. The NOx index itself reads
 *  as 0 (not yet valid) for the algorithm's 45 s initial-blackout period
 *  after that, then 1..500 afterwards, growing more stable over several
 *  hours as the algorithm's mean/variance estimators mature.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** @brief Probe the bus for an SGP41, verify its serial number, and — if
 *         found — start the background ~1 Hz sampling task.
 *
 *  Always returns ESP_OK; sensor absence is non-fatal. Check
 *  sgp41_present() to know whether NOx data is available. The bus must
 *  already be up.
 */
esp_err_t sgp41_init(i2c_master_bus_handle_t bus);

/** @brief True if an SGP41 was detected and the sampling task started. */
bool sgp41_present(void);

/** @brief Most recent NOx index (1..500, higher = more NOx-family gases).
 *
 *  Safe to call from any task — just reads a mutex-protected cache written
 *  by the background sampling task. Returns ESP_FAIL if not present, still
 *  conditioning, or still in the algorithm's initial blackout period (no
 *  valid index produced yet).
 */
esp_err_t sgp41_get_nox_index(int32_t *out);
