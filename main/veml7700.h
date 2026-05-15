#pragma once

/** @file
 *  @brief V2.3.30: Vishay VEML7700 ambient-light sensor driver (I²C, 0x10).
 *
 *  Adafruit STEMMA QT breakout (PID 4162) — single fixed I²C address
 *  0x10. Two-channel sensor: ALS (filtered to match human-eye photopic
 *  response) + white-light. Lux is computed from raw ALS counts using
 *  a resolution that depends on configured gain + integration time,
 *  with a polynomial non-linearity correction at high light levels.
 *
 *  Driver uses fixed defaults (gain 1/8×, IT 100 ms → 0.54 lux/count,
 *  range ~0–35 klux) — good general-purpose setting that handles
 *  indoor through outdoor-shade light without needing auto-gain logic.
 *  See `reference_veml7700.md` memory for the full design rationale.
 *
 *  ALS-PT19 (analog, FeatherS3-D onboard) and VEML7700 (I²C, external)
 *  can coexist — both will appear in the /status "Ambient light" block
 *  if present. VEML7700 is the more accurate of the two; ALS-PT19 is
 *  the always-there fallback.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** @brief Probe + initialise the VEML7700 on the given bus.
 *
 *  Returns ESP_OK if a VEML7700 ACK'd at 0x10 and was configured for
 *  the default operating mode. Returns ESP_ERR_NOT_FOUND if no ACK
 *  (caller can move on or call again with a different bus). Other
 *  errors propagate from i2c_master_bus_add_device or config writes.
 *
 *  Idempotent: safe to call multiple times; subsequent calls return
 *  ESP_OK without re-probing if already initialised.
 */
esp_err_t veml7700_init(i2c_master_bus_handle_t bus);

/** @brief True if a VEML7700 was successfully initialised. */
bool veml7700_present(void);

/** @brief Take one measurement.
 *  @param raw_als   16-bit raw ALS count from register 0x04. NULL to skip.
 *  @param raw_white 16-bit raw white-channel count from register 0x05.
 *                   NULL to skip.
 *  @param lux       Computed lux value (resolution × als_count, with
 *                   non-linearity polynomial correction). NULL to skip.
 *  @return ESP_OK on success, ESP_FAIL if not present.
 */
esp_err_t veml7700_read(uint16_t *raw_als, uint16_t *raw_white, float *lux);
