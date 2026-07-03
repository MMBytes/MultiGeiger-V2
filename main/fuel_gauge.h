#pragma once

/** @file
 *  @brief V2.6.6: MAX17048 battery fuel-gauge driver (I²C, 0x36, FeatherS3-D only).
 *
 *  The FeatherS3-D (the "[D]" revision — distinct from plain FeatherS3/
 *  TinyS3/ProS3, which use an ADC-divider VBAT pin instead) has an onboard
 *  MAX17048 wired to the same pins as the STEMMA1 connector (GPIO8 SDA /
 *  GPIO9 SCL — the primary I²C bus). That bus is powered from the always-on
 *  3.3V LDO1 rail, independent of the battery, so the chip ACKs and
 *  free-runs on I²C regardless of whether a LiPo is attached — no init
 *  register writes needed.
 *
 *  Battery presence is NOT a config toggle — it's auto-detected from VCELL
 *  with a hysteresis band (present > 2000 mV, absent < 1500 mV). See
 *  `project_fuel_gauge_max17048_design.md` memory and
 *  `docs/superpowers/specs/2026-07-03-fuel-gauge-max17048-design.md` for
 *  the full threshold derivation: an empirical ~0V no-battery reading
 *  (CHANGELOG V2.4.28) sits far below the ~2.4V lowest voltage a real,
 *  physically-connected cell will ever show (LiPo protection-IC
 *  over-discharge cutoff), leaving a wide safe margin on both sides.
 *
 *  Compiled out on boards without HAL_HAS_FUEL_GAUGE — fuel_gauge_present()
 *  returns false and every caller (`/status`, mqtt.c, mqtt_discovery.c,
 *  transmission.c) already treats "not present" as "skip this block", so
 *  no board-specific #ifdef is needed at any call site.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** @brief Probe + initialise the MAX17048 on the given bus.
 *
 *  Returns ESP_OK if the chip ACK'd at 0x36 (no register writes needed —
 *  it free-runs once powered). Returns ESP_ERR_NOT_FOUND if no ACK.
 *  No-op (returns ESP_OK without touching the bus) on boards without
 *  HAL_HAS_FUEL_GAUGE.
 *
 *  Idempotent: safe to call multiple times; subsequent calls return
 *  ESP_OK without re-probing if already initialised.
 */
esp_err_t fuel_gauge_init(i2c_master_bus_handle_t bus);

/** @brief True if the chip is initialised AND the latest VCELL reading
 *  crossed into the "battery present" side of the hysteresis band (see
 *  file header for the exact thresholds). Reads VCELL once per call —
 *  cheap (~sub-ms at 400 kHz), safe to call every /status load and every
 *  TX cycle. Always false on boards without HAL_HAS_FUEL_GAUGE (no I²C
 *  traffic at all in that case).
 */
bool fuel_gauge_present(void);

/** @brief Read voltage / state-of-charge / charge-rate. Pointers may be
 *  NULL to skip that field.
 *  @param volts           Cell voltage in volts (78.125 uV/LSB raw scale).
 *  @param soc_pct         State of charge, 0-100+ % (1/256 %/LSB raw scale).
 *  @param rate_pct_per_hr Charge rate, %/hr, signed: positive = charging,
 *                         negative = discharging (0.208 %/hr/LSB raw scale).
 *  @return ESP_OK on success, ESP_FAIL if not present/initialised.
 */
esp_err_t fuel_gauge_read(float *volts, float *soc_pct, float *rate_pct_per_hr);
