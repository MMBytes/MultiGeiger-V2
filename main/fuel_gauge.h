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
 *  V2.6.6 CORRECTION: fuel_gauge_present()'s VCELL threshold (present >
 *  2000 mV) was designed around an assumed ~0V no-battery reading
 *  (CHANGELOG V2.4.28). Real bench testing (2026-07-03, two FeatherS3-D
 *  units, confirmed no LiPo attached) showed VCELL sitting at 4.2-4.4V
 *  instead — the onboard LiPo charger IC's output floats up near its
 *  ~4.2V regulation setpoint when USB power is present and unloaded
 *  (no battery to servo against). That's indistinguishable from a real
 *  battery by voltage alone, so fuel_gauge_present() is UNRELIABLE
 *  whenever VBUS is present (the common case — most nodes are USB-powered
 *  with no LiPo). It's kept for now (still gates /status, mqtt.c,
 *  mqtt_discovery.c) but should not be trusted as ground truth; see
 *  `project_fuel_gauge_max17048_design.md` memory for the full trail.
 *
 *  fuel_gauge_ready() is the honest signal for "does this board have the
 *  physical chip" (I2C ACK'd, driver initialised) — independent of whether
 *  a real battery is attached. Use this to gate output that should always
 *  show power-supply data on a FeatherS3-D regardless of battery presence
 *  (e.g. the per-TX-cycle log line, which reports raw VCELL/SoC/rate plus
 *  VBUS state rather than claiming to know if a battery is really there).
 *
 *  Compiled out on boards without HAL_HAS_FUEL_GAUGE — every bool-returning
 *  function here returns false and every caller already treats that as
 *  "skip this block", so no board-specific #ifdef is needed at any call site.
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

/** @brief True if the MAX17048 ACK'd at init and is ready to read — i.e.
 *  this board physically has the chip. Does NOT imply a battery is
 *  attached (see file header) — just that VCELL/SoC/CRATE reads are
 *  meaningful register reads rather than a no-op stub. Always false on
 *  boards without HAL_HAS_FUEL_GAUGE.
 */
bool fuel_gauge_ready(void);

/** @brief True if VBUS (USB 5V) is present, per the dedicated digital
 *  detect pin. Always true while running on USB power (the ESP32-S3 can't
 *  execute code on LiPo-only without it); reads false only when running
 *  purely off battery with USB unplugged. Always false on boards without
 *  HAL_HAS_FUEL_GAUGE.
 */
bool fuel_gauge_vbus_present(void);

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
