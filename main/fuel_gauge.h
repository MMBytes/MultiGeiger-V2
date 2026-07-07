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
 *  V2.6.6 CORRECTION: fuel_gauge_present()'s original VCELL threshold
 *  (present > 2000 mV) was designed around an assumed ~0V no-battery reading
 *  (CHANGELOG V2.4.28). Real bench testing (2026-07-03, two FeatherS3-D
 *  units, confirmed no LiPo attached) showed VCELL sitting at 4.2-4.4V
 *  instead — the onboard LiPo charger IC's output floats up near its
 *  ~4.2V regulation setpoint when USB power is present and unloaded
 *  (no battery to servo against). That's indistinguishable from a real
 *  battery by voltage alone whenever VBUS is present (the common case —
 *  most nodes are USB-powered with no LiPo), so no voltage-only heuristic
 *  can be made reliable.
 *
 *  V2.6.6 FIX: fuel_gauge_present() is now driven by the `batt_present`
 *  config bool (set via the /config "Battery attached" checkbox,
 *  HAL_HAS_FUEL_GAUGE boards only) instead of a VCELL guess. This is a
 *  deliberate manual override, not a step back from auto-detection — no
 *  digital "battery attached" signal exists on this board's wiring (no
 *  charger-IC status pin reaches a GPIO), so a config bool is the only way
 *  to get a definite answer. See `project_fuel_gauge_max17048_design.md`
 *  memory for the full trail.
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
 *  Idempotent: safe to call multiple times. A second call is a hard no-op
 *  — it returns ESP_OK immediately without re-probing the chip, re-adding
 *  the I2C device, or reconfiguring PIN_VBUS_DETECT; it does not refresh
 *  or re-validate any state.
 */
esp_err_t fuel_gauge_init(i2c_master_bus_handle_t bus);

/** @brief True if VBUS (USB 5V) is present, per the dedicated digital
 *  detect pin. True while running on USB power, false when running purely
 *  off battery with USB unplugged — but only once fuel_gauge_init() has
 *  succeeded; if the MAX17048 never ACK'd at init, this returns false
 *  unconditionally regardless of actual VBUS state (the GPIO read is gated
 *  on the same "ready" flag as the I2C-backed functions, since the pin is
 *  only configured inside a successful fuel_gauge_init()). Always false on
 *  boards without HAL_HAS_FUEL_GAUGE.
 */
bool fuel_gauge_vbus_present(void);

/** @brief True if the chip is ready AND the user has confirmed (via the
 *  /config "Battery attached" checkbox, `batt_present`) that a real LiPo is
 *  plugged in. No voltage heuristic — see file header for why VCELL alone
 *  can't tell this apart from a floating no-battery reading. Always false
 *  on boards without HAL_HAS_FUEL_GAUGE.
 */
bool fuel_gauge_present(void);

/** @brief Set the user-confirmed battery-presence flag (mirrors the
 *  `batt_present` config bool). Call once at boot after fuel_gauge_init(),
 *  and again on every live /config Save so a checkbox change takes effect
 *  without a reboot. No-op on boards without HAL_HAS_FUEL_GAUGE.
 */
void fuel_gauge_set_user_present(bool present);

/** @brief Read voltage / state-of-charge / charge-rate. Pointers may be
 *  NULL to skip that field.
 *  @param volts           Cell voltage in volts (78.125 uV/LSB raw scale).
 *  @param soc_pct         State of charge, 0-100+ % (1/256 %/LSB raw scale).
 *  @param rate_pct_per_hr Charge rate, %/hr, signed: positive = charging,
 *                         negative = discharging (0.208 %/hr/LSB raw scale).
 *  @return ESP_OK on success, ESP_FAIL if not present/initialised.
 */
esp_err_t fuel_gauge_read(float *volts, float *soc_pct, float *rate_pct_per_hr);

/** @brief Read raw VERSION/STATUS registers for diagnostic logging.
 *  Pointers may be NULL to skip that field.
 *  @param version Chip/firmware version register (0x08), raw 16-bit value.
 *                  This is the register Adafruit's own MAX1704x libraries
 *                  use as their sole "battery attached" sentinel (0xFFFF =
 *                  no response) — see `project_fuel_gauge_max17048_design.md`
 *                  memory for why that check doesn't transfer to this board.
 *  @param status  Alert-flags register (0x1A): bit0 RI (reset indicator),
 *                  bit1 VH, bit2 VL, bit3 VR, bit4 HD, bit5 SC.
 *  @return ESP_OK on success, ESP_FAIL if not present/initialised.
 */
esp_err_t fuel_gauge_read_diag(uint16_t *version, uint8_t *status);
