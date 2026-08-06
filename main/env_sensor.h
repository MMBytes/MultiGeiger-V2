#pragma once

/** @file
 *  @brief Unified environmental sensor interface — SHT45 plus a Bosch
 *         pressure/THP chip (BMP581 / BMP390 / BME688 / BME280).
 *
 *  Detection order on init:
 *    1. SHT45  at 0x44       — temperature + humidity (±0.1 °C, ±1 % RH)
 *    2. BMP581 at 0x46/0x47  — pressure + temperature (±0.4 Pa relative,
 *                              0.21 PaRMS noise) — independent of 0x77 family
 *    3. BMP390 at 0x77       — pressure + temperature (±0.5 hPa)
 *    4. BME688 at 0x77       — temperature + humidity + pressure (only if
 *                              BMP390 didn't claim 0x77)
 *    5. BME280 at 0x76 or 0x77 — legacy fallback (only 0x76 if a previous
 *                              0x77-family chip claimed 0x77)
 *
 *  V2.3.29: bus ownership moved out to `i2c_bus.c`. This module is now a
 *  pure consumer — it accepts a bus handle in env_sensor_init() and runs
 *  the cascade probe on it. main.c orchestrates which bus(es) to try.
 *  Same module can be re-init'd against a second bus if the first call
 *  found nothing (idempotent at the sub-driver level: each sht45_init /
 *  bmp*_init / bme*_init returns early if already bound).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** @brief Probe for env sensors on the given I²C bus, in priority order.
 *
 *  Always returns ESP_OK — sensor absence is non-fatal. Check
 *  env_sensor_present() to know whether any THP data is available.
 *
 *  Caller (main.c) typically calls once with the primary bus; if no
 *  sensor was found, may call again with the secondary bus. Sub-driver
 *  inits are idempotent so a second call against a different bus
 *  re-probes cleanly.
 */
esp_err_t env_sensor_init(i2c_master_bus_handle_t bus);

/** @brief True if at least one sensor capable of temperature reading is present. */
bool env_sensor_present(void);

/** @brief Per-field driver-presence predicates.
 *
 *  V2.4.12: needed by MQTT HA Discovery so it can register the env_t /
 *  env_h / env_p entities independently. Pre-V2.4.12 all three were gated
 *  on `env_sensor_present()` (= "any env sensor"), which caused HA to
 *  show a phantom 0.00 hPa pressure entity on SHT45-only setups (SHT45
 *  has no pressure channel). Boot-time, fixed at driver-detect — does
 *  not reflect per-cycle read failures.
 *
 *    env_t_present  → SHT45 || BMP581 || BMP390 || BME688 || BME280
 *    env_h_present  → SHT45 || BME688 || BME280
 *    env_p_present  → BMP581 || BMP390 || BME688 || BME280
 */
bool env_t_present(void);
bool env_h_present(void);
bool env_p_present(void);

/** @brief Read one set of compensated measurements.
 *
 *  Any pointer may be NULL. Fills from the highest-accuracy source available:
 *    temperature — SHT45 > BMP390 > BME280
 *    humidity    — SHT45 > BME280 (BMP390 has no humidity channel)
 *    pressure    — BMP390 > BME280 (SHT45 has no pressure channel)
 *
 *  Units: temperature °C, humidity %RH, pressure Pa.
 *  Returns ESP_OK on success; ESP_FAIL if no sensor is ready or read failed.
 *
 *  V2.4.12: `have_t / have_h / have_p` (each NULL-able) report per-field
 *  validity for THIS read — distinct from `env_*_present()` which reports
 *  driver presence at boot. A SHT45-only setup that reads OK gives
 *  have_t=have_h=true, have_p=false; a multi-chip setup where SHT45's H
 *  read failed mid-cycle gives have_t=true, have_h=false, have_p=true.
 *  Callers needing field-level publish gating (mqtt.c) use these instead
 *  of the all-or-nothing return code.
 *
 *  V2.3.26: optional `raw_log` buffer (NULL to skip) is filled with one comma-
 *  separated segment per present-and-called sensor, e.g.
 *    "SHT45: T=18.86°C  H=0.00%, BMP390: T=18.86°C P=1026.60hPa"
 *  Failed reads emit "<NAME>: read failed". Trailing ", " is stripped so the
 *  caller can append " <fused>" with one space. Suggested cap: 160 bytes.
 */
esp_err_t env_sensor_read(float *temperature_c, float *humidity_pct,
                          float *pressure_pa,
                          bool *have_t, bool *have_h, bool *have_p,
                          char *raw_log, size_t raw_log_cap);

/** @brief Short human-readable label describing the active sensor combination.
 *
 *  Examples: "SHT45+BMP390", "SHT45", "BMP390", "BME280". Never NULL.
 */
const char *env_sensor_name(void);

/** @brief Call once per measurement cycle to conditionally fire the SHT45
 *         built-in heater (see sht45.h for policy details).
 *
 *  `now_ms` must be esp_timer_get_time()/1000 — sht45_read() compares the
 *  heater blackout deadline against that clock, so a tick-derived value
 *  would silently mis-time the window.
 */
void env_sensor_heat_periodic(uint32_t now_ms, float humidity_pct);

/** @brief Standalone-SD-logging only: read every PRESENT sub-driver's own
 *  T/H/P this cycle, bypassing env_sensor_read()'s fused priority
 *  short-circuit (final review A3).
 *
 *  The standalone CSV gives every attached env chip its own column (spec
 *  §4.1) — but env_sensor_read() skips a chip once an earlier one already
 *  satisfied its fields (e.g. BME280 skipped once SHT45+BMP581 already cover
 *  T/H/P), and a chip's telemetry cache is only ever refreshed inside its
 *  own `*_read()`. Left alone, a skipped chip's CSV column stays empty
 *  forever, or worse, freezes on a stale reading from the one cycle its
 *  higher-priority sibling happened to fail. This call reads every present
 *  chip unconditionally so every registered telemetry column is fresh every
 *  cycle. Return values are discarded — only the driver-internal cache
 *  (read by the telemetry callbacks) matters here; the fused values used by
 *  the networked path come from env_sensor_read() alone, untouched.
 *
 *  Intentionally a second full I²C pass, standalone-only, called once per
 *  TX cycle from the main task (150 s cadence) — acceptable per spec;
 *  telemetry read callbacks themselves must still never issue fresh I²C.
 */
void env_sensor_refresh_all_for_telemetry(void);
