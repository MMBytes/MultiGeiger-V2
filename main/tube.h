#pragma once

/** @file
 *  @brief Geiger-Müller tube driver — HV generation and pulse counting.
 *
 *  Runs two interrupt-driven paths (pin map per `hal.h`):
 *    - A 100 µs gptimer drives the HV charge-pump state machine (pulse the
 *      FET until the cap-full comparator fires, then idle until next charge).
 *    - A GPIO edge ISR on PIN_GMC_COUNT_INPUT tallies tube pulses, rejecting
 *      double-counts inside GMC_DEAD_TIME_US (no Schmitt on the input pin).
 *
 *  Consumers poll tube_read() to snapshot and reset the window accumulators.
 *  An optional per-pulse callback fires from ISR context for speaker/LED tick.
 */

#include <stdbool.h>
#include <stdint.h>

#include "hal.h"   // PIN_HV_FET_OUTPUT, PIN_HV_CAP_FULL_INPUT, PIN_GMC_COUNT_INPUT

// Si22G dead time (µs). Rejects rising-edge noise on the count input.
#define GMC_DEAD_TIME_US 190

/** @brief Configure GPIOs, install ISRs, and start the recharge timer.
 *
 *  @param enabled  When false, configures HV_FET as a static LOW output
 *                  (keeps the boost MOSFET off) and skips ISR install +
 *                  gptimer setup entirely. tube_read() then returns zeros.
 *                  Lets the firmware run as a non-Geiger node without the
 *                  HV charge pump cycling.
 */
void tube_setup(bool enabled);

/** @brief Snapshot and reset the accumulators.
 *  @param counts_delta  Out: new pulses since the last call.
 *  @param dt_ms         Out: wall-clock ms since the last call (for CPM).
 *  @param min_us        Out: min inter-pulse gap in the window (UINT32_MAX if no data).
 *  @param max_us        Out: max inter-pulse gap in the window (0 if no data).
 *  @param hv_pulses     Out: cumulative HV charge pulses since boot.
 *  @param hv_error      Out: true if the last charge cycle hit MAX_CHARGE_PULSES
 *                       (indicates tube vacuum failure or HV fault).
 *
 *  When the tube is disabled (tube_setup(false)), all outputs are zeroed
 *  except dt_ms, which still tracks the wall-clock window so callers can
 *  compute rates from other sensors against a known interval.
 */
void tube_read(uint32_t *counts_delta, uint32_t *dt_ms,
               uint32_t *min_us, uint32_t *max_us,
               uint32_t *hv_pulses, bool *hv_error);

/** @brief True if tube_setup(true) was called at boot. */
bool tube_is_enabled(void);

/** @brief Callback fired from the GMC pulse ISR when a valid pulse is counted.
 *
 *  Runs in IRAM/ISR context — the function must be IRAM_ATTR and must not
 *  block or touch non-ISR-safe APIs. Pass NULL to unregister.
 */
typedef void (*tube_pulse_cb_t)(void);
void tube_set_pulse_callback(tube_pulse_cb_t cb);
