#pragma once

/** @file
 *  @brief Geiger-Müller tube driver — HV generation and pulse counting.
 *
 *  Runs two interrupt-driven paths on Heltec Wireless Stick V2:
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

// Heltec Wireless Stick V2 pin map.
#define PIN_HV_FET_OUTPUT     23
#define PIN_HV_CAP_FULL_INPUT 22
#define PIN_GMC_COUNT_INPUT    2

// Si22G dead time (µs). Rejects rising-edge noise on the count input.
#define GMC_DEAD_TIME_US 190

/** @brief Configure GPIOs, install ISRs, and start the recharge timer. */
void tube_setup(void);

/** @brief Snapshot and reset the accumulators.
 *  @param counts_delta  Out: new pulses since the last call.
 *  @param dt_ms         Out: wall-clock ms since the last call (for CPM).
 *  @param min_us        Out: min inter-pulse gap in the window (UINT32_MAX if no data).
 *  @param max_us        Out: max inter-pulse gap in the window (0 if no data).
 *  @param hv_pulses     Out: cumulative HV charge pulses since boot.
 *  @param hv_error      Out: true if the last charge cycle hit MAX_CHARGE_PULSES
 *                       (indicates tube vacuum failure or HV fault).
 */
void tube_read(uint32_t *counts_delta, uint32_t *dt_ms,
               uint32_t *min_us, uint32_t *max_us,
               uint32_t *hv_pulses, bool *hv_error);

/** @brief Callback fired from the GMC pulse ISR when a valid pulse is counted.
 *
 *  Runs in IRAM/ISR context — the function must be IRAM_ATTR and must not
 *  block or touch non-ISR-safe APIs. Pass NULL to unregister.
 */
typedef void (*tube_pulse_cb_t)(void);
void tube_set_pulse_callback(tube_pulse_cb_t cb);
