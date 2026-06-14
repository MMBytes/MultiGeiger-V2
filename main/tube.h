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

// V2.5.12: number of edge-to-edge spacing buckets for the raw-edge count
// profiler (see tube_get_diag) — lets a clean Poisson count path be told
// apart from count-node ringing/noise.
#define TUBE_DIAG_NBUCKETS 8

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

/** @brief Monotonic count of valid tube pulses since boot (never reset).
 *
 *  V2.5.6: separate from the tube_read() window accumulator. Lets a sampler
 *  (history.c) take its own fixed-interval deltas without disturbing the TX
 *  cycle's destructive tube_read(). Returns 0 while the tube is disabled (the
 *  count ISR is never installed). Unsigned wrap is safe for bounded deltas.
 */
uint32_t tube_get_total_counts(void);

/** @brief V2.5.30: set the opt-in retriggerable dead-time-guard window (µs).
 *
 *  Layered on top of the fixed GMC_DEAD_TIME_US gate: once set, an edge can
 *  start a new count only after a quiet gap longer than @p guard_us; edges
 *  inside the window extend the dead zone and are dropped, collapsing an
 *  afterpulse / re-trigger TRAIN to a single count (reaches the 1-5ms band the
 *  190µs gate is too short for). 0 = OFF (default; legacy behaviour). Set once
 *  at boot from config (reboot-required, like the PCNT filter). Diagnostic only
 *  — it alters the dead-time loss; see config_fields.def / tube.c.
 */
void tube_set_guard_us(uint32_t guard_us);

/** @brief V2.5.12: snapshot + reset the raw-edge count profiler.
 *
 *  Permanent diagnostic on the Geiger count path, emitted as the per-cycle
 *  DIAG log line — distinguishes clean counting from count-node ringing/noise.
 *  @param raw_edges     Out: every GMC ISR entry since the last call, counted
 *                       BEFORE the dead-time gate. raw_edges - counts isolates
 *                       edges suppressed as ringing/double-counts (< GMC_DEAD_TIME_US).
 *  @param guard_removed Out: V2.5.30 — edges suppressed by the optional dead-time
 *                       guard since the last call (0 when the guard is off).
 *  @param hist          Out: edge-to-edge spacing histogram, TUBE_DIAG_NBUCKETS bins
 *                       (<50, <190, <500, <1k, <5k, <50k, <500k, >=500k µs). Real
 *                       ~1.2 cps pulses land in the top two bins; a fat low-bin
 *                       population is count-node ringing/noise.
 */
void tube_get_diag(uint32_t *raw_edges, uint32_t *guard_removed,
                   uint32_t hist[TUBE_DIAG_NBUCKETS]);

/** @brief Callback fired from the GMC pulse ISR when a valid pulse is counted.
 *
 *  Runs in IRAM/ISR context — the function must be IRAM_ATTR and must not
 *  block or touch non-ISR-safe APIs. Pass NULL to unregister.
 */
typedef void (*tube_pulse_cb_t)(void);
void tube_set_pulse_callback(tube_pulse_cb_t cb);
