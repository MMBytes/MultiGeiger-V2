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

// V2.6.9: coincidence window (µs after the START of an HV charge pulse,
// see tube.c's recharge_tick S_PULSE_H) used to tag a counted edge as
// HV-correlated rather than a genuine tube pulse. Lower bound clears the
// ~190µs dead-time gate itself plus the FET's own 1500µs H-phase so a
// pulse right on the gate edge isn't misattributed; upper bound covers the
// 2.5ms (1500+1000µs) full charge-pulse period plus margin — the field
// evidence (radiation_overcounting_independent_review.md) shows spurious
// counts landing ~2.5-3ms after HV activity starts, matching the firmware's
// own 2-pulse charge-train spacing.
#define HV_COINCIDENT_MIN_US 200
#define HV_COINCIDENT_MAX_US 3000

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
 *  190µs gate is too short for). 0 = OFF (default; legacy behaviour). Applied
 *  LIVE — the ISR reads the window each edge, so /config Save takes effect
 *  immediately (no reboot, unlike the hardware-latched PCNT filter); also set at
 *  boot. Diagnostic only — it alters the dead-time loss; see config_fields.def.
 */
void tube_set_guard_us(uint32_t guard_us);

/** @brief V2.6.29: set the opt-in HV blanking window (µs). 0 = OFF (default).
 *
 *  Phantom-pulse suppression for the Rev B/C PCB coupling defect: each HV
 *  charge pulse couples one phantom NEGEDGE into GMZ_COUNT ~10 µs after FET
 *  turn-off (the first L1 flyback ring-down trough — see
 *  reference_radiation_data_analysis 2026-07-21/26). Once set, a would-be
 *  count landing within @p blank_us of the most recent FET turn-off (stamped
 *  every pulse in recharge_tick's S_PULSE_L, so mid-train and boot-train
 *  pulses are covered too) is dropped and tallied as `hv_blanked` instead.
 *  Keyed on the CAUSE (the recharge state machine's own clock), so unlike the
 *  4 µs PCNT width filter it is immune to the coupled pulse's width varying
 *  with MCU input stage or hardware mods. Dead-time cost ~= hv_pulses ×
 *  blank_us per cycle (a few hundred µs per 3-min cycle — ppm-level).
 *  Applied LIVE like the guard (ISR re-reads the volatile each edge); also
 *  set at boot. Steps CPM down by the node's phantom share (~6-9% on
 *  affected boards) — archive-continuity decision, hence opt-in; see
 *  config_fields.def.
 */
void tube_set_blank_us(uint32_t blank_us);

/** @brief V2.6.29: monotonic total of blanked would-be counts since boot.
 *
 *  Twin of tube_get_total_counts() (never reset; callers take their own
 *  deltas, unsigned wrap-safe; 0 while the tube or blanking is off). Exists
 *  for the PCNT subtract mode: when pcnt_filter is authoritative, phantoms
 *  wide enough to pass the width filter are still inside the PCNT hardware
 *  count, which ISR-side blanking cannot reach — so the per-minute history
 *  sampler (history.c) subtracts this total's delta from the filtered total,
 *  and do_tx_cycle subtracts the per-cycle hv_blanked from the filtered
 *  cycle count. ONLY correct on boards whose phantom pulses pass the filter
 *  (S3 family, coupled pulses >= 4 µs); see config_fields.def hv_blank.
 */
uint32_t tube_get_blanked_total(void);

/** @brief V2.5.12: snapshot + reset the raw-edge count profiler.
 *
 *  Permanent diagnostic on the Geiger count path, emitted as the per-cycle
 *  DIAG log line — distinguishes clean counting from count-node ringing/noise.
 *  @param raw_edges     Out: every GMC ISR entry since the last call, counted
 *                       BEFORE the dead-time gate. raw_edges - counts isolates
 *                       edges suppressed as ringing/double-counts (< GMC_DEAD_TIME_US).
 *  @param guard_removed Out: V2.5.30 — REAL counts the optional dead-time guard
 *                       suppressed since the last call: only edges that would
 *                       have cleared the 190µs gate, so it's the guard's true
 *                       marginal effect and a subset of (raw_edges - counts).
 *                       0 when the guard is off.
 *  @param hv_coincident Out: V2.6.9 — counted edges (i.e. included in `counts`,
 *                       NOT a subset of raw_edges-counts) whose gap since the
 *                       START of the most recent HV charge pulse fell inside
 *                       [HV_COINCIDENT_MIN_US, HV_COINCIDENT_MAX_US]. Tests the
 *                       HV-recharge-coupling hypothesis in
 *                       radiation_overcounting_independent_review.md: a real
 *                       Poisson tube shows this tracking background rate
 *                       (near-zero at typical HV cadence); electrical pickup
 *                       from the charge pump shows it tracking hv_pulses 1:1.
 *  @param hv_blanked    Out: V2.6.29 — would-be COUNTS the opt-in HV blanking
 *                       window suppressed since the last call (its true
 *                       marginal effect: counts_without_blank = counts +
 *                       hv_blanked; NOT included in `counts`, NOT a subset of
 *                       guard_removed/rejected). With blanking active on an
 *                       affected board, expect hv_blanked ~= hv_pulses per
 *                       cycle while hv_coincident collapses toward 0 (the
 *                       phantoms stop being counted). 0 when off.
 *  @param hist          Out: edge-to-edge spacing histogram, TUBE_DIAG_NBUCKETS bins
 *                       (<50, <190, <500, <1k, <5k, <50k, <500k, >=500k µs). Real
 *                       ~1.2 cps pulses land in the top two bins; a fat low-bin
 *                       population is count-node ringing/noise.
 */
void tube_get_diag(uint32_t *raw_edges, uint32_t *guard_removed,
                   uint32_t *hv_coincident, uint32_t *hv_blanked,
                   uint32_t hist[TUBE_DIAG_NBUCKETS]);

/** @brief Callback fired from the GMC pulse ISR when a valid pulse is counted.
 *
 *  Runs in IRAM/ISR context — the function must be IRAM_ATTR and must not
 *  block or touch non-ISR-safe APIs. Pass NULL to unregister.
 */
typedef void (*tube_pulse_cb_t)(void);
void tube_set_pulse_callback(tube_pulse_cb_t cb);
