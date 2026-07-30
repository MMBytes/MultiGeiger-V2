#pragma once

/** @file
 *  @brief Pure decision logic for the GMC count path — no IDF/FreeRTOS/HW.
 *
 *  Header-only and dependency-free (only <stdint.h>/<stdbool.h>) for the
 *  same reason as util.h: the host-side test runner under `test/` can
 *  include it directly and exercise the branchy counting/guard logic that
 *  otherwise lives buried in the IRAM count ISR (gmc_count_isr in tube.c),
 *  where it is impossible to unit-test on a host.
 *
 *  V2.5.31 (CI hardening): extracted the dead-time-guard decision out of the
 *  ISR into gmc_classify() so the three-way outcome (count / guard-removed /
 *  reject) — which took two MAX code reviews to get right in V2.5.30 — is
 *  pinned by host tests. The functions are `always_inline` so they fold into
 *  the IRAM_ATTR ISR at zero call cost and stay in IRAM (the ISR runs with
 *  the flash cache disabled and must not call into flash).
 */

#include <stdint.h>
#include <stdbool.h>

/** @brief Saturating uint64 → uint32 cast for µs timestamp deltas.
 *
 *  now-X deltas are uint64 µs; a gap over 2^32 µs (~71.6 min) overflows a
 *  bare uint32 cast and wraps to a small value — mis-binning the edge and
 *  (worse) tricking the dead-time guard into blocking a genuine well-separated
 *  pulse. Clamp to UINT32_MAX so a long quiet gap always reads as "maximally
 *  separated". Moved here from tube.c in V2.5.31 so it is host-testable.
 */
__attribute__((always_inline)) static inline uint32_t clamp_u32(uint64_t v) {
    return v > UINT32_MAX ? UINT32_MAX : (uint32_t)v;
}

/** @brief Outcome of classifying one GMC tube edge in the count ISR. */
typedef enum {
    GMC_COUNT,          /**< Real pulse: increment counts; becomes new "last counted". */
    GMC_GUARD_REMOVED,  /**< Would have counted, but the dead-time guard blocked it. */
    GMC_REJECT,         /**< Rejected by the fixed 190 µs dead-time gate (sub-gate edge). */
} gmc_class_t;

/** @brief Classify one tube edge given timing already computed by the ISR.
 *
 *  Pure restatement of the V2.5.30 ISR decision, split out so it is testable:
 *
 *  @param edt        Gap (µs) since the PREVIOUS edge (counted or not), clamped;
 *                    UINT32_MAX for the first-ever edge so it can never be guarded.
 *  @param past_gate  True if this edge clears the fixed dead-time gate
 *                    (last==0 OR gap-since-last-COUNTED-pulse > GMC_DEAD_TIME_US).
 *  @param guard_us   Retriggerable refractory window; 0 disables the guard.
 *
 *  The guard keys on @p edt (the previous EDGE, not the previous count) so it is
 *  retriggerable: every edge in a burst keeps the channel dead, collapsing an
 *  afterpulse / re-trigger train to its single leading count.
 *
 *  - guard blocks + past_gate  → GMC_GUARD_REMOVED (a real count the guard cost us)
 *  - guard blocks + !past_gate → GMC_REJECT (sub-gate edge, already owned by the gate)
 *  - no guard     + past_gate  → GMC_COUNT
 *  - no guard     + !past_gate → GMC_REJECT
 *
 *  So counts_without_guard == counts + guard_removed, and guard_removed is a clean
 *  subset of the DIAG `rejected` total (raw_edges - counts).
 */
__attribute__((always_inline)) static inline gmc_class_t
gmc_classify(uint32_t edt, bool past_gate, uint32_t guard_us) {
    bool guard_block = (guard_us != 0) && (edt <= guard_us);
    if (guard_block) return past_gate ? GMC_GUARD_REMOVED : GMC_REJECT;
    return past_gate ? GMC_COUNT : GMC_REJECT;
}

/** @brief Effective window (µs) for an ISR-path count feature vs pcnt_filter.
 *
 *  Single source of the on/off policy (V2.5.30): the feature is OFF (0) when
 *  its enable is clear OR when pcnt_filter is on — pcnt_filter makes the PCNT
 *  hardware path authoritative for the uploaded count, which no ISR-side
 *  suppression can reach, so the two are mutually exclusive and pcnt_filter
 *  wins. V2.6.29: shared by BOTH ISR-path features — the dead-time guard
 *  (config_effective_guard_us) and the HV blanking window
 *  (config_effective_blank_us); the config.c wrappers are thin adapters over
 *  this so the rule is host-testable without pulling in config_t / NVS.
 */
__attribute__((always_inline)) static inline uint32_t
guard_effective_us(bool enabled, bool pcnt_filter, uint32_t window_us) {
    return (!enabled || pcnt_filter) ? 0u : window_us;
}

/** @brief V2.6.29: should the HV blanking window drop this would-be count?
 *
 *  The Rev B/C PCB couples one phantom NEGEDGE into GMZ_COUNT per HV charge
 *  pulse — deterministically ~10 µs (9-16 observed; 3-10 during a boot train
 *  as the rail rises) after FET turn-off, the first ring-down trough of the
 *  L1 flyback (80 kHz SRF). See reference_radiation_data_analysis 2026-07-21/26.
 *  Because the trigger time is known from the recharge state machine itself,
 *  a short deaf window keyed on the FET turn-off stamp removes the phantom
 *  by CAUSE — immune to the pulse-width variation (MCU- and hardware-mod-
 *  dependent) that makes the 4 µs PCNT width filter a non-fix fleet-wide.
 *
 *  @param off_gap  Gap (µs) from the most recent FET turn-off to this edge,
 *                  clamped (clamp_u32); callers pass UINT32_MAX when no HV
 *                  pulse has fired yet so the edge can never be blanked.
 *  @param blank_us Blanking window; 0 disables (default).
 *
 *  Applied ONLY to edges classified GMC_COUNT (past gate + guard): blanked
 *  edges are tallied separately (hv_blanked) as the window's true marginal
 *  effect, so counts_without_blank == counts + hv_blanked.
 */
__attribute__((always_inline)) static inline bool
hv_blank_hit(uint32_t off_gap, uint32_t blank_us) {
    return (blank_us != 0) && (off_gap <= blank_us);
}
