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

/** @brief Effective dead-time-guard window given the three config inputs.
 *
 *  Single source of the on/off policy (V2.5.30): the guard is OFF (0) when its
 *  enable is clear OR when pcnt_filter is on — pcnt_filter makes the PCNT
 *  hardware path authoritative for the uploaded count, which the ISR guard
 *  cannot reach, so the two are mutually exclusive and pcnt_filter wins.
 *  config_effective_guard_us() (config.c) is a thin wrapper over this so the
 *  rule is host-testable without pulling in config_t / NVS / esp_err.
 */
__attribute__((always_inline)) static inline uint32_t
guard_effective_us(bool enabled, bool pcnt_filter, uint32_t window_us) {
    return (!enabled || pcnt_filter) ? 0u : window_us;
}
