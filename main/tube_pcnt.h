#pragma once

/** @file
 *  @brief V2.5.16 — parallel PCNT pulse-WIDTH comb on the GMC count pin.
 *
 *  A diagnostic to test whether the cross-board CPM gap (socketed Feather
 *  ESP32-S3 reads a stable +13–14 % over the Heltec ESP32 — see
 *  reference_radiation_data_analysis) is caused by the S3 registering
 *  *narrow* count-line glitches the ESP32 rejects.
 *
 *  The production count path (tube.c gmc_count_isr) measures pulse SPACING
 *  (the 190 µs dead-time gate + the V2.5.12 edge-spacing histogram). It has
 *  never measured pulse WIDTH. This module taps the SAME pad
 *  (PIN_GMC_COUNT_INPUT) with up to TUBE_PCNT_NWIDTHS hardware PCNT units,
 *  each running the peripheral glitch filter at a different width. Unit 0 is
 *  unfiltered; units 1..N reject pulses narrower than their threshold. The
 *  difference between adjacent teeth is the count of pulses in that width band
 *  — i.e. a width histogram of every falling edge.
 *
 *  OBSERVER ONLY. PCNT taps the digital input through the GPIO matrix (one pad
 *  fans out to the CPU interrupt AND every PCNT unit simultaneously); it never
 *  gates, clears, or alters the production ISR count. Electrically nothing
 *  changes — the input buffer is already enabled for the ISR. A node running
 *  the comb reports exactly the same CPM it would without it.
 *
 *  Unlike steadramon/ESPGeiger (which uses PCNT as a *replacement* for the ISR,
 *  chosen at compile time, on the legacy driver/pcnt.h API), this runs PCNT
 *  *alongside* the ISR on the IDF v6 driver/pulse_cnt.h API, keeping all the
 *  per-pulse timing/HV diagnostics the ISR provides.
 *
 *  Ships dark on every board — gated by the `pcnt_filter` config flag (default
 *  off). The module itself is a pure counter (observer); when `pcnt_filter` is
 *  on, do_tx_cycle (main.c) elects pc[NWIDTHS-1] as the authoritative count, so
 *  the same comb both filters CPM and logs the full width diagnostic.
 */

#include <stdbool.h>
#include <stdint.h>

/// Number of comb teeth (PCNT units). ESP32-S3 has exactly 4 PCNT units;
/// the original ESP32 has 8. Using 4 consumes all S3 units (nothing else in
/// this firmware uses PCNT).
#define TUBE_PCNT_NWIDTHS 4

/** @brief DEFAULT glitch-filter width (ns) per unit; index 0 = 0 = filter OFF.
 *
 *  The widest tooth (index N-1) is overridden at init by the configured filter
 *  width; the lower teeth are fixed diagnostic references. APB-clocked, so each
 *  value must stay under the ~12.8 µs ceiling (1023 cycles @ 80 MHz). Use
 *  tube_pcnt_width_ns() for the ACTUAL widths in use this session.
 */
extern const uint32_t TUBE_PCNT_WIDTHS_NS[TUBE_PCNT_NWIDTHS];

/** @brief Actual width (ns) of comb tooth @p i in use this session (0 if the
 *  comb is inactive or @p i is out of range). Tooth N-1 = the filter width. */
uint32_t tube_pcnt_width_ns(int i);

/** @brief Bring up the PCNT width-comb on PIN_GMC_COUNT_INPUT.
 *
 *  Call AFTER tube_setup() so the count pin is already configured as an input
 *  with the GMC ISR installed (this only adds matrix taps; it does not touch
 *  the pad's pull or interrupt config). Safe to call once; idempotent.
 *
 *  @param filter_width_ns  Glitch-filter width (ns) for the widest tooth = the
 *         FILTER source (config pcnt_filter_width_ns). The lower teeth stay at
 *         the fixed diagnostic defaults. Must be ≤ ~12.8 µs (APB ceiling) — an
 *         out-of-range value just fails set_glitch_filter and self-disables the
 *         comb, leaving the ISR count untouched.
 *  @return true if all units came up. On any error it rolls back every unit it
 *          created and self-disables (returns false) — the ISR count path is
 *          never affected either way.
 */
bool tube_pcnt_init(uint32_t filter_width_ns);

/** @brief True if the comb is running (tube_pcnt_init succeeded). */
bool tube_pcnt_active(void);

/** @brief Tear down all PCNT units/channels, releasing their internal DRAM and
 *  the GPIO-matrix taps. Idempotent (no-op if not active). Called on the OTA
 *  teardown path so the comb doesn't pin internal DRAM during the OTA's
 *  contiguous-alloc window (matters most on the heap-tight Heltec). Not
 *  re-armed afterwards — a successful OTA reboots; a failed one leaves the comb
 *  down until the next boot, acceptable for an opt-in diagnostic. */
void tube_pcnt_stop(void);

/** @brief Per-cycle deltas of all teeth (no clear).
 *  @param out  Out: out[i] = falling edges since the last call that survived
 *              width filter i (out[0] = unfiltered ≈ DIAG raw_edges). Filled
 *              with zeros when the comb is inactive.
 *
 *  Each value is a software delta of the unit's MONOTONIC accumulator (current
 *  total minus this consumer's last-read base) — the units are never cleared,
 *  so there is NO read-then-clear lost-edge window. The last-read base is
 *  private to this call path, independent of tube_pcnt_filtered_total()'s
 *  consumer, so both can read the same totals without interfering.
 */
void tube_pcnt_read(uint32_t out[TUBE_PCNT_NWIDTHS]);

/** @brief Monotonic count total (since boot) surviving the WIDEST (4 µs) width
 *  filter — the filtered analogue of tube_get_total_counts().
 *
 *  Read non-destructively (no clear), so an independent consumer (history.c's
 *  60 s cpm5/cpm15 sampler) can take its own delta off it without disturbing
 *  the per-cycle tube_pcnt_read() delta. Returns 0 when the comb is inactive.
 *  Lets the rolling averages be filtered consistently with the per-cycle CPM
 *  when pcnt_filter is on — keeping the hardware width accuracy (no software
 *  width measurement). */
uint32_t tube_pcnt_filtered_total(void);
