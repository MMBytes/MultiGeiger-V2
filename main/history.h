#pragma once

/** @file
 *  @brief In-RAM CPM history + rolling averages (V2.5.6).
 *
 *  Two-tier ring buffer of recent counts-per-minute, sampled every 60 s from a
 *  monotonic tube counter — the raw ISR total (tube_get_total_counts()), or the
 *  PCNT width-filtered total (tube_pcnt_filtered_total()) when the width filter
 *  is on (V2.5.16; see history_tick's use_filtered). Independent of the
 *  destructive per-cycle tube_read(). Backs the /status graph and the
 *  rolling 5-/15-minute averages used for GMC ACPM + ThingSpeak field3/4.
 *
 *  Live only — resets on reboot (no flash persistence by design). Radiation-
 *  only: the sampler is inert while the tube is disabled. ~250 bytes RAM;
 *  enabled on all boards.
 *
 *  V2.8.3 adds a second, independent consumer: history_live_cpm(), a 60 s
 *  sliding-window CPM updated ~1 Hz for the radiation display. Separate ring,
 *  separate baseline — shares only the count-source choice (source_total()).
 *  Adds ~590 bytes RAM (72 × 8-byte samples + bookkeeping) on every board.
 *
 *  Concurrency: history_tick() is the single writer (main task); history_get()
 *  is the reader (HTTP task). A mutex guards the snapshot. history_live_cpm()
 *  is main-task only (its own writer and reader, own state) — no mutex. No
 *  ISR touches this module's state. (When the filter is on, history_tick
 *  reads the PCNT accum total, which the driver maintains under its own
 *  spinlock — an atomic read, not a hazard to this module's state.)
 */

#include <stdbool.h>
#include <stdint.h>

#define HIST_MIN_DEPTH   60       // last 60 min @ 1 sample/min
#define HIST_HOUR_DEPTH  24       // last 24 h  @ 1 sample/hour
#define HIST_EMPTY       0xFFFF   // sentinel for unfilled ring slots

/** @brief Snapshot for the HTTP reader. Arrays are oldest..newest; only the
 *         first *_count entries are valid (the rest are HIST_EMPTY). */
typedef struct {
    uint16_t cpm_min[HIST_MIN_DEPTH];
    uint16_t cpm_hour[HIST_HOUR_DEPTH];
    uint8_t  min_count;     // valid minute samples (0..HIST_MIN_DEPTH)
    uint8_t  hour_count;    // valid hour samples  (0..HIST_HOUR_DEPTH)
    uint16_t cpm_now;       // most recent 1-min CPM
    uint16_t cpm5;          // mean of last min(5, min_count) minute samples
    uint16_t cpm15;         // mean of last min(15, min_count) minute samples
} history_snapshot_t;

/** @brief Create the mutex and zero all state. Call once at boot. */
void history_init(void);

/** @brief Drive the 60 s sampler. Call every main-loop iteration with the
 *         monotonic ms clock; it fires internally once per 60 s. No-op while
 *         the tube is disabled.
 *  @param use_filtered  V2.5.16: when true, sample the PCNT width-filtered
 *         monotonic total (tube_pcnt_filtered_total) so cpm5/cpm15 match the
 *         filtered per-cycle CPM; when false, the raw ISR total. The caller
 *         (main.c) passes the live `pcnt_filter && tube_pcnt_active()` decision.
 *         A source switch re-primes the baseline (one skipped sample) so the
 *         cross-source delta can't produce a garbage minute. */
void history_tick(uint32_t now_ms, bool use_filtered);

/** @brief Copy the current history + rolling averages out under the mutex. */
void history_get(history_snapshot_t *out);

/** @brief V2.8.3: live ~60 s sliding-window CPM for the 1 Hz radiation display.
 *
 *  Display-only. Keeps its OWN ring of (monotonic total, timestamp) samples,
 *  one per call at most every ~1 s, and never touches the 60 s sampler's
 *  state above — cpm5/cpm15 are uploaded (GMC ACPM, ThingSpeak f3/f4) and
 *  must not shift because a display is fitted. Never calls the destructive
 *  tube_read()/tube_pcnt_read(), so the uploaded per-cycle CPM is untouched.
 *
 *  The rate is taken over the ACTUAL elapsed time to the newest sample at
 *  least 60 s old, so a stalled caller (main-task FTPS upload, slow PM read)
 *  lengthens the window, never skews the rate: after a stall > 60 s the window
 *  spans back to the last pre-stall sample until post-stall samples are 60 s
 *  old — the rate keeps showing and never drops to a short window. A source
 *  switch (runtime pcnt_filter toggle) empties the ring, like history_tick's
 *  re-prime, and so does PCNT subtract mode's per-cycle wide-phantom step
 *  (detected exactly, from tube_get_blanked_wide_total() changing), which
 *  would otherwise read low — or as 0 CPM on a slow node — for up to a
 *  minute every cycle.
 *
 *  Single-task contract: the main task is the only caller (writer and reader
 *  in one), so no mutex. Not for the uploaded values — the 60 s window is
 *  deliberately different from the TX-cycle window.
 *
 *  @param now_ms        Monotonic ms clock (same source as history_tick).
 *  @param use_filtered  Same count-source decision as history_tick.
 *  @param cpm_out       Out: CPM over the window. Untouched on false.
 *  @return false while the tube is disabled or the window spans < 10 s
 *          (just after boot, a source switch, or a subtract-mode step-down)
 *          — the caller keeps whatever it was showing rather than paint a
 *          1-count-in-1-s = 60 CPM spike.
 */
bool history_live_cpm(uint32_t now_ms, bool use_filtered, uint32_t *cpm_out);
