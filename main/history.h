#pragma once

/** @file
 *  @brief In-RAM CPM history + rolling averages (V2.5.6).
 *
 *  Two-tier ring buffer of recent counts-per-minute, sampled every 60 s from
 *  the monotonic tube counter (tube_get_total_counts()) — independent of the
 *  destructive per-cycle tube_read(). Backs the /status graph and the
 *  rolling 5-/15-minute averages used for GMC ACPM + ThingSpeak field3/4.
 *
 *  Live only — resets on reboot (no flash persistence by design). Radiation-
 *  only: the sampler is inert while the tube is disabled. ~250 bytes RAM;
 *  enabled on all boards.
 *
 *  Concurrency: history_tick() is the single writer (main task); history_get()
 *  is the reader (HTTP task). A mutex guards the snapshot. No ISR touches this
 *  module's state — the ISR only bumps tube.c's isr_gmc_total.
 */

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
 *         the tube is disabled. */
void history_tick(uint32_t now_ms);

/** @brief Copy the current history + rolling averages out under the mutex. */
void history_get(history_snapshot_t *out);
