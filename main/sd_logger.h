#pragma once

/** @file
 *  @brief V2.6.19: standalone-mode CSV logger — one row of every attached
 *         sensor's readings per TX cycle, appended to the microSD card.
 *         Spec: docs/superpowers/specs/2026-07-15-standalone-sd-logging-design.md §5.
 *
 *  File lifecycle: creation WAITS for the first GNSS clock sync
 *  (gnss_clock_synced()) so the filename timestamp and every row share one
 *  valid GPS-set time base — a node with a dead GPS antenna logs nothing,
 *  by design. One file per boot session: esp32-<chipid>_YYYYMMDD_HHMMSS.csv
 *  (V2.6.21: timestamp is local time per cfg tz_posix, applied at boot).
 *
 *  Column order (spec §4/§5): DateTime (V2.6.21: local time with numeric
 *  UTC offset, was hardcoded UTC "Z"), Uptime [s], GPS Lat/Lon/Alt/
 *  Sats/HDOP fixed and pinned first; then every registered telemetry
 *  column sorted by header string — order is a pure function of the
 *  attached sensor set, stable across boots/OTAs, so a node's files always
 *  merge cleanly. Header strings ARE column identity; renaming one is a
 *  breaking format change.
 *
 *  Durability: fprintf → fflush → fsync per row bounds power-loss data
 *  loss to the in-flight row (~200 B / 150 s cadence, write amplification
 *  irrelevant). Write failure → unmount; next cycle remounts and starts a
 *  NEW file (the old FILE* is stale after remount — never reused).
 */

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool     mounted;
    bool     file_open;
    char     filename[64];   ///< basename only ("" until first file created)
    uint32_t rows_written;
    uint32_t fail_streak;    ///< consecutive failed cycles (mount or write)
    int      last_err;       ///< 0 = OK, else errno/esp_err of last failure
    bool     last_err_is_esp; ///< V2.6.22: true = last_err is an esp_err_t (mount path), false = errno (stdio path)
} sd_logger_status_t;

/** @brief One-time init. Stores the chip-id string used in filenames
 *  (static storage — main.c's g_chip_id, already "esp32-<num>"). Call once
 *  at boot before the first sd_logger_cycle(). */
void sd_logger_init(const char *chip_id);

/** @brief Log one CSV row. Call once per TX cycle from the main service
 *  task, standalone mode only. Handles mount retry, file creation gating
 *  (waits for GNSS clock sync), row write + fsync, and error recovery.
 *  No-op on HAL_HAS_SD_CARD=0 boards. */
void sd_logger_cycle(void);

/** @brief Status snapshot for the /status page. */
void sd_logger_get_status(sd_logger_status_t *out);
