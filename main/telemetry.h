#pragma once

/** @file
 *  @brief V2.6.19: neutral telemetry column registry for the standalone
 *         SD-logging CSV (see docs/superpowers/specs/
 *         2026-07-15-standalone-sd-logging-design.md §4).
 *
 *  Inversion-of-dependency registry: each sensor DRIVER registers its own
 *  columns inside its own init, at detection time — there is no central
 *  column table to forget when a future sensor is added. Drivers depend on
 *  this header only, never on sd_logger.h; sd_logger.c is the registry's
 *  (currently sole) consumer and sorts descriptors by header string at CSV
 *  creation, so registration ORDER is irrelevant to the file format.
 *
 *  Concurrency: registrations happen exclusively during app_main() init
 *  (single task, before the service loop starts); reads happen exclusively
 *  on the main service task inside the TX cycle. No locking by design.
 */

#include <stdbool.h>
#include <stddef.h>

// 48 slots: 10 sensor drivers x <=5 columns each today (~25 used worst-case),
// with headroom for future sensors. Registry array is 48 * 12 bytes static.
#define TELEMETRY_MAX_COLUMNS 48

/** @brief Fill `cell` (NUL-terminated, plain number, no comma) with the
 *  column's current value. Return false to emit an EMPTY cell instead —
 *  used when this cycle's read failed or the value is stale/invalid. */
typedef bool (*telemetry_read_fn)(char *cell, size_t cap, void *arg);

typedef struct {
    const char       *header;  ///< Static string, e.g. "SHT45 Temperature [C]". Column identity — renaming is a breaking change (spec §4).
    telemetry_read_fn read;
    void             *arg;
} telemetry_desc_t;

/** @brief Register one CSV column. Call from driver init, at detection time.
 *  Logs an error and drops the column if the registry is full (never fails
 *  the caller). `header` must be a string with static storage duration. */
void telemetry_register(const char *header, telemetry_read_fn read, void *arg);

size_t telemetry_count(void);

/** @return descriptor at `idx` (0..count-1), or NULL if out of range. */
const telemetry_desc_t *telemetry_get(size_t idx);