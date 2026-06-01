#pragma once

/** @file
 *  @brief V2.5.8: Generic GNSS receiver driver (I²C, NMEA over DDC).
 *
 *  ONE driver, TWO supported breakouts — both stream NMEA-0183 text over
 *  I²C; they differ only in how bytes are pulled off the bus, so the
 *  NMEA parser + clock discipline + fix snapshot are shared and the
 *  per-chip part is a thin transport:
 *
 *    * Adafruit 4415 — CDTop PA1010D (MediaTek MT3333). I²C addr 0x10.
 *      Plain streaming reads; idle/no-data padding byte is 0x0A.
 *      NOTE: 0x10 is the SAME address as the VEML7700 ambient-light
 *      sensor. They cannot share a bus. `gnss_enable` (config) takes the
 *      address and the VEML7700 probe is skipped — the two are never
 *      used together by design.
 *
 *    * SparkFun MAX-M10S Qwiic — u-blox MAX-M10S. I²C addr 0x42 (u-blox
 *      DDC). Bytes-available count at registers 0xFD/0xFE, data stream at
 *      0xFF; idle padding byte is 0xFF. Outputs NMEA + UBX binary by
 *      default — the '$'-anchored, checksum-validated line accumulator
 *      simply discards the interleaved UBX frames. No conflict with the
 *      VEML7700 (different address), but the same `gnss_enable` rule keeps
 *      the wiring uniform across boards.
 *
 *  `gnss_init()` auto-detects which chip is present (probes 0x42 then
 *  0x10) and binds the matching transport. Time discipline is GPS-primary:
 *  on every valid fix the system clock is set from the GNSS UTC (rate-
 *  limited to avoid sub-second jitter), overriding SNTP whenever a fix is
 *  available. SNTP keeps running untouched as the no-fix fallback.
 *
 *  Draining (`gnss_poll()`) runs on the main service task at ~1 Hz; the
 *  status page reads only the cached snapshot via `gnss_get_fix()`, so the
 *  I²C device is touched from exactly one task.
 */

#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** @brief Snapshot of the most recent GNSS solution. */
typedef struct {
    bool    valid;    ///< RMC status == 'A' (navigation-valid position + time)
    bool    fix_3d;   ///< GGA fix quality >= 1 with an altitude (vs 2D / none)
    double  lat;      ///< Decimal degrees, +N / -S. Valid only when `valid`.
    double  lon;      ///< Decimal degrees, +E / -W. Valid only when `valid`.
    float   alt_m;    ///< Altitude above mean sea level (m), from GGA.
    float   hdop;     ///< Horizontal dilution of precision, from GGA.
    uint8_t sats;     ///< Satellites used in the solution, from GGA.
    time_t  utc;      ///< UTC of fix (Unix epoch). Valid only when `valid`.
} gnss_fix_t;

/** @brief Probe + initialise a GNSS receiver on the given bus.
 *
 *  Probes the u-blox DDC address (0x42) first, then the PA1010D (0x10),
 *  and binds the matching transport on the first ACK. Idempotent — once
 *  bound, subsequent calls return ESP_OK without re-probing (so the
 *  caller can retry on a second bus without disturbing a prior hit).
 *
 *  @return ESP_OK if a receiver was bound, ESP_ERR_NOT_FOUND if neither
 *          address ACK'd, or a propagated i2c error.
 */
esp_err_t gnss_init(i2c_master_bus_handle_t bus);

/** @brief True once a receiver has been bound by gnss_init(). */
bool gnss_present(void);

/** @brief Drain the I²C stream, parse complete NMEA sentences, and
 *  discipline the system clock from any valid fix. No-op if not present.
 *  Call at ~1 Hz from the main service task. */
void gnss_poll(void);

/** @brief Copy the latest fix snapshot. @return false if not present. */
bool gnss_get_fix(gnss_fix_t *out);

/** @brief Human-readable chip name of the bound receiver
 *  ("PA1010D" / "MAX-M10S"), or "none" if not present. */
const char *gnss_chip_name(void);

/** @brief 7-bit I²C address of the bound receiver (0 if not present). */
uint8_t gnss_i2c_addr(void);

/** @brief True if GNSS has disciplined the system clock within the last
 *  few minutes — i.e. GPS is the active wall-clock source (vs SNTP). */
bool gnss_time_is_source(void);
