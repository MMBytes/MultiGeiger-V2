#pragma once

/** @file
 *  @brief FTP log upload — streams the applog ring directly to a LAN FTP
 *  server (zero-copy via applog_stream_begin/_end since V2.3.16).
 *
 *  Passive-mode FTP. Anonymous when user is empty. Intended for local use
 *  such as a router USB share. The blocking send runs on the main loop task —
 *  a 60 KB upload over LAN finishes in under 3 s and does not meaningfully
 *  perturb the ~150 s TX cycle.
 *
 *  Blocking-safety: all socket reads go through select() with an explicit
 *  deadline, so a half-open TCP connection (WiFi dropped mid-transfer) cannot
 *  wedge the main loop. If a data write stalls, the 226 confirmation read is
 *  skipped and both sockets are closed immediately.
 */

#include <stdint.h>
#include "config.h"

/** @brief Call once at boot. @p chip_id is embedded in the upload filename. */
void log_ftp_init(const char *chip_id, const config_t *cfg);

/** @brief Call from the main loop with the current uptime in ms.
 *
 *  First upload fires one hour after boot; subsequent uploads every
 *  cfg->ftp_interval_min minutes. No-op if disabled, WiFi down, time
 *  invalid, or not yet due.
 */
void log_ftp_loop(uint32_t now_ms);

/** @brief FTP upload stats snapshot — surfaced on the / status page.
 *
 *  `have_last` is false until the first upload attempt completes.
 *  `last_ok` is the result of that attempt (false = failed). `last_at` is
 *  the unix-epoch wall-clock timestamp of the last attempt (0 = never).
 *  `last_bytes` is the body length of the last *successful* upload (last
 *  failed attempt does not overwrite this). `next_due_ms` is the monotonic
 *  uptime-ms when the next regular upload is scheduled — caller subtracts
 *  current uptime to display "next in X seconds".
 */
typedef struct {
    bool     have_last;
    bool     last_ok;
    int64_t  last_at;
    uint32_t last_bytes;
    uint32_t next_due_ms;
} log_ftp_stats_t;

/** @brief Read the current FTP stats snapshot. Safe from any task. */
void log_ftp_get_stats(log_ftp_stats_t *out);
