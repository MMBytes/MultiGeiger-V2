#pragma once

/** @file
 *  @brief FTP log upload — ships applog_snapshot() to a LAN FTP server.
 *
 *  Passive-mode FTP, no TLS. Anonymous when user is empty. Intended for local
 *  use such as a router USB share. The blocking send runs on the main loop
 *  task — a 40 KB upload over LAN finishes in under 2 s and does not
 *  meaningfully perturb the ~150 s TX cycle.
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
