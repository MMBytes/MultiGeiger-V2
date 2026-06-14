#pragma once

/** @file
 *  @brief V2.4.15: UDP syslog client (RFC 5424 framing since V2.5.27).
 *
 *  Fire-and-forget per-line shipping to a LAN syslog receiver (rsyslog or
 *  syslog-ng on a server, typically the same Pi already running Mosquitto).
 *  No TLS, no retries, no persistent state — the structural cheapest
 *  log-shipping path on this firmware.
 *
 *  Heap cost: ~0 KB persistent (just the int socket fd + a 64-byte sockaddr).
 *  Per-line cost: ~50-100 µs CPU + one UDP packet sent via lwIP. The packet
 *  goes through the existing WiFi/lwIP stack — no new buffers, no
 *  per-connection state.
 *
 *  Loss tolerance: UDP. Packets dropped on weak WiFi or while Pi is down are
 *  gone. Acceptable for log shipping — the on-device applog ring still holds
 *  the line, viewable via `/log`.
 *
 *  Pairs with `rsyslog` on the Pi side (already installed on Raspberry Pi OS).
 *  See `[[reference_syslog_pi_setup]]` for the rsyslog drop-in config.
 *
 *  Boot-time logs are NOT shipped — syslog_init runs after `GOT_IP`, so
 *  everything before the first STA connect lives only in the applog ring.
 *  That's the same constraint MQTT has post-V2.4.12 and is generally fine
 *  because the most valuable boot logs are accessible via `/log` after the
 *  device comes up.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief Initialize the syslog UDP socket and resolve the server.
 *
 *  Safe to call once after `GOT_IP`. If host is empty or port is 0, returns
 *  immediately as a no-op (disabled). After this returns, `syslog_emit()`
 *  will start shipping lines.
 *
 *  @param host        Server hostname or IPv4 string (e.g. "10.11.12.150")
 *  @param port        UDP port (typically 514)
 *  @param hostname    Hostname to embed in the syslog header — typically
 *                     the device's WiFi hostname so rsyslog can route by
 *                     `$hostname startswith "MultiGeiger"`.
 */
void syslog_init(const char *host, uint16_t port, const char *hostname);

/** @brief Tear down the UDP socket. Idempotent.
 *
 *  Called from the OTA / FTPS teardown paths to free the socket (~1 KB lwIP
 *  state). After this returns, `syslog_emit()` is a silent no-op until
 *  `syslog_init()` is called again. Main-loop poll re-inits as needed,
 *  same pattern as `mqtt_stop()` / `mqtt_init()`.
 */
void syslog_stop(void);

/** @brief True if the syslog socket is currently open. */
bool syslog_is_initialized(void);

/** @brief Cumulative UDP send stats since boot (either pointer may be NULL).
 *
 *  `dropped` counts `sendto()` failures — almost always lwIP pbuf-pool
 *  exhaustion when a burst (e.g. the boot config dump) outruns the WiFi/lwIP
 *  drain. A non-zero count is a definitive device-side-loss signal; **zero
 *  does NOT prove zero loss** (WiFi-driver-late, network, and server-side
 *  drops aren't visible here). Read from the TX cycle, never the emit path.
 */
void syslog_get_stats(uint32_t *sent, uint32_t *dropped);

/** @brief Emit one log line via UDP. Called from `applog_vprintf()`.
 *
 *  Severity is inferred from the first character of the formatted line:
 *  'E' → error, 'W' → warning, 'D' → debug, anything else → info.
 *  Facility is hardcoded to local0 (16).
 *
 *  Re-entrancy safe: a guard flag short-circuits the second call if the
 *  first triggered a recursive ESP_LOG (e.g. via the sendto error path —
 *  this code does NOT call ESP_LOG to avoid that path, but the guard is
 *  belt-and-braces).
 *
 *  No-op if syslog isn't initialized. Trailing newlines stripped before
 *  send — rsyslog adds its own.
 */
void syslog_emit(const char *line, size_t len);
