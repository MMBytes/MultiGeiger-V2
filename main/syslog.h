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
 *  syslog_init runs after `GOT_IP`, so everything logged before the first
 *  STA connect is in the applog ring only. V2.8.17: once the socket is up,
 *  syslog_init() replays that part of the ring once, after the boot banner,
 *  each line prefixed with "boot " ("boot:" banner lines excepted; capped at
 *  300 lines / 32 KB; skipped if the ring has already wrapped). Until
 *  V2.8.16 boot-time logs were not shipped at all.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief Initialize the syslog UDP socket and resolve the server.
 *
 *  Safe to call once after `GOT_IP`. If host is empty or port is 0, returns
 *  immediately as a no-op (disabled). After this returns, `syslog_emit()`
 *  will start shipping lines. V2.8.17: before returning it sends the boot
 *  banner and the paced boot replay (see the file header), which keeps the
 *  calling (main) task busy for typically ~1 s, at most ~3.5 s. V2.8.18: plus
 *  a warm-up line and a 0.2 s pause before the banner.
 *
 *  @param host        Server hostname or IPv4 string (e.g. "192.0.2.10")
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

/** @brief Log the "boot: …" banner lines (V2.8.7).
 *
 *  Firmware / Build / SoC / Display / Flash / PSRAM / Sensors / TLS. Called
 *  by syslog_init() once the socket is live, and by main.c's loop for nodes
 *  whose syslog never starts, so the lines always reach the RAM /log ring.
 *  Paces 10 ms per line only while the syslog socket is open. Main task only:
 *  logs via ESP_LOG, so it must never run inside the applog/syslog emit path.
 */
void syslog_boot_banner(void);

/** @brief True once syslog_boot_banner() has run at least once this boot. */
bool syslog_boot_banner_logged(void);

/** @brief Cumulative UDP send stats since boot (either pointer may be NULL).
 *
 *  `dropped` counts `sendto()` failures — usually a DOWN LINK (no route, so
 *  every emit during the outage fails), not buffer pressure. Burst-driven
 *  pbuf-pool exhaustion was the pre-V2.5.29 boot-dump failure mode and has not
 *  recurred since that release paced the dump.
 *  A non-zero count is a definitive device-side-loss signal; **zero
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
