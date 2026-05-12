#pragma once
// Bump before build; commit after successful flash.
//
// V2.3.24 — the wrap-corruption fix + UX polish.
//
// Headline fix: the V2.3.16-era streaming snapshot in applog_stream_begin had
// a wrap-specific pointer aliasing bug. In the wrapped case the oldest-half
// segment (seg_a) started at exactly `s_ring + s_pos + skip`, which is also
// where the writer's next ring_append() lands. Releasing the mutex with that
// pointer exposed the segment's start to in-flight overwrites — visible from
// V2.3.16 onwards as torn lines at the head of every uploaded FTPS file
// (and would have hit any concurrent /log browser request the same way).
// The bug only manifested on Heltec because PSRAM boards' giant 4 MB ring
// barely wraps in production lifetime; Heltec's 60 KB ring wraps every ~3 h
// so files #2 onwards always took the wrapped branch.
//
// Fix: applog_stream_begin now copies the danger zone (first
// HAL_LOG_SNAP_SCRATCH_BYTES of seg_a's content) into a pre-allocated
// scratch buffer under the mutex, and exposes a third segment so callers
// stream `scratch + ring-tail-remainder + newer-pre-wrap-half` in
// chronological order. Per-board scratch sizing (in hal.h):
//   Heltec V2 (8 MB + 4 MB clone) : 4 KB internal DRAM
//   FeatherS3-D                   : 16 KB external PSRAM
//   QT Py ESP32-PICO              : 16 KB external PSRAM
// Worst-case writes during a 3-30 s upload are <500 B (after the FTPS-
// internal handshake noise was downgraded — see below), so 4 KB on Heltec
// gives ~8× headroom while costing only ~4 % of free_heap. Failure mode if
// scratch overflows: degrades to V2.3.23 torn-line behaviour — never worse.
// Init-time scratch allocation failure is non-fatal: falls back to the
// V2.3.16 zero-copy path.
//
// FTPS-internal log lines downgraded INFO→DEBUG. Six lines per upload
// (~660 B): TLS-cipher on ctrl/data channels, NewSessionTicket received ×2,
// TLS-shutdown drain summary ×2. They're now in source as ESP_LOGD —
// re-enable any time via `esp_log_level_set("ftp", ESP_LOG_DEBUG)` from
// the console (or with CONFIG_LOG_DEFAULT_LEVEL=DEBUG at build time) when
// investigating a TLS regression. The drain summary was production-validated
// in V2.3.22; with that arc closed it's no longer needed at INFO every hour.
//
// Status page small wins:
//   - FTP "next" duration now uses the same H/M/S split as "ago" (e.g.
//     "in 11m 12s" instead of "in 672s").
//   - NTP line: "synced · now <ts>" → "synced · clock now <ts>" so it's
//     clearer that the timestamp is current device time (not last-sync time).
//   - <code> elements no longer shrink (browser default is ~85 % of body);
//     forced to 1em so chip-id and MAC visually match the surrounding label.
//
// Config page UX rework — two submit buttons:
//   - Save        : persists to NVS, no reboot. Live-applied for fields
//                   read per-cycle / per-request (TX targets + their HTTPS
//                   toggles + credentials, FTP settings, BME280 altitude,
//                   speaker/LED tick, web admin password).
//   - Save and restart : persists + flags reboot (V2.3.23 behaviour).
// Reboot-required fields are visually marked with a red `*`; a legend at
// the top of the form explains. The user owns the decision — no server-
// side diff logic. Saved-no-restart response page links back to /config
// and / so the user isn't dead-ended.
//
// Reboot response page now also links back to / (was a dead-end).
//
// OTA-safe from V2.3.23 (no partition layout changes, no sdkconfig
// changes). 20 release artefacts (5 × 4 boards). Heltec gains 4 KB of
// permanent internal-DRAM allocation for the snapshot scratch; FeatherS3-D
// and QT Py gain 16 KB of permanent PSRAM allocation (negligible vs pool).
//
// V2.3.23 — closes out the V2.3.5 → V2.3.22 FTPS+TLS 1.3 arc + adds slow-
// fragmentation prevention. Two small changes:
//
//   1. **`ftp_tls12_only` default flipped from true to false.** TLS 1.3 is
//      now the FTPS default after V2.3.22's bidirectional-close fix landed
//      and was production-validated against the project's LAN FTPS server
//      (esp32-176432 → esp32-12276328:48807, ~22 successful uploads
//      observed across ~1 hour with no 426 events).
//
//      The /config "Limit FTPS to TLS 1.2" checkbox stays as a safety net
//      for any future server we haven't yet handled. Existing devices
//      upgrading from V2.3.22 keep their saved value via NVS — only fresh
//      / NVS-erased devices get the new default. Pre-V2.3.22 the default
//      was true because TLS 1.3 against this server got "426 Connection
//      reset" (now fixed by draining server's NewSessionTickets before TCP
//      close). See `reference_ftps_tls13_investigation.md` for the full
//      arc.
//
//   2. **Scheduled PSA crypto refresh every 24h.** Adds a third trigger
//      to the existing PSA-reset machinery (already fires on 5 consecutive
//      OOMs and on TCP write stalls). New trigger is purely time-based —
//      flushes any slow heap fragmentation that accumulates across hundreds
//      of TLS handshakes (HTTPS + FTPS combined).
//
//      Observed in production V2.3.22 1-hour soak: min_free dropped ~14 KB
//      across 22 uploads, in step-shaped chunks of 6-8 KB on specific
//      uploads (free_heap stayed flat — no leak, just fragmentation).
//      Likely stabilises but the 24h scheduled refresh is cheap insurance.
//
//      Implementation in `log_ftp.c::log_ftp_loop` BEFORE the
//      ftp_enabled early return, so it fires on every device regardless
//      of FTP being configured. Gating:
//        - `tx_is_idle()` — no HTTPS handshake mid-flight on TX worker
//          (CPU1). If busy, defer to next loop tick (1 s); eventually hits
//          an idle window between TX cycles (~10 s active out of every
//          150 s).
//        - FTPS not in progress: structurally guaranteed, since both
//          FTPS upload and this timer check run on the main task which is
//          single-threaded.
//
//      Cost: ~20-50 ms extra latency on the FIRST TLS handshake after the
//      reset (PSA key material re-derived on first use). Sensor / WiFi /
//      HTTP server / NVS / log ring all unaffected. Worst-case race
//      window if HTTPS handshake starts between `tx_is_idle()` check and
//      `mbedtls_psa_crypto_free()` call: one HTTPS retry. Same race
//      acceptance the existing reset triggers have used since V2.3.15.
//
//      24h interval is comfortably below any conceivable fragmentation
//      accumulation rate; if min_free ever shows linear-not-stabilising
//      drop on overnight soaks, drop to 12 h or add a heap-threshold
//      trigger alongside this time-based one.
//
// OTA-safe from V2.3.22 (no partition layout changes, no sdkconfig
// changes). 20 release artefacts (5 × 4 boards). All four boards share
// the FTPS code path and benefit from both changes.
#define VERSION_STR "V2.3.24"
