#pragma once
// Bump before build; commit after successful flash.
//
// V2.3.26 — env-sensor diagnostics (no behaviour change).
//
// **Headline:** make a flaky SHT45 (or any silently-failing env sensor) visible
// in the logs. Two changes:
//
//   1. `sht45_read()` previously returned `esp_err_t` from the two I²C steps
//      (CMD_MEASURE_HIGH write, post-measure receive) without any log. The
//      CRC-mismatch path already logged. Now both silent paths emit
//      `ESP_LOGW(TAG, "measure_high write: <err>")` /
//      `ESP_LOGW(TAG, "post-measure read: <err>")` mirroring the init path's
//      `try_init_pass()` step-by-step verbosity. Triggered by esp32-176432
//      (knock-off Heltec): SHT45 ACK'd at init and read 61.62 % RH on the
//      verification probe, then on every cycle after that env_sensor_read
//      reported H=0.00 % with T tracking BMP390 → SHT45 was failing silently
//      and the cascade quietly fell through to BMP390 (which has no humidity
//      channel), leaving the local `h` at its initial 0.0f.
//
//   2. `env_sensor_read()` gained an optional `(char *raw_log, size_t cap)`
//      tail so the caller can see WHAT each present-and-called sensor
//      returned, alongside the existing fused result. Each sensor segment is
//      `"<NAME>: T=...  H=... [P=...]"` (only fields the chip provides) or
//      `"<NAME>: read failed"`, comma-separated. `main.c`'s cycle log line
//      now reads e.g.
//        SHT45: T=18.86°C  H=0.00%, BMP390: T=18.88°C P=1026.60hPa SHT45+BMP390: T=18.86°C  H=0.00%  P=1026.60hPa
//      Per-sensor segment is omitted for sensors not present (so the line
//      shrinks naturally on single-chip configurations). The fused
//      `<combined-name>:` block at the end is unchanged in shape.
//
// **Code surface:** sht45.c (+2 ESP_LOGW), env_sensor.c (per-sensor RL append
// inside the existing cascade), env_sensor.h (signature + size_t include +
// docstring), main.c (160-byte stack buffer + new format string). No CPU/heap
// cost in the steady state — RL macro is a snprintf into a stack buffer that
// gets discarded after the log line is emitted.
//
// **No behaviour changes.** Same I²C transactions per cycle, same fallback
// cascade priority, same fused output to /status, transmission, FTP, OLED.
// All four boards build identically; OTA-safe from V2.3.25 (no partition or
// sdkconfig changes). 20 release artefacts (5 × 4 boards).
//
// V2.3.25 — aqi.eco compatibility fix + body trim.
//
// **Headline fix:** aqi.eco's `devices.esp8266_id` column is `bigint` (per
// `mysql-schema.sql` in `trekawek/air-quality-info`). Our V2 firmware was
// emitting `"esp8266id": "esp32-5965048"` in the body — a string. Server
// tried to `UPDATE devices SET esp8266_id = 'esp32-5965048'`, MySQL failed
// to coerce the string to bigint, PHP threw an unhandled exception, nginx
// returned 500 with empty body. Symptom: every aqi.eco POST returned 500
// and our circuit breaker would eventually open. NAMF firmware sends bare
// numeric IDs (e.g. `"7738603"`) because ESP8266 chip IDs ARE bigints — we
// happen to wrap ours with `"esp32-"` for display, which broke the parse.
//
// Fix: in `build_luftdaten_body`, when `prefix_aqi_id == true`, strip the
// leading `"esp32-"` from `c->chip_id` before emitting the `esp8266id`
// field. Verified end-to-end 2026-05-12 by manual POST: `"5965048"` →
// HTTP 200 OK; `"esp32-5965048"` → HTTP 500. ~5 LOC change.
//
// **Other aqi.eco body trims** (per the same investigation — full table in
// `reference_aqi_eco.md`'s VALUE_MAPPING section):
//   * Drop `Si22G_*` radiation block entirely (aqi.eco has no radiation
//     column; data was being silently discarded). Includes `samples`,
//     `min_micro`, `max_micro` which only made sense alongside Si22G.
//   * Drop `SPS30_TS` (typical particle size) — not in VALUE_MAPPING.
//   * Drop `DNMS_noise_LA_min` / `DNMS_noise_LA_max` — only LAeq is mapped
//     (canonical column `noise_level`).
//
// **Other aqi.eco body additions** (NAMF spray-and-pray pattern):
//   * Add `SHT3X_temperature`, `SHT3X_humidity` alongside `BME280_*` —
//     aqi.eco's VALUE_MAPPING lists SHT3X first in the temperature and
//     humidity alias arrays. Same numeric value, two aliases — server
//     picks whichever it prefers.
//   * Add `BMP_pressure` alongside `BME280_pressure` — also in the
//     pressure alias array (after BME280_pressure).
//
// Net body size: ~30 % smaller per aqi.eco POST (radiation and noise tail
// dropped). One more handshake's worth of bytes saved per upload.
//
// **openSenseMap path unchanged** — still gets the full bundle including
// Si22G, SPS30_TS, DNMS min/max. Per-box channel mapping on openSenseMap
// can route each field where the user wants. The aqi.eco-specific trims
// are gated on `prefix_aqi_id == true`.
//
// **No other behaviour changes.** All other targets (Madavi, sensor.community
// X-PIN 1/11/15/19, Radmon) use separate body builders and are untouched.
//
// OTA-safe from V2.3.24 (no partition layout changes, no sdkconfig
// changes). 20 release artefacts (5 × 4 boards).
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
#define VERSION_STR "V2.3.26"
