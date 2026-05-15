#pragma once
// Bump before build; commit after successful flash.
//
// V2.3.29 — multi-page display + dual-bus auto-detect + brightness control
// + ALS-PT19 + i2c_bus refactor.
//
// **Headline:** Major V2 architectural cleanup plus four user-facing
// additions, all OTA-safe from V2.3.28:
//
//   1. **Multi-page display rotation** (5 pages, 7 s each) on FeatherS3-D
//      and QT Py — replaces V2.3.28's single Env page. Single binary
//      auto-detects which display is fitted (SerLCD at 0x72 OR SSD1309/6
//      at 0x3C) AND which I²C bus it lives on. Page set adapts to the
//      sensors actually present.
//
//   2. **Dual-bus auto-detect** for both displays AND sensors. Probes
//      STEMMA1 first, falls through to STEMMA2 (FeatherS3-D only). LDO2
//      is enabled lazily and torn down at end-of-init if no consumer
//      bound to STEMMA2 — saves ~5–10 mA quiescent + NeoPixel idle.
//
//   3. **OLED contrast / SerLCD backlight brightness dropdown** in
//      /config — 10 % steps (10 % to 100 %). Live-applied on Save (no
//      reboot). Default 80 % matches the V2.3.28 hardcoded register.
//
//   4. **Onboard ALS-PT19 ambient-light sensor** exposed (FeatherS3-D
//      only — GPIO 4 = ADC1_CH3). New /status row "Ambient light:
//      245 mV (~127 lux, indoor lit)". Two-layer calibrated (eFuse mV
//      + nominal 1.6 mV/lux, ±50 % typical accuracy).
//
// **Pages (logical 4-row layout, mirrored on OLED 8-char and SerLCD 20-char):**
//   Env       — T / RH / P (+ Noise on row 3 if DNMS present)
//   PM Mass   — PM1.0 / PM2.5 / PM4.0 / PM10  (ug/m³)
//   PM Number — n0.5 / n1.0 / n2.5 / n4.0     (per cm³, k-suffix on OLED ≥1000)
//   Uploads   — Madavi / Sensor / OSM / AQI
//               OLED: percentages ("MA 100%")
//               SerLCD: success/attempt counts ("Madavi   919/919")
//               Only enabled targets shown; Radmon excluded by design.
//   System    — TX cycles / uptime / heap stats
//               OLED: 2x font for cycles+uptime, 1x font for Free/Min/Max.
//                     Uptime adapts: "%2dh %2dm" < 1 day, "%3dd%2dh" ≥ 1 day.
//                     TX cycles "C%5.1fk" k-suffix at ≥ 100000 (~4.75 yr).
//               SerLCD: Cycles / Uptime / Free / Max (no Min row).
//
// **Architectural changes:**
//   1. New `i2c_bus.c/.h` module — owns BOTH I²C buses. Lazy init for
//      both; secondary (FeatherS3-D STEMMA2) is sheddable via
//      i2c_bus_finalize() if no consumer. Replaces env_sensor's previous
//      bus ownership and display.c's previous bring_up_stemma2_bus().
//      Per-board bus power handling (Heltec Vext, FeatherS3-D LDO2)
//      lives here, not scattered across consumers.
//   2. `env_sensor.h` API change: `env_sensor_init(bus)` takes the bus
//      handle. Removed `env_sensor_get_i2c_bus()`. Sub-driver inits
//      (sht45, bmp581, etc.) are idempotent so main.c can re-call init
//      with the secondary bus if the first call found nothing.
//   3. main.c sensor probing uses a `PROBE_ON_BOTH_BUSES` macro: try
//      bus 1, fall through to bus 2 if `*_present()` returns false.
//      Each successful bus 2 bind calls `i2c_bus_secondary_keep_alive()`.
//   4. New `HAL_MULTIPAGE_ROTATION` hal.h flag — 1 on FeatherS3-D + QT
//      Py, 0 on Heltec V2 / V2 4MB. Drives display task spawn + main.c
//      snapshot push. Heltec keeps the V2.3.x radiation `display_running()`
//      path entirely.
//   5. New `HAL_HAS_ALS` hal.h flag — 1 on FeatherS3-D, 0 elsewhere.
//      `als.c` stubs out when 0; /status skips the ambient-light row.
//   6. New `display_snapshot_t` in display.h — sensor data pushed once
//      per TX cycle from main.c. Single-writer torn-tolerant pattern.
//      Dynamic data (uptime, heap, cycles, upload counters) is read
//      live at render time via existing accessors, not stored in the
//      snapshot — System and Uploads pages refresh every 7 s.
//   7. New `display_task` FreeRTOS task (gated by HAL_MULTIPAGE_ROTATION,
//      4 KB stack, priority 5). Boot splash visible 7 s before rotation
//      starts. Page enum + per-backend dispatch via switch.
//   8. Per-page render functions:
//      - OLED: `render_oled_env / _pm_mass / _pm_number / _uploads /
//        _system` in display.c
//      - SerLCD: `display_serlcd_render_env / _pm_mass / _pm_number /
//        _uploads / _system` in display_serlcd.c
//   9. OLED invert (0xA6/0xA7) toggles at the start of each new rotation
//      — anti-burn-in. First rotation keeps boot-splash polarity (normal).
//  10. New `display_set_contrast(uint8_t pct)` — backend-aware live
//      brightness apply. OLED writes contrast register; SerLCD writes
//      RGB backlight at white. Called from display_setup() at boot and
//      from /config save handler for live changes.
//  11. The old `display_environment()` / `display_serlcd_environment()`
//      APIs are gone. main.c now calls `display_update_snapshot()` once
//      per do_tx_cycle on multi-page boards. `display_running()` is
//      preserved for Heltec.
//  12. `display_set_status()` is a no-op on multi-page boards
//      (s_status[] still updated for symmetry) — the radiation-era
//      status line would otherwise overlay rotation pages.
//  13. New `main_target_enabled(int)` accessor — main.c exposes per-
//      target enable state so display.c (Uploads page) can hide disabled
//      targets without needing a config pointer.
//
// **Madavi compatibility fix:** `build_madavi_env_body()` previously
// emitted `BME280_pressure: 0.00` whenever any env sensor was present
// — sending fake 0 Pa data when only an SHT45 was fitted (no Bosch
// pressure chip). V2.3.29 uses sentinel-based field selection: emits
// the full BME280_* trio only when pressure is real (>1 hPa); falls
// back to DHT-style "temperature"/"humidity" for SHT45-only setups,
// or just "temperature" for sensors with neither H nor P. Madavi's
// hardcoded value_type whitelist routes the unprefixed names to its
// dht-highres.rrd file. (The same bug still exists in the
// sensor.community / openSenseMap / aqi.eco body builders — fixed
// only for Madavi at user request.)
//
// **Affected boards:** FeatherS3-D and QT Py get the full feature set.
// Heltec V2 / V2 4MB get the i2c_bus.c refactor (transparent — same
// bus, same Vext drive) plus the Madavi fix; everything else is bit-
// identical from V2.3.28 (display_running unchanged, no display task,
// no ALS, brightness dropdown UI present but only contrast register
// to drive on the Heltec OLED).
//
// **Rollback:** revert hal.h, display.h/.c, display_serlcd.h/.c,
// als.h/.c, i2c_bus.h/.c, env_sensor.h/.c, http_server.c, config.h/.c,
// main.c, transmission.c, version.h, CMakeLists.txt. The
// display_environment / display_serlcd_environment / env_sensor_get_i2c_bus
// APIs were removed — note for any external code that referenced them
// (none in this repo).
//
// OTA-safe from V2.3.28 (no partition layout / sdkconfig changes).
//
// V2.3.28 — external SSD1309 OLED on FeatherS3-D STEMMA2 (MVP, single page).
//
// **Headline:** add support for an external 2.42" SSD1309 128x64 OLED
// (Core Electronics CE09964) on the FeatherS3-D's SECOND STEMMA QT
// connector (STEMMA2 — IO15 SCL / IO16 SDA, powered from LDO2 / 3V3.2).
// Routing the OLED to STEMMA2 instead of STEMMA1 keeps cable runs short
// when the panel sits on the opposite side of the box from the sensors.
//
// **What's new:**
//   1. `hal.h` FeatherS3-D branch: `HAL_HAS_OLED` 0 → 1.
//   2. `display.c` `display_setup()`:
//      - Brings up STEMMA2 by driving IO39 HIGH (LDO2 enable) + creating
//        a second `i2c_master_bus_handle_t` on I2C_NUM_1 (IO15/IO16).
//      - Reset block (was unconditional `gpio_set_level(PIN_OLED_RST, ..)`)
//        is now `#ifdef PIN_OLED_RESET` — the external SSD1309 breakout
//        is 4-pin I²C (no reset line); chip POR handles it.
//      - SSD1306 init sequence reused unchanged — SSD1309 is register-
//        compatible. Charge-pump command (0x8D 0x14) stays in the
//        sequence; SSD1309 modules with onboard charge pumps ignore
//        it harmlessly.
//   3. New `display_environment(valid, t, h, p, noise_valid, noise_db,
//      use_display)` renders a single-page T/RH/P (+ optional LAeq) view.
//   4. `main.c` `do_tx_cycle()`: on FeatherS3-D, calls
//      `display_environment()` AFTER env + noise reads complete, instead
//      of the radiation-focused `display_running()`. Other boards
//      unchanged.
//
// **NOT in this release** (deferred):
//   - SENSORS on STEMMA2 — requires a per-driver bus-handle refactor
//     (see deferred memory `project_stemma2_software_enable_deferred`).
//   - Page rotation / multiple pages — single Environment view only.
//   - Radiation page on FeatherS3-D — display_running() suppressed on
//     this board variant since radiation isn't the headline metric in
//     the dust-sensor deployment context this targets.
//
// **Side effect: NeoPixel power.** Enabling LDO2 also powers the onboard
// WS2812 NeoPixel on IO40. We never drive its data line, so the WS2812's
// internal POR keeps it dark (no valid 24-bit RGB frame ever arrives).
// Quiescent current ~1 mA.
//
// **Affected boards:** FeatherS3-D ONLY. Heltec V2 / Heltec V2 4MB / QT Py
// builds are bit-identical to V2.3.27 — same hal.h non-FeatherS3-D
// branches, same call site for `display_running()` (gated by
// `#if !defined(BOARD_FEATHERS3_D)`).
//
// **Rollback:** revert the 4 files (hal.h, display.c, display.h, main.c).
// Or runtime: toggle `show_display=false` on /config to clear the panel
// without reverting firmware. Deployment context for this MVP is the
// dust sensor (esp32-5965048) in a clear-window box where the OLED is
// visible — not the Geiger which lives in a sealed PVC tube.
//
// OTA-safe from V2.3.27 (no partition layout changes, no sdkconfig
// changes).
//
// V2.3.27 — FeatherS3-D pin map: HV_FET + speaker moved off A2..A4.
//
// PCB harness rev: HV_FET_OUT moved from A2 (IO14) to A5 (IO5); SPEAKER_P
// moved from A3 (IO12) to D10 (IO3); SPEAKER_N moved from A4 (IO6) to D9
// (IO1). Frees the contiguous A2..A4 trio for future analog expansion
// (true ADC-capable channels live there on every ESP32-family Feather).
//
// Notes:
//   * IO3 (now PIN_SPEAKER_P) is an ESP32-S3 boot strap (JTAG vs USB-Serial-
//     JTAG select). The chip's internal pull-up holds it HIGH at boot →
//     default USB-Serial-JTAG mode. We only drive the line in speaker_setup()
//     which runs AFTER config + WiFi bring-up, so the strap reads correctly.
//     A piezo at hi-Z does not pull the line down at boot.
//   * A5 (IO5) was previously listed as the reserved future "HWTESTPIN" slot
//     in the wiring harness; that reservation is dropped here. No firmware
//     ever read HWTESTPIN in V2, so this is purely a wire-harness rename.
//
// Affected boards: FeatherS3-D ONLY. Heltec V2 / Heltec V2 4MB / QT Py
// builds are bit-identical to V2.3.26 — same diagnostic log changes, same
// transmission code, same UI. We still cut all 4 board binaries so each
// board's release artefact carries a consistent V2.3.27 tag.
//
// OTA-safe from V2.3.26 (no partition layout changes, no sdkconfig changes).
// 20 release artefacts (5 × 4 boards). FeatherS3-D installs MUST be
// flashed BEFORE re-attaching the new PCB harness — the old V2.3.26 will
// PWM-drive IO14 expecting HV_FET, which on the new harness is now A5's
// IO5 with no MOSFET attached (and IO5 is being driven HIGH from the new
// PCB side, harmless on the chip side).
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
#define VERSION_STR "V2.3.29"
