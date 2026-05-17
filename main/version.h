#pragma once
// Bump before build; commit after successful flash.
//
// V2.3.33 — fix silent /config page truncation + accidental-scroll value
// drift on number fields + drop auth from /log + web-UI security hardening.
//
// **Four unrelated web-UI changes shipping together.**
//
// **(A) Silent page truncation under longer field values.**
//
// *Symptom:* the /config page rendered partially on some boards — different
// devices were cut at different points (one feather stopped after the bold
// "Reboot" header, the Heltec stopped just before the "Firmware update"
// link, the other feather rendered fully). Replicable across reboots,
// fixed config — no transient state involved.
//
// *Root cause:* `config_get` builds the whole page into a single 8 KB
// heap buffer via one big `snprintf`, then `httpd_resp_send`s it. The
// static template alone is ~7 KB; once you add field values (escaped
// SSID, FTP host/user/pw/path, three NTP servers, TZ string, openSenseMap
// box+token, aqi token), longer configs blow past 8192. `snprintf`
// silently truncates and returns the would-have-been length. Different
// boards have different field values → different cut points; same board
// always cuts at the same place. Latent since V2.3.3 (the buffer was
// last sized then for the openSenseMap + aqi.eco additions).
//
// *Fix:*
//   1. `CFG_FORM_BUF_SIZE` 8192 → 16384 in `http_server.c`. Heap-only
//      during the few ms of request handling, freed immediately. Gives
//      ~8 KB headroom over the current worst case, plenty for many more
//      releases of form growth.
//   2. New `ESP_LOGE` after the `snprintf` if `n >= CFG_FORM_BUF_SIZE`
//      so any future truncation surfaces in serial instead of producing
//      another batch of silently-broken pages. Also clamps the send
//      length to the buffer so we never read past it.
//
// *Why not chunked sends:* the natural "robust" fix would be to convert
// `config_get` to chunked `httpd_resp_send_chunk` like `status_get`
// already does, but the cost/benefit didn't justify it: ~100 LOC of
// mechanical refactor across a critical UI page, multiplied retest
// matrix, and the actual problem here was "we had no alarm on
// truncation" — fixed by the one-line ESP_LOGE. The 16 KB ceiling is
// plenty until form growth or a future fragmentation issue forces the
// hand.
//
// **(B) Number inputs converted to text-with-inputmode to disable the
// wheel-decrements-value trap.**
//
// *Symptom:* `station_altitude_m` (and previously) values drifting by small
// multiples of the field's `step` between saves — Heltec set to 63.0 m
// reading back as 62.8 m (2 × 0.1 m), prior incident on 66.0 m → 65.8 m.
// Same UX trap caused, but never identified for years.
//
// *Root cause:* `<input type="number">` on both Chrome and Firefox treats
// the focused element as a mouse-wheel target — scrolling decrements or
// increments by `step`. User edits the field, finishes typing, then
// scrolls the page to reach the Save button; the focused number input
// eats the wheel events, value silently ticks down, Save persists the
// wrong value. Affected the three number fields on the form
// (`alt_m`, `ftp_int`, `tx_int_ms`).
//
// *Fix:* converted all three to `type="text" inputmode="decimal|numeric"`.
// No spinner arrows; no wheel-value behaviour; mobile keyboards still
// pop the numeric/decimal layout via the `inputmode` hint. Loses HTML5
// `min`/`max` live validation, but the POST handler already enforces
// the same bounds server-side (alt_m [-500, 9000], ftp_int [1, 1440],
// tx_int_ms [10000, 3600000]) — so functionally identical, just no
// in-browser red outline.
//
// **(C) /log no longer auth-gated.**
//
// One-click view from the unauth'd /status page. The ring buffer is
// diagnostic output (boot banner, WiFi/upload status, sensor cycle
// summaries) — same class of information the device already publishes
// to Madavi / sensor.community / Grafana publicly, so the password
// prompt added friction without protecting anything sensitive. /config,
// /update, /reboot remain password-protected.
//
// **(D) Web-UI security hardening — 4 fixes from the /config + /update
// audit.**
//
// All in `http_server.c`. Closes the realistic attack surface for a
// LAN sensor without pulling in HTTPS / signed-boot / NVS encryption
// (deferred: see audit notes).
//
//   1. **CSRF protection on POST handlers.** Basic-auth credentials are
//      cached per-origin by browsers and auto-attached to any subsequent
//      same-origin request — including cross-origin form POSTs from a
//      malicious page the admin happens to visit in the same browser
//      session. Without protection, attacker.com can submit
//      `<form action="http://device/config" method=POST>` with arbitrary
//      fields and the browser attaches cached `Authorization`. New
//      `check_same_origin()` helper requires the request's `Origin`
//      header (or `Referer` as fallback) to match `Host`. Applied to
//      `/config`, `/update`, `/reboot` POST handlers. Programmatic
//      clients (curl/scripts) need `-H "Origin: http://<device>:<port>"`.
//
//   2. **Constant-time credential compare.** `check_auth` previously
//      used `strcmp` to compare the received Authorization header
//      against the expected base64, which short-circuits on the first
//      differing byte — leaks position via response-time variance,
//      enables byte-at-a-time brute force on a timing-attack-capable
//      adversary. New `ct_memcmp()` inline helper runs over the full
//      buffer regardless of position of the mismatch. Length check is
//      still early-exit (length is non-secret).
//
//   3. **OTA `content_len` clamp to partition size.** `update_post`
//      previously trusted the client-claimed `content_len` as the recv
//      loop bound. Auth'd attacker could claim a giant size and dribble
//      bytes (slowloris) to hold the OTA partition open. Now rejected
//      with HTTP 400 before any erase happens.
//
//   4. **X-Frame-Options: DENY on /config, /update, /reboot.** Free
//      clickjacking protection — blocks framing from any origin (the
//      device never frames itself either). New `set_security_headers()`
//      helper called on success-response paths.
//
// **Deferred** (called out in audit, not addressed in this release):
// HTTPS migration (LAN-only, not worth heap+UX cost), signed-app OTA
// (production-hardening, key-management overhead), flash + NVS
// encryption (same), auth-failure rate limit / lockout (mostly
// informational without HTTPS).
//
// **Files touched:** `http_server.c` only. ~16 sites total: (A) buffer
// #define + truncation guard + log; (B) three input element
// conversions; (C) `check_auth` removal in `log_get` + status-page
// link text; (D) `ct_memcmp` helper, `check_same_origin` helper,
// `set_security_headers` helper, length-check + ct_memcmp in
// `check_auth`, `check_same_origin` calls in 3 POST handlers, size
// clamp in `update_post`, `set_security_headers` calls on 5 success-
// response paths.
//
// **Compatibility:** no NVS key changes, no sdkconfig changes, no
// partition changes. OTA-safe from V2.3.32. 20 release artefacts
// (5 × 4 boards). +8 KB transient heap during /config render only.
// Programmatic POSTs from outside a browser now require an explicit
// `Origin` or `Referer` header (curl needs `-H "Origin: http://..."`).
//
// V2.3.32 — config-page polish + status-page Madavi link + display OFF.
//
// **Three small UX changes — all in http_server.c + display.c:**
//
//   1. Config page label rename. "Drive OLED display" → "Enable Display".
//      "OLED brightness" → "Display brightness". The OLED-only labels were
//      misleading on FeatherS3-D + Heltec where the field also drives the
//      SerLCD backlight (V2.3.28 added SerLCD support but didn't relabel
//      the form). NVS key stays `oled_bright` — no migration needed.
//
//   2. Brightness dropdown gains an "OFF" entry (value 0). Lets the user
//      put the display fully dark without unchecking "Enable Display"
//      (which would also disable the multi-page render task). On OLED,
//      0xAE puts the panel into sleep mode — segment + common drivers
//      off, charge pump retained, RAM contents preserved → instant
//      re-enable on the next non-zero brightness write. On SerLCD, the
//      RGB backlight goes to (0,0,0); the LCD glass is still being
//      driven so a strong external light would show faint text, but in
//      a sealed-tube deployment that's invisible.
//      `display_set_contrast()` previously clamped pct < 10 to 10; that
//      clamp is removed so 0 reaches the backend.
//
//   3. Status page: per-chip Madavi link in the bottom links block. Only
//      emitted when `send_madavi=1` in /config — no point linking to a
//      graph page with no data behind it. Resolved at HTML render time
//      (no JavaScript), so toggling Madavi in /config only changes the
//      link's visibility on the next status-page load. URL pattern:
//      api-rrd.madavi.de:3000/grafana/d/q87EBfWGk/temperature-humidity-pressure?var-chipID={chip}
//      (the dashboard ID is what the user's browser was on 2026-05-16 —
//      if Madavi ever restructures Grafana paths we'll need to refresh).
//
// **Files touched:**
//   * `http_server.c` — 4 sites: brightness <option> builder (prepend OFF),
//     config form labels, POST validator (accept 0), status page links
//     split into HEAD/TAIL with conditional Madavi link injection.
//   * `display.c` — 1 site: `display_set_contrast()` interprets 0 as OFF
//     for both backends; documented sleep-mode semantics.
//
// **Compatibility:** no NVS key changes, no sdkconfig changes, no
// partition changes. OTA-safe from V2.3.31. Existing brightness values
// (10..100) keep working unchanged. 20 release artefacts (5 × 4 boards).
//
// V2.3.31 — fix sub-tick `vTaskDelay` timing in I²C drivers (SHT45 H=0% root
// cause + audit-driven sweep across the rest of the env / PM stack).
//
// **Headline:** at the ESP-IDF default `CONFIG_FREERTOS_HZ=100` (10 ms tick),
// `vTaskDelay(pdMS_TO_TICKS(N))` for N ≤ 10 evaluates to `vTaskDelay(1)` —
// a 1-tick yield that actually sleeps **0..10 ms** depending on the call's
// phase relative to the next tick boundary. Several driver post-command
// waits assumed millisecond precision and were silently shorter than the
// chip's conversion / response time, causing intermittent failures that
// looked like flaky hardware.
//
// Most visible symptom: SHT45 returning H=0.00% with valid T (cycles 1358
// + 1409 on esp32-5965048), or post-measure NACK `ESP_ERR_INVALID_RESPONSE`.
// SHT45 measures T first then H; if the read happens at 4-9 ms the T
// register is fresh but the H register is still 0x0000, with CRC byte 0x81
// (CRC-8 of `00 00`) which **passes** the integrity check. Looked like a
// chip fault for V2.3.0..V2.3.30. The other SHT45 in the user's stock had
// the same behaviour, just less often → diagnostic clue that proved it
// wasn't a chip fault.
//
// **Fix pattern:** for sub-20 ms timing-critical waits, replace
// `vTaskDelay(pdMS_TO_TICKS(N))` with `esp_rom_delay_us(N * 1000)` — a
// precise busy-wait via the system RTC. CPU cost is real but trivial:
// 15 ms once per 150 s TX cycle = 0.01 % CPU. For ≥ 20 ms waits,
// `vTaskDelay` is fine (the worst-case ±10 ms quantisation is small
// relative to the wait).
//
// **Files touched:**
//   1. `sht45.c` (4 sites) — post-measurement wait in `sht45_read()` (THE
//      cycle-bug fix), post-measurement wait in `try_init_pass()`
//      (init-time variant of same bug), serial-read wait in
//      `sht45_read_serial()`, post-soft-reset wait in `try_init_pass()`
//      (conditional: busy-wait if requested wait < 20 ms, else vTaskDelay).
//   2. `bmp581.c` (1 site) — forced-mode 12 ms post-conversion wait.
//      Chip needs 11.4 ms; 1-tick vTaskDelay was 0..10 ms = below spec.
//   3. `bme280.c` (1 site) — bumped 55 → 70 ms target on post-measurement
//      wait. Chip needs 46.1 ms; vTaskDelay(pdMS_TO_TICKS(55)) at 100 Hz
//      = 5 ticks = 40..50 ms actual = sometimes below spec. New value
//      gives 60..70 ms minimum 60 ms. Kept as vTaskDelay (busy-wait is
//      heavy at 50 ms; an extra 10 ms on the yield is cheap).
//   4. `sps30.c` (4 sites) — Sensirion-style 5 ms inter-command waits in
//      serial read, data-ready poll, measurement read, status read.
//      Datasheet allows up to 5 ms response; 1-tick vTaskDelay was 0..10 ms
//      = could miss the response window.
//   5. `veml7700.c` (1 site) — post-wake 5 ms wait. Datasheet tWAKE = 2.5 ms.
//
// **Files audited and intentionally NOT changed:**
//   * `bme688.c` — post-measure 60 ms (6 ticks = 50..60 ms actual; chip
//     max 46 ms; thin margin but always above spec). Reset-poll waits are
//     inside iteration loops where natural retry covers any short wait.
//   * `bmp390.c` — post-measure 30 ms (3 ticks = 20..30 ms; chip needs
//     ~13 ms; comfortable margin).
//   * `i2c_bus.c` — LDO2 settle 10 ms (LDO2 hardware turn-on is ~1 ms;
//     even 0 ms would be fine).
//   * `bme280.c` reset/NVM polling — short waits inside polling loops.
//
// **Pin-cite primary source:** Sensirion SHT45 datasheet §"Measurement
// Conversion Time" specifies typ 8.2 ms / max 9.4 ms for high-precision
// command 0xFD, and §"Measurement Sequence" confirms T-then-H ordering.
// FreeRTOS ESP-IDF port docs confirm `pdMS_TO_TICKS()` truncates and
// `vTaskDelay(N)` actual sleep is `(N-1)..N × tick_period`.
//
// **Affected boards:** all four (FeatherS3-D, QT Py ESP32-PICO, Heltec V2,
// Heltec V2 4MB) — same FreeRTOS tick rate, same drivers, same fix.
// Production dust node esp32-5965048 (FeatherS3-D, SHT45 + BMP581 + SPS30)
// gets all five driver fixes.
//
// **Memory notes:** `reference_sht45.md` (NEW) — full SHT45 reference
// including the FreeRTOS sub-tick trap section. `feedback_freertos_subtick_vtaskdelay.md`
// (NEW) — generic feedback so future drivers don't repeat the pattern.
//
// **Rollback:** revert sht45.c, bmp581.c, bme280.c, sps30.c, veml7700.c,
// version.h. No public API changes, no sdkconfig changes, no partition
// changes — pure timing fix. OTA-safe from V2.3.30. 20 release artefacts
// (5 × 4 boards).
//
// V2.3.30 — sensor serials at boot + VEML7700 ambient-light driver.
//
// **Headline:** small additive release — diagnostic-friendly serial-number
// logging for two sensors that have factory-burned unique IDs, plus a new
// I²C ambient-light sensor driver. No architecture changes, no removed
// APIs. OTA-safe from V2.3.29.
//
//   1. **SHT45 serial logged at init** (`sht45.c`) — read via cmd 0x89,
//      logged as 8-char hex. Diagnostic aid for distinguishing physical
//      chips when several are in dev rotation. Surfaced after a faulty
//      Adafruit #6174 SHT45 (post-measure NACKs in steady state, init
//      OK) needed identification across board swaps.
//   2. **SPS30 serial logged at init** (`sps30.c`) — read via cmd 0xD033,
//      32-char ASCII (typically null-padded to ~16). All 16 word-pair
//      CRCs validated; failure non-fatal.
//   3. **New Vishay VEML7700 driver** (`veml7700.c/.h`) — I²C ambient-
//      light sensor at fixed address 0x10. Auto-detected via the
//      existing `PROBE_ON_BOTH_BUSES` chain — works on STEMMA1, STEMMA2
//      (FeatherS3-D), or the single shared bus on Heltec / QT Py.
//      Returns 3 measurements: lux (computed from raw ALS × resolution
//      with polynomial non-linearity correction), raw 16-bit ALS count,
//      raw 16-bit white-channel count. Default config: gain 1/8×, IT
//      100 ms → 0.54 lux/count, ~0–35 klux range. Designed to coexist
//      with the existing onboard ALS-PT19 (analog, FeatherS3-D only).
//   4. **`/status` "Ambient light" block extended** to render BOTH
//      sensors when present — VEML7700 (lux + raw ALS + raw white) and
//      ALS-PT19 (mV + approx lux). Block is omitted entirely if
//      neither is fitted.
//
// **Affected boards:** all four (FeatherS3-D, QT Py, Heltec V2, Heltec
// V2 4MB). All gain the SHT45 + SPS30 serial logging. VEML7700 driver
// compiles into every board's binary but only activates when the chip
// is physically present (probe at 0x10) — no impact on boards without
// one. Heltec / QT Py users can plug a VEML7700 breakout into their
// shared I²C bus and the /status row appears automatically.
//
// **No removed APIs.** SHT45 / SPS30 / display / i2c_bus / env_sensor
// public surfaces unchanged. Pure additive release.
//
// OTA-safe from V2.3.29 (no partition layout / sdkconfig changes).
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
#define VERSION_STR "V2.3.33"
