# MultiGeiger-V2 — Changelog

Per-release WHAT/WHY notes. Extracted from `main/version.h` in **V2.4.1** so the release archaeology doesn't bloat every build's parse cycle.

- **V2.4.1 down to V2.3.23**: full notes preserved from the pre-V2.4.1 `version.h` header.
- **V2.3.22 down to V2.1.11**: one-line headlines only — full bodies live in the GitHub release for each tag at <https://github.com/MMBytes/MultiGeiger-V2/releases>.

For build / flash / release workflow see `README.md` and the `_build.cmd` / `_merge.cmd` / `_flash.cmd` helpers at the repo root.

---

## V2.6.24 — led_tick prefers the NeoPixel on dual-LED boards

Boards that have both a plain user LED and a WS2812 NeoPixel
(`adafruit_esp32s3_feather_4mb_2mbpsram` #5477 and
`adafruit_esp32s3_tft_feather` #5483) now flash the **NeoPixel** (blue,
40 ms) on each Geiger pulse instead of the red "#13" LED.

- WHY: the original port decision routed `led_tick` to `PIN_LED_BUILTIN`
  purely because that reused `speaker.c`'s existing `#ifdef` chain with zero
  code changes — not for any GPIO or electrical reason (design spec §2,
  "no new code, existing precedent"). At #5477 bench bring-up the user
  chose the NeoPixel as the intended pulse indicator wherever one exists.
- `speaker.c` `tick_start()` now checks `HAL_HAS_NEOPIXEL` first and falls
  back to `PIN_LED_BUILTIN` only on NeoPixel-less boards (Heltec V2/V4,
  feathers3_d). Boards with a NeoPixel and no plain LED (QT Py, SparkFun
  S3/C5, Feather V2) are unchanged — they already used the NeoPixel.
- The red "#13" LED on the two dual-LED Feathers is still configured and
  driven to a deterministic OFF at boot, reserved for future status use.

---

## V2.6.23 — LoRaWAN uplink (heltec_wifi_lora32_v4_r2) + generalized keep-AP-on

Ships the LoRaWAN feature line built across Tasks 1-8: OTAA/Class A uplink
support for the `heltec_wifi_lora32_v4_r2` board via RadioLib 7.2.1, plus a
config-UI checkbox (keep the AP on permanently) that was previously
SD-standalone-only and is now available on every board.

### LoRaWAN uplink (heltec_wifi_lora32_v4_r2 only)

- New `lorawan.cpp` drives an SX1262 radio over the module's dedicated
  internal SPI bus (RadioLib 7.2.1), joining via OTAA and transmitting
  Class A uplinks on a duty-cycle-guarded schedule.
- Uplink payloads on FPort 1 (GM counts/interval/firmware version/tube type)
  and FPort 2 (temperature/humidity/pressure) are byte-identical to the
  Multigeiger V1.9 Arduino firmware's TTN frames (`lorawan_codec.h`,
  ported from `transmission.cpp`'s `send_ttn_geiger`/`send_ttn_thp`), so
  existing TTN Console decoders and ttn2luft ingestion keep working
  unchanged. A ready-to-paste TTN "Custom Javascript formatter" is now
  published at `docs/ttn_payload_formatter.js`.
- `/config` gained an all-region LoRaWAN section (frequency-plan dropdown
  covering the regions RadioLib supports, DevEUI/JoinEUI/AppKey entry,
  join status). Session state (DevAddr, session keys, frame counters)
  persists across reboots in NVS so a device doesn't have to rejoin on
  every power cycle.
- Two checkboxes — `lora_fem_en` ("Drive FEM enable (GPIO2)") and
  `lora_hp` (FEM high-power/PA select, GPIO46) — are exposed for
  **reworked hardware only**. On stock/unmodified boards, GPIO2 is
  double-booked with the HV cap-full comparator input and GPIO46 with the
  mainboard's DIP1 switch; asserting either pin on unmodified wiring would
  contend with the live signal already driving it, so both default off and
  must stay off unless the board has had the corresponding copper rework
  done. See `hal.h`'s `PIN_LORA_FEM_EN`/`PIN_LORA_FEM_PA` comments for the
  full double-booking rationale, and note that on unmodified boards the
  GC1109 front-end's RX-enable line is left wiggling by the comparator, so
  LoRaWAN RX (downlink) reliability there remains an open bench question.
- Bench status, stated plainly: radio bring-up (SPI init, RadioLib object
  construction) and the join-request TX + backoff/retry path are
  bench-verified on real V4-R2 hardware. Join-accept and full end-to-end
  uplink delivery are **not yet** confirmed — that leg is pending
  availability of a TTN gateway in range of the bench unit, not a known
  code defect.

### Generalized keep-AP-on checkbox (all boards)

- The "keep config AP on permanently" checkbox (`sd_ap_on` internally, for
  history — it started as an SD-standalone-only option) is now honored by
  `main.c` in every operating mode, not just SD-standalone. One latch
  (`s_standalone_ap_on_latched`), captured once at boot and never
  re-read from `g_cfg` afterward, decides whether the AP + httpd stay up
  forever or the radio fully powers down at the config window's close —
  same semantics as before, just no longer conditional on SD-standalone
  being active.

---

## V2.6.22 — V2.6.19 deferred items + V2.6.20 review follow-ups

Two batches in one (unreleased) version: the deferred-item list from the
V2.6.19 final review (Part 1), and the findings of a single-agent Fable 5
MAX review of the v2.6.20 release diff (Part 2 — verdict: sound as shipped,
zero Critical/High/Medium; all findings were documentation hygiene plus one
deliberately-investigated robustness item).

### Part 2 — V2.6.20 review follow-ups: Vext doc rot + loud GPIO bring-up

#### F1/F2 — the "no gating GPIO" falsehood, eradicated (Low)

- v2.6.20 fixed the wrong "Vext doesn't gate the OLED" claim in three places
  but left it standing in three others: `i2c_bus.h`'s per-board behaviour
  block, its `i2c_bus_get_secondary()` doc comment ("there is no LDO to drop
  — the bus stays up regardless"), and `i2c_bus_finalize()`'s V4-R2 branch
  ("No gating GPIO on this board"). All three now state the truth: the OLED
  bus is Vext_Ctrl-gated (GPIO36, active-LOW), driven before controller
  creation, and the rail is deliberately never dropped (a missing consumer
  means a failed OLED probe, not a power-save case). The original V2.6.7 bug
  was born from a wrong power claim outliving its refutation — this class is
  now clean.

#### F3/F5 — hal.h PIN_VEXT comment accuracy (Informational)

- Reserved-list GPIO36 bullet no longer claims it was "moved out of this
  list" while sitting in it: it stays reserved (it's the module's power-gate
  control, never free GPIO), it's just firmware-driven now.
- PIN_VEXT comment documents the side effect that the same MOSFET feeds the
  external "Ve" header (500 mA peripheral rail) — energized from boot on
  every unit since v2.6.20; relevant to battery deployments.

#### F6 — ESP_ERROR_CHECK on boot-time GPIO bring-up (investigated, applied)

- The review called the unchecked `gpio_reset_pin`/`gpio_set_direction`/
  `gpio_set_level` triples "house pattern" — investigation showed the house
  is split: `gpio_config()` users check loudly (tube.c, speaker.c ×
  ESP_ERROR_CHECK; led.c, neopixel.c × log), the triple was never checked
  anywhere. Per IDF v6.0 source these calls fail ONLY on invalid or
  input-only pin constants — a compile-time board-port mistake (classic
  ESP32's GPIO34-39 are input-only, and `gpio_set_direction` silently no-ops
  on them; three classic-ESP32 boards live in this tree). Such a mistake is
  deterministic per board, so it can never pass the bench then fail in the
  field — abort semantics are safe, same argument as the radio-off
  `ESP_ERROR_CHECK` pair.
- Applied to the run-once bring-up sites: all four rail-gate blocks in
  `i2c_bus.c` (Heltec V2 Vext, TFT-Feather/#5477 I2C power gate, FeatherS3-D
  LDO2, V4-R2 Vext), the OLED reset-pin setup in `display.c`, and the
  antenna-select in `main.c`. Hot-path `gpio_set_level` calls (speaker/tube/
  led per-tick) deliberately left bare — abort risk in pulse paths for zero
  diagnostic value.
- Converts a future misport from "every I2C probe times out, bench session
  to root-cause" (the exact V2.6.7 dead-OLED signature) into an instant
  first-boot abort naming the offending line.

#### F4 (failure-path Vext asymmetry) — no change

- On `i2c_new_master_bus()` failure the V4-R2 branch leaves Vext on (unlike
  FeatherS3-D's LDO2 back-off). Verified unreachable in practice: one call
  site per boot, and bus-controller creation failing is effectively fatal.
  Documented here; not worth code.

### Part 1 — V2.6.19-review deferred items (diagnostic accuracy + doc rot)

Clears the deferred-item list from the V2.6.19 final review (items 2–7; items
10/11 re-verified as already matching house style / already in place, no
change — though item 10's "house pattern" claim was later overturned for the
GPIO case, see Part 2 F6).

#### /status "Last error" now names the actual error (M2, item 5)

- `sd_logger.c::fail_cycle()` mixed two error domains into one `int` rendered
  with a bare `%d`: `sd_card_mount()` failures are `esp_err_t` (so
  `ESP_ERR_TIMEOUT`/`0x107` displayed as a baffling "263"), while file
  create/write failures are `errno`. `fail_cycle()` now tags the domain,
  `sd_logger_status_t` carries `last_err_is_esp`, and /status renders
  `ESP_ERR_TIMEOUT` vs `errno 5 (I/O error)` accordingly (serial log line
  likewise).
- `errno` is now captured BEFORE `close_file_for_remount()` — the cleanup's
  own `fclose`/unmount could overwrite it, so the number shown on /status
  could be the cleanup's errno, not the real failure's. Diagnostic accuracy
  only; retry/alert behaviour unchanged.

#### /status SD card cosmetics after card pull (M2, items 7)

- Row count no longer lingers: after a card pull, /status showed the dead
  file's `rows_written` next to no filename. `close_file_for_remount()` now
  zeroes the count together with the filename.
- Torn-filename read closed: the httpd task snapshots the filename while the
  main service task creates/clears it. Both sides now go through a tiny
  `portMUX` critical section (`s_status_mux`), and `create_file()` publishes
  the new name only after the header row is safely fsync'd — /status can
  never show a half-written name or a name with no live file behind it.

#### Header-length contract made explicit (item 4)

- A future sensor registering a CSV column header longer than 23 chars would
  have silently truncated the CSV header row (data rows still full-width — a
  misaligned CSV with no error anywhere). New `TELEMETRY_HEADER_MAX_LEN` (23)
  in `telemetry.h` documents the limit; `telemetry_register()` now rejects
  over-long headers loudly (same non-fatal drop contract as registry-full);
  `sd_logger.c` static-asserts the cap against its `CELL_MAX` row budget.
  Today's longest header is 22 chars — no behaviour change for any current
  sensor.

#### Doc rot + cosmetics (items 2, 3, 6)

- `sd_logger.c`: two stale claims fixed — "status read unconditionally on
  every board" (the /status card is `HAL_HAS_SD_CARD`-gated since the final
  review) and "init called unconditionally from main.c" (mode-gated since the
  A2 boot-latch).
- `sd_logger.h`: "DateTime UTC" column doc updated for V2.6.21's local-time
  change (header is now "DateTime", local time with numeric UTC offset;
  filename stamp likewise local).
- `fuel_gauge.c`: comment claimed env_sensor's dual-bus probe can call
  `fuel_gauge_init()` twice — main.c calls it exactly once and the `s_ready`
  guard would short-circuit a second call anyway; the `s_tm_registered` guard
  is belt-and-braces, not load-bearing. Justification corrected, guard kept.
- `hal.h`: `HAL_HAS_SD_CARD` value column was 2 chars right of its neighbours
  in the four narrow-alignment board sections (heltec_v2, feathers3_d,
  adafruit_qtpy_esp32_pico, seeed_xiao_esp32s3); the V2.6.19 GPIO25
  reserved-pin comment rewrapped to house width.

#### Geiger-pulse NeoPixel flash: blue, was red (bench-test follow-up)

- On the four boards whose pulse indicator is the NeoPixel (QT Py, SparkFun
  Thing Plus S3/C5, Adafruit Feather V2), the per-count flash is now dim
  blue (0,0,20) instead of dim red — a tube pulse can no longer be confused
  with the solid-red SD-failure alert at a glance, and during an active
  alert the pixel reads "blue blink per count, solid red in between".
  Boards whose pulse tick is a plain onboard LED (TFT Feather, #5477, all
  non-NeoPixel boards) are unaffected. Alert colour unchanged (red, 64,0,0).

#### Per-cycle write confirmation in /log (bench-test follow-up)

- `sd_logger: row N written to <file>` at INFO after every successful
  (fsync'd) CSV row. Previously only failures logged, so /log showed the
  sensor readings but never that they landed on the card — "logging fine"
  and "logging silently stopped" looked identical between error lines.

#### Verified no-change items (10, 11)

- Trailing `append_safe()` calls without `(void)`: grep confirms zero
  `(void)append_safe` casts exist anywhere in `main/` — bare trailing calls
  ARE the house style (http_server.c, transmission.c, mqtt.c, sd_logger.c
  all agree); cppcheck gate is clean on it. Left as-is.
- `ESP_ERROR_CHECK` on the standalone radio-off `esp_wifi_stop()`/
  `esp_wifi_deinit()` pair (main.c): abort re-verified unreachable — the
  branch only runs after WiFi init succeeded at boot, and both calls can
  only fail with `ESP_ERR_WIFI_NOT_INIT`. Consistent with the adjacent
  networked AP-window branch. Left as-is.

## V2.6.21 — Standalone SD-logging: fix card-pull crash, local-time CSV, unified heap diagnostics

### Fix SD card-pull crash (LoadProhibited, httpd task)

- Bench-crashed twice (2026-07-16) pulling the SD card live on
  `sparkfun_thing_plus_esp32s3`: `LoadProhibited` at `excvaddr=0xc`, backtrace
  `esp_vfs_select → httpd_server → httpd_thread`, always on the `httpd` task,
  always right after an unmount.
  - Root cause (confirmed via coredump SHA-verified decode, not a race):
    ESP-IDF's `esp_vfs_unregister_with_id()` NULLs the freed slot in the
    global `s_vfs[]` VFS table but never shrinks `s_vfs_count`, leaving a
    *persistent* hole until the next successful mount refills it.
    `esp_vfs_select()`'s own NULL-slot guard (`vfs_calls.c:660`) then executes
    `ESP_LOGD(..., vfs->offset)` on a NULL `vfs` — `offset` sits at struct
    byte-offset `+0xc`, matching both coredumps exactly. Deterministic on
    httpd's very next `select()`, no concurrency required, once the card is
    out and every remount keeps failing.
  - Fix: `CONFIG_LOG_MAXIMUM_EQUALS_DEFAULT=y` (was
    `CONFIG_LOG_MAXIMUM_LEVEL_DEBUG=y`) in `sdkconfig.defaults` — compile-time
    ceiling drops to INFO, dead-code-eliminating that `ESP_LOGD` line
    project-wide. Doesn't touch any `wifi:`-tagged driver output (that's all
    I/W-level, unaffected by the ceiling either way).
  - `sd_card.c` also gained `run_serialized_with_httpd()`: mount/unmount now
    run via `httpd_queue_work()` onto the httpd task itself (new
    `http_server_get_handle()` getter), kept as defense-in-depth against the
    separate, genuine ESP-IDF-acknowledged race where a cross-task unregister
    can free a `vfs_entry_t` while another task's in-flight `select()` has
    already fetched a pointer to it.
  - Bench-confirmed fixed: full pull → write-fail → unmount → retry-mount-fail
    (card still out) → retry-mount-succeed (card reinserted) → new CSV
    created → normal steady-state cycles, zero crashes throughout.

### Local time in standalone CSV (was hardcoded UTC)

- `ntp_setup()`'s `setenv("TZ", ...)` step split into its own
  `ntp_set_timezone()`, now called unconditionally at boot right after
  `config_load()`. Standalone boards never reach `ntp_setup()` (no STA, so no
  `GOT_IP`), so they never got past the UTC default despite `tz_posix` being
  configured. Harmless on networked boards — `ntp_setup()` re-applies the
  same value once `GOT_IP` fires.
- `sd_logger.c`'s CSV filename timestamp and per-row `DateTime` column switch
  from `gmtime_r()`/hardcoded `"Z"` to local time (`localtime_r()` /
  `ntp_localtime_str()`, the same helper syslog.c already used) — the
  underlying clock is still GPS-set UTC underneath, only the display
  conversion changes.

### Unified per-cycle heap diagnostics

- New `diag_log_heap_standalone()` (`diag.c`/`diag.h`): one combined
  free/min_free/max_alloc + INTERNAL/DMA capability-split line. Standalone
  mode never reaches `tx_run()` (bypasses `tx_transmit()` entirely), so it had
  zero per-cycle heap visibility in `/log` until now; `do_tx_cycle()` in
  `main.c` calls it directly.
- `transmission.c`'s `tx_run()` switched to the same helper, replacing its old
  two-line `ESP_LOGI(...)` + `diag_log_heap("per-cycle")` pair (V2.3.17 /
  V2.4.32) with one shared line — networked and standalone `/log` output now
  match exactly here.

## V2.6.20 — Heltec WiFi LoRa32 V4-R2: fix dead onboard OLED (Vext-gated rail)

- First real V4-R2 unit shipped with the OLED's dedicated I²C bus
  (I2C_NUM_1, GPIO17/18) timing out on every transaction (not NACKing —
  timing out), the classic signature of an unpowered I²C target. The
  V2.6.7 port's `HAL_HAS_VEXT_GATE=0` for this board rested on a single,
  never-bench-tested datasheet reading that GPIO36 (`Vext_Ctrl`) only
  gates the external "Ve" header, not the OLED.
- Root cause confirmed on real hardware: that reading was wrong. This
  board's Vext_Ctrl gates the OLED rail, matching the well-documented
  Heltec V3-generation gotcha (same SDA=17/SCL=18/RST=21 OLED pinout this
  board reuses) where the OLED stays dark until Vext is pulled low.
- Fix: `i2c_bus_get_secondary()` now drives GPIO36 LOW (new `PIN_VEXT` for
  this board) before the OLED bus comes up, mirroring the same active-LOW
  gate pattern already used for Heltec V2's shared OLED+sensor rail.
  Bench-verified 2026-07-15: I2C timeouts cleared (`i2c_err=0`), OLED
  enumerates as SSD1315 @ 0x3C and displays correctly.
- Kept as a bus-local drive in `i2c_bus_get_secondary()` rather than
  folding into `HAL_HAS_VEXT_GATE` — that flag's one existing code path
  gates the *primary* bus (`i2c_bus_get_primary()`), which on this board
  is the unrelated external sensor header, not the OLED.

## V2.6.19 — Standalone SD-logging mode (offline field logger); log STA IP/gateway/netmask/DNS to syslog

### Standalone SD-logging mode (offline field logger)

- NEW standalone mode (SparkFun Thing Plus ESP32-S3 / ESP32-C5, the two
  boards with microSD slots): after the boot AP config window the radio
  turns fully off and every TX cycle appends one CSV row of all attached
  sensors' readings to a FAT32 card. Optional "keep AP on permanently"
  sub-mode for in-field monitoring via /status (uses more battery).
- GPS becomes the coarse clock in standalone mode (first-fix step, then
  hourly, 10 s threshold); networked nodes keep NTP-only (V2.5.11 rule).
- CSV: pinned DateTime/Uptime/GPS columns, then per-sensor columns sorted
  by name (stable across boots/OTAs — files always merge). Per-sensor
  readings (no fused values): SHT45/BMP581/BMP390/BME688/BME280, SPS30,
  DNMS, SGP41 NOx, VEML7700, battery %/V, tube CPM/dose/counts/HV/window.
- File per boot session: esp32-<chipid>_YYYYMMDD_HHMMSS.csv; creation
  waits for first GPS fix; fsync per row; card-pull recovery with new
  file on remount; 3 consecutive failures = red NeoPixel + /log error.
- S3 mounts via SDMMC 4-bit (native protocol, data CRC); C5 via SPI (no
  SDMMC peripheral on that chip). exFAT unsupported (IDF hardcodes it
  off) — cards >32 GB must be reformatted FAT32.

### Log STA IP/gateway/netmask/DNS to syslog

- New `ESP_LOGI` line ("wifi ip: ...") in `main.c`, fired right after the
  "wifi link: ..." line added in V2.6.16 — same one-shot block that already
  emits the `config:` dump (V2.5.23). Reports the STA's IP address, gateway,
  network mask, and both DNS servers via `esp_netif_get_ip_info()` /
  `esp_netif_get_dns_info()`.
  - Motivation: `on_ip_event()` already logs this at `GOT_IP`, but that fires
    before `syslog_init()` (syslog only starts once STA has an IP) so it
    never reaches the rsyslog server — the same gap the V2.6.16 `wifi link:`
    line closed for RF/association details.

## V2.6.18 — Fix OTA upload stall on heavy-TX-target PSRAM nodes (recv-mailbox ceiling)

- Repeated `/update` OTA failures on esp32-5965048 (feathers3_d): the 1.3+ MB
  upload stalled at an exact TCP-MSS-multiple byte offset (`8640` = 6×1440,
  or `11520` = 8×1440), then sat with zero received bytes through all
  5 × 30s `httpd_req_recv()` retries before aborting — reproduced both after
  multi-day uptime and immediately after a fresh reboot (~48 min uptime),
  ruling out the older long-uptime heap-fragmentation theory as sufficient
  on its own.
  - Root cause: `CONFIG_LWIP_TCP_RECVMBOX_SIZE` was never set by this
    project and defaulted to IDF's stock value of 6 — the depth of the
    queue lwIP uses to hand received TCP segments to the app before
    `recv()` drains them, decoupled from the byte-level
    `CONFIG_LWIP_TCP_WND_DEFAULT` (16384). `update_post_inner()`'s recv
    loop (`main/http_server.c`) calls `esp_ota_write()` — synchronous flash
    erase/write — between `httpd_req_recv()` calls; on a node whose
    per-cycle TX load is heavy enough (this fleet's most TX targets: Madavi,
    sensor.community, ThingSpeak PM, aqi.eco, openSenseMap ×2, MQTT — 7
    sequential TLS sessions every ~3 minutes churning internal RAM harder
    per wall-clock minute than lighter nodes), the mailbox fills before the
    byte window does, lwIP stops accepting further segments, and the
    resulting stall can outlast the retry budget. The RECVMBOX ceiling
    itself is universal to every PSRAM board — only this node's heavier
    per-cycle churn has pushed it far enough to actually trip it.
  - Fix: `CONFIG_LWIP_TCP_RECVMBOX_SIZE=24` added to
    `sdkconfig.defaults.psram` (all 8 PSRAM boards), not the feathers3_d
    overlay alone — the mechanism isn't board-specific, and every PSRAM
    board shares the same OTA code path and the same stock default. Scoped
    to PSRAM boards (not the shared base `sdkconfig.defaults`, which the
    non-PSRAM Heltec boards also read) because the mailbox entries ride the
    same `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP` net-stack-to-PSRAM offload
    as the rest of the WiFi/lwIP buffers, so the extra depth doesn't cost
    internal RAM here the way it would on Heltec.

- Fix ESP32-C5 STA bandwidth still failing under `WIFI_BAND_MODE_AUTO`
  after V2.6.17 (`set_bandwidths(...) failed: ESP_ERR_INVALID_ARG` with
  `2g:0x47` — 11B/11G/11N/11AX — still logged live on a C5 node).
  - Root cause: V2.6.17 added the `max_legal_bw()` helper in
    `main/main.c` specifically to keep the requested bandwidth legal for
    whatever protocol mask was just set (`WIFI_BW40` is rejected outright
    when `WIFI_PROTOCOL_11AX`/`11AC` is in the mask), and its doc comment
    claimed both bands "now go through the same helper instead of a
    hardcoded `WIFI_BW20`" — but `apply_radio_limits_sta()` itself was
    never actually changed to call it: `bw.ghz_2g`/`bw.ghz_5g` were still
    assigned `WIFI_BW40`/`WIFI_BW20` directly, so an HE-capable 2.4 GHz
    mask kept requesting the illegal 40 MHz width every boot.
  - Fix: wire `bw.ghz_2g`/`bw.ghz_5g` through `max_legal_bw(proto.ghz_2g)`/
    `max_legal_bw(proto.ghz_5g)` as the comment already described, only
    short-circuiting to `WIFI_BW20` when the user's `ht20_only` checkbox
    is set.

---

## V2.6.17 — Fix ESP32-C5 STA radio-limit APIs under WIFI_BAND_MODE_AUTO

- `apply_radio_limits_sta()` (`main.c`) previously always called the
  single-band `esp_wifi_set_protocol()`/`esp_wifi_set_bandwidth()`, which
  IDF returns `ESP_ERR_NOT_SUPPORTED` for under `WIFI_BAND_MODE_AUTO` — the
  default band mode on the ESP32-C5 (this fleet's only dual-band chip),
  since the firmware never calls `esp_wifi_set_band_mode()`. Both calls
  failed silently, leaving the `wifi_11bg_only`/`wifi_ht20_only` config
  flags un-applied on that board.
  - Fix: branch at runtime on `esp_wifi_get_band_mode()`. Under
    `WIFI_BAND_MODE_AUTO`, use the plural per-band
    `esp_wifi_set_protocols()`/`esp_wifi_set_bandwidths()` (separate
    `ghz_2g`/`ghz_5g` fields) instead; single-band boards keep the original
    calls unchanged.
  - `wifi_11bg_only`/`wifi_ht20_only` are 2.4 GHz-only concepts (5 GHz has
    no 11b/g), so `ghz_5g`'s protocol set is always the fullest
    (11A/11N/11AC/11AX) regardless of `wifi_11bg_only`.
  - `ghz_2g`'s protocol set includes `WIFI_PROTOCOL_11AX` when
    `!wifi_11bg_only` on `CONFIG_SOC_WIFI_HE_SUPPORT` chips, matching the
    unrestricted-2.4GHz default IDF already used on the C5 before this fix
    — so HE-capable boards keep their 802.11ax rate rather than being
    silently downgraded to HT (802.11n).
  - Bandwidth on both bands is derived via a `max_legal_bw()` helper rather
    than a bare `wifi_ht20_only` ternary: IDF only permits `WIFI_BW40` when
    802.11n is in the protocol mask and neither 11AC nor 11AX is —
    requesting `WIFI_BW40` against an 11AX-inclusive 2.4GHz mask (or an
    11bg-only, no-11n mask) is rejected outright and previously left both
    bands' bandwidth unset. This also fixes the same latent bug on legacy
    2.4GHz-only boards when `wifi_11bg_only` is checked.
  - `/config`: the "Limit to 802.11b/g" and "Limit to 20MHz" checkboxes
    each show a caveat where they don't do what the label implies — the
    former discloses it only restricts 2.4GHz (5GHz stays unrestricted) on
    `CONFIG_SOC_WIFI_SUPPORT_5G` boards; the latter discloses it has no
    effect on `CONFIG_SOC_WIFI_HE_SUPPORT` boards, whose 802.11ax radio is
    capped at 20MHz regardless.
  - Corrected a stale comment in `hal.h`'s C5 board section claiming the
    firmware "only ever configures 2.4 GHz WiFi station mode" — not calling
    `esp_wifi_set_band_mode()` means the chip runs its dual-band `AUTO`
    default, not a 2.4GHz-only mode.

## V2.6.16 — Log wifi_ap_record_t link details (RSSI/channel/security/phy) to syslog

- New `ESP_LOGI` line ("wifi link: ...") in `main.c`, fired once right after
  the syslog UDP client comes up (same one-shot block that already emits
  the `config:` dump — V2.5.23). Reports BSSID, channel, secondary-channel/
  bandwidth, RSSI, auth mode, pairwise/group cipher, the AP's 11b/g/n/a/ac/
  ax/WPS/FTM capability bits, the negotiated PHY mode, and AID — all pulled
  from `esp_wifi_sta_get_ap_info()` / `esp_wifi_sta_get_negotiated_phymode()`
  / `esp_wifi_sta_get_aid()`. Fields are logged as their raw enum/bitfield
  values, not translated to strings.
  - Motivation: the WiFi driver's own internal `wifi:`-tagged trace lines
    (association/PHY negotiation detail — SNR, phy type, rate, etc.) print
    synchronously during `esp_wifi_connect()`, which is structurally before
    syslog can be up (syslog only starts after `GOT_IP`, which follows
    association) — so that trace never reaches the rsyslog server. This adds
    an equivalent (though coarser — see below) summary that we control the
    timing of, so it lands in both `/log` and syslog.
  - Deliberately does NOT attempt to reproduce SNR, noise floor, DTIM
    period, beacon interval, PMF confirmation, TWT state, or AMPDU state:
    verified against the ESP-IDF v6.0.2 `esp_wifi` headers that none of
    these are exposed by any public API on this IDF version — they only
    exist inside the driver's own internal trace output.

## V2.6.15 — Adafruit SGP41 gas sensor (NOx), fix SHT45 read-concurrency corruption, fix ESP32-C5 "Chip: ?" display, quiet misleading WiFi retry-connect log, fix silent NTP fallback

- New sensor: Adafruit SGP41 (SKU 6455) VOC/NOx gas sensor, I²C address
  `0x59`, board-agnostic auto-detect (`sgp41_init`/`sgp41_present`,
  `PROBE_ON_BOTH_BUSES` in `main.c`) — no HAL gating, works on any board with
  a free STEMMA QT/I²C bus. Only the NOx index is surfaced; the VOC index is
  intentionally not exposed (out of scope for this integration).
  - Unlike every other sensor in this codebase, the SGP41 needs a dedicated
    background FreeRTOS task (`sgp41.c`) sampling at a steady ~1 Hz via
    `vTaskDelayUntil`, rather than a per-TX-cycle read: the sensor's raw NOx
    signal only becomes a meaningful index when fed continuously through
    Sensirion's Gas Index Algorithm, which requires a consistent sampling
    cadence and has a ~45 s initial "blackout" plus multi-hour maturation
    (mean-estimator ~4.75 h, variance ~5.7 h) before its output stabilises.
    Boot sequence: `executeConditioning()` once/second for 10 s (VOC-only,
    NOx unused), then `measureRawSignals()` at 1 Hz indefinitely, each with a
    70 ms post-command wait (guarantees the datasheet's 50 ms conversion time
    even at the fleet's worst-case tick-rounding). Reads live RH/T from the
    SHT45 (if present) each tick for compensation, falling back to the
    sensor's disable-compensation defaults (50% RH / 25°C) otherwise. Latest
    index cached behind a mutex-guarded getter (`sgp41_get_nox_index()`);
    the cache ages out and is withheld (`ESP_FAIL`) after 15 s without a
    fresh sample rather than serving an arbitrarily old reading, and the
    `/status` page and per-cycle log line distinguish first-boot warmup from
    a sensor that later went unresponsive (`sgp41_had_valid_reading()`).
    Cache-mutex allocation failure during init is treated as sensor-absent
    rather than risking unserialized access. Measurement failures log a WARN
    on the first occurrence and every 30th thereafter (instead of one per
    second for as long as the sensor stays wedged), plus a one-shot note
    once a failure streak crosses 15 consecutive samples.
  - Vendored `sensirion_gas_index_algorithm.{c,h}` verbatim (float variant —
    ESP32 has HW FPU, so `_fixpoint` wasn't needed) from
    `github.com/Sensirion/gas-index-algorithm`, BSD-3-Clause, with a
    provenance header citing source/subdir/branch/fetch-date and a
    do-not-hand-edit note — the first vendored-third-party-code convention
    in this repo, since CI's cppcheck gate scans the entire flat `main/`
    directory with no exclusion mechanism.
  - Deliberately does NOT implement the sensor's documented soft-reset
    command: it's an I²C *general call* (broadcast address `0x00`), not
    device-specific, so issuing it would also reset every other sensor
    sharing the bus (SHT45, BMP581, fuel gauge).
  - Wired into all four existing per-sensor surfaces: per-cycle log line in
    `main.c`, `/status` page block (`format_sgp41()` in `http_server.c`),
    MQTT state JSON (`nox_index` field, suppressed while unavailable so HA
    shows "unavailable" rather than a misleading `0`), and MQTT HA Discovery
    (`mqtt_discovery.c`, `device_class=NULL` — Sensirion's 1..500 scale has
    no HA-standard class, same convention as the existing PM4/NC entities).
  - The serial-number probe (`sgp41_get_serial()`, used once at init) uses a
    precise 2 ms busy-wait instead of a 1 ms `vTaskDelay`, which rounds down
    to a no-op at the fleet's 100 Hz tick rate — same fix already in place
    for `sht45.c`'s own serial-number read.
- SHT45 driver (`sht45.c`) read/write access is now mutex-serialized. The
  new SGP41 background task reads live RH/T from the SHT45 once per second
  for compensation, alongside the main task's own per-TX-cycle
  `sht45_read()` call; the sensor's multi-step I²C protocol (write command
  → wait → read result) isn't safe against two tasks interleaving on the
  same device, and an unlucky interleave could silently return a CRC-valid
  but wrong reading. All `sht45_read()`/`sht45_heat_periodic()` access now
  goes through a shared mutex; a failed mutex allocation is treated as
  sensor-absent rather than falling back to unserialized access.
- `chip_model_str()` in `main/sysinfo.h` — the shared model-string lookup
  used by the boot `chip:` log line, the `/status` device block, and the
  syslog boot banner — never had a case for `CHIP_ESP32C5` (added to
  `esp_chip_info.h` for the board port in V2.6.10), so it fell through to
  `default: return "?"`. Purely cosmetic: WiFi, telemetry, and OTA were
  unaffected — the C5 board correctly identified itself as
  `sparkfun_thing_plus_esp32c5` via `BOARD`/hal.h the whole time, only the
  silicon-model string was wrong. Found on first real bench deployment of
  the C5 (V2.6.14) via its live `/status` page.
- Added `case CHIP_ESP32C5: return "ESP32-C5";`.
- `main.c`'s `EV_DISCONNECTED` retry path logged `"retry connect (attempt
  #N)"` even when `esp_wifi_connect()` was rejected with
  `ESP_ERR_WIFI_CONN` — meaning the driver's own internal auth/SAE retry
  from the previous cycle was still in flight and our call did nothing.
  Observed on the same C5 bench unit: 5 consecutive WPA3-SAE `AUTH_FAIL`
  (reason=202) rounds against its 5GHz AP, each internal SAE round taking
  several seconds, with every one of our 500ms-later retry calls bounced —
  the log implied 5 fresh attempts when only the driver's own retries were
  actually happening. Harmless (the driver keeps retrying regardless,
  connection still recovers on its own), purely a misleading log. Now
  checks the return value and logs `"...driver already reconnecting,
  skipped"` instead when this happens.
- `CONFIG_LWIP_SNTP_MAX_SERVERS` was left at ESP-IDF's stock default of `1`
  on every board in the fleet (never overridden anywhere in this repo).
  `ntp.c`'s `ntp_setup()` registers up to 3 configured NTP servers via
  `esp_sntp_setservername(0..2, ...)`, but lwIP bounds-checks that index
  against `SNTP_MAX_SERVERS` and silently drops any call with `idx >= 1` —
  so server 2 and 3 were configured in name only and never actually used.
  With `SNTP_MAX_SERVERS == 1`, lwIP also compiles its multi-server
  failover function (`sntp_try_next_server()`) as a `#define` alias for
  `sntp_retry()` — the whole round-robin-on-failure state machine doesn't
  exist in the binary, so a dead/unreachable server 1 is retried forever
  (exponential backoff, capped ~1h) instead of ever falling through to
  servers 2/3. Root-caused on a C5 bench unit: `10.11.12.150`'s
  `chrony.service` stopped, and the device never fell back to its two
  configured public NTP servers despite them being reachable the whole
  time — the boot log's `"SNTP started (3 server(s): ...)"` line was
  itself misleading, since it only counts our configured strings, not how
  many the underlying client actually accepted. Bumped to `3` in
  `sdkconfig.defaults` (shared, applies fleet-wide) and directly in all 11
  committed per-board `sdkconfig.<board>` caches, since `sdkconfig.defaults`
  only fills keys missing from an existing cache.

## V2.6.14 — Adafruit ESP32-S3 Feather (4MB/2MB PSRAM, STEMMA QT) board port

- New board: `adafruit_esp32s3_feather_4mb_2mbpsram` (Adafruit #5477 —
  ESP32-S3 LX7 dual-core, 4 MB external QSPI flash + 2 MB external QSPI
  PSRAM, feathers3_d-class not in-package SiP). The 11th build target, on
  the shared `Feathers3d_new_pcb` carrier alongside `feathers3_d` /
  `sparkfun_thing_plus_esp32s3` / `adafruit_esp32s3_tft_feather`. Full pin
  map and rationale in `main/hal.h` under
  `BOARD_ADAFRUIT_ESP32S3_FEATHER_4MB_2MBPSRAM` and in
  `docs/superpowers/specs/2026-07-11-adafruit-esp32s3-feather-board-port-design.md`.
- No onboard display — unlike the sibling TFT Feather (#5483), this variant
  has no screen; the sensor OLED is external, probe-detected on the STEMMA
  QT bus (`HAL_HAS_OLED=1`), same as `feathers3_d` / `sparkfun_thing_plus_esp32s3`.
- Pin map cross-verified against three independent sources (arduino-esp32's
  `pins_arduino.h`, CircuitPython's SKU-specific `pins.c`, and the user's
  own visual read of Adafruit's pin-identical #5323 sibling-SKU picture —
  #5477 itself has no published picture) — all three agree.
- No always-on I²C bus, like the TFT Feather: the single STEMMA QT bus
  (also the onboard MAX17048 fuel gauge's bus) is dead until
  `PIN_I2C_POWER_GATE` (GPIO7, Adafruit's own `PIN_I2C_POWER` net) is
  driven high. Confirmed required — not just precautionary — by Adafruit's
  own product documentation. Reuses the TFT Feather's
  `i2c_bus_get_primary()` gate plumbing (`main/i2c_bus.c`), widened to
  cover both boards under one guard.
- Dual-LED note: this board defines both `PIN_LED_BUILTIN` and
  `HAL_HAS_NEOPIXEL`. `speaker.c`'s `tick_start()` prefers
  `PIN_LED_BUILTIN`, so the onboard red "#13" LED — not the NeoPixel —
  flashes on each Geiger pulse, matching the TFT Feather's behavior.
- New `sdkconfig.defaults.adafruit_esp32s3_feather_4mb_2mbpsram` overlay
  (4 MB flash / `partitions_4mb.csv`, 2 MB external QSPI PSRAM at 80 MHz
  quad mode, USB-Serial-JTAG console) — copied from the TFT Feather's
  overlay verbatim minus the display-specific commentary.
- `http_server.c`'s `UPLOAD_PROMPT_BOARD` OTA-page label added at port
  time (this board has been missed on three prior ports — XIAO in
  V2.4.25, SparkFun Thing Plus ESP32-S3 in V2.6.8, TFT Feather in
  V2.6.11 — each shipped without a label and silently fell through to
  "(unknown board)" on the `/update` page until fixed retroactively).
- Corrected a stale wiring-reference memory in the process: the D9/D10
  speaker-pin GPIOs recorded for the #5323 sibling SKU were wrong
  (previously GPIO6/5); confirmed correct as GPIO9/10 by two independent
  sources during this port.
- CI: added to `_build-boards.yml`'s matrix (esp32s3 target) and
  `_cppcheck.yml`'s per-board leg; `release.yml`'s `EXPECTED_BOARDS`
  bumped 10 → 11.

## V2.6.13 — ESP-IDF v6.0.1 → v6.0.2 toolchain upgrade

- Bumped the pinned ESP-IDF version from v6.0.1 to v6.0.2 (local dev
  toolchain at `C:\esp\v6.0\esp-idf`, `_build-boards.yml`'s
  `esp_idf_version`, and `main/idf_component.yml`'s `idf: ">=6.0.2"` floor).
  All in-tree submodules (mbedTLS, esp_wifi/esp_phy/esp_coex libs, BT
  controller libs, NimBLE, OpenThread) moved to their v6.0.2-pinned commits.
- Driven by a genuine security fix: ESP-TLS resolved a bug that allowed
  **CA verification to be bypassed during TLS session resumption** —
  directly relevant since this firmware does HTTPS/MQTT TLS uploads.
- mbedTLS itself moved to 4.1.0 (upstream-flagged Breaking Change). All
  three chip families this project builds for (esp32, esp32s3, esp32c5)
  compile clean against the new toolchain; FTPS and HTTPS upload paths
  still need a real-hardware bench re-test given the project's prior
  history with mbedTLS buffer-handling regressions
  (`CONFIG_MBEDTLS_DYNAMIC_BUFFER=y` previously caused heap corruption in
  `log_ftp.c`) before this is considered fully verified.
- Also picks up (not independently verified, assessed low-risk):
  `esp_http_client_connect()` now actually reuses connections instead of
  always reconnecting, and a console REPL busy-loop fix for
  USB-Serial-JTAG consoles without an attached host.

## V2.6.12 — Adafruit ESP32 Feather V2 board port (2nd-source classic-ESP32 MCU for the shared Feather carrier)

- New board: `adafruit_esp32_feather_v2` (Adafruit #5400/#5900 — identical,
  #5900 just ships with headers pre-soldered — ESP32-PICO-MINI-02 SiP, 8 MB
  flash + 2 MB in-package PSRAM). The 10th build target, and a second-source
  MCU for the `Feathers3d_new_pcb` shared carrier alongside `feathers3_d` /
  `sparkfun_thing_plus_esp32s3` / `adafruit_esp32s3_tft_feather` — same
  physical Feather-format header positions, but this is the original ESP32
  LX6 dual-core (not S3), same module *class* as `adafruit_qtpy_esp32_pico`'s
  ESP32-PICO-V3-02. Full pin map and rationale in `main/hal.h` under
  `BOARD_ADAFRUIT_ESP32_FEATHER_V2` and in
  `docs/superpowers/specs/2026-07-10-adafruit-esp32-feather-v2-board-port-design.md`.
- First board on this shared carrier with no I²C fuel-gauge IC — only a raw
  ADC `BAT_VOLT_PIN`/GPIO35 battery-voltage divider. An ADC-based battery
  driver is out of scope for this port (design spec §8) and is not
  implemented; `HAL_HAS_FUEL_GAUGE=0` on this board.
- `NEOPIXEL_I2C_POWER` (GPIO2) gate scope confirmed directly against
  Adafruit's own Learn guide: it switches only the STEMMA QT connector's own
  3.3V regulator, not the header SDA/SCL pins (which ride the board's main,
  always-on 3.3V rail). Matches `sparkfun_thing_plus_esp32s3`'s
  gate-is-irrelevant-to-sensors precedent, not
  `adafruit_esp32s3_tft_feather`'s only-I2C-bus-is-gated precedent — no
  `i2c_bus.c` pre-gate needed, `HAL_HAS_VEXT_GATE=0`.
- New `sdkconfig.defaults.adafruit_esp32_feather_v2` overlay (8 MB flash,
  in-package PSRAM at 40 MHz, UART0 console via the CH9102F/CP2102N USB
  bridge). `CONFIG_ESP32_REV_MIN_3` deliberately left unset pending a
  first-boot log check — Espressif's PCN20220901 hints the module shares
  QT Py Pico's rev-3 die, but this module's own datasheet doesn't state the
  ECO revision directly.
- Drive-by fix: `main/http_server.c`'s `/update` page board-label chain
  (`UPLOAD_PROMPT_BOARD`) was missing a branch for `adafruit_esp32s3_tft_feather`
  (shipped in V2.6.11) in addition to the new board here — both fell
  through to "(unknown board)", the same recurring miss as XIAO (V2.5.19)
  and SparkFun Thing Plus ESP32-S3 (V2.6.10). Both boards now get a proper
  label.

---

## V2.6.11 — Adafruit ESP32-S3 TFT Feather board port (onboard color ST7789 TFT)

- New board: `adafruit_esp32s3_tft_feather` (Adafruit #5483, ESP32-S3, 4 MB
  flash + 2 MB external QSPI PSRAM, feathers3_d-class, not in-package). The
  9th build target, and the first board
  with an onboard color TFT instead of an I²C OLED/SerLCD. Plugs into the
  same FeatherS3-D-format carrier PCB as `feathers3_d` /
  `sparkfun_thing_plus_esp32s3`. Full pin map and bring-up parameters in
  `main/hal.h` under `BOARD_ADAFRUIT_ESP32S3_TFT_FEATHER` and in
  `docs/superpowers/specs/2026-07-10-adafruit-esp32s3-tft-feather-board-port-design.md`.
- New display backend: `main/display_tft.c` / `.h` drive the onboard
  240×135 ST7789 SPI panel via ESP-IDF's built-in `esp_lcd` component
  (`esp_lcd_panel_io_spi` + `esp_lcd_panel_st7789`) — no Component Registry
  dependency, no GFX/bitmap library. A single PSRAM-resident RGB565
  framebuffer is drawn with primitives built on the same `FONT8` 8×8 font
  the OLED backend already uses, then pushed with one
  `esp_lcd_panel_draw_bitmap()` call per page change.
- New feature flag `HAL_HAS_TFT`, mutually exclusive with `HAL_HAS_OLED`
  (this board has no I²C OLED/SerLCD path). `main/display.c` gained a third
  top-level `#elif HAL_HAS_TFT` branch alongside the existing
  `HAL_HAS_OLED` / no-display branches, implementing the 5-page rotation
  (Env / PM Mass / PM Number / Uploads / System) only — the radiation
  single-page layout (`display_running`) isn't implemented for this
  backend, since the landscape color panel suits the rotation grid.
  Explicitly defined as `0` on all 8 other boards per `hal.h`'s
  every-branch-defines-every-flag convention.
- Shared TFT/STEMMA-QT power gate on GPIO21 (`TFT_I2C_POWER`): powering the
  panel also powers the primary I²C rail, unlike the other Feather-format
  boards' independently-gated STEMMA connectors. Driven from
  `i2c_bus_get_primary()` (`main/i2c_bus.c`), before any I²C bus creation —
  fixed post-implementation per a Fable 5 MAX code review, which caught it
  being driven inside `display_tft_init()` instead, too late in `main.c`'s
  boot sequence (after the fuel-gauge and every env/PM/noise/GNSS/VEML
  probe, all of which NACK'd on the still-unpowered bus).
- Review fixes (Fable 5 MAX): `display_tft_init()` now unwinds cleanly
  (`spi_bus_free()` / `esp_lcd_panel_io_del()` / `esp_lcd_panel_del()`) on
  every mid-init failure instead of leaking the SPI bus/panel-IO handle,
  and checks every `esp_lcd_panel_*` bring-up call instead of ignoring
  their return values; `fb_push()` now logs `esp_lcd_panel_draw_bitmap()`
  failures instead of swallowing them; `display_boot_screen()` and
  `display_set_contrast()` for this backend now match the OLED backend's
  `s_show`-gating/storage behavior (`display.h`'s documented contract);
  stale "5 s" dwell comment corrected to match `PAGE_DWELL_MS` (7000 ms).

---

## V2.6.10 — SparkFun Thing Plus ESP32-C5 board port; PSRAM mode/speed now visible in `/log`; fix SparkFun OTA page board label

- New board: `sparkfun_thing_plus_esp32c5` (SparkFun WRL-30678,
  ESP32-C5-WROOM-1, 8 MB in-package flash + 8 MB in-package quad PSRAM).
  The 8th build target, and the codebase's first single-core, first
  RISC-V target — every other board is dual-core Xtensa. `tx_setup()` in
  `main/transmission.c` now branches on `BOARD_SPARKFUN_THING_PLUS_ESP32C5`
  to create the upload worker with plain `xTaskCreate()` instead of
  `xTaskCreatePinnedToCore(..., 1)`, since pinning to a second core is
  meaningless on single-core silicon. Own standalone Thing Plus carrier PCB
  (not a drop-in for the FeatherS3-D-format carrier the other Thing
  Plus/FeatherS3-D boards share): external I2C OLED via Qwiic, onboard
  MAX17048 fuel gauge, onboard WS2812 NeoPixel (always-on rail via
  pull-up, no power-gate GPIO). Full pin map and design rationale in
  `main/hal.h` under `BOARD_SPARKFUN_THING_PLUS_ESP32C5` and in
  `docs/superpowers/specs/2026-07-09-sparkfun-thing-plus-esp32-c5-board-port-design.md`.
- `http_server.c`'s OTA cross-family chip guard had no `CHIP_ESP32C5` case,
  so an upload to this board silently skipped the upfront chip-mismatch
  rejection every other board gets and fell through to the bootloader-only
  safety net. Added the case (`ESP_CHIP_ID_ESP32C5` / `CHIP_ESP32C5`, both
  present in this IDF version's headers) alongside the existing per-board
  cases.
- ESP-IDF's own "found N MB PSRAM, speed X, mode Y" line prints during
  `esp_psram_init()`, which runs before `app_main()` — before `applog`'s
  vprintf hook exists — so it never reached `/log` or syslog on any PSRAM
  board. Confirming PSRAM config after a first boot required a serial
  capture. `applog_init()` now logs `psram: NNNN KB total, mode=X speed=YMHz`
  right after the existing ring-placement line, reconstructed from the
  sdkconfig baked into the binary (mode/speed are build-time choices, not
  runtime-queryable) plus the live total size from `esp_psram_get_size()`.
  Applies to all 6 PSRAM boards (`feathers3_d`, `adafruit_qtpy_esp32_pico`,
  `heltec_wifi_lora32_v4_r2`, `seeed_xiao_esp32s3`,
  `sparkfun_thing_plus_esp32s3`, `sparkfun_thing_plus_esp32c5`); the two
  non-PSRAM Heltec V2 targets skip it.
  Bench-verified on the SparkFun Thing Plus ESP32-S3: `2048 KB total,
  mode=quad speed=80MHz`, matching that board's port design spec with no
  fallback needed.
- Fixed the `/update` OTA page reporting `sparkfun_thing_plus_esp32s3` as
  "(unknown board)" — it shipped in V2.6.8 without a label branch in
  `http_server.c`'s board-name `#if`/`#elif` chain, the same miss the
  V2.5.19 XIAO fix already documented in a comment one branch up.

---

## V2.6.9 — `hv_coincident` diagnostic: test HV-recharge coupling as a spurious-count source

- New permanent diagnostic on the DIAG log line: `hv_coincident` counts, per
  cycle, how many of the pulses actually COUNTED (i.e. a subset of `counts`,
  not of `raw_edges - counts` like `rejected`/`guard_removed`) land within
  [200µs, 3000µs] of the START of the most recent HV charge pulse
  (`HV_COINCIDENT_MIN_US`/`MAX_US`, `tube.h`).
- Built to test the leading hypothesis from an independent field-data review
  (`docs/radiation_overcounting_independent_review.md`, 2026-07-09) that the
  new small-form-factor PCB (both the FeatherS3-D and XIAO ESP32-S3 carriers)
  shows spurious counts electrically correlated with its own HV recharge
  activity (r=0.98-0.99 vs. the original Heltec board's r=-0.02), confirmed
  independently by a tube-disconnected bench test (12 phantom counts per
  cycle, exactly matching `hv_pulses`, spaced with clockwork ~10.0025s
  regularity — inconsistent with genuine Poisson background radiation).
- Implementation: `tube.c`'s `recharge_tick` (the 100µs HV charge-pump
  gptimer ISR) stamps `isr_last_hv_pulse_us` at every `S_PULSE_H` FET-on
  edge; `gmc_count_isr` (the GPIO count ISR) reads that stamp under its own
  lock (`mux_hv` — a 64-bit value isn't atomic on this core, and the two
  ISRs run on independent interrupt sources that can preempt each other) and,
  for each edge it counts, checks whether the gap since that stamp falls in
  the coincidence window. `tube_get_diag()` gained a fourth out-param
  (`hv_coincident`) alongside the existing `raw_edges`/`guard_removed`,
  snapshotting and resetting under the same `mux_gmc` critical section.
- A real tube should show this near-zero (background rate is far too low to
  land inside a ~2.8ms window by chance most cycles); HV-pickup should show
  it tracking `hv_pulses` roughly 1:1, mirroring the tube-disconnected bench
  result above. Intended for an overnight capture on the XIAO (and ideally
  the Heltec as a zero-control) to get definitive per-edge attribution before
  deciding on a permanent fix.

---

## V2.6.8 — New `sparkfun_thing_plus_esp32s3` board target (7th build target)

- **New board**: SparkFun Thing Plus ESP32-S3 (WRL-24408), ESP32-S3-MINI-1
  SiP with 2 MB in-package quad PSRAM and 4 MB flash. Second-source MCU for
  the existing FeatherS3-D-format carrier PCB (`project_feathers3d_new_pcb`,
  in fab since 2026-05-14) — same physical Feather footprint and wiring,
  drop-in alternative to `feathers3_d` on the same board with a different
  GPIO-to-header-position mapping.
- Reuses the FeatherS3-D peripheral set unmodified: external I²C OLED via
  Qwiic (probe-detected), onboard MAX17048 fuel gauge at 0x36, native
  USB-Serial-JTAG console. No ALS (the shared carrier PCB carries none) and
  no VBUS-detect GPIO (this board's MCP73831 STAT pin drives an onboard LED
  only, no MCU-visible signal).
- Onboard WS2812 NeoPixel (GPIO46 DIN) enabled via `HAL_HAS_NEOPIXEL=1`.
  Its VDD ties to the board's general peripheral rail ("3.3V_P", shared with
  the Qwiic connector and microSD), which defaults to always-on via the
  onboard RT9080 LDO's resistor-biased EN pin — unlike the QT Py's switched
  NeoPixel rail, this board has no dedicated power-gate GPIO. `neopixel.c`'s
  power-gate step is now conditional on `PIN_NEOPIXEL_POWER` being defined
  (same "intentionally undefined optional pin" idiom already used for
  `PIN_OLED_RESET`) so the shared driver works on both rail topologies
  without a board-specific branch.
- `fuel_gauge.c`'s VBUS-present check is now conditional on `PIN_VBUS_DETECT`
  being defined (same idiom as the NeoPixel power-gate above) — this board's
  MCP73831 STAT-only wiring left it undefined, and the driver previously
  assumed every `HAL_HAS_FUEL_GAUGE` board had a dedicated sense GPIO.
  `fuel_gauge_vbus_present()` returns `false` (unknown) where the pin doesn't
  exist.
- Found and fixed a latent single-callback collision: `speaker.c` and
  `neopixel.c` both call `tube_set_pulse_callback()` to claim the one tube
  ISR pulse-callback slot, and no existing board had both `HAL_HAS_SPEAKER`
  and `HAL_HAS_NEOPIXEL` set at once, so the collision was never exercised.
  On this board the NeoPixel's registration (run after the speaker's, per
  `main.c`'s boot order) would have silently stolen the slot back whenever
  `led_tick` was enabled, breaking the speaker's audio tick for as long as
  it stayed enabled. Fixed by having `speaker.c` keep sole ownership of the
  callback on such boards and drive the NeoPixel directly through a new
  `neopixel_notify_pulse()` entry point instead of `PIN_LED_BUILTIN` (also
  now conditional, same idiom); `neopixel_register_pulse_tick()` still
  creates its worker task but skips re-registering the callback whenever
  `HAL_HAS_SPEAKER` is set. Verified no regression by rebuilding
  `adafruit_qtpy_esp32_pico` (NeoPixel-only path) and `feathers3_d`
  (`PIN_LED_BUILTIN` path) alongside this board.
- Post-port MAX review (9 passes) fixed three further issues: `neopixel_notify_pulse()`
  now carries its own `IRAM_ATTR` (previously only its callee did, despite the
  header doc promising ISR-safety); the `/config` page's `led_tick` checkbox
  label — hardcoded "LED flash on each GM pulse" — is genericized since it now
  also drives a NeoPixel on this board; and a structural CI gap where
  `_cppcheck.yml` hardcoded one board's feature-flag macros, so cppcheck had
  never statically analyzed the real `neopixel.c`/`fuel_gauge.c` driver
  implementations (only their stub branches) on *any* board — fixed by
  matrixing cppcheck across all 7 build targets, one leg per board with the
  same defines `CMakeLists.txt` uses.
- 4 MB flash reuses `partitions_4mb.csv` (dual OTA, no factory partition,
  1.875 MB per slot) — same file already serving `heltec_v2_4mb`. Headroom
  checked against the other four PSRAM/S3 boards' measured binary sizes
  (largest is ~1.36 MB, ~72% of the slot); no partition-table change needed
  for this port.
- Full pin-sourcing rationale, schematic verification (WS2812 power rail,
  strapping-pin review, GPIO26-37 in-package-PSRAM exclusion), and the
  binary-size headroom analysis are preserved in the design spec at
  `docs/superpowers/specs/2026-07-09-sparkfun-thing-plus-esp32s3-board-port-design.md`
  (local working notes, not tracked in git).

## V2.6.7 — New `heltec_wifi_lora32_v4_r2` board target (6th build target)

- **New board**: Heltec WiFi LoRa 32 V4 — base/R2 variant (ESP32-S3R2, 2 MB
  in-package quad PSRAM). Runs on a third-party PCB: the standard Multigeiger
  V2 mainboard populated with this Heltec module instead of the Heltec V2.
  NOT the V4-R8 variant (ESP32-S3R8, 8 MB PSRAM, different GPIOs) — the
  `_r2` suffix permanently disambiguates from a possible future R8 port.
- Two-I²C-bus architecture: the onboard SSD1315 OLED lives on a
  module-internal bus (GPIO17 SDA / GPIO18 SCL) completely separate from the
  external env-sensor bus exposed on the mainboard header (GPIO48 SDA /
  GPIO47 SCL) — every other supported board that has a second bus uses it
  as an opportunistic fallback shared with sensors; here it is dedicated
  exclusively to the OLED. Reuses the dual-bus abstraction built for
  FeatherS3-D's STEMMA1/STEMMA2
  split (`i2c_bus.c`'s `i2c_bus_get_secondary()`), extended with an
  always-on (no LDO gating) branch for this board's fixed OLED bus.
  `display.c` gains `SSD1315` as a third register-compatible OLED chip
  identity alongside SSD1306/SSD1309.
- Onboard LED (GPIO35) is active-HIGH per independent third-party firmware
  cross-reference (Heltec's own Arduino library defines no LED pin for this
  module at all). Since `HAL_HAS_SPEAKER=0` (see below) stubs out
  speaker.c entirely, `led.c`'s own pulse-tick driver now owns this pin.
- **Piezo speaker disabled (`HAL_HAS_SPEAKER=0`)**: the pin matrix wires the
  onboard piezo to GPIO26 (P) / GPIO5 (N), but GPIO26 is this chip's
  internal PSRAM chip-select (SPICS1) — confirmed against the Espressif
  ESP32-S3 datasheet v2.2 (§2.3.5 Table 2-9, Priority 4: "SPI0/1 interface
  connected to the in-package flash and PSRAM"), against Heltec's own
  `pins_arduino.h` for V3/V4/V4_R8 (all three omit GPIO26-37 entirely,
  regardless of whether the chip variant has in-package flash or PSRAM),
  and against a KiCad/Eagle trace review of the Multigeiger V2 mainboard.
  Driving GPIO26 as a PWM tone output risks bus contention with live SPI0
  flash traffic, i.e. crashes/hangs correlated with speaker use — this is a
  chip-hardware conflict, not a firmware bug, and disabling PSRAM in
  firmware does not fix it (the chip-select bond is fixed at chip
  fabrication, and the surrounding GPIO27-32 still carry the live flash
  bus the firmware itself runs from). Also checked and ruled out as a
  workaround: swapping to the WiFi LoRa 32 V3 module (ESP32-S3FN8) — same
  restriction applies for the mirror-image reason (in-package flash instead
  of in-package PSRAM), and V3/V4 share the same module footprint, so the
  mainboard's hardwired trace lands on the same physical GPIO26 either way.
  Full writeup with all three source citations:
  `docs/superpowers/reviews/2026-07-08-v2.6.7-max-review/GPIO26_Befund_Zusammenfassung_DE.txt`
  (German). Hardware-team decision pending on a GPIO34/GPIO37 rework for a
  future board revision; this release ships without speaker/tone output on
  this board.
- Hardware-reservation only for future LoRaWAN/Meshtastic work: GPIO
  7-14 (the SX1262 radio's dedicated internal SPI bus + front-end enable)
  are documented as reserved in `hal.h` but not driven by any code — no
  `HAL_HAS_LORA` flag, no radio driver, no config fields. LoRaWAN is a
  parallel connectivity mode (own OTAA/ABP join flow, no WiFi/internet
  dependency), not another TX-dispatch-table target, so it needs its own
  dedicated design work once scoped.
- CI matrix (`_build-boards.yml`) and release artefact count
  (`release.yml`'s `EXPECTED_BOARDS`) updated for the 6th board.
- **Not bench-verified** — no hardware in hand this session. LED polarity,
  PSRAM speed (80 MHz assumed), and the assumed 128x64/0.96" SSD1315 panel
  size (`DISPLAY_MODE_AUTO`) are flagged in `hal.h`/`display.c`/the
  sdkconfig overlay as first-flash verification items. (Speaker P/N
  assignment is no longer a verification item — the speaker is disabled,
  see above.) The GPIO3 strap used for `PIN_GMC_COUNT_INPUT` was
  investigated against the Espressif ESP32-S3 datasheet v2.2 ch.3 and
  downgraded from a verification item: GPIO3 only selects JTAG signal
  source (not the SPI-boot/download-mode decision, which is GPIO0+GPIO46),
  this board doesn't use native USB-Serial-JTAG for console/flashing
  anyway (separate UART bridge on GPIO43/44), and the eFuse bits that would
  make the chip act on this strap default to not-burnt on stock chips — see
  `hal.h`'s `PIN_GMC_COUNT_INPUT` comment for the full citation.

## V2.6.6 — MAX17048 battery fuel gauge (FeatherS3-D): /status, MQTT, HA discovery, per-cycle log, config checkbox

- New `fuel_gauge.c`/`.h` driver for the onboard MAX17048 (I²C 0x36,
  FeatherS3-D's STEMMA1/primary bus only — every other board compiles a
  zero-cost stub). Registers used: `VCELL` (voltage), `SOC` (state of
  charge), `CRATE` (signed charge rate, +charging/-discharging). No init
  writes needed — the chip lives on the always-on 3.3V rail, independent
  of the battery, and free-runs once powered.
- **Battery presence is a `/config` checkbox (`batt_present`), not an
  auto-detected guess.** The original design tried auto-detecting via a
  VCELL threshold, but bench testing (first with USB power and no LiPo,
  later confirmed with a real LiPo on 2026-07-07 — see below) found this
  fundamentally unworkable: the onboard LiPo charger IC's unloaded output
  floats to ~4.2-4.4V whenever USB is present, regardless of whether a
  battery is attached, and no charger-status pin reaches a GPIO on this
  board — so no voltage-only or digital signal can disambiguate "USB +
  real battery" from "USB, no battery." The new "Battery attached
  (MAX17048 fuel gauge)" checkbox is the honest fix: it's greyed out on
  boards without `HAL_HAS_FUEL_GAUGE` (same convention as the
  external-antenna and I²C pin-out switches) and directly gates
  `fuel_gauge_present()`, which in turn controls every surface below.
  Live-apply — `fuel_gauge_set_user_present()` runs once at boot from the
  saved config and again on every `/config` Save, so ticking/unticking
  takes effect immediately, no reboot needed.
- `/status` gains a Battery block (voltage / charge % / rate), MQTT
  rich-state JSON gains `batt_v` / `batt_soc` / `batt_rate`, and three new
  Home Assistant discovery entities appear (Battery voltage — V,
  device_class `voltage`; Battery — %, device_class `battery`; Battery
  charge rate — %/h, the first entities in this codebase to use HA's
  `battery`/`voltage` device classes) — all three gated on
  `fuel_gauge_present()`, i.e. only shown once the checkbox is ticked.
- One `ESP_LOGI` line per TX cycle, also gated on `fuel_gauge_present()`,
  reports `"Battery: <V> <SoC%> <rate%/hr> (VBUS present|absent,
  version=.. status=..)"`. VBUS state comes from a dedicated digital
  detect pin (`PIN_VBUS_DETECT`/GPIO34, `fuel_gauge_vbus_present()`) and
  gives context for the reading (charging vs. running off battery);
  `version`/`status` are raw MAX17048 diagnostic registers, logged
  alongside voltage/SoC/rate for field debugging. A node with the
  checkbox left unticked logs nothing here, rather than presenting a
  charger-IC float as if it were a real reading.
- **Diagnostic register logging**: `fuel_gauge_init()` logs all 7
  readable MAX17048 registers once at startup
  (`version`/`hibrt`/`config`/`valert`/`vreset`/`chip_id`/`status`) —
  `version` because it's the register Adafruit's own MAX1704x libraries
  use as their sole "battery attached" sentinel (`0xFFFF` = no response,
  which doesn't transfer to this board's always-on-rail wiring — see
  `fuel_gauge.h`); the rest are pure config registers this driver never
  writes, so logging them once at boot is enough. New `fuel_gauge_read_diag()`
  API for the two registers (`version`/`status`) worth reading again later.
- **Real-battery bench validation** (2026-07-07, FeatherS3-D with a
  genuine LiPo, after fixing a reversed-polarity JST-PH connector):
  confirmed both power states read correctly — charging: 3.956-3.960V /
  67-67.4% / +4.6-7.7%/hr, VBUS present; battery-only: 3.924V / 67.6% /
  +2.3%/hr, VBUS absent — validating the checkbox-gated design above
  against a real cell in both states.
- **I²C driver consolidation (no functional change).** A code-simplifier
  survey of `main/` found the same low-level I²C boilerplate copy-pasted
  across every register-based driver. Two rounds of cleanup, both
  verified with a clean build across all 5 board targets and no change
  to wire behavior (byte order, timeouts, log messages, error codes):
  - `i2c_bus.h` gains shared `static inline` per-device helpers
    (`i2c_dev_write_reg`, `i2c_dev_read_regs`, `i2c_dev_read_u16_be`/`_le`,
    `i2c_dev_write_u16_le`, `i2c_add_device`, `i2c_probe_and_add`,
    `i2c_dev_teardown`). `bmp581.c`, `bmp390.c`, `bme280.c`, `bme688.c`,
    `veml7700.c`, and `fuel_gauge.c` now use these instead of each
    hand-rolling its own `write_reg`/`read_regs`/16-bit-read wrapper and
    probe→add_device→teardown-on-failure ceremony; `sht45.c` and
    `sps30.c` adopt the add/teardown helpers for their command-based
    protocol. This is the same class of bug the fuel-gauge's original
    VCELL byte-order handling needed to get right — centralizing it makes
    that mistake harder to reintroduce.
  - New `sensirion_crc.h` centralizes the Sensirion CRC-8 (poly 0x31,
    init 0xFF) that `sps30.c`, `sht45.c`, and `dnms.c` each reimplemented
    independently (`sps30.c` even had a comment noting the duplication).
    All three now call the shared `sensirion_crc8()`.

## V2.6.5 — roaming-app safety-net widened to 5 min; t_attempt_start_us torn-read fix; three int64→32-bit demotions

**Roaming-app reconnect safety-net widened 30 s → 5 min** (`main.c`, `ROAM_RECONNECT_SAFETY_NET_US`): A 2026-07-01 router restart on a live node showed the 30 s backstop firing four times while the ESP-IDF roaming app was still legitimately retrying (all `reason=201`, AP unreachable during the router's own reboot) — three were harmless no-ops (`sta is connecting, return error`), but one collided with a link the roaming app had just re-established, forcing a spurious extra disconnect/reconnect (`sta is connected, disconnect before connecting to new ap`). The roaming app went on to resolve the reconnect itself in ~111 s. Widened to 5 min so the experimental roaming app is left to fully own ordinary roams/reconnects (a router reboot alone can take 1-2 min of its own retries); the safety-net now only fires on a genuine stall. No other watchdog is affected — `STA_STARTUP_TIMEOUT_US` (10 min) only guards the *first* post-boot connection and disarms permanently after it, and MQTT's own `reconnect_timeout_ms` (30 s) is independent of WiFi link state.

**`t_attempt_start_us` torn-read fix** (`main.c`): Investigating the safety-net incident above surfaced a torn-read hazard the V2.6.3 sweep missed: `t_attempt_start_us` (`int64_t`) is written by `mark_attempt()` from *both* the main task (retry / roaming-safety-net paths) and the WiFi event task (`STA_START`), then read back on the WiFi event task to compute `last_assoc_s` — a genuine two-instruction-store race on 32-bit Xtensa. Unlike the V2.6.3 counters, this feeds a sub-second diagnostic (`assoc=2.503s`), so it wasn't a demotion candidate; guarded instead with a dedicated `portMUX_TYPE` (`t_attempt_start_mux`), following the existing `g_last_cycle_at_mux` precedent. A follow-up full-codebase sweep of every cross-task 64-bit variable found no other unprotected instances — the rest of the codebase already carries this lesson consistently (spinlock-guarded, demoted, single-task-only, or write-once-at-boot).

**Three bounded-duration `int64_t` → 32-bit demotions** (found by a general "does this need 64 bits" sweep, independent of thread-safety):
- `mqtt.c`: `ftp_age_s` (seconds since last FTP upload, MQTT state payload) → `int32_t`.
- `http_server.c`: OTA-upload `recv_ms` (elapsed receive time) → `uint32_t`, dropping the `(long long)` print cast.
- `ntp.c`: `uptime_s` inside `sync_cb()` → `uint32_t`, matching the `sync_tv_sec_off`/`s_boot_epoch_off` convention two lines above it in the same function — the closest thing to an actual inconsistency found. All three are bounded durations (at most weeks), nowhere near the ~136-year range of `uint32_t`; wall-clock `time_t` epochs and absolute `esp_timer_get_time()` microsecond timestamps elsewhere were deliberately left 64-bit (Y2038 safety / multi-day uptime range).

**Why now:** Prompted by reviewing a real router-restart log; each fix led to the next (safety-net tuning → torn-read audit → general 64-bit sweep) rather than being separately planned.

---

## V2.6.4 — fix stack overflow on POST /config + config_save ordering + static sweep

**Stack overflow (primary):** `config_t next = *s_cfg` in `config_post` (`http_server.c`) put a ~4 KB struct on the IDF httpd task's 8 KB stack. The struct is dominated by `mqtt_tls_ca[2401]` (added in V2.4.6). Combined with `char page[1024]` and ~800 bytes of IDF httpd framework overhead the 8 KB budget was exhausted — confirmed via coredump: `***ERROR*** A stack overflow in task http has been detected.`

**Fix:** `config_t next` → `static config_t cfg_next` (BSS). Safe because `esp_http_server` runs all URI handlers serially on one task (documented at `http_server_start`), so `config_post` is never re-entered. Every call reinitialises with `cfg_next = *s_cfg` before any field read. Follows the same pattern established by V2.4.22 for `config_get`'s `e_*` escape buffers and `br_opts`.

**config_save ordering (pre-existing fix):** Previously `*s_cfg = next` committed the new config to the live in-memory struct before `config_save()` wrote it to NVS. If the NVS write failed, the device would run the new config in RAM but revert to the old config on the next reboot (silent split-brain). Fixed: `config_save(&cfg_next)` is called first; `*s_cfg = cfg_next` only executes on success.

**Static sweep — config_get post-V2.4.22 regressions (secondary):** The V2.4.22 comment claimed all large locals in `config_get` were moved to BSS, but four `e_*` arrays added in V2.5.1 / V2.5.4 (`e_gmc_aid`, `e_gmc_gid`, `e_ts_key`, `e_ts_pm_key`) and the V2.6.1 `tube_opts[384]` were left on the stack. All five are now `static`. The `char page[1024]` response buffer in `config_post` is also made static.

**Why now:** Overflow latent since V2.4.6 (mqtt_tls_ca field). The V2.6.x rebuild (slightly different register/stack layout) pushed it over the edge. Triggered by a user POST /config that panicked the device; confirmed and root-caused via coredump decode.

---

## V2.6.3 — atomic WiFi counters: n_attempts/n_connects/n_got_ip uint64→uint32, sync_tv_sec offset, dead code removal

**What:** Four torn-read hazards and one dead variable removed, found by a post-V2.6.2 sweep of all cross-task 64-bit globals.

1. **`n_attempts`, `n_connects`, `n_got_ip` demoted `uint64_t` → `uint32_t`** — all three are written by the WiFi event task and read by the main task with no mutex. On 32-bit Xtensa (ESP32/S3), a 64-bit store is two `S32I` instructions; a reader on the other core can see one half of one write and one half of the next (torn read). `uint32_t` is a single instruction. Wraps after ~136 years at 1 event/s. All five `PRIu64` format specifiers updated to `PRIu32`.
2. **`sync_tv_sec` in `ntp.c` demoted `volatile time_t` → `volatile uint32_t` EPOCH_2026 offset** — this variable carries the timestamp from the SNTP lwIP timer callback to `ntp_poll()` on the main task. `time_t` is 64-bit on IDF newlib (same torn-read hazard). Fixed using the same `(uint32_t)(tv_sec - EPOCH_2026)` offset pattern as `s_boot_epoch_off`. `ntp_poll()` reconstructs via `EPOCH_2026 + off`.
3. **`t_last_got_ip_us` deleted** — declared as `static int64_t`, written in `on_ip_event`, but never read by any code path. Dead write removed along with the declaration.

**Why now:** The V2.6.2 MAX review established the `uint32_t`-offset-for-atomic-stores pattern. A follow-up sweep confirmed the three WiFi counters and the SNTP timestamp variable had the same structural hazard. The V2.6.2 `n_disconnects` fix was the template; this release applies it to the remaining unprotected cross-task 64-bit stores.

**No behaviour change** under normal operation. A torn read on `n_got_ip` or `n_connects` could briefly delay MQTT startup or roaming-app deference by one main-loop tick (~1 s); a torn read on `sync_tv_sec` could produce a garbled NTP sync log line (cosmetic). Neither manifests reliably.

---

## V2.6.2 — boot-epoch hardening: atomic storage, per-sync refresh, clamped uptime helper

**What:** Seven correctness fixes and two cleanups to the `ntp_boot_epoch()` infrastructure introduced in V2.5.22 / V2.6.1, surfaced by a MAX independent code review. No user-visible behaviour change under normal conditions; the fixes only kick in under adverse NTP conditions or on a warm reboot.

**Fixes:**

1. **Atomic boot-epoch storage** (`ntp.c`): `s_boot_epoch` was a `volatile time_t` (64-bit `__int_least64_t` on ESP32 newlib). On 32-bit Xtensa, a 64-bit write compiles to two `S32I` instructions — not atomic. HTTP and TX workers on core 1 could observe a torn 64-bit value if a sync fired on the tcpip thread (core 0) between the two stores. Changed to `volatile uint32_t s_boot_epoch_off` (seconds since `EPOCH_2026`): a 32-bit store/load is a single atomic instruction on Xtensa, requires no mutex, and covers dates to ~2162.

2. **Per-sync epoch refresh** (`ntp.c`): The `if (s_boot_epoch_off == 0)` write-once guard meant a plausible-but-wrong first NTP response (pool server fallback while GPS stratum-1 is offline, GPS rollover, leap-second error) permanently corrupted the "Started" timestamp and every `CYCLE log` `uptime=` field for the device's lifetime. Removing the guard so every hourly SNTP poll (CONFIG_LWIP_SNTP_UPDATE_DELAY = 3600 s) refreshes the epoch caps crystal drift at < 1 s per sync interval and allows self-correction from a bad first sync. Inner guard `if (epoch > EPOCH_2026)` prevents storing a negative offset when a very long pre-sync crystal run pushes `(tv_sec - uptime_s)` below the epoch floor.

3. **Warm-reboot regression** (`http_server.c`): The V2.6.1 "Started" gate changed from `ntp_time_valid()` (true immediately from RTC carryover on warm reboot) to `boot_epoch_off != 0` (requires an SNTP callback), so `/status` showed "NTP: synced · clock now \<time\>" but no "Started" suffix for ~60 s after a warm reboot. Resolved by fix #2 (the re-sync that fires ~60 s in now refreshes `s_boot_epoch_off`).

4. **Unsigned-wrap on step-back** (`ntp.c`, `http_server.c`, `main.c`, `transmission.c`): `(time_t)time(NULL) - boot_epoch` cast to `uint32_t`/`unsigned long` with no clamp: a negative result (NTP clock step-back after a bad first sync is corrected) wrapped to ~4 billion seconds (~136-year uptime). New `ntp_uptime_s()` helper clamps negative results to 0 before returning; all three callers now use it.

5. **Triplicated fallback pattern → `ntp_uptime_s()`** (`ntp.h/c`, `http_server.c`, `main.c`, `transmission.c`): The `boot_epoch ? time(NULL)-boot_epoch : crystal_fallback` pattern was copy-pasted in three files with inconsistent cast types (`unsigned long` vs `uint32_t`). Centralised in `ntp_uptime_s()` (returns clamped `unsigned long`); each call site is now a single line.

6. **`n_disconnects` torn read** (`main.c`): Was `uint64_t` — two 32-bit stores on a 32-bit CPU. Written on the Wi-Fi event task, read on the TX task without a lock; a disconnect firing mid-read produced a torn counter in the `CYCLE` log line. Demoted to `uint32_t` (wraps after 4.3 billion disconnects ≈ 136 years at 1/s — display use only) which is a single atomic store on Xtensa.

7. **`disconnects=` field rename** (`main.c`): The `CYCLE` log field `reconnects=%lu` counted **disconnect events** (`WIFI_EVENT_STA_DISCONNECTED`), not successful reconnects — overcounting by 1 while the node was mid-reconnect. Renamed to `disconnects=%lu` to match the counter semantics. (Syslog parsers watching `reconnects=` will need updating.)

8. **Consistent "clock now" / "Started + Uptime" timestamps** (`http_server.c`): `status_get()` and `format_system()` each called `time(NULL)` independently, so "Started + Uptime" and "clock now" could differ by up to 1 s. `now_wall = time(NULL)` is now captured once at the top of `status_get()` and threaded through to `format_system()`.

9. **`format_uptime_hm` d>0 branch dedup** (`util.h`): The day-resolution branch was byte-identical to `format_uptime()`'s d>0 branch — two copies that could silently diverge. The branch now delegates to `format_uptime()` (outputs are identical for d>0; only the d==0 branch differs by omitting seconds).

---

## V2.6.1 — selectable Geiger tube type (SBM-20 / SBM-19 / Si22G)

**What:** The Geiger-Müller tube is now a runtime `/config` setting ("Geiger tube type", `tube_type`, 0–3) instead of a hard-coded Si22G calibration. Four options — **Unknown** (no dose conversion), **SBM-20**, **SBM-19**, **Si22G** (default) — each carrying the upstream MultiGeiger cps→µSv/h factor. The selected tube is shown on the `/status` Radiation card and in the boot config dump. Default is Si22G, so existing nodes are unchanged.

**Why:** The firmware previously baked the Si22G conversion factor (`SI22G_CPS_TO_USVPH = 1/12.2792`) into five dose computations, so connecting any other tube (e.g. an **SBM-19**) reported correct CPM but a wrong µSv/h — the dose is the *only* tube-dependent quantity, off by the tube's sensitivity ratio (an SBM-19 read through the Si22G factor under-reports dose by ~20 %). Making it a config field lets one firmware serve a mixed fleet (Si22G + SBM-19 + SBM-20) and pick the tube per node without a rebuild.

**How:** New `X_U32 tube_type` (default 3 = Si22G) in `config_fields.def`; a `tube_type_t` enum + `k_tubes[]` characteristics table + `tube_cps_to_usvph()` / `tube_type_name()` helpers in `transmission.{h,c}` replacing the lone `#define`. The factor table is the validated ecocurious2 / t-pi `tube.cpp` calibration (SBM-20 1/2.47, SBM-19 1/9.81888, Si22G 1/12.2792). `tx_context_t` gained a `tube_type` field snapshotted in `build_tx_context()` so the worker derives dose with a stable factor across a cycle; the five former `SI22G_CPS_TO_USVPH` sites (`main.c` live dose + Madavi/Radmon/ThingSpeak/OSM-aqi in `transmission.c`) now call `tube_cps_to_usvph()`. The `/config` page renders a four-option dropdown mirroring the display-mode select; dose is recomputed each cycle so a Save applies **live** (no reboot). **Deliberately unchanged:** the sensor.community / OSM / aqi `value_type` field *names* (e.g. `Si22G_counts_per_minute`) stay fixed — they are wire-format identifiers tied to the multi-year upload archive, and renaming them would split historical graphs server-side.

---

## V2.5.34 — FTP interval max 10090 + out-of-range reporting; Wi-Fi roaming app + BSSID logging

### FTP interval ceiling + `/config` out-of-range reporting

**What:** Three small `/config` changes: (1) the FTP log-upload interval ceiling (`ftp_int`) is raised from **1440** (24 h) to **10090** minutes (~7 days); (2) its form label now reads "Upload interval (minutes) (Max 10090)"; (3) the `/config` POST result page now lists any field whose value was **out of range and therefore not saved** (prior value kept), and logs it via `ESP_LOGW`.

**Why:** A value above a field's max was discarded silently — the X-macro dispatcher keeps the prior value and still returns "handled", so an entry like `ftp_int=1450` produced a "Saved." page with no change and **no log line**, looking like a broken save. Hit on two nodes (`.193` on V2.5.29, `.198` on V2.5.33) — and confirmed not version-specific (the apply path is unchanged across V2.5.29..V2.5.33). The ceiling bump covers realistic >24 h cadences; the result-page notice + warning log turn the silent no-save into explicit feedback.

**How:** `ftp_interval_min`'s `hi` bound in `config_fields.def` → 10090 (struct / NVS / POST all auto-follow the X-macro; the field stays `type="text"` per the V2.3.x wheel-scroll fix, so the server-side range is the single enforcement point — now a visible one). `config_post_apply_field()` gained a `bool *out_rejected` out-param that the `X_U32` / `X_F32` / `X_U8` macros set when a key matched a field but the value failed its range test (X_STR / X_BOOL never reject). `config_post()` accumulates those keys — plus the OLED-brightness step special-case — and renders a red "out of range … NOT saved (previous value kept): …" banner built with the existing `append_safe()` clamped accumulator, plus an `ESP_LOGW`. Only fixed schema keys can reach the echoed list, so no escaping is needed.

### Wi-Fi roaming app (PSRAM boards) + per-cycle BSSID logging

**What:** Enable the ESP-IDF Wi-Fi roaming app on the PSRAM boards (FeatherS3-D, QT Py PICO, XIAO S3; Heltec **excluded**) with low-RSSI + legacy roaming, and append the connected AP's `bssid`/`ch` to every `CYCLE` log line.

**Why:** A node (`esp32-5965048`, `.198`) lost its strong AP to a beacon timeout (`reason=200`), reconnected to a **−82 dBm** mesh BSSID, and **stuck there for ~75 minutes** (≈1 in 3 uploads failing — `HTTP_EAGAIN`, connect timeouts, TLS `Socket is not connected`) until a second beacon timeout *luckily* landed it on a −31 dBm AP. Connect-time selection was already optimal (`WIFI_ALL_CHANNEL_SCAN` + `WIFI_CONNECT_AP_BY_SIGNAL`), but nothing re-evaluated the link **while associated**, so a node that lands on a weak BSSID never recovers until a full disconnect. The per-cycle line logged `rssi` but not *which* AP, so the pattern was invisible in syslog.

**How:** Per-board `sdkconfig.defaults` (PSRAM only) set `CONFIG_ESP_WIFI_ENABLE_ROAMING_APP=y` (with its `IDF_EXPERIMENTAL_FEATURES` gate): **low-RSSI trigger** (threshold −72 dBm; healthy APs here are −29..−31) + **legacy roam** (forcible disconnect→reconnect to a stronger BSSID — works without AP 802.11k/v, which this mesh lacks: `pmf:0`); **periodic-scan monitor OFF** (active scans every 30 s would punch holes in the 180 s TX cycle) and **802.11v/BTM OFF**. The app runs in the supplicant `eloop` (no new task) and auto-wires through the default `esp_netif_create_default_wifi_sta` handlers we already use. **Reconnect ownership:** the roaming app's disconnect hook calls `esp_wifi_connect()` itself, so under `CONFIG_ESP_WIFI_ENABLE_ROAMING_APP` `main.c` now **defers** its own `EV_DISCONNECTED → esp_wifi_connect()` (avoiding two lifecycle owners racing the supplicant); non-PSRAM Heltec keeps the hand-rolled reconnect via the `#else`. Both `CYCLE` log shapes now trail `bssid=… ch=…` so a roam (or a stuck weak link) is visible in syslog. **⚠️ Espressif marks the roaming app EXPERIMENTAL — bench-validate on one node before fleet OTA.**

**Review hardening (pre-tag, local MAX code review):** (1) **reconnect-ownership gap fixed** — the deferral to the roaming app now triggers only *after the first association* (`n_connects > 0`); the app's `allow_reconnect` is false until then, so a failed first connect at boot (AP down/slow) was previously retried by *nobody* until the 10-min startup watchdog rebooted. Pre-association we keep our own 500 ms retry (can't race the idle app). (2) `mark_attempt()` on both reconnect paths so `n_attempts` no longer freezes under roaming. (3) the roaming kconfig block (was triplicated across the 3 PSRAM board overlays) moved to a shared **`sdkconfig.defaults.psram`** consumed via CMakeLists `SDKCONFIG_DEFAULTS` (one source of truth; Heltec still excluded). (4) the two `config_post` result-page branches collapsed to a single render path, dropping the separate `warn[]` stack buffer. (5) removed redundant `# CONFIG_LWIP_DHCP_DEBUG`/`NETWORK_ASSISTED_ROAM=n` lines that tripped kconfig "disabled symbol with user-set value" advisories (DHCP debug stays off via the lwIP master switch; BTM off via its unmet WNM dependency). **Round 2** added (6) a **reconnect safety-net** — since the roaming app is experimental, if it ever stalls and leaves the node disconnected (no IP) for >30 s post-association, the main loop forces one `esp_wifi_connect()` and re-arms (cleared on `GOT_IP`); the long window means it never races the app's normal ms–seconds reconnect; and (7) trimmed the roaming rationale that had been restated in three places down to one canonical home (CHANGELOG + the main.c comment), leaving `sdkconfig.defaults.psram` with a short pointer.

---

## V2.5.33 — heap-guard root-cause fix: PSRAM offload + configurable confirm window

**What:** Two related changes to stop the heap-guard auto-reboot from firing prematurely on `.198` (and any PSRAM board):
1. **Tier-1 PSRAM offload** (all PSRAM boards — feathers3_d, QT Py PICO, XIAO S3; Heltec excluded, no PSRAM): WiFi/lwIP pbufs+PCBs and mbedTLS record buffers now allocate from PSRAM (`CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`, `CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y`), and the malloc internal-only threshold drops 16384 → 4096.
2. **Configurable heap-guard confirm window** — new `/config` field "Heap-guard confirm cycles" (`hg_confirm`, default 10, range 2–240), replacing the hard-coded 5. The heap-guard log line now also reports node **uptime**. The explanatory blurb under the floor field was removed.

**Why:** Field forensics on esp32-5965048 (`.198`) showed the guard was rebooting on a **transient, self-healing** fragmentation dip, not the slow month-scale creep it was designed for. The INTERNAL largest-free block sat steady at 68 KB for 37 h, then a single inbound `/config` (plus per-cycle outbound TLS churn) left a small long-lived buffer mid-arena that bisected it to 39 KB — below the 44 KB floor. The dip self-heals in 3–5 cycles (observed coalescing back to 68 KB), but the 5-cycle confirm window occasionally caught it and rebooted for nothing. Routing those network/TLS buffers to PSRAM removes the bisecting allocator at the source (the real fix); the confirm-window bump to 10 rides out any residual transient (the belt-and-braces); the uptime line lets the syslog reader tell a genuine slow creep from a same-day false-positive at a glance.

**How:** `tx_heap_guard()` takes `confirm_cycles` from the new config field (wired through `config_fields.def` → `tx_context_t` → `main.c`) and appends `Uptime: Nd HHh MMm` (via `esp_timer_get_time()`) to the reboot log line. PSRAM knobs added to each `sdkconfig.defaults.<board>` for the three PSRAM boards only. The 4 MB `/log` ring was already PSRAM-resident (V2.3.18) and is unaffected. ESP32-PICO-V3-02 is rev-3 silicon so the rev<3 PSRAM cache workaround does not apply.

---

## V2.5.32 — openSenseMap dispatched last (slow-target isolation)

**What:** Reorder the `TX_TABLE[]` dispatch so the production openSenseMap target runs **last** of the upload targets (staging, normally disabled, sits just above it). No other behaviour change.

**Why:** The TX dispatch loop runs each target's `send()` **synchronously**, so a slow or hung target stalls every target listed below it. OSM's ingress has been the worst offender for connect-timeouts (`ESP_ERR_HTTP_EAGAIN`, "Connection timed out before data was ready") and `502`s — at ~16–18 s/attempt × 4 retries that is ~70 s of a cycle spent blocked. Observed live on 2026-06-16 (~04:00–04:32 UTC). With OSM dispatched after the reliable targets (Madavi, sensor.community, Radmon, GMC, ThingSpeak, aqi.eco), their data is already away before OSM is allowed to spend the time budget.

**How:** Pure data reorder of the V2.5.5 table-driven dispatch — the prod `TX_TARGET_OSM` row moved to the end of `TX_TABLE[]`. Safe because the circuit-breaker, fail-streak, and per-target stats are all indexed by `tx_target_id_t`, not array position, so failure tracking and the `/status` display are unchanged. OSM's OLED slot is `-1` (no display slot), so there is no UI ordering to disturb either.

---

## V2.5.31 — CI hardening + host-testable count logic (no firmware behaviour change)

**What:** A test/CI-only release that adds host coverage for previously-untested pure logic and removes long-standing duplication and flake sources from the GitHub Actions pipeline. The firmware is **behaviourally inert** vs V2.5.30 — the only `main/` change is a pure refactor that extracts existing decision logic into a testable header.

**Why:** Two real gaps surfaced reviewing the pipeline: (1) `url_encode_query_value()` was shipped in V2.5.20 (R2, to fix raw-credential mangling in the Radmon URL) with **zero** tests, and the V2.5.30 dead-time-guard decision lived buried in the IRAM count ISR where it could not be unit-tested despite taking two MAX reviews to get right; (2) the `build.yml` and `release.yml` cppcheck steps had **already drifted** (only the release copy passed `--std=c11`) despite a "keep in sync" comment, the board matrix was duplicated across both files, and a transient component-registry outage had reddened a release build (V2.5.30).

**How:**
- **New `main/tube_logic.h`** (pure, no IDF/FreeRTOS/HW) holds `clamp_u32()` (moved from `tube.c`), the new `gmc_classify()` — the count / guard-removed / reject decision extracted verbatim from `gmc_count_isr()` — and `guard_effective_us()`, the guard on/off policy `config_effective_guard_us()` now wraps. All `always_inline`, so they still fold into the IRAM ISR at zero cost and stay in IRAM. The ISR and `config.c` call into them; behaviour is byte-identical (verified by a real `heltec_v2` build).
- **+18 host tests** in `test/test_main.c`: `url_encode_query_value` (unreserved passthrough, every reserved char, high byte, empty, zero-dstsz, two truncation boundaries), `clamp_u32` (below / at / above the 2³² wrap), `gmc_classify` (guard-off, first-edge-never-guarded, in-window GUARD_REMOVED with inclusive boundary, sub-gate-is-reject-not-removed, outside-window count), and `guard_effective_us` (disabled / pcnt-wins / enabled).
- **Reusable workflows** `_cppcheck.yml` + `_build-boards.yml` are now the single source of the cppcheck gate and the 5-board matrix, called by both `build.yml` and `release.yml` — the `--std=c11` drift is now structurally impossible. `_build-boards.yml` toggles merge-bin / version==tag / staging via a `release` input.
- **Pipeline resilience:** the network-touching IDF Component Manager step (`reconfigure`) is **retried** 3× (the durable fix for the V2.5.30 registry flake — we deliberately do **not** commit `dependencies.lock`, gitignored in V2.5.13 for per-board `target:` churn); **ccache** is wired through `extra_docker_args` with `CCACHE_DIR` inside the mounted workspace so `actions/cache` persists it across runs; `esp-idf-ci-action` is pinned to the **v1.2.0 commit** (`e6f5c74`) instead of the moving `@v1` branch; `build.yml` gained **concurrency cancel-in-progress** (release never cancels); and a **CHANGELOG preflight** job fails a tag release in seconds (using the extractor's exact matcher) instead of after a 5-board build with a placeholder body.

**Note:** the reusable-workflow refactor renames the CI status checks (e.g. `build / heltec_v2` → `build / build / heltec_v2`); update any required-status-check rules on `main` to match.

---

## V2.5.30 — opt-in dead-time guard (afterpulse / re-trigger burst-collapse)

**What:** New opt-in dead-time guard — a `deadtime_guard` checkbox + a `deadtime_guard_us` window (µs), modelled on the `pcnt_filter` checkbox+width pair. A *retriggerable* refractory layered on top of the fixed 190 µs ISR dead-time gate: an edge can only start a new count after a quiet gap longer than the window; edges arriving inside it extend the dead zone and are dropped, collapsing an afterpulse / re-trigger **train** to a single count. The per-cycle count of suppressed edges is surfaced on the `DIAG:` line as `guard_removed=N`. **Mutually exclusive with `pcnt_filter`** (see below).

**Why:** The 2026-06-14 overnight bench analysis decomposed the QT Py's +10 % over-count vs the Heltec into **~60 % spurious 1–5 ms re-triggers + ~40 % genuine over-sensitivity** (see `reference_radiation_data_analysis`). The existing 190 µs gate only catches the 50–190 µs afterpulse, and the V2.5.16 PCNT width filter is a *sub-pulse* (4 µs) glitch filter — 3 orders of magnitude too short in time to touch a re-trigger arriving 1–5 ms later. The dead-time guard is the tool that reaches that band. It is a **diagnostic / characterisation** feature, NOT a production fix: it introduces a board-dependent dead-time-loss term (would step the multi-year archive) and only removes the spurious ~60 %, so Heltec stays the dose baseline and the 44205 µSv constant is unchanged.

**How:** The GMC count ISR keys the window on the *previous edge* (`isr_last_edge_us`, captured before its per-edge update), which makes it retriggerable — every edge in a burst, counted or not, keeps the channel dead, so the whole train collapses to the leading count. `guard_removed` rides the same locked snapshot+reset as the raw-edge profiler (`tube_get_diag` gained one out-param). The config field auto-generates through the existing X-macro schema (struct / NVS / POST), with a `/config` numeric input (greyed via `syncTube()` and force-cleared when the tube is off) and a boot config-dump line. **Live-applied** on `/config` Save (the ISR reads the volatile window each edge — no reboot, unlike the hardware-latched `pcnt_filter`) and also set at boot. `guard_removed` counts only **real** suppressed counts (edges that would have cleared the 190 µs gate — its true marginal effect, a *subset* of the DIAG `rejected`, not an orthogonal column). **Bench-validated on the QT Py** at guard = 5000 µs: the cycle count drops ~6 % (matching the predicted spurious fraction), `guard_removed` reports exactly those suppressed real counts, and the raw-edge histogram is unchanged — same pulse train, spurious 1–5 ms population no longer counted. Off by default on every board, so the release is behaviourally inert until a node opts in.

**Review hardening (pre-tag, local MAX code review):** (1) `guard_removed` now tallies only the *marginal* edges (past the 190 µs gate) instead of every in-window edge, so it no longer double-credits the 50–190 µs afterpulses the gate already owned; (2) the edge-delta and inter-pulse-gap `uint32` casts are clamped to `UINT32_MAX`, closing a latent wrap (a >71.6 min quiet gap could otherwise mis-block a genuine pulse / mis-record min-max) — the clamp also covers the pre-existing count-path `dt`; (3) live-apply replaces the reboot-required handling (the guard is a software volatile, not a hardware latch); (4) the `edt` subtraction is computed once and reused by the guard; (5) the field became a checkbox+window pair (was a bare numeric, which mis-implied a dependency on PCNT) with the window range enforced at 200–20000. A **second MAX review over the whole `V2.5.29..HEAD` range** found **no correctness bugs** (the `/config` arg realignment, the ISR count semantics, and the 3-way mutual exclusion all verified clean); minor cleanup applied — the saturating µs-delta cast factored into a `clamp_u32()` helper, the first-edge guard term dropped (`edt` now defaults to `UINT32_MAX`), and the boot-dump "superseded" label derives from `config_effective_guard_us()` (the single-source policy) instead of re-deriving it.

**Mutual exclusion with `pcnt_filter` (the footgun the review surfaced as #6):** the guard runs in the GMC ISR, but `pcnt_filter` makes the separate PCNT *hardware* path authoritative for the uploaded count — which the ISR guard cannot reach. So with both on, the guard silently affects only the logged pre-filter value, not the real CPM (caught live on the QT Py: status showed *"raw CPM 90 → filtered 97"* — the guard pulled the ISR count to 90 while the uploaded value stayed 97 from the un-guarded PCNT path). The two are now **mutually exclusive, `pcnt_filter` wins**, enforced in three places: `config_post` force-clears `deadtime_guard` when `pcnt_filter` is set, the `/config` UI greys + unticks the guard box (`syncGuard()`) whenever PCNT is on, and `config_effective_guard_us()` (the single source of the on/off policy, used by both the boot and live-apply paths) returns 0. They target different physics regardless — `pcnt_filter` the ESP32-S3 narrow-*width* pulses, the guard the *time-domain* 1–5 ms re-triggers (so on the classic-ESP32 QT Py, `pcnt_filter` is pointless and the guard is the right tool).

**Also (`/config` page tidy):** added a **Network** `<h3>` above the WiFi fields; renamed **Other** → **Time** (it now holds only NTP + timezone); moved the **web-admin/AP password** under the AP SSID (Network), the **sensor-data upload interval** to the top of Transmission targets, and the **heap-guard floor** to the bottom of Hardware. Pure form-layout move (field `name=`s unchanged, so NVS/POST untouched); the format string and its `%`-arg list were relocated as matched pairs (verified by `-Wformat` under `-Werror`). Also corrected the syslog blurb's framing label **RFC 3164 → RFC 5424** (the code has emitted 5424 since V2.5.27 — `<PRI>1 … - - -` with the version field + NILVALUEs; the `/config` text, the `config_fields.def`/`syslog.h` headers, and a buffer-sizing comment were stale).

**Also (mbedTLS dynamic-buffer re-test on ESP-IDF 6.0.1 — FAILED, reverted):** 6.0.1's TLS 1.3 / dynamic-buffer fix prompted a re-test of `CONFIG_MBEDTLS_DYNAMIC_BUFFER` on heltec_v2 (disabled since V2.4.7). **Both variants** — `DYNAMIC_BUFFER` alone (the Kconfig-canonical config), then all three with `FREE_CONFIG_DATA` + `FREE_CA_CERT` (the aggressive ~20–30 KB variant) — PANICKED on the *first* FTPS upload with the same TLSF `block_is_last` heap-corruption assert inside our hand-rolled `log_ftp.c::io_close` → `mbedtls_ssl_session_reset`. The asserting free site **wandered across the 3 historical runs** (msg-layer buffer / peer-cert ASN.1 / PSA transform key) = heap corruption *upstream* of `session_reset`, not a single double-free. **Reverted** — the sdkconfig config *values* are back to the V2.4.7 baseline (only documentation comments changed). The real fix would be a `log_ftp.c` teardown rewrite (skip `session_reset`, `mbedtls_ssl_free` only — we never reuse the session) or wrapping FTPS in `esp_tls`, not a kconfig toggle; don't re-test the kconfig alone on future IDF bumps. Coredumps decoded + saved under `Geiger_Log/`.

## V2.5.29 — fix cross-task log-fragment interleaving (ring + syslog)

**What:** Logical-line reassembly moved up into `applog_vprintf()`, so a complete log line is forwarded to **both** the `/log` ring and the UDP syslog at once. Fixes the cross-task splice where a second task logging between our fragments mixed its line into the middle of ours (the mangled `syslog: started` boot line).

**Why:** ESP-IDF v6 splits one `ESP_LOG` into ~3 vprintf fragments, and `applog`'s mutex is taken/released *per fragment*. Pre-V2.5.29 each fragment was forwarded immediately: the `/log` ring concatenated fragments verbatim (so it spliced), and `syslog.c` re-joined them in its *own* accumulator — so the two surfaces could even disagree about the same boot. The interleaving was diagnosed via a separate design-review agent, which also showed the original "fix it in syslog only" idea was a trap (it would leave the ring — the primary forensics surface — still spliced).

**How:** `applog_vprintf` accumulates fragments into a single BSS line buffer (reusing the existing `LOG_LINE_MAX` allocation — **~0 extra RAM**) and flushes to `ring_append` + `syslog_emit` only on end-of-line (or buffer-full). A `xTaskGetCurrentTaskHandle()` owner check flushes a *different* task's pending partial before appending, so fragments never mix. `syslog_emit()` collapses to a dumb framer — its `s_accum` accumulator is **deleted** (net code reduction). `strip_ansi`/`rewrite_boot_ts` now run once per whole line instead of per fragment. The interrupted line still splits into a stub + orphan tail (whole-line reassembly across an interruption would need per-task buffers — not worth the RAM on the tight-DRAM Heltec for a cosmetic, boot-mostly defect), but it is no longer *mixed* and both surfaces now agree.

**Also (config-dump UDP pacing):** validating the reassembly on hardware showed the device `/log` ring holds the *complete* config dump while the syslog server was missing ~⅔ of it — i.e. pure UDP burst loss, not a reassembly bug. The ~28-packet boot dump overruns lwIP's pbuf pool on the tight-heap heltec faster than the V2.5.24 "yield every ~5 lines" pacing drains it. `config_log_summary()` now yields **one tick after every line** (a `LOG_PACED` macro replacing the 5 scattered `vTaskDelay`s), so lwIP transmits between sends — adds ~150–280 ms to one boot event. The yield is **gated on `syslog_enable`** so the syslog-off boot path (`config_load`) pays nothing.

**Also (syslog drop counter):** `emit_packet()` previously discarded the `sendto()` return, so device-side UDP drops (lwIP pbuf-pool exhaustion under burst) were invisible. It now counts send successes/failures (silently — never `ESP_LOG` from the emit path), exposed via `syslog_get_stats()` and reported once per TX cycle as `syslog udp: tx=N drops=M (since boot)`. Lands on `/log` even when syslog itself is dropping, so the loss is self-evident. `drops>0` is a definitive device-side-loss signal; zero doesn't prove zero loss (driver-late / network / server-side drops aren't visible here).

**Also (toolchain → ESP-IDF 6.0.1):** built on **ESP-IDF v6.0.1** (`idf_component.yml` `idf: ">=6.0.1"`). Point release; for our stack the relevant fixes are WPA3/SAE crypto + memory-leak, an HTTP-client `Content-Length`/`Transfer-Encoding` fix, an I²C-driver-allocated-from-PSRAM-when-cache-disabled **crash** fix (S3 boards), and a TLS 1.3 / dynamic-buffer handshake fix. All 5 boards build clean on 6.0.1, `-Werror` green.

**Also (heltec /config buffer 16→24 KB):** the worst-case `/config` page outgrew the heltec's 16 KB `HAL_CFG_FORM_BUF_SIZE` — the openSenseMap-staging rows (V2.5.26) + a custom MQTT-TLS CA cert push it to ~16.9 KB, so the page was silently truncated mid-form (bottom half + Save buttons missing). Latent since V2.5.26; only the heltec hit it (PSRAM boards are at 32 KB). The device's own `config page truncated: needed N bytes, buffer is M` ERROR line pinned it; both heltec variants share the `#if` block, both fixed. 24 KB transient malloc is safe against the heltec's ~64 KB largest free block.

## V2.5.28 — OTA logging reaches the syslog server (detailed flash trace)

**What:** The full firmware-update process is now visible on the syslog server — a start line, **128 KB progress ticks** (`OTA progress: 512/1289 KB (40%)`), a receive-complete line with elapsed time + throughput, the verify/commit steps, and a final **`OTA SUCCESS: V2.5.27 -> V2.5.28 (N bytes) — rebooting in ~2s`**. Every failure path now logs a greppable **`OTA FAILED: <stage> — <reason>`** headline.

**Why:** The OTA-prep teardown called `syslog_stop()` *before* the flash, so the rich logging that already existed in `update_post()` only ever reached the device ring buffer + serial — never the server. Failed OTAs in particular were invisible to server-side forensics. The teardown's actual heap win is `mqtt_stop()` (~50 KB TLS) + `log_ftp_pause()`; the syslog UDP socket is a few hundred bytes, so keeping it open through the flash is negligible and does not reintroduce the OTA-OOM the teardown was built to prevent.

**How:** Dropped `syslog_stop()` from the teardown (`http_server.c`) so the socket survives the whole flash; `mqtt_stop` / `log_ftp_pause` / `tube_pcnt_stop` are unchanged. Added wall-clock telemetry around the receive loop (`esp_timer_get_time()` start, a 128 KB progress gate, a receive-complete summary), relabelled the terminal error logs with an `OTA FAILED:` prefix, added an `esp_ota_end ok — verifying` phase marker, and rewrote the commit log line as `OTA SUCCESS: <running> -> <new>`. The existing 2 s main-loop pre-restart delay flushes the final line; a 100 ms yield after it adds belt-and-suspenders margin since syslog send is fire-and-forget (`MSG_DONTWAIT`).

**Also (boot-slot visibility):** the boot banner now reports the running OTA partition (`… Reset reason: … - Partition: ota_1 - Board: …`) via `esp_ota_get_running_partition()`, and the OTA success line names the target slot (`… boot set to ota_1 …`). Together they close the OTA loop: write → `boot set to ota_1` → after reboot the banner's `Partition:` confirms the switch stuck (or exposes a bootloader rollback if it shows the other slot).

**Also (syslog timestamp, local instead of UTC):** when the clock is synced, the RFC 5424 timestamp is now **local time with the numeric UTC offset** (e.g. `2026-06-13T20:45:50+10:00`) instead of UTC `Z` — so on a collector you don't control the header matches the device's in-message time and the status-page/NTP line. The dead `ntp_localtime_str()` helper was repurposed to emit that format (one `strftime` with `%z` + a colon splice, DST-aware via the configured TZ string), and `syslog.c`'s timestamp block collapses to `ntp_time_valid() ? ntp_localtime_str() : "-"` (replacing the V2.5.27 UTC branch). Pre-sync still emits NILVALUE `-`.

## V2.5.27 — syslog RFC 5424 framing (pre-NTP NILVALUE timestamp)

**What:** The UDP syslog client now frames messages as **RFC 5424** instead of RFC 3164. Before the wall clock is valid, the timestamp field is emitted as the NILVALUE `-`; once NTP has synced (or a soft-reboot RTC carry-over is sane) it carries a real RFC 3339 UTC timestamp (e.g. `2026-06-13T09:00:34Z`).

**Why:** The boot banner + the full `config:` dump are emitted in the early-boot window, *before* the first NTP sync (which can take minutes). Under RFC 3164 there is no "unknown time" token, so the pre-sync path sent a well-formed but bogus `Jan  1 00:00:00`. A collector configured to trust the message's *reported* time then filed those lines under the wrong day — on our rsyslog server they landed at `2026-01-01T00:00:00`, invisible to any `grep 2026-06-13` and mis-sorted in the file. RFC 5424's NILVALUE `-` is defined to mean "originator has no reliable time," so a stock collector falls back to its own receive time automatically — no server-side template change required.

**How:** `emit_packet()` in `syslog.c` reframed to `<PRI>1 TIMESTAMP HOSTNAME geiger - - - MSG` (APP-NAME `geiger`; PROCID / MSGID / STRUCTURED-DATA all NILVALUE). The clock-sane gate now reuses **`ntp_time_valid()`** (the V2.5.24 single-source predicate) instead of a duplicated `1735689600` literal. That predicate's floor was also bumped **2025-01-01 → 2026-01-01** (`EPOCH_2025` → `EPOCH_2026`): the firmware only runs from 2026 onward, so any 2025-dated clock is a stale/bad sync and is now correctly treated as not-yet-valid. This tightens the same gate used for TLS-cert and FTP timestamp readiness — safe, since a genuinely synced clock reads 2026-06+. **Note:** the rendered server tag changes from `geiger:` to `geiger` (RFC 5424 APP-NAME carries no colon) — cosmetic.

## V2.5.26 — new TX target: openSenseMap STAGING (beta)

**What:** A ninth upload target, **"openSenseMap STAGING (HTTPS only)"**, on the `/config` page directly below the existing openSenseMap block. Independent Box ID + Access Token; posts the standard Luftdaten body to `upload.staging.opensensemap.org/boxes/<id>/data?luftdaten=1`.

**Why:** The "new openSenseMap" public beta runs a separate staging stack. This lets a node mirror to production *and* staging at once during the beta without disturbing the prod openSenseMap config. Verified 2026-06-13 that our existing upload format is accepted unchanged by the staging endpoint (201 "Measurements saved in box").

**How:** `send_osm()` refactored into a shared `send_osm_to(host, box_id, token, use_insecure)` core — production (`ingress.opensensemap.org`) and staging (`upload.staging.opensensemap.org`) are now thin wrappers over it. New `TX_TARGET_OSM_STAGING` + config fields `send_osm_staging`/`osm_staging_box_id`/`osm_staging_token` (X-macro schema), wired through the dispatch table, the status-page Uploads block, and the config boot-dump. HTTPS-only (no plaintext staging endpoint).

## V2.5.25 — sensor.community pressure unit fix (hPa → Pa)

**What:** The barometric pressure POSTed to sensor.community is now in **pascals**, not hectopascals. `build_sensorc_bme_body` no longer divides `bme_pressure_pa` by 100; the optional sea-level-reduced `pressure_sealevel` is likewise emitted in Pa.

**Why:** sensor.community expects pressure in Pa. Confirmed against three independent sources: airrohr sends `BME280_pressure` as the raw Bosch Pa reading (its `/100` is display-only), the original MultiGeiger sends `Adafruit_BME280::readPressure()` (Pa), and SC's own example payloads use Pa (e.g. `"BMP_pressure":"100590"`). Our Madavi / openSenseMap / aqi.eco bodies already sent the raw Pa value — only the sensor.community body had the stray `/100`, so every node reporting a BME/BMP to SC had been publishing pressure ~100× too low (~1019 instead of ~101900). It never errored because SC doesn't range-check magnitude (HTTP 201 throughout), so it was silently wrong for the entire BME-on-sensor.community history.

**How:** Send `c->bme_pressure_pa` directly; the sea-level barometric factor is dimensionless so Pa-in → Pa-out. Field names and X-PINs unchanged — only the unit (value magnitude) changes. Verified against airrohr / original MultiGeiger and live `geiger/bme rc=201` on both radiation nodes.

## V2.5.24 — DRY cleanups + config-dump pacing

Follow-up to the independent review of V2.5.22/23:
- **Shared `chip_model_str()`** in `sysinfo.h` replaces the model-string ternary ladder that had been copy-pasted in three spots (`main.c` boot `chip:` log, `http_server.c` /status device block, `syslog.c` boot banner). Switch + `default` keeps `-Wswitch` happy.
- **`http_server.c` reuses `ntp_time_valid()`** instead of two raw `1735689600` ("> 2025-01-01") literals — the clock-sane predicate now has a single source of truth (`EPOCH_2025` in `ntp.c`).
- **Config dump is paced** — `config_log_summary()` yields one tick every few lines. The boot `config:` dump is ~27 syslog UDP packets sent back-to-back; on the tight-heap heltec (plain ESP32, ~99 KB) ~1 in 5 were lost to random pbuf-alloc failures, because the main task never yielded for the WiFi/lwIP task to drain its TX queue. Five yields fix it; no-op on the S3 boards and on the syslog-off boot path. (Confirmed by device-vs-server log diff: the dropped lines were scattered *mid-dump* = random loss, not a priming/ARP-timing issue.)

The first two are pure refactors; the pacing adds ~50 ms to the once-per-boot dump.

## V2.5.23 — full config dump now reaches the syslog server

**What:** The boot `config:` trace (the ~28-line dump of every setting, secrets masked) now lands on the rsyslog server, not just the device's `/log` and serial console.

**Why:** Like the boot banner, the config dump was logged at `config_load()` time — long before WiFi + syslog come up — so it never reached the server. Server-side you couldn't see a node's actual config without hitting its `/config`.

**How:** The dump body was extracted into a single `config_log_summary(const config_t *cfg)` (so adding a config field still touches only one place). It's printed **once per boot**: at boot when `syslog_enable` is **off** (there's no server to forward to later, so the boot copy is the only chance), or right after the syslog UDP client comes up when syslog is **on** (so it reaches `/log` *and* the server). Syslog-disabled nodes therefore still get their dump locally; syslog-enabled nodes get it on the server. No duplicate printing.

## V2.5.22 — boot diagnostics: /status start time + a server-side boot banner

**1. /status Uptime line shows the boot time.** The System block's `Uptime:` line now appends the boot wall-clock — e.g. `Uptime: 00h 03m 37s (Started 2026-06-12 15:36:00)`. Boot instant = `time(NULL) − uptime`, rendered as local time via the existing `format_wallclock()` helper, gated on the same `>2025-01-01` clock-sanity check as the NTP line (pre-sync shows the bare uptime, never a nonsense "Started 1970-…"). Rendered into a separate suffix buffer so it stays a single bounded `snprintf`.

**2. Syslog boot-summary line.** The real boot banner (firmware version / board / chip / reset reason) is logged before WiFi + syslog come up, so it never reached the rsyslog server — leaving the firmware version and reset reason invisible to server-side forensics (the gap that once hid an OTA behind an unexplained count-rate jump). `syslog_init()` now emits a one-line summary as the **first** packet once the socket is live, under a greppable `boot` tag:

```
boot: Firmware V2.5.22 (IDF v6.0) - Reset reason: SOFT (esp_restart) - Board: seeed_xiao_esp32s3 - Chip: ESP32-S3 rev v0.2 (2 cores) - Coredump: none - Free heap: 4330740 B (largest 4128768 B)
```

The reset reason distinguishes a clean reboot from `BROWNOUT` / `TASK_WDT` / `PANIC` / `POWER_ON`; `Coredump: PRESENT` pairs with `PANIC` to point at `/coredump.elf`; the free/largest-heap pair gives a fragmentation baseline. Reuses `reset_reason_str()` (sysinfo.h), `BOARD_NAME` (hal.h), and the chip-model formatting from `main.c`.

## V2.5.21 — fix: GNSS I²C "hardware timeout" floods under a live multi-GNSS fix

**What:** Added `.scl_wait_us = 13000` to the GNSS `i2c_device_config_t` in `gnss_init()`.

**Why:** The u-blox MAX-M10S (and, less so, the PA1010D) **clock-stretches** — it holds SCL low while assembling NMEA. With the per-device timeout left at its short default, a sustained multi-GNSS fix (a real 12-satellite GPS+Galileo+BeiDou fix emits a heavy GSV burst every second) routinely stretches past that timeout, so `gnss_poll()` reads trip `I2C hardware timeout` — which IDF logs with the misleading `GPIO X is not usable, maybe conflict with others` wording — several times a second. The drain loop retries, so fixes keep flowing and `i2c_err` stays 0, but the log floods. Raising the SCL-low timeout to ~13 ms (≈ the classic-ESP32 I²C timeout-register ceiling) lets the stretch complete instead of erroring.

**Scope:** per-device — does **not** change any other sensor's timeout, and the field is never built on boards without a GNSS (a no-op there). Covers both GNSS parts (M10S `0x42` and PA1010D `0x10`), which share the one `devcfg`. Bench-validated on a QT Py (esp32-15033992) with a live 12-sat fix: the timeout floods stopped, fixes unaffected.

## V2.5.20 — full-codebase review batch: 5 bug fixes + hygiene (no feature changes)

Outcome of a whole-tree review (bugs / security / memory / practices) of all ~16.6 KLOC in `main/`. Review IDs R1–R11 below match the review report.

**R1 — MQTT TLS Mode D could never connect.** "Mode D — skip server verification" (and the Mode B empty-CA fallback) set only `skip_cert_common_name_check`, which merely skips the CN match; with no CA attached, esp-tls aborts the handshake with "No server verification option set" unless `CONFIG_ESP_TLS_INSECURE` + `CONFIG_ESP_TLS_SKIP_SERVER_CERT_VERIFY` are enabled — and they weren't, on any board. Both symbols now set in `sdkconfig.defaults` **and** in the five committed per-board `sdkconfig.<board>` caches (defaults only fill *missing* keys). Verified-mode connections (Mode A bundle / Mode B CA, all HTTPS uploads) still verify fully — the change only converts the deliberate no-CA case from hard-fail to encrypted-but-unverified, which is what the UI advertised.

**R2 — Radmon credentials now percent-encoded.** User/password were interpolated raw into the submit query string, so a password containing `&` `=` `+` `%` or space broke or silently mangled the request. New generic `url_encode_query_value()` in `util.h` (RFC 3986 unreserved-set, 3× expansion bound); URL buffer 256 → 512 to fit worst-case encoded credentials.

**R3 — FTPS TLS handshake can no longer wedge the main task.** `io_upgrade_tls()`'s retry loop had no wall-clock deadline — on a half-open socket (WiFi drop mid-handshake) `WANT_READ` repeated forever; worse, the DATA socket had no `SO_RCVTIMEO` at that point, so `recv()` inside `mbedtls_ssl_handshake` could block indefinitely. FTPS runs on the **main task**, so this wedged TX scheduling, history, and the restart path. Now: 5 s socket timeouts set at upgrade entry + a 30 s handshake deadline (matching the `io_send_all`/`io_close` guard discipline the file header always promised).

**R4 — truncation-safe JSON/HTML body building.** The `transmission.c` body builders accumulated with bare `n += snprintf(buf + n, cap - n, …)`: one truncating call pushes `n` past `cap`, the next computes `cap - n` as a huge `size_t`, and silent truncation becomes an out-of-bounds write. Worst-case OSM body (~1.4 KB) had only ~13 % margin against its 1600 B buffer — one added field could have crossed it. New clamped `tx_append()` helper (mirrors `mqtt.c`'s `APPEND`); all three builders (Madavi env / sensor.community BME / Luftdaten) converted. `http_server.c`'s `format_als`/`format_gnss`/`format_pm_info` likewise converted to the existing `append_safe()` (forward-declared).

**R5 — AP SSID length fixed for 32-char names.** `ssid_len` was taken from snprintf's *untruncated* return, so a maximum-length `ap_name` beaconed 31 chars + a trailing 0x00 as its "32-char" SSID. Now `strlen()` of what's actually in the buffer.

**R6 — OTA refusal message labels the real board.** The chip-family check's `expected_board` strings were hardcoded: a QT Py reported itself as "heltec_v2 (ESP32)", a XIAO as "feathers3_d (ESP32-S3)". Now `BOARD_NAME " (ESP32[-S3])"` — always the running build's identity.

**R8 — upload credentials copied by value into `tx_context_t`.** They were `const char *` aliases into `g_cfg`, which the worker task reads for the whole multi-second upload cycle while a concurrent `/config` Save rewrites `g_cfg` wholesale (`*s_cfg = next`) on the httpd task — a torn/changed credential could reach an in-flight request. Nine fields (Radmon ×2, OSM ×2, aqi, GMC ×2, ThingSpeak ×2) are now fixed arrays filled by `safe_strcpy` in `build_tx_context`; `sw_version`/`chip_id` stay pointers (static literals). Always-non-NULL array checks simplified accordingly (cppcheck `knownConditionTrueFalse` guard).

**R9 — circuit-breaker counters now under `s_stats_mux`.** `breaker_open_cycles` was read/written lock-free in `tx_dispatch_one` while every other `s_stats[]` access takes the spinlock. Benign on Xtensa (aligned 32-bit) but inconsistent with the documented discipline; logging stays outside the critical section.

**R10 — syslog long lines: truncate-send + full-line accumulator.** Frames longer than the 600 B emit buffer were silently *dropped* (`n < sizeof` guard); now clamped and sent. Buffers resized for reality: emit 600 → 1200 B, accumulator 768 → 1100 B (the old size was derived from a misremembered 256 B line max; applog's `LOG_LINE_MAX` is 1024) — long lines now arrive whole instead of split across syslog rows.

**R11 — dead code removed.** `applog_snapshot()` — unused since the V2.3.16/17 streaming rewrites and the only API that malloc'd the entire ring (up to 4 MB on PSRAM boards) — deleted from `applog.c`/`.h`. `main.c`'s write-only `g_last_counts` removed. Stale comments fixed (`log_ftp.c` snapshot NOTE, `tx_transmit` "static literals" claim). `ntp.c`'s two `localtime()` stragglers → `localtime_r` for tree-wide consistency.

**Also (carried in from the V2.5.19 review, same version): L1 + L2.** L1 — `neopixel_init()` is now gated on `led_tick` too, so the WS2812 power rail (`PIN_NEOPIXEL_POWER`) is not energised when the per-pulse flash is disabled. L2 — the boot config dump prints the `i2c: pinout=` line only on boards with `HAL_HAS_I2C_PINOUT_SWITCH` instead of an inert `pinout=0` everywhere.

## V2.5.19 — fix: label the Seeed XIAO ESP32-S3 on the OTA page (was "(unknown board)")

**What:** Added an `#elif BOARD_SEEED_XIAO_ESP32S3` branch to the `UPLOAD_PROMPT_BOARD` compile-time chain in `http_server.c` so the `/update` page reads "Select a firmware .bin for **Seeed XIAO ESP32-S3**" instead of falling through to the `#else` "**(unknown board)**".

**Why:** The XIAO target shipped in V2.4.25 but was never given a label branch in that chain (which covered only Heltec 4 MB/8 MB, FeatherS3-D, and QT Py ESP32-PICO). Every XIAO build from V2.4.25 onward therefore showed "(unknown board)" on its OTA page. Cosmetic only — OTA itself always worked — but it made the page misleading. (Ironically it also made the page an accidental XIAO fingerprint: of the five supported targets, the XIAO was the only one that hit the `#else`, so "(unknown board)" uniquely identified a XIAO build.)

**How:** Label-string `#define` only; no HAL/pin/partition change. `BOARD_SEEED_XIAO_ESP32S3` is defined `=1` by CMakeLists like the other board macros, so the bare-macro `#elif` evaluates correctly.

**Also (same version): per-pulse LED now honours `led_tick`, + XIAO user LED support.** Two parts: (1) **Bug fix** — the QT Py NeoPixel registered its tube-pulse flash on `tube_enabled` alone, ignoring the `led_tick` config, so it flashed even with `led_tick` off. Now `main.c` gates the pulse-tick registration on `tube_enabled && led_tick` (the Heltec/FeatherS3-D already honoured it inside `speaker.c`'s `on_gm_pulse`). (2) **New** — the Seeed XIAO ESP32-S3 has an onboard user LED (GPIO21, **active-LOW** per Seeed's ESPHome `inverted: true`) that nothing drove: `speaker.c` is stubbed (`HAL_HAS_SPEAKER=0`), there's no NeoPixel, and `PIN_LED_BUILTIN` was undefined — so the single `tube_set_pulse_callback` slot went unclaimed. New `led.c`/`led.h` mirrors `neopixel.c` (ISR notifies a worker that blinks ~40 ms then auto-off), gated to boards with `PIN_LED_BUILTIN && !HAL_HAS_SPEAKER && !HAL_HAS_NEOPIXEL` (i.e. only the XIAO; no-op stubs elsewhere, so no callback-slot contention). New `PIN_LED_BUILTIN 21` + `HAL_LED_ACTIVE_LOW 1` in the XIAO HAL block; polarity abstracted via `HAL_LED_ACTIVE_LOW`. Reboot-required (matches the `led_tick` `*` marker; `led_tick` is boot-applied only). Built + cppcheck-clean on the XIAO (active), QT Py, and Heltec branches.

**Also (same version): QT Py I²C pin-out route toggle.** New board-gated `i2c_pinout` config bool (default false): on the QT Py ESP32-PICO it moves the primary I²C master bus from the onboard STEMMA QT connector (IO22/IO19) to the broken-out SDA/SCL castellated pads (IO4/IO33), for wiring a sensor/display straight to the header instead of via a Qwiic cable. Reboot-required (the bus is created once at first sensor probe). Implemented on the existing `ext_antenna` board-gated pattern: new `HAL_HAS_I2C_PINOUT_SWITCH` (1 on QT Py, 0 elsewhere) + `PIN_I2C_*_ALT` pins in `hal.h`; `i2c_bus_set_primary_pinout()` called from `main.c` right after `config_load()` and before the first `i2c_bus_get_primary()`; `i2c_bus.c` picks the pin pair and logs the chosen route; `/config` renders the checkbox (greyed "(not available on this board)" off-QT-Py, like the antenna switch) and the POST handler force-disables it where the HAL flag is 0. Boot dump prints `i2c: pinout=N`. Built + cppcheck-clean on both the active (QT Py) and inactive (Heltec) branches.

**Also (same version):** completed the boot-time config dump in `config.c`, which had drifted from the `config_fields.def` schema. 11 fields across 4 groups were loaded/saved but never printed at startup — **GMCMap** (`send_gmc`/`gmc_account_id`/`gmc_geiger_id`, V2.5.1), **ThingSpeak** (`send_thingspeak`/`thingspeak_https`/`thingspeak_api_key`, V2.5.1), **ThingSpeak PM** (`send_thingspeak_pm`/`thingspeak_pm_https`/`thingspeak_pm_api_key`, V2.5.4), and **PCNT filter** (`pcnt_filter`/`pcnt_filter_width_ns`, V2.5.16). Added four log lines mirroring the existing style (account/geiger IDs in clear like `radmon_user`; ThingSpeak write keys `MASK()`ed). Visibility-only — these values were always stored and used correctly; they just gave no startup evidence. The dump is deliberately hand-written (not generated from the X-macro), which is why it was the one consumer that could silently desync.

## V2.5.18 — fix: relocate the heap-guard auto-reboot off the 1 Hz tick onto the per-cycle resting snapshot (stop the /log-fetch misfire)

**What:** The `heap_guard_floor_kb` INTERNAL-fragmentation auto-reboot (V2.5.14) moved from `periodic.c::periodic_heap_guard()` — called every main-loop tick (~1 Hz) — to `transmission.c::tx_heap_guard()`, evaluated **once per TX cycle** (~180 s) on the same resting heap snapshot the per-cycle `diag: per-cycle heap` line already reports. The predicate is now a fragmentation test, not a bare floor test: it counts a cycle only when `INTERNAL largest < floor` **AND** `INTERNAL free_total > 2×floor`; 5 consecutive qualifying cycles (~15 min) trigger the reboot.

**Why:** The V2.5.14 guard sampled at 1 Hz on the main task, so it saw the **transient** largest-free dip that *any* inbound HTTP connection causes — the lwIP/TCP receive buffers are DMA-capable INTERNAL RAM, and a held-open `GET /log` (a chunked stream) drops INTERNAL `largest` for the duration. Root-caused 2026-06-09 from the .150 syslog: each `HEAP GUARD` line landed 2–3 s after a `GET /log from 10.11.12.67` (the monitor host) — the monitoring's own log fetches were rebooting the Feather every ~4 h, while the per-cycle heap was flat-healthy (`largest=70k`) the rest of the time. The old gating (2 h arm-delay, 6 h rate-limit, `tx_is_idle()` sample, 3-sample debounce) couldn't catch it: `tx_is_idle()` only sees the TX worker, not the HTTP server task serving `/log`, and the 3-sample (~3 s) window sat right inside the stream.

**How:** The `free_total > 2×floor` arm is the load-bearing transient filter — a connection that grabs net buffers drops **both** largest and free, so it fails the arm and never counts; only genuine fragmentation (largest low while free stays high — the OTA-stall / long-tail-OOM precursor) qualifies. The check rides the per-cycle snapshot via a new `tx_context_t.heap_guard_floor_kb` field (copied from `g_cfg` in `build_tx_context`), so it reads exactly the floor it logs, on the TX worker's quiescent pre-upload moment. Reboot still goes through `main_request_restart()` (the same path `/reboot` uses). **Deleted:** `periodic_heap_guard()`, the `tx_is_idle()` guard-gate, the arm-delay, the 6 h rate-limit, and `s_last_reboot_ms`; `periodic_loop()` loses its `heap_guard_floor_kb` parameter. The `heap_guard_floor_kb` config knob is unchanged (existing nodes' setting carries over). **No NVS persistence:** a production node (no PCNT comb) boots with a healthy largest block and cannot enter the state, so it can't boot-loop; the one residual loop risk is the experimental PCNT width-comb taking an unlucky INTERNAL boot-split, accepted on the bench (`pcnt_filter` off by default).

## V2.5.17 — fix: the PCNT filter checkbox could never be enabled (form-name vs NVS-key mismatch)

**What:** In V2.5.16 the `/config` "PCNT pulse-width filter" checkbox was `name="pcnt_filter"`, but the POST handler dispatches on the **NVS key**, which is `"pcnt_filt"` (`config.c::config_post` → `strcmp(key, k)`). `"pcnt_filter" != "pcnt_filt"`, so ticking the box was silently ignored: the bool stayed pre-cleared to `false` and **`pcnt_filter` could never be set true from the web UI** — the entire feature was un-enableable as shipped.

**Why it happened:** the `pcnt_diag → pcnt_filter` rename. When the field was `pcnt_diag` the struct name and NVS key were both `"pcnt_diag"`, so the HTML `name=` matched by coincidence; the rename made them diverge (struct `pcnt_filter`, key `pcnt_filt`) and the HTML `name=` was left as the struct name. The `pcnt_filter_width_ns` input (`name="pcnt_filt_w"`) was already correct. No other field is affected (every pre-existing field's HTML name already equals its NVS key).

**Fix:** checkbox `name`/`id` → `"pcnt_filt"`, and the `syncTube()` id list updated to match. Not an NVS-size or save-path issue — `config_save()`'s error handling is correct and NVS (0x6000 = 24 KB) is not full. Three V2.5.16 reviews missed it because they checked C-side rename consistency + printf-arg order, not the HTML-name == NVS-key contract.

## V2.5.16 — PCNT pulse-width filter (+ width diagnostic) for the board-gap, off by default

**What:** A new opt-in **width filter** (`tube_pcnt.c`) on the GMC count pin, built from up to 4 parallel hardware PCNT units running the peripheral glitch filter at 0 / 250 ns / 1 µs / 4 µs. When the `pcnt_filter` flag is **on**, the authoritative per-cycle count switches from the ISR (dead-time-gated) count to the **4 µs-filtered PCNT count** — dropping the narrow pulses behind the board gap — and that filtered count drives CPM, dose, `/status`, MQTT and all uploads. The full diagnostic is still logged every cycle: the `DIAG:` line (raw edges + spacing histogram), the `PCNT:` line (the 4-width comb), plus a new `FILTER:` line carrying the **pre-filter** counts/cpm so the unfiltered reading stays visible.

**Why:** The socketed Feather (ESP32-S3) reads a stable **+15–17 %** CPM over the Heltec (ESP32), localised to the MCU board (controlled PCB/HV/parts swaps ruled out everything else) and characterised as ESP32-S3 input over-sensitivity — about half of the excess is a 1–4 µs narrow-pulse population the ESP32 never registers. A 4 µs width filter removes that half (gap → ~+8 %). The measures the *width* axis the production ISR never saw (it only gates on *spacing*: the 190 µs dead-time + V2.5.12 edge histogram).

**How:** PCNT taps the same pad through the GPIO matrix, parallel to the ISR (unlike ESPGeiger's compile-time PCNT replacement on legacy `driver/pcnt.h`); built on IDF v6 `driver/pulse_cnt.h`. The ISR count is retained as `counts_raw` (pre-filter reference + the `rejected` calc); the snapshot is read once per cycle and reused for both the filter election and the `PCNT:` dump. New `esp_driver_pcnt` REQUIRES. Gated by a new `pcnt_filter` config flag (default **off** on every board), shown indented under "Enable Geiger tube" on `/config`, tube-gated like the radiation TX targets. Reboot-required (units come up at tube setup). The filter is **not recommended on the production radiation node** (it only halves the gap and may clip genuine pulses — see reference notes); it's offered for new nodes or after a check-source validates the narrow pulses as noise.

**Consistency + tunability (added before close):**
- The PCNT comb units are **monotonic accumulators** (`flags.accum_count` + a high-limit watch point), read as software deltas (no clear → also removes the old read/clear lost-edge bias). A new `tube_pcnt_filtered_total()` lets `history.c`'s 60 s sampler read the *filtered* monotonic total, so the **rolling 5-/15-min averages (GMC ACPM / ThingSpeak f3/f4) are now filtered too** when the filter is on — consistent with the per-cycle CPM. `history_tick()` takes the live `pcnt_filter && tube_pcnt_active()` decision; a source switch re-primes one sample.
- **Configurable filter width** via `pcnt_filter_width_ns` (default 4000, bounded 250–12000 ns) — sets the widest comb tooth = the filter source; the diagnostic teeth stay 0/250 ns/1 µs. The optimum is temperature-sensitive (the Heltec's narrow-pulse fraction climbs as it warms), so it's tunable per node from the syslog `PCNT:`/`FILTER:` lines. Reboot-required.
- **`/status` indicator** — when the filter is active the Radiation card shows "PCNT width filter: ON @N ns — raw CPM X → filtered Y", so the effect is visible without the syslog.
- OTA teardown calls `tube_pcnt_stop()` to release the comb's DRAM (+watch ISRs) for the OTA window. Two independent code reviews (no Critical/High); off/default path verified byte-for-byte unchanged.

## V2.5.15 — DNMS noise: fix sensor.community field naming (HTTP 400)

### Bug: every DNMS noise POST to sensor.community 400'd
- **Symptom:** with a DNMS connected (first deployed on dust node `esp32-5965048`
  2026-06-07), every cycle logged `sensor.community: ... noise rc=400` while BME (PIN 11)
  and PM (PIN 1) on the same node returned 201. Madavi / openSenseMap / aqi.eco all
  accepted the noise data fine.
- **Root cause:** the PIN-15 body sent **`DNMS_`-prefixed** value names
  (`DNMS_noise_LAeq` / `DNMS_noise_LA_min` / `DNMS_noise_LA_max`). sensor.community
  expects the **unprefixed** `noise_LAeq` / `noise_LA_min` / `noise_LA_max` on PIN 15 and
  rejects the prefixed form: `{"value_type":["\"DNMS_noise_LAeq\" is not a valid choice."]}`.
  This is the **same prefix-strip rule as SPS30 on PIN 1** — upstream airrohr builds the
  JSON with `DNMS_`-prefixed names (for Madavi) then *strips* the prefix before the SC POST
  (the `"DNMS_"` argument to `sendSensorCommunity()` is the replace string, not proof the
  prefix is kept). Verified live against `api.sensor.community`: prefixed → HTTP 400,
  unprefixed → HTTP 201.
- **Fix:** `build_sensorc_noise_body()` in `transmission.c` now emits the unprefixed
  `noise_*` names. Madavi / openSenseMap / aqi.eco builders are unchanged (they correctly
  keep `DNMS_noise_*`). No hardware/config change — the SC node was already registered with
  DNMS on pin 15; only the field naming was wrong.

---

## V2.5.14 — heap-guard auto-reboot (opt-in fragmentation safety-net)

### New config knob `heap_guard_floor_kb` (default 0 = off)
- **Problem:** INTERNAL-DRAM **fragmentation** (total free heap stays flat, but the
  *largest contiguous block* slowly shrinks across days of TLS-handshake churn) is the
  OTA-stall precursor and a long-tail OOM risk on month+ uptimes. The existing 24h PSA
  crypto refresh (`periodic.c`) *slows* it but doesn't fully prevent it — the dust node
  `esp32-5965048` soaked `INTERNAL largest` from **71.7 K → 62.5 K over 4.8 days** despite
  the refresh, on a clear downward (not plateauing) trend.
- **Fix:** new `periodic_heap_guard()` in `periodic.c`. When `heap_guard_floor_kb > 0`, it
  reboots the node (via the clean `main_request_restart()` path — persists state, flushes
  applog, `esp_restart` from the main loop) once `heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)`
  stays below the floor. A reboot is the only certain reset of fragmentation.
- **Boot-loop hardened** (a loop on an unreachable field node is worse than fragmentation):
  - **2h arm-delay** — never acts in the first 2h of uptime (a board that boots
    already-fragmented, or with a mis-set floor, can't reboot itself in a loop).
  - **6h rate-limit** — at most one guard-reboot per 6h.
  - **TX-idle sampling** — only samples `largest` when `tx_is_idle()`, so transient
    mid-handshake dips are filtered (no separate de-glitch timer).
  - **3-sample debounce** — needs 3 consecutive idle sub-floor reads before acting.
- **Opt-in & live:** default `0` = disabled (no behaviour change for the fleet). Set per
  node on `/config` ("Heap-guard auto-reboot floor (KB)"); read live each tick by
  `periodic_loop()`, so a plain **Save** applies it **without a reboot**. `~44` KB is a
  sane starting floor — validate against an actual OTA's contiguous-alloc need first. The
  trigger logs a loud `HEAP GUARD: …` WARN to `/log` + syslog before rebooting.
- Plumbing: one schema line in `config_fields.def` (X-macro auto-generates struct + NVS
  load/save + POST parse); `periodic_loop()` gains a `heap_guard_floor_kb` arg (passed
  from `g_cfg` in `main.c`); boot-dump line in `config.c`; form field in `http_server.c`.

## V2.5.13 — /status uploads fix + Heltec MQTT boot‑defer

### `/status` Uploads block: per-cell inline styles → CSS classes (`http_server.c`)
- **Bug:** the shared `/status` scratch buffer (`buf[1600]`) overflowed on nodes with
  ~5+ enabled upload targets — the Uploads table plus the FTPS line exceeded 1600 B, so
  `append_safe()` silently truncated the **FTPS line** (rendered last) mid-string and
  dropped its closing `</div>`, leaving the MQTT block mis-nested. Confirmed live on the
  dust node (5 targets): the line died at "…next i".
- **Fix is RAM-neutral** (deliberate — the Heltec V2 is heap-tight; the FTPS/TLS
  handshake already dips min-free to ~2 KB, which is why MQTT is torn down before FTPS).
  Rather than enlarging the buffer (which would add BSS), the repeated
  `align=right style="padding-left:10px"` and `style='color:#…'` on every table cell are
  factored into CSS classes (`.u/.ar/.g/.r/.o/.d`) added to the `STATUS_HEAD` `<style>`,
  which is `static const` → flash `.rodata`, not RAM. Each upload row shrinks ~210→~110 B,
  so the block now fits the existing 1600 B buffer even at the 8-target worst case, and
  every `/status` render is ~600 B lighter. No `buf`, BSS, stack, or heap change.

### Heltec only: defer MQTT start until after the first TX round (`main.c`)
- **Why:** on the no-PSRAM Heltec, starting MQTT eagerly on GOT-IP overlapped the MQTT
  TLS handshake + 16-entity HA-discovery burst with the first TX cycle's HTTPS upload
  handshakes — 2–3 concurrent TLS contexts at boot drove the DMA-capable heap to
  `min_free=280 B` and triggered an MQTT connect→disconnect→reconnect churn (observed on
  esp32-12276328). Steady state was fine; the failure window was boot.
- **Fix:** gated `#ifdef BOARD_HELTEC_V2`, the MQTT-start poll requires
  `tx_cycles >= 1 && tx_is_idle()`. `tx_cycles >= 1` gates past boot; `tx_is_idle()` (the
  existing accessor: TX worker not busy + queue empty) ensures the CPU1 upload worker
  isn't mid-cycle — so MQTT starts in the ~168 s gap *between* cycle #1's uploads and
  cycle #2, with no concurrent upload TLS, instead of overlapping them. (A first cut with
  `tx_cycles >= 1` alone still collided: `tx_cycles` flips at the cycle *log* line, but
  the HTTPS uploads run async on the worker *after* `do_tx_cycle`'s non-blocking
  `tx_transmit` — `tx_is_idle()` is what actually tracks completion. Confirmed on
  esp32-12276328: MQTT started 1 s into cycle #1, mid-Madavi-handshake.) Race-free since
  `do_tx_cycle` bumps the counter and enqueues the worker in one pass on the same task as
  the gate. Also stops the FTPS/OTA/PSA re-inits (same poll) colliding with an in-flight
  upload. PSRAM boards unchanged (both terms inside the `#ifdef`; cppcheck byte-identical).
  Boot-only — steady-state heap (the tight margin that can OOM the big `/config` render on
  a Heltec+MQTT node) is unchanged.

---

## V2.5.12 — GNSS per-cycle log + MAX-M10S identity + raw-edge count profiler

### Per-cycle GNSS log line (`main.c`)
- Each TX cycle now emits a `v2_main:` GNSS line (after the SHT45/BMP env line), gated on `gnss_present()` so boards without a receiver log nothing. Two shapes mirror the `/status` card: a full fix (`<chip>: Fix: 3D, N satellites, HDOP x.x Position: lat, lon Altitude: a m MSL UTC: <iso8601>`) or `Fix: acquiring (N satellites visible)` before RMC goes valid. Reads only the cached snapshot via `gnss_get_fix()` — no extra I²C, the bus is still touched only by `gnss_poll()` on the main task.

### MAX-M10S unique chip ID + firmware via UBX (`gnss.c`, `gnss.h`, `http_server.c`)
- Added a one-shot UBX poll at `gnss_init()`, **u-blox-only** (gated on addr 0x42): `UBX-MON-VER` (logs `sw`/`hw` version strings) and `UBX-SEC-UNIQID` (factory unique chip ID → hex, the analogue of the SHT45/SPS30 serial). Logged once at boot (`MAX-M10S serial = 0x…`) and surfaced as a `Serial:` line on the `/status` GNSS card. The PA1010D has no per-unit serial (plain-NMEA part) and skips this entirely — its log/card are unchanged.
- Implementation: `ublox_send_poll()` builds the `B5 62 | class id | len | ckA ckB` frame with an 8-bit Fletcher checksum and prepends `0xFF` (the DDC stream register — correct whether or not the first written byte is treated as a register address); `ublox_poll()` runs a UBX framing state machine over the live stream (via the existing `ublox_read_chunk`), ignoring interleaved NMEA, with a per-message timeout. Best-effort — a missing reply logs a warning, position/fix work regardless.
- ⚠️ **Unvalidated on hardware** — the SparkFun MAX-M10S board is on backorder (~2026-06-16). This is the first code in the driver to *write* to the u-blox DDC; the write semantics and response-scan timing need bench confirmation before this ships. cppcheck 2.20.0 clean; builds for `adafruit_qtpy_esp32_pico` (verified embedded V2.5.12).

### Per-cycle raw-edge profiler on the Geiger count path (`tube.c`, `tube.h`, `main.c`)
- New `DIAG:` log line each TX cycle, gated on `tube_enabled` (PM-only nodes log nothing): `raw_edges` (every count-ISR edge, tallied *before* the 190 µs `GMC_DEAD_TIME_US` gate) + `rejected` (= raw_edges − counts; edges suppressed as ringing/double-counts) + an 8-bin edge-to-edge spacing histogram (`<50/<190/<500/<1k/<5k/<50k/<500k/≥500k µs`). Distinguishes a clean Poisson count path from count-node ringing/noise at a glance.
- `tube_get_diag()` snapshots+resets the profiler under `mux_gmc` (mirrors `tube_read()`); the count ISR adds one volatile increment + a few comparisons per edge (negligible at ~1.2 cps, IRAM-safe) and returns zeros on tube-disabled nodes (count ISR never installed). No heap, no network.
- Used 2026-06-02 to confirm the V1.4 (Heltec) vs Rev B (FeatherS3-D) ~14.7% CPM gap is a *genuine* detection difference — both boards count clean Poisson at their own rates — not a counting artifact. The HV-setpoint-vs-MCU-input-threshold sub-cause remains open (pending an MCU-swap / HV measurement).

---

## V2.5.11

**GNSS is now display-only — it never sets the system clock. NTP is the sole time source.**

### Why
Field logs (esp32-5965048) showed the GNSS clock-discipline re-firing every few seconds with ~2 s drift, repeatedly stepping the clock **backward** (non-monotonic — log timestamps jumped, lines went out of order), plus periodic ~26 s spikes where SNTP clawed the clock back. Root cause: GNSS-over-1 Hz-NMEA-over-I²C is inherently ~1–2 s laggy (no PPS pin is broken out on the Qwiic/STEMMA wiring), so comparing the parsed fix-time to "now" and hard-setting whenever they differ by ≥2 s is a latency-chasing oscillation — and SNTP, the other clock owner, fought it. On these **networked** nodes LAN NTP is ~100× more accurate than the GNSS path, and an **offline** node uploads nothing anyway, so GPS-as-time-fallback added zero value while churning `settimeofday()`.

### Change (`gnss.c`, `gnss.h`, `http_server.c`)
- Removed `discipline_clock()`, `gnss_time_is_source()`, the `s_last_clock_set_ms` tracker, and the `DISCIPLINE_DRIFT_S` / `TIME_SOURCE_TTL_MS` constants. GNSS **never calls `settimeofday()`** now.
- `gnss_poll()` parses RMC/GGA only to populate the cached fix snapshot for `/status`. `parse_datetime` is retained (still digit/range/`GNSS_MIN_YEAR`-validated) but now only renders the receiver's UTC on the page — never the clock.
- `/status` GNSS card: dropped the "System time source" line (GNSS no longer affects the clock); still shows chip / fix / sats / HDOP / lat-lon + OSM map / altitude / UTC.
- Dropped the now-unused `<sys/time.h>` and `ntp.h` includes.

Deleting the loop (rather than rate-limiting/deadbanding it) removes the churn and the non-monotonic-clock hazard by construction — no tuning constants, no NTP-vs-GNSS coordination. cppcheck 2.20.0 clean.

---

## V2.5.10

**GNSS made actually work on hardware (parser fix), auto-detect (no config toggle), and an MQTT/FTP timing fix. All bench-validated on a real PA1010D + VEML7700.**

### GNSS NMEA parser fix (`gnss.c`) — the one that matters
- `parse_sentence` matched the sentence type at `fields[0] + 2`, but the accumulator keeps the leading `$`, so `fields[0]` is `"$GNRMC"` — the offset landed on `"NRMC"` and **every RMC/GGA sentence was silently dropped**. Detection worked (chip bound), but no fix ever parsed: `/status` sat at "acquiring… 0 satellites / time source NTP" even with a valid 12-satellite fix on the wire. Fixed to `fields[0] + 3` (skip `$` + 2-char talker), guard widened to `< 6`. Present (latent) since the driver shipped in V2.5.8.

### GNSS auto-detect — config toggle removed
- `gnss_init()` now auto-detects: probes u-blox **0x42** (bound on ACK — unambiguous) then PA1010D **0x10**, where it **sniffs for a checksum-valid `$…*HH` NMEA sentence** (up to ~1.5 s) before binding. A VEML7700 at the shared 0x10 fails the sniff (it returns `00 00 FF…`, which can't pass an NMEA checksum) and is left for the ambient-light probe. `main.c` probes GNSS unconditionally, falling through to the VEML7700 probe only if none is found.
- The `gnss_enable` config field, its `/config` checkbox, and form arg are **removed** — both supported modules now "just work" when plugged in, no setting required. The orphaned `gnss_en` NVS key on existing devices is ignored. (Bench-validated: PA1010D auto-detected + GPS fix on the dust node; VEML7700 correctly falls through on a QT Py.)

### MQTT/FTP timing — stop a redundant MQTT thrash (`main.c`, `log_ftp.c/.h`)
- When the 24h PSA-crypto refresh (stops MQTT) and a scheduled FTPS upload (also stops MQTT) landed on **adjacent service-loop ticks**, the main loop's MQTT-restart fired in the gap — bringing MQTT up (full TLS connect + HA-discovery publish) only for the FTPS prep to tear it down ~180 ms later, then a third bring-up after the upload. New `log_ftp_imminent()` lets the MQTT (re)start gate skip when an FTPS upload is due, so MQTT comes back up exactly once, after the upload. Cosmetic on PSRAM boards; removes a transient TLS heap spike on tight-heap Heltec. (Diagnosed from a PSA+FTP schedule alignment on esp32-5963724.)

> Reviewed with cppcheck 2.20.0 (clean) + an independent code/security audit (no Critical/High; device-handle lifecycle, snprintf arg-balance, and boot-blocking all verified clean).

---

## V2.5.9

**Post-V2.5.8 hardening from a focused security/memory code audit of the GNSS driver. No new features.**

### GNSS clock-discipline input validation (`gnss.c`)
- **The fix that matters:** `parse_datetime` now strictly validates the NMEA date/time before it can reach `settimeofday()`. New `two_digits()` helper rejects non-digit characters (plain `atoi` silently accepts garbage like `"1x"→1`), every field is range-checked (hh≤23, mm≤59, ss≤60, mon 1-12, day 1-31), and a `GNSS_MIN_YEAR` (2020) plausibility floor is enforced. **Why:** an NMEA checksum only proves wire integrity, not sanity — RF multipath or a glitching module can present a self-consistent garbage date. Since GNSS is the GPS-primary time source, an unchecked bad date could yank the wall clock outside TLS cert-validity windows and break every HTTPS/MQTT upload until a good fix re-disciplined it. A bad fix is now a no-op, not a clock-corrupting event.

### Concurrency tightening (`gnss.c`)
- `parse_rmc` now does all parsing (`nmea_to_deg`, `parse_datetime`/`timegm`) **before** taking the `portMUX` spinlock — only the struct stores run inside the critical section (interrupts are disabled on-core under a portMUX).
- The 64-bit `s_last_clock_set_ms` is now written (in `discipline_clock`) and read (in `gnss_time_is_source`, HTTP task) under the fix mux, closing a benign torn-read that could mislabel the `/status` time source for one tick on a 32-bit core.

### cppcheck `variableScope` nits (pre-existing, unrelated to GNSS)
- `display.c::format_time` — `days` moved into the branch that uses it.
- `log_ftp.c` — the FTP interval computation moved into the `is_scheduled` branch. Pre-emptive: a stricter cppcheck (2.20.0 locally) flags these `style` findings; squashed so a future CI cppcheck bump can't trip the gate.

> Reviewed with cppcheck 2.20.0 (now available locally) — full tree clean. GNSS driver remains clean-room / not yet bench-validated against real hardware.

---

## V2.5.8

**GNSS receiver — GPS-primary time source + position on `/status`.**

### New: `gnss.c/h` driver (all boards, opt-in)
- One driver auto-detects either I²C breakout and binds the matching transport: **u-blox MAX-M10S** (SparkFun Qwiic, addr `0x42`, DDC byte-count at `0xFD/0xFE` → stream at `0xFF`) or **CDTop PA1010D** (Adafruit 4415, addr `0x10`, plain NMEA stream).
- Shared `$`-anchored, **XOR-checksum-validated** NMEA accumulator parses `RMC` (time/date/position/validity) + `GGA` (satellites/HDOP/altitude). Talker-ID-agnostic, so it handles both `GP*` (GPS-only) and `GN*` (multi-constellation) sentences; interleaved UBX binary on the MAX-M10S is harmlessly discarded by the framing.
- Drained at ~1 Hz on the main service tick (`gnss_poll()`); `/status` reads a `portMUX`-guarded cached snapshot, so the device is touched from exactly one task.

### Time discipline — GPS-primary, NTP fallback
- On each valid fix the system clock is set from GNSS UTC via `settimeofday()`, **rate-limited to drift ≥ 2 s** to avoid sub-second jitter. SNTP keeps running untouched: NTP naturally owns the clock first (Wi-Fi/DHCP beats GPS cold-start lock), then GPS takes over once it has a fix. If GPS loses the fix for >5 min, the indicator reverts to NTP.

### `/status` "GNSS / Position" card
- Shows detected chip + address, fix type / satellites / HDOP, lat/lon with an OpenStreetMap link, altitude (MSL), UTC, and the **active system time source (GPS / NTP / none)**.

### Config
- New opt-in toggle `gnss_enable` (`gnss_en`). The PA1010D shares `0x10` with the VEML7700 ambient-light sensor and the two are never used together, so enabling GNSS takes the address and the light-sensor probe is skipped. Reboot-required.

### Scope (deferred)
- No GPS→`station_altitude_m` auto-fill (GPS MSL is geoid-noisy; that field stays LiDAR-sourced). No GNSS position in upload payloads this pass — `/status` + time only.

> Clean-room from the PA1010D / u-blox datasheets — pending bench validation against real hardware.

---

## V2.5.7

**UI polish — `/status` graph + `/config` layout. No behaviour change.**

### `/status` CPM graph
- **Faint horizontal gridlines at every 1 CPM** so you can read where the trace sits, with emphasised + labelled lines at round steps (the step auto-adapts to the range: every 1 / 5 / 10 CPM). Minor 1-CPM lines are skipped when the range is very wide (>45) so a radiation event can't emit hundreds of `<line>`s.
- **Reading times on the x-axis** (`HH:MM` ticks, newest = now, evenly spaced) with faint vertical guides; y-axis CPM labels on the left. Chart height 120→140 to fit the time row. Still hand-drawn inline SVG (no JS lib), drawn in the browser.

### `/config` transmission targets
- Each TX target's config fields (credentials/tokens/IDs) are now **indented in a left-bordered block under the target's enable checkbox**, so the main enable toggles (and their inline HTTPS toggles) read as a clean left-aligned column.
- **Station altitude + sea-level-pressure toggle moved under sensor.community** — its only consumer (the sea-level reduction is emitted only in the sensor.community body) — and the standalone **"BME280 (environmental)"** heading was retired. Form field `name`s are unchanged, so `config_post` parsing and the tube-disable greying of Radmon/GMCMap/ThingSpeak are unaffected.

---

## V2.5.6

**CPM history graph on `/status` + rolling 5-/15-min averages** (modelled on ESPGeiger; closes the long-standing "rolling averages for GMC/ThingSpeak" follow-up in the same change).

### New: in-RAM CPM history (`history.c/h`, all boards)
- Two-tier ring — 60 samples @ 1/min + 24 samples @ 1/hour (~250 B RAM). Sampled every 60 s from a **new monotonic pulse counter in `tube.c`** (`tube_get_total_counts()`), which the count ISR bumps alongside the existing window accumulator. The history sampler takes its own 60 s deltas off that counter, so it's **fully decoupled from the destructive `tube_read()`** — the TX cycle's per-cycle CPM path is untouched and the TX interval stays freely settable without affecting the 1-min graph resolution.
- Live only (resets on reboot — no flash persistence by design). Radiation-only: inert while the tube is disabled.

### Rolling averages (all boards)
- `cpm5`/`cpm15` = windowed means of the last 5 / 15 one-minute samples. Now feed **GMC `ACPM`** and **ThingSpeak `field3` (cpm5) / `field4` (cpm15)** (were the current per-cycle CPM placeholders). Ramp-up before the first minute sample falls back to the current CPM so uploads never send 0.

### `/status` graph (all boards)
- Hand-drawn inline SVG polyline + ~25 lines of vanilla JS (no external/CDN library — offline device). The device serves static markup + an ~84-value data array over the existing chunked response; the browser does the drawing. 60-min / 24-h toggle. ~2-4 KB flash; shown only on radiation nodes.

### MQTT/HA (rich-state boards only)
- `cpm5`/`cpm15` added to the rich-state JSON + two HA-discovery entities, gated on the existing `MQTT_RICH_STATE` (so no phantom entities on Heltec).

No new per-board flag — history, rolling averages, uploads and the graph are universal; only the MQTT fields ride `MQTT_RICH_STATE`.

---

## V2.5.5

**Pure refactor — audit finding A2: table-driven TX dispatch.** No user-visible behaviour change. `tx_run()` in `transmission.c` carried **8 copy-pasted ~28-LOC per-target dispatch blocks** (~240 LOC) that shared one skeleton — enable → gate → circuit-breaker → send → success-check → fail-streak/breaker-trip → log + OLED status. With 8 upload targets the duplication had become the place divergence creeps in (a wrong success code, a missed streak reset). Collapsed to a `static const tx_dispatch_t TX_TABLE[]` (one row per target) + a single `tx_dispatch_one()` interpreter, looped. ~240 → ~80 LOC; adding target #9 is now one table row + its `send_*()`.

### Changes

- **`TX_TABLE[]` + `tx_dispatch_one()`** encode the 6 axes that actually varied per target: enable flag, gate (`any_payload` / `tube_enabled` / `pm_valid`), send fn, success rc(s) (200 / 201 / 200‖201), OLED slot (only Madavi/sensor.community/Radmon have one), skip-reason string. Each target's payload/body builder (`send_madavi` etc.) is **untouched** — the table just holds a function pointer, so Madavi's SDS011/BME280 relabelling and every other per-target quirk are unchanged.
- **OSM + aqi.eco unified to `tx_target_t`** (were bare `send_*` / `*_use_insecure` bools). Their URLs are dynamic + HTTPS-only, so `url_*` stay NULL via the `s_target_urls` `{NULL,NULL}` rows; `tx_target_configure()` sets `use_insecure=false` (matching the prior literal). This lets the table reach every target's `enabled`/`use_https` by a uniform `offsetof`, with no special-case row.
- The 8 function-static `*_fail_streak` ints became one `fail_streak[TX_TARGET_COUNT]` array indexed by `tx_target_id_t`.

### Only output difference

The GMC "Sending" log line now reads `Sending to GMC (http)` instead of `Sending to GMC (gmcmap.com, http)` (the protocol label is derived uniformly). Cosmetic; every other log/result/breaker/skip line is byte-identical. Verified by building all 5 boards + diffing a TX cycle's logs.

---

## V2.5.4

**New `/config` polish + a second ThingSpeak channel for the dust node.**

### `/config` page

- Dropped the standalone "GMCMap + ThingSpeak" sub-heading — those rows now sit directly under **Transmission targets** with the rest.
- Added "— radiation-only" to the **Radmon** and **ThingSpeak** labels (matching GMCMap), so it's clear they carry only the Geiger payload.
- When **Enable Geiger tube** is unticked, Radmon / GMCMap / ThingSpeak are now greyed + force-unchecked (`syncTube()` JS), and `config_post` re-enforces it server-side (a hand-crafted POST can't enable a radiation-only target on a tube-less node). Credential fields stay editable so values survive a tube off→on round-trip. Unlike `ftp_ps_dis`, "tube off" genuinely clears these to off rather than just hiding them.

### ThingSpeak (Particulate Matter) — new TX target

A **second, independent ThingSpeak channel** dedicated to the SPS30 dust node, so radiation and PM live in separate channels (free-tier accounts get 4 channels × 8 fields). Modelled 1:1 on the existing ThingSpeak target — its own enable + HTTPS tickboxes and write-API-key field on `/config`.

- **Field map** (fills the 8-field channel exactly): field1=PM1.0, field2=PM2.5, field3=PM4.0, field4=PM10 (µg/m³); field5/6/7=temp/humidity/pressure; field8=typical particle size (µm).
- **Not tube-gated.** Gated on a live PM reading (`pm_valid`) instead, so the dust node uploads with the tube *off*, and a hypothetical combined node would fill both the radiation and PM channels independently.
- Full framework wiring: enum `TX_TARGET_THINGSPEAK_PM`, circuit breaker, `/status` per-target stats, and — on PSRAM/rich-state boards — MQTT JSON fields (`tspm_*`) + four HA-discovery entities. Rich-state JSON buffer 1664 → 1792 B for the 8th target. Heltec/base boards unaffected.

---

## V2.5.3

**Republish HA discovery on `/config` Save (no reboot) so a newly-enabled upload target appears in Home Assistant immediately.** Gap found 2026-05-30: a TX target toggled via plain "Save" starts uploading right away (the TX path reads `g_cfg` fresh each cycle), but MQTT's rich-state JSON gating and the HA-discovery entity-presence predicates read **cached** enable flags that were only refreshed inside `mqtt_init()` — i.e. at boot/reconnect. So the new target's `*_ok`/`*_att` entities (and JSON fields) didn't show in HA until a reboot. (`mqtt.c`'s own comment already claimed config-Save re-entered the cache refresh; that wire was missing.)

### Changes

- New **`mqtt_apply_config(cfg)`** — re-syncs the cached enable gates (HA-discovery, tube-enabled, the per-target upload flags) and republishes HA discovery if connected. Called from `config_post`'s save path, guarded by `mqtt_is_initialized()`.
- Extracted the flag-cache logic `mqtt_init()` already did into a shared `mqtt_cache_cfg_gates()` helper so `mqtt_init()` and `mqtt_apply_config()` can't drift (adding a future target now touches one place). Republish runs under the same `s_state_mux` TOCTOU guard as `mqtt_publish_state()`.

### Scope

**Enable case only.** A target just *disabled* keeps its (now-stale) HA entity until a reconnect/reboot — removing it cleanly needs a retained-empty discovery delete, deliberately out of scope here. Broker/port/TLS/prefix changes remain reboot-required (not live-tunable). No-op when MQTT isn't connected; works on Heltec too (refreshes the base gates + republishes its core entities).

---

## V2.5.2

**Grow the MQTT rich-state JSON buffer 1280 → 1664 B.** The rich-state publish buffer was sized at V2.4.26 for **5** upload targets, but has since grown: V2.4.33 added 3 internal/DMA heap fields, and V2.5.1 added **GMC + ThingSpeak (now 7 targets × 4 fields ≈ 560 B of upload stats)**. Worst case on a fully-loaded FeatherS3-D (tube + env + PM + DNMS + lux + all 7 upload targets + FTP, with uint32-max counters) is ~1.27 KB — at the old 1280 B ceiling. `APPEND()` truncates safely (no overflow/crash), but a truncated JSON fails Home Assistant's parse for that publish, briefly dropping the entity. Bumped to 1664 B to restore the original ~380 B slack. PSRAM/rich-state boards only; the Heltec base buffer (768 B, no upload stats / heap split) is unchanged. Stack-allocated; the cycle task has 4 KB+ stack.

---

## V2.5.1

**New upload targets: GMCMap (gmcmap.com) and ThingSpeak.** Two community/generic radiation outputs modelled on ESPGeiger's `GMC` and `Thingspeak` modules, wired into the existing TX-target framework (per-cycle dispatch, retry, circuit breaker, status page, MQTT/HA).

### Changes

- **GMCMap** (`TX_TARGET_GMC`) — `GET http://www.gmcmap.com/log2.asp?AID=&GID=&CPM=&ACPM=&uSV=`. HTTP-only (gmcmap has no TLS). Radiation-only (gated on `tube_enabled`, like Radmon). Success = body contains `OK` (`ERR1`/`ERR2` = bad account/counter ID). Config: enable + Account ID + Geiger Counter ID.
- **ThingSpeak** (`TX_TARGET_THINGSPEAK`) — `GET …/update?api_key=&field1=CPM&field2=µSv&field3=CPM&field4=CPM` (+ `field5/6/7=T/H/P` when a BME-class sensor is present). HTTPS supported (default on). Generic (gated on any payload). Success = response body ≠ `0`. Config: enable + HTTPS + channel write API key.
- **Current-values-only mapping** (per design decision): no 5/15-min rolling windows in this firmware, so `ACPM`/`field3`/`field4` carry the current per-cycle CPM. A rolling-average accumulator can be added later if the averaged columns are wanted.
- **Config page**: new "GMCMap + ThingSpeak" section on `/config` so both are configurable post-deploy (fields auto-flow through the `config_fields.def` X-macro → struct/NVS/POST; form rows added by hand like the other targets).
- **MQTT + HA Discovery** (PSRAM/rich-state boards only): `gmc_*` and `ts_*` per-target upload stats in the rich-state JSON + 8 HA diagnostic entities, gated on the target being enabled — same pattern as the existing per-target stats.

### Memory

The MQTT/HA additions are entirely inside `#ifdef MQTT_RICH_STATE`, which the **Heltec V2 build intentionally does not define** — so they add nothing to the Heltec image. The Heltec only carries the base feature (two GET send functions + config). On PSRAM boards the rich-state JSON grows ~2 targets × 4 fields (~120 B, well within the 1280 B buffer). See the per-board partition-free figures in the release build.

---

## V2.4.33

**Publish the internal/DMA-RAM split to MQTT so Home Assistant can graph the long-uptime net-stack drain.** Follow-on to V2.4.32: the per-cycle `diag_log_heap()` lands in `/log` → FTP, which means watching the multi-day trend meant pulling and diffing log files. The MQTT rich state already published `heap_free` / `heap_min` / `heap_max_alloc` — but those are the **PSRAM-dominated totals**, i.e. the same misleading numbers that hid the problem. This adds the gauge that matters.

### Changes

- **Three new rich-state fields** (`mqtt.c`): `heap_int_free`, `heap_int_largest`, `heap_dma_largest` (`heap_caps_*` for `MALLOC_CAP_INTERNAL` / `MALLOC_CAP_DMA`), emitted next to the existing `heap_*` totals.
- **Three HA discovery entities** (`mqtt_discovery.c`): "Heap internal free / internal largest / DMA largest", `data_size`/`B`/`measurement` (so HA keeps long-term statistics), diagnostic category — same shape as the existing heap entities.
- **Gated `MQTT_RICH_STATE` (PSRAM boards only).** On the Heltec (no PSRAM) the total heap *is* the internal heap, so the existing `heap_free`/`heap_max_alloc` already serve as the internal gauge there — the split would only duplicate them. The split is informative exactly where total ≠ internal, which is the rich-state boards.

### Why

`heap_int_largest` is the high-value one — the contiguous-block ceiling on internal/DMA RAM is what a sustained inbound TLS/OTA receive needs, and a shrinking `largest` over days is the fragmentation signature behind the OTA-upload stalls (see V2.4.32 / the long-uptime OTA-stall investigation). Graphing it in HA — with an optional alert when it drops below ~40 KB — catches a node *before* its OTA starts failing, instead of after. Diagnostic/observability only; no data-path change. Adds ~72 B to the rich-state JSON (well within the 1280 B buffer; observed publish ~676 → ~748 B).

---

## V2.4.32

**Diagnose long-uptime OTA-upload stalls + give the OTA prep enough time to wait out a TX retry storm.** Background: on 2026-05-30, esp32-5965048 (dust node, ~56 h uptime) failed every OTA — the 1.3 MB upload stalled after exactly one TCP window (`8640`/`11520` B) while all outbound HTTPS kept working and the heap looked healthy. **A reboot fixed it**, proving the cause is *accumulated runtime state in the net stack*, not config/RF/mesh. The "healthy heap" was misleading: `esp_get_free_heap_size()` is PSRAM-dominated, but WiFi + lwIP RX buffers live in **internal/DMA-capable RAM**, so a drain there is invisible in the numbers we log.

### Changes

- **Tier-1 net-stack instrumentation** — new `diag_log_heap(where)` logs the heap split by capability: `INTERNAL free/largest/min` and `DMA free/largest`. Called **per-cycle** (right after the existing `free heap before TX` line) and **at OTA-prep** (the failure moment, before the service teardown). The per-cycle line lands in `/log` → the hourly FTP upload, so we get the full multi-day time series into the next stall and can see *which* bucket drains (internal/DMA RAM, vs. needing Tier-2 `LWIP_STATS` per-pool counts). Read all three counters — stable free + falling largest/min = fragmentation, not a leak.
- **OTA-prep "wait for TX idle" 10 s → 60 s** — a sensor.community/openSenseMap retry storm is up to 4 × ~15 s ≈ 60 s. The old 10 s budget guaranteed `OTA prep: TX still busy after 10s — proceeding anyway`, so the OTA write contended with an in-flight TLS upload (observed 2026-05-30, 16:07). 60 s covers the worst-case retry run so the OTA almost always starts on a quiet radio.

### Notes

Diagnostic + robustness only — no behaviour change to the data path. Workaround for the underlying stall remains **reboot-before-OTA** until the drained pool is identified from the Tier-1 data; Tier-2 (`CONFIG_LWIP_STATS`/`MEMP_STATS` per-pool) is the planned follow-up if the capability split alone doesn't pin it.

---

## V2.4.31

**Fix: the 24h PSA crypto refresh broke the live MQTT connection.** Diagnosed 2026-05-30 from three serial logs (esp32-5963724 + esp32-5965048). Once per 24h `periodic_loop()` runs `mbedtls_psa_crypto_free()` + `psa_crypto_init()` to defragment the heap by emptying the PSA key-slot pool — but it did so **while the persistent MQTT client's TLS session was still live.** In mbedTLS 4.x / IDF 6 the MQTT session's AES-GCM record keys live in PSA slots, so freeing the pool invalidated them and the next MQTT TLS write failed.

### Root cause

The PSA-refresh safety gate (`tx_is_idle()`) only accounts for the **transient** HTTPS/FTPS handshakes on the TX worker. The **persistent** MQTT TLS connection (added V2.4.2) holds PSA slots full-time and the gate never saw it. Symptom: immediately after `psa_crypto_init: ok`, the next MQTT op failed `esp-tls-mbedtls: write error :-0x0084` — a keepalive ping (`mqtt_client: Error sending ping` → `DISCONNECTED`, on its own, before any FTP) or, on FTP nodes, the FTP-prep DISCONNECT (`errno=128`). The subsequent `mqtt_stop()` then blocked the main task ~4.5 s (vs ~18 ms when no refresh coincided). Earlier this was misattributed to a "benign already-dead idle socket" during FTP teardown — wrong: normal FTP uploads with no PSA refresh tear MQTT down cleanly; only the upload coinciding with the refresh errored.

### Changes

- **`periodic.c`** — stop the MQTT client (`mqtt_stop()`, guarded by `mqtt_is_initialized()`) **before** `mbedtls_psa_crypto_free()`, so its DISCONNECT goes out over still-valid crypto. main.c's "STA has IP + clock sane → starting MQTT client" poll re-inits it on the next tick (the same restart path FTP relies on); FTP-prep's own `mqtt_is_initialized()` guard then sees it already down and skips its stop → one clean bounce.
- **Bonus** — this also makes the defrag actually reclaim MQTT's PSA slots. Previously the live session held its slots across the refresh, so the pool never fully emptied, partly defeating the chore's purpose.

### Scope

Affects every board running MQTT (all current targets). The break failed *safe* (PSA returns an invalid-handle error, no crash / use-after-free) and MQTT auto-recovered, so no data was lost — but it flapped availability + re-ran HA discovery every 24h and blocked the main task during the dirty stop. No behaviour change on the once-daily window beyond a clean MQTT bounce instead of a dirty one.

---

## V2.4.30

**MQTT reconnect handling on WiFi recovery.** Prompted by a router-reboot trace (2026-05-29) on esp32-5965048: STA dropped at 00:16:54 and recovered at 00:18:40, but MQTT didn't reconnect until 00:19:10 — ~10 s after `GOT_IP` — and esp-mqtt fired 8 blind TCP+TLS connect attempts during the outage, each failing fast with `ENETUNREACH` but still building/tearing an esp-tls context.

### Changes

- **`mqtt_kick_reconnect()`** — new API called from main.c's `EV_GOT_IP` handler. Forces an immediate `esp_mqtt_client_reconnect()` on every (re)association instead of waiting for esp-mqtt's internal retry timer. No-op when the client is uninitialised (first boot, before `mqtt_init`) or already connected. Mux-guarded against the OTA-teardown TOCTOU like `mqtt_publish_state`/`mqtt_stop`.
- **`reconnect_timeout_ms` 10 s → 30 s** — slows the blind auto-reconnect cadence during an outage to cut esp-tls context churn (negligible on PSRAM boards; real fragmentation pressure on the Heltec V2's tight heap). Recovery latency is unaffected because the `GOT_IP` kick owns it; the slow timer only governs the broker-down-but-WiFi-up case (e.g. broker host reboot), where 30 s is fine.

### Not changed (deliberately)

Did **not** tear down the MQTT client on `STA_DISCONNECTED` — that's heavier (`esp_mqtt_client_stop` joins the task) and a WiFi flap would stop/start it repeatedly, fighting esp-mqtt's tested reconnect logic. The publish path is already double-guarded (TX cycle returns on `!wifi_up()`; `mqtt_publish_state` bails on `!s_connected`), so no publish-side wrapper was needed.

---

## V2.4.29

**Fix abort() crash: `time()` called inside `portENTER_CRITICAL`.** Diagnosed from coredump on esp32-5965048 (Oatlands prod, V2.4.24) — `tx` task panicked in `lock_acquire_generic` at `locks.c:150`.

### Root cause

Three call sites called `time(NULL)` inside a `portENTER_CRITICAL` spinlock section. Critical sections disable interrupts and make `xPortCanYield()` return false. When `time()` internally acquires `s_time_lock` via `_lock_acquire`, the IDF lock code sees "ISR context", uses `xSemaphoreTakeFromISR` instead of `xSemaphoreTake`, and if the lock is held by another task at that instant → `abort()`. A race condition: only triggers when `s_time_lock` is contended at the exact microsecond.

### Fix

Hoist `time(NULL)` above the critical section in all three sites:

- `transmission.c:record_outcome()` — **crash site** (V2.4.1+)
- `log_ftp.c:log_ftp_loop()` FTP stats update (V2.4.1+)
- `main.c:do_tx_cycle()` cycle-at timestamp (V2.4.1+)

The timestamp is captured before the spinlock, then the pre-computed value is stored inside the critical section. No semantic change — the few-microsecond difference between sampling `time()` before vs inside the lock is irrelevant for a 150s cycle.

---

## V2.4.28

**I²C sensor-read-error counter — cheapest possible observability for bus / supply health.** Surfaces in three places: CYCLE log line (lands in /log via applog), /status System block, MQTT diagnostic entity.

### What it does

Adds a single global atomic counter (`main/diag.c`) that increments once per failed top-level sensor read in `do_tx_cycle`. Failure call sites covered:

- `env_sensor_read()` — env cascade (BME280 / BMP581 / SHT45 / BMP390 / BME688)
- `pm_sensor_read()` — SPS30 over I²C
- `noise_sensor_read()` — DNMS over I²C

The counter is cumulative since boot; only way to reset is reboot (matches `reconnects` and the per-target upload counters). One increment per failed read, not per failed underlying I²C transaction — keeps the signal interpretable.

### Where it shows up

| Surface | Form |
|---|---|
| Serial / applog `/log` | `i2c_err=<N>` appended to the existing `CYCLE #...` line |
| `/status` page | "I²C errors: N since boot" in the System block |
| MQTT `state` JSON | `"i2c_err": N` always (alongside `cycles`, `reconnects`) — adds 17 B per cycle on Heltec; well under the existing buffer |
| HA Discovery | `<chip>_i2c_err` entity, `device_class: total_increasing`, `entity_category: diagnostic`, icon `mdi:bus-alert` |

### Why this and not VBUS / fuel-gauge measurement

V2.4.27 left the question "could a marginal supply be causing the +30% CPM / odd spikes?" The first guess was a MAX17048 fuel gauge driver + VBUS-present digital read, both of which got dropped after talking through the actual hardware:

- **MAX17048 fuel gauge** — measures battery cell voltage. Without a LiPo attached (current Oatlands deployment), both VCELL and SoC report ~0 / 0 %. No signal, just clutter on HA. Dropped.
- **GPIO 34 VBUS-present** — digital line, not analog (ESP32-S3 ADCs only exist on GPIO 1-20). Without a battery, the firmware can't run with USB unplugged, so this would publish a constant `1` forever. Dropped.
- **Precise 5 V / 3.3 V measurement** — neither rail has an exposed sense pin on the FeatherS3-D. Would require external resistor divider mod to a free ADC1 pin (GPIO 6 = A4 is the only candidate). Not in scope.

What IS observable without hardware mods:
- **Brownout-grade 3.3 V dips** → `reset_reason: BROWNOUT` (already in MQTT since V2.4.26)
- **Sub-brownout I²C reliability degradation** → the new counter (this release)
- **WiFi stack stress** → `reconnects` (already exposed)

A rising `i2c_err` paired with stable `reset_reason` = bus or sensor problem (cable, contamination, marginal pull-ups). A rising `i2c_err` paired with periodic `BROWNOUT` resets = supply problem.

### Files touched

- `main/diag.c` + `main/diag.h` — new module, atomic counter
- `main/main.c` — include `diag.h`, increment on three sensor read failures, snapshot counter into `main_status_t`, log on CYCLE line
- `main/main_status.h` — new `i2c_errors` field
- `main/http_server.c` — render "I²C errors: N since boot" in the System block
- `main/mqtt.c` — emit `"i2c_err": N` in the state JSON
- `main/mqtt_discovery.c` — add the diagnostic entity
- `main/CMakeLists.txt` — add `diag.c` to SRCS
- `main/version.h` — V2.4.27 → V2.4.28

### Notes

- **DNMS-equipped nodes only**: on first boot, `noise_sensor_read()` *may* return ESP_FAIL if the LAeq integration window (~150 s, started by `noise_sensor_init()`) hasn't quite finalised by the time the first TX cycle runs — a millisecond-level race. If it loses the race, the counter ticks once at the first cycle and is stable afterwards. Nodes without DNMS attached (e.g. the Oatlands SPS30+SHT45+BMP581 dust node) skip the call entirely and start the counter at 0.
- All sensor drivers in V2.4.x use the new IDF v6.0 I²C master driver (`i2c_master.h`) — no v4.x legacy paths involved.

---

## V2.4.27

**Restore the V1.x per-cycle semantic for `hv_pulses` in legacy HTTPS uploads + status page, and stop POSTing the Madavi-rejected radiation body.** Two unrelated cleanups bundled because both touch the legacy-HTTPS upload path.

### What changed (`hv_pulses` semantic)

Two related fixes, both single-line in effect:

1. **Legacy HTTPS upload body** (`transmission.c` via `main.c`): the `hv_pulses` value POSTed to sensor.community / Madavi / Radmon now carries the **per-cycle delta**, matching V1.x firmware (where `multigeiger.ino` computed `delta = current - last; last = current;` at upload time). The V2.0 rewrite uploaded the cumulative-since-boot counter raw, so the published value grew unboundedly with uptime instead of representing "pulses needed this cycle". Sensor.community / Madavi / Radmon CSV archives produced under V2.0-V2.4.26 carry the cumulative semantic and need re-interpretation; from V2.4.27 onward the field is consistent with the pre-V2 historical baseline (~18-63 per cycle for a healthy Si22G + Rev B HV section).

2. **`/status` page "HV pulses" rate display** (`http_server.c`): the "X / min" figure now computes `delta_this_cycle * 60000 / dt_ms` instead of `cumulative * 60000 / dt_ms`. The previous form divided a monotonic counter by a one-cycle duration, producing a number that grew unboundedly with uptime.

3. **CYCLE log line** now shows both: `hv_pulses=<delta> (cum=<cumulative>) ...` so the firmware's serial trace exposes both quantities at a glance.

### What did NOT change

- **MQTT `hv_pulses` field** keeps publishing the cumulative-since-boot value. HA Discovery declares it as `device_class: total_increasing` (`mqtt_discovery.c:117`), which is the correct semantic for that consumer. Long-term-statistics graphs in HA will keep working.
- **No hardware behaviour change.** The HV regulation itself, the ISR pulse counting, and the tube operating point are all unchanged. The Rev B PCB is fine; this was purely a published-field semantic mismatch.

### How it slipped in

`Git_Repository_Geiger/multigeiger/multigeiger.ino:150` (V1.x): the delta computation lived in `publish()` / `transmit()`, called once per upload cycle. The V2.0 rewrite moved the cumulative ISR counter into a `tube_read()` accessor (`tube.h:39` — "cumulative HV charge pulses since boot") and the new `main.c::do_tx_cycle()` passed the cumulative value through to `transmission.c` without ever taking a difference. The field name `hv_pulses` was preserved, the unit silently changed. Lesson saved to memory as `[[feedback_check_field_semantics_across_firmware_versions]]`.

### Diagnostic context (2026-05-24)

Detected when comparing recent sensor.community CSV downloads (`hv_pulses` reaching 40,000 per row) against `radiation.txt` baselines (which documented monthly averages of 18-63 per cycle through March 2026 from V1.x firmware). The carbon→metal-film theory for the +30% CPM step on Rev B remains correct and unrelated to this fix. The 100 CPM spike on 23/05 is statistical Poisson noise (~1.8σ on the daily mean), not a hardware event.

### What changed (Madavi: drop the radiation POST)

`send_madavi()` previously issued two POSTs per cycle: a dedicated "geiger" body with `Si22G_counts_per_minute` / `Si22G_hv_pulses` / `Si22G_counts` / `Si22G_sample_time_ms` / `signal`, then the environmental body. **Madavi's hardcoded `value_type` whitelist (`api-rrd.madavi.de/data.php`) recognises DHT / HTU21D / BME280 / BMP / BMP280 / DS18B20 / SDS011 / PMS / HPM / PPD42NS / GPS only** — every `Si22G_*` field has been silently dropped at the server for the entire lifetime of V2 (see `[[reference_madavi]]`). The "geiger" POST was therefore pure wasted bandwidth + TLS handshake overhead + RTT for zero stored data.

V2.4.27 removes the geiger body entirely. The environmental body still goes (and still carries `samples` / `min_micro` / `max_micro` / `signal` when the tube is enabled — all whitelisted by Madavi), so tube-only Heltec V2 deployments keep uploading the information Madavi can actually graph.

**Side effects:**
- ~1 POST/cycle saved across all 5 boards on the Madavi path. Per-cycle airtime + heap usage drops by roughly half on the Madavi side.
- The legacy "Madavi: geiger rc=X, env rc=Y" log line shortens to "Madavi: rc=X" (one POST = one rc).
- No HA / MQTT / sensor.community / Radmon / openSenseMap / aqi.eco behaviour change. Si22G radiation data still flows to sensor.community (X-PIN 19) and Radmon as before.

### Files touched

- `main/main.c` — track `g_last_hv_pulses_delta`, compute it in `do_tx_cycle()` via a function-static `s_last_uploaded_hv_pulses`, pass delta to `build_tx_context()`, update CYCLE log line
- `main/main_status.h` — new `last_hv_pulses_delta` field in `main_status_t`
- `main/http_server.c` — use `last_hv_pulses_delta` in the per-minute rate calc
- `main/transmission.c` — remove `build_madavi_geiger_body()` and the dedicated POST that called it; collapse `send_madavi()` to a single env-body POST
- `main/version.h` — V2.4.26 → V2.4.27

---

## V2.4.26

**Richer MQTT state — system diagnostics for every board, per-target upload counters for PSRAM boards.** One bigger JSON per existing TX cycle, no new timers, no extra publishes. HA Discovery advertises the new entities automatically; existing entities and state-topic shape are unchanged so HA setups need no manual reconfiguration.

### What changed

**All boards (Heltec V2 included)** — `geiger/<id>/state` JSON gains six system fields:

- `ip` (string, "10.11.12.198")
- `rssi` (dBm)
- `heap_free`, `heap_min`, `heap_max_alloc` (bytes)
- `reset_reason` (string: "POWER_ON" / "PANIC" / "TASK_WDT" / etc. — same enum the /status page already shows)

HA discovers all six as `entity_category: diagnostic`, so they land under the device card's Diagnostics panel instead of cluttering dashboards. RSSI gets `device_class: signal_strength`; heap fields get `device_class: data_size`.

JSON growth per cycle: ~120-130 bytes on Heltec (~120 B current → ~250 B). Negligible airtime impact.

**PSRAM boards only** (FeatherS3-D, QT Py PICO, XIAO ESP32-S3 — gated on a new `MQTT_RICH_STATE` CMake compile flag) — additional per-target upload-stats fields, only for targets actually enabled in cfg:

- `<target>_ok` / `<target>_att` / `<target>_rc` / `<target>_breaker` for each of madavi / sc / radmon / osm / aqi
- `ftp_ok` / `ftp_bytes` / `ftp_age_s` if `ftp_enabled`

These are primary entities (not diagnostic) so they show on dashboards — `*_ok` and `*_att` are HA `total_increasing` counters suitable for long-term statistics, `*_rc` and `*_breaker` are `measurement` snapshots. Same accessors the `/status` page already uses (`tx_get_stats`, `log_ftp_get_stats`) — no new counters, no new state, just exposed through MQTT.

### Why gated on a build flag

JSON growth on a fully-loaded FeatherS3-D (PM + DNMS + light + 5 upload targets + FTPS) lands around 940 bytes, vs ~615 bytes if Heltec got the upload stats too. The Heltec V2 has historically been heap-tight (see V2.4.13 and V2.4.22 stack-pressure audits), so the per-cycle stack buffer was bumped from 512 → 768 bytes on Heltec and 512 → 1280 bytes on PSRAM boards. Splitting it this way costs Heltec only what's most valuable for in-the-field debugging (RSSI, heap, reset reason) while letting PSRAM boards carry the full operational picture.

### Files touched

- `CMakeLists.txt` — `MQTT_RICH_STATE=1` compile def added to the three PSRAM-board branches
- `main/sysinfo.h` — new header, holds `reset_reason_str()` (extracted from http_server.c so mqtt.c can reuse the same enum→string map)
- `main/http_server.c` — drops its local copy of `reset_reason_str()`, includes `sysinfo.h`
- `main/mqtt.c` — new system-stats block (always), new upload-stats block (`#ifdef MQTT_RICH_STATE`), bumped JSON buffer
- `main/mqtt_discovery.c` — six new diagnostic entities + 23 new primary entities under the ifdef, new `entity_cat` field on `ha_entity_t`, new `mqtt_discovery_set_upload_flags()` setter
- `main/version.h` — V2.4.25 → V2.4.26

### Notes

- **HA migration: nothing to do.** Existing entities keep their `uniq_id` and JSON keys; new entities are added automatically when HA receives the new discovery payloads on next sensor reconnect.
- **Heltec /status page unchanged** — the existing Uploads block already shows the same per-target stats in HTML; this release just makes them visible via MQTT on the boards that have heap budget for the JSON growth.
- **Field naming**: short keys (`madavi_ok` not `madavi.ok`) keep HA `value_template` expressions trivial (`{{ value_json.madavi_ok }}`) and avoid HA's nested-object handling, matching the rest of the JSON.

---

## V2.4.25

**Shared-PCB pin map for QT Py PICO + XIAO ESP32-S3, plus the v6.0 kconfig drift fixes from earlier in the day.** Three things in one tag — pure config / pin-map changes, no behavioural code touched.

### What changed

1. **New `seeed_xiao_esp32s3` board target** (5th build target). ESP32-S3 LX7, 8 MB flash, 8 MB OPI PSRAM, native USB-C. CI matrix in both `build.yml` and `release.yml` now covers it; release artefacts ship for all 5 boards.

2. **QT Py PICO and XIAO ESP32-S3 now share a single Geiger pin map**: A0 / A1 / SCK (HV_CAP_FULL / GMC_COUNT / HV_FET). A0+A1 are the only two pads strap-free on both boards; SCK is the third strap-free pad and lives on the opposite long edge of the board, which physically separates the switching HV_FET output from the sensitive GMC pulse pickup. One PCB design can host either board.

   - PICO: HV_FET moved from A2 (GPIO 27) → SCK (GPIO 14). A2 and A3 now free for future analog use.
   - XIAO: Geiger pins moved from D0/D1/D2 placeholders → D0/D1/D8 (= A0/A1/SCK). D2 was unsafe anyway (GPIO 3 is the USB-Serial-JTAG selector strap on S3).

3. **IDF v6.0 kconfig drift cleanup**:
   - `CONFIG_SPIRAM_MODE_OCTAL` (v5.x name) → `CONFIG_SPIRAM_MODE_OCT` (v6.0 name) in the XIAO overlay. The old name was silently ignored, so the XIAO was actually falling back to QUAD mode — would have crashed at bootloader-level PSRAM init on real hardware.
   - Removed `CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF/_BIN` and `CONFIG_ESP_COREDUMP_CHECKSUM_CRC32/_SHA256` from the base `sdkconfig.defaults`. Honest history: these were added in **V2.4.18** (the coredump feature commit), not pre-v5→v6 leftovers — I copy-pasted them from v5.x reference docs without realising v6.0 had removed them. In v6.0 the four symbols have no `config` declaration anywhere in IDF (verified against `components/espcoredump/Kconfig`); they're only referenced via `select`/`if` inside `ESP_COREDUMP_ENABLE`, which unconditionally selects ELF + SHA256. The `core_dump_sha.c` source file is the only checksum implementation present in v6.0 — `core_dump_crc.c` and `core_dump_bin.c` were removed by Espressif. So V2.4.18–V2.4.24 were **always running on SHA256+ELF** despite the sdkconfig saying CRC32; the four lines have been generating "unknown kconfig symbol" warnings on every build for 7 releases. Pure noise cleanup, no functional change in V2.4.25.

### Migration note for QT Py PICO users with existing hardware

If you have a QT Py PICO with a Geiger PCB wired for the old A0/A1/A2 pin map (HV_FET on A2 = GPIO 27): **do NOT OTA to V2.4.25** until you've rewired HV_FET from A2 to SCK. The firmware will drive GPIO 14 (SCK) for HV PWM and ignore GPIO 27 — your tube won't bias. There is no production QT Py + Geiger deployment in the field (the PICO target was always experimental), so this is purely a heads-up for any bench builds. The FeatherS3-D and Heltec V2 pin maps are unchanged.

### Files touched

- `main/hal.h` — both BOARD_ADAFRUIT_QTPY_ESP32_PICO and BOARD_SEEED_XIAO_ESP32S3 blocks (pin map + comment rewrite)
- `main/version.h` — V2.4.24 → V2.4.25
- `README.md` — boards table entry for XIAO updated to reflect new shared-PCB story
- `sdkconfig.defaults.seeed_xiao_esp32s3` — MODE_OCTAL → MODE_OCT
- `sdkconfig.defaults` — removed 4 deprecated COREDUMP symbols

---

## V2.4.24

**OTA receives a clear WiFi link.** Scheduled TX cycles (Madavi / sensor.community / Radmon HTTPS POSTs every 2 minutes) now skip while an OTA upload is in progress. The OTA gets the full WiFi airtime to itself instead of competing with three TLS handshakes per cycle. Non-sticky — TX resumes immediately on the next main-loop tick after OTA completes or aborts.

### Motivation

2026-05-22 log review of a failed V2.4.22 → V2.4.23 OTA on a FeatherS3-D revealed two scheduled TX cycles (CYCLE #220, #221) firing during the OTA's recv-retry window. The actual root cause of that particular failure was client-side WiFi flakiness (browser stalled after sending 1 MSS), not airtime competition — but the TX cycles eating ~10 s of airtime + 3 TLS handshakes every 2 minutes during a sensitive upload was a real latent issue. Strict improvement on every OTA on every board, particularly on marginal WiFi.

The V2.4.13 teardown already drains the in-flight TX worker before starting the OTA recv loop, but does nothing about NEW cycles that fire during the upload. V2.4.24 closes that gap.

### What changed

New non-sticky flag pair in `main.c`:

```c
void main_ota_begin(void);    // set
void main_ota_end(void);      // clear
bool main_ota_in_progress(void);
```

(Declared in `main_status.h` alongside the existing `main_services_suspended` pattern. Deliberately a separate flag — `main_services_suspended` is sticky-until-reboot because MQTT/syslog re-init mid-OTA would defeat the heap-teardown intent; TX cycle skipping wants the opposite semantics, resuming immediately after a failed OTA so the sensor's primary purpose isn't tank by a single bad upload.)

Wired in:

- `http_server.c::update_post` split into a thin wrapper + `update_post_inner`. The wrapper does auth + CSRF + `main_ota_begin()` + calls inner + `main_ota_end()` + returns. This avoids touching every one of update_post's ~17 internal return paths.
- `main.c` main loop's TX-cycle scheduler now wraps `do_tx_cycle()` in `if (!main_ota_in_progress())`. Deliberately does NOT advance `next_tx` when skipped — the deferred cycle fires immediately on resume rather than waiting another `tx_interval`.

### Why this was discussed but not bundled

- **Surface OTA progress on `/status`** — considered, REJECTED. `esp_http_server` is single-threaded, so a concurrent GET / from a monitoring client would sit in the accept queue unread until OTA completes. Worse: if we worked around the threading constraint, the polling itself would consume the airtime the OTA needs. Browser-side `<progress>` element (already wired in `/update` page) is the right monitoring story.
- **Hard total OTA timeout** — defensive but adds another knob. Current per-recv 5×30s timeout already bounds individual stalls. Leaving the wall-clock total open lets a genuinely slow upload over a weak link still succeed if it makes steady progress.

### Files touched

- `main/main.c` (+22 LOC): flag + 3 trivial accessors + 5-LOC gate in TX scheduler with explanatory comment
- `main/main_status.h` (+27 LOC): 3 function decls with WHY doc
- `main/http_server.c` (+13 LOC): wrapper + forward decl + rename of body to update_post_inner
- `main/version.h`: V2.4.23 → V2.4.24
- `CHANGELOG.md`: this entry

No partition change. No `sdkconfig` change. Drop-in OTA upgrade across all 4 board targets.

---

## V2.4.23

**Audit follow-up release — closes the remaining items flagged in the 2026-05-21 codebase audit.** Five small changes bundled. No partition change. Drop-in OTA from V2.4.22.

### 1. `/update` page shows current firmware version (committed 2026-05-21 ahead of tag)

Adds `**Current firmware:** V2.4.23` under the heading on the OTA upload page so operators can verify before uploading that they aren't OTA-ing the same version onto itself. Compile-time string-literal concatenation with `VERSION_STR` — zero runtime cost, baked into the static page literal.

### 2. MQTT publish-state TOCTOU race fix (`mqtt.c`)

`mqtt_publish_state()` previously did `if (!s_client || !s_connected) return; ... esp_mqtt_client_publish(s_client, ...)`. If `mqtt_stop()` ran on the httpd task (OTA teardown) between the check and the publish, the handle could be destroyed mid-call (use-after-free of the IDF mqtt-client struct). Window was microseconds and never observed in production, but not strictly synchronised.

Fix: new `s_state_mux` FreeRTOS mutex serialises `mqtt_init` / `mqtt_stop` / `mqtt_publish_state`. The mutex is created lazily on first `mqtt_init` and held across the check-and-publish in publish_state, around the destroy in stop, and around the handle assignment in init. `esp_mqtt_client_publish` is a non-blocking enqueue per IDF docs (returns after putting the message on its internal queue), so holding a mutex across it is safe — no risk of starving the IDF event task. Falls back gracefully to pre-fix behaviour if mutex creation fails at boot (logs a warning, doesn't crash).

`mqtt_is_initialized` / `mqtt_is_connected` / `mqtt_publish_count` deliberately stay lock-free — they're single-word reads, torn-tolerant on 32-bit Xtensa.

### 3. Document the single-httpd-task assumption (`http_server.c`)

10-line comment block above the `httpd_config_t` setup spelling out the invariant V2.4.20 + V2.4.22's static buffers depend on: ONE httpd thread processing all URI handlers serially via `select()`. The comment names the affected functions (`status_get`, `config_get`, `format_system`, `applog_vprintf`) and the recovery path (revert the V2.4.22 statics to mutex-guarded shared or per-handler heap allocs) if anyone ever bumps `max_open_sockets` past 1 AND a future IDF release introduces per-connection worker threads. Pure documentation — no runtime change.

### 4. Reconnect counters → `uint64_t` (`main.c`)

`n_attempts`, `n_connects`, `n_got_ip`, `n_disconnects` were `uint32_t` which wraps at 4.3 B. At one reconnect per minute (worst-case marginal-WiFi node) wrap is ~34.5 days; at realistic rates (1/day) it's millions of years. Either way, wrap was harmless (display rolls to 0). Switched to `uint64_t` to eliminate the conceptual wrap entirely — 584 million years of headroom, zero perf cost on 32-bit Xtensa (just two-word stores instead of one). `main_status_t.reconnects` stays `uint32_t` with a truncating cast at assignment time (the status-page consumer doesn't care about post-2^32 precision). `<inttypes.h>` added; four `%lu` + `(unsigned long)` cast sites converted to `%" PRIu64 "`.

### 5. Documented the `log_ftp_note_psa_refreshed()` cross-module coupling pattern (memory only — `reference_periodic_module.md`)

V2.4.19's tiny export pattern works for one caller (periodic.c → log_ftp.c) but does not scale past one. The memory note records the scaling limit (refactor at N=3 callers, lift the OOM counter into a shared `psa_health.{c,h}` module) and the deliberate 2026-05-22 decision to keep the V2.4.19 form for now since there's only one consumer. No code change this release.

### Files touched

- `main/mqtt.c` (+52 LOC: mutex declaration with comment, three take/give pairs across init/stop/publish_state)
- `main/main.c` (+/- ~17 LOC: counter type change, inttypes include, four format-specifier conversions, truncating cast)
- `main/http_server.c` (+10 LOC: single-task-assumption comment; +5 LOC from 2026-05-21: /update version display)
- `main/version.h`: V2.4.22 → V2.4.23
- `CHANGELOG.md`: this entry

No partition change. No `sdkconfig` change. Drop-in OTA upgrade across all 4 board targets.

---

## V2.4.22

**Codebase-wide audit of the V2.4.20 stack-overflow class of bug — moves ~7.4 KB of httpd-task stack buffers to BSS.** Same proven pattern (V2.4.16 syslog → V2.4.20 applog → this), expanded to every large stack allocation on the 8 KB httpd task's path.

### Motivation

V2.4.20's hotfix freed 1 KB from `applog_vprintf` after a httpd stack overflow corrupted the task TCB and triggered a FreeRTOS assertion. That fix kept the immediate symptom (V2.4.19 `/config` from an unauth client) from recurring, but the underlying fragility — large stack allocations on a single 8 KB task — remained latent in other code paths. A codebase audit identified ~7.4 KB of stack buffers that should have been BSS from the start.

### What changed

All in `main/http_server.c`. All static-ified — `esp_http_server` runs one task processing URI handlers serially via `select()`, so re-entry is structurally impossible and the static buffers are race-free.

#### `config_get` — ~4.3 KB freed

The ~25 HTML-escape buffers used to render the `/config` form (e_ssid, e_pw, e_chip, e_ru, e_rp, e_ntp1/2/3, e_ap, e_tz, e_apn, e_host, e_fhost, e_fuser, e_fpw, e_fpath, e_osm, e_osm_tok, e_aqi, e_mhost, e_muser, e_mpw, e_mpfx, e_slh) plus `br_opts[512]`. All simultaneously in scope before `config_get` finishes the form composition. Total ~4.3 KB consumed over half the httpd task's 8 KB budget every authenticated `/config` render. Worked today because V2.4.20 had just freed up 1 KB, but the headroom was thin — any new caller on the path would have re-tripped the same overflow.

#### `/` status handler — 1.6 KB freed

`buf[1600]` shared chunked-render scratch (`http_server.c:951`). Single buffer reused across all status blocks via `httpd_resp_send_chunk` between block fills. Single function, single thread.

#### `format_system` — 0.96 KB freed

`cd_summary[320]` + `cd_line[640]` used to render the V2.4.18 coredump status row. Called from the `/` handler, so previously stacked with the `buf[1600]` above on every `/status` render — adjacent allocations consuming ~2.6 KB of the 8 KB.

### Not changed

- OTA error `msg[128/256/384]` buffers in `update_post` (lines 1799, 1928, 1972). Only one in scope at a time, all triggered on rare error paths. Total budget impact negligible.
- TX worker's `body[1280/1600/1700]` in `send_madavi` / `send_osm` / `send_aqi`. TX task has 16 KB stack (`TX_TASK_STACK_BYTES`); each function uses ~1.5 KB body + url + TLS handshake stack (~6 KB) — sits comfortably within budget.
- main task's `log_ftp.c` arrays + `mqtt.c::buf[512]`. Main task has 16 KB stack.

### Files touched

- `main/http_server.c`: 4 edits adding `static` to 28 array declarations + 3 WHY comment blocks
- `main/version.h`: V2.4.21 → V2.4.22
- `CHANGELOG.md`: this entry

BSS cost ~7.4 KB once, replacing ~7.4 KB off every httpd handler invocation. No partition change, no `sdkconfig` change. Drop-in OTA upgrade from V2.4.21.

---

## V2.4.21

**Hotfix for openSenseMap auth-token format.** OSM's ingest endpoint wants the raw token in the `Authorization` header — NOT `Bearer <token>` as V2.3.16 shipped speculatively. Confirmed 2026-05-21 by direct A/B curl against the user's real auth-enabled box: raw token → `HTTP 201 Created`, `Bearer <token>` → `HTTP 401 "Box access token not valid!"`.

### Symptom user reported

User enabled "access token required" on their `Dusty-Feather` box on openSenseMap.org and pasted the 64-char token into `/config` → Save (no restart). Every subsequent OSM upload then failed with:

```
W (...) HTTP_CLIENT: This request requires authentication, but does not provide header information for that
E (...) HTTP_CLIENT: Error response
W (...) tx: openSenseMap perform error: ESP_ERR_NOT_SUPPORTED
W (...) tx: openSenseMap rc=-1 (retry 1/4)
```

(Madavi, sensor.community, aqi.eco, FTPS, MQTT — all other targets continued working normally.)

### Why the firmware log message was misleading

The `HTTP_CLIENT: This request requires authentication, but does not provide header information for that` line is **not** "we forgot to send the header" — it's IDF's `esp_http_client.c:1966` auth-retry handler logging: "I got a 401 back and looked for a `WWW-Authenticate` response header so I'd know which scheme to retry with (Basic / Digest), didn't find one, returning `ESP_ERR_NOT_SUPPORTED`". OSM's 401 response is a JSON body (`{"code":"Unauthorized","message":"Box access token not valid!"}`) with no `WWW-Authenticate` header, so IDF's auth-retry logic gives up with that misleading log. The firmware WAS sending the `Bearer <token>` header all along — it just was the wrong format for OSM.

### Diagnosis path

Direct curl against `https://ingress.opensensemap.org/boxes/<BOX_ID>/data?luftdaten=1` with the user's real token, comparing auth header formats:

| `Authorization:` value | Result |
|---|---|
| `Bearer <64-hex>` (current firmware format) | 401 + `"Box access token not valid!"` |
| `<64-hex>` (raw) | 422 → 201 (422 when sensor IDs don't match box's mapping; 201 when they do) |
| `X-ApiKey: <64-hex>` (alternate header name) | 401 |
| `?accessToken=<64-hex>` (query param) | 401 |

201 with raw token confirms OSM accepts it; 401 with Bearer confirms OSM rejects it. The 422 vs 401 split is the diagnostic that auth IS validated before body content checks — 422 means "auth OK, body unprocessable".

### What changed

One-character-class change in `main/transmission.c::send_osm()`:

```c
- if (have_token) snprintf(authz, sizeof(authz), "Bearer %s", c->osm_access_token);
+ if (have_token) snprintf(authz, sizeof(authz), "%s", c->osm_access_token);
```

Plus a fat WHY comment block above so the next reader doesn't re-add the `Bearer ` prefix thinking it's "standard practice".

### Backward compatibility

- Boxes WITHOUT auth required (every existing deployment until today): user leaves token field empty → `have_token == false` → no `Authorization` header sent at all. Unchanged from V2.4.20.
- Boxes WITH auth required: user pastes token into `/config` → raw token in `Authorization` header → OSM accepts. Fixed by this release.

No checkbox or extra config field needed. The presence/absence of the token IS the gate.

### Files touched

- `main/transmission.c` (+15 LOC: 1-character format-string change, +13 lines of WHY comment)
- `main/version.h`: V2.4.20 → V2.4.21
- `CHANGELOG.md`: this entry

No partition change. No `sdkconfig` change. Drop-in OTA upgrade across all 4 board targets.

---

## V2.4.20

**Hotfix for V2.4.19 httpd stack overflow on `/config` from an unauth client.** Same class of bug V2.4.16 fixed for `syslog_emit`'s 600 B stack buffer — `applog_vprintf`'s `line[1024]` had been on the stack since the module was written. Moved to BSS (protected by the existing `s_mtx`, taken before the buffer is touched and released after).

### Symptom

V2.4.19 on a freshly-erased FeatherS3-D, AP-mode, browse to `http://192.168.4.1/config` without credentials → assert + reboot:

```
http: GET /config from ::FFFF:192.168.4.2
http: auth failed for /config from ::FFFF:192.168.4.2

assert failed: xTaskPriorityDisinherit tasks.c:5157 (pxTCB->uxMutexesHeld)
```

Backtrace (addr2line):

```
panic_abort → esp_system_abort → __assert_func
  ← xTaskPriorityDisinherit              (uxMutexesHeld == 0)
  ← prvCopyDataToQueue / xQueueGenericSend / xQueueGiveMutexRecursive
  ← _lock_release_recursive
  ← usb_serial_jtag_write                (vfs_calls.c:74)
  ← bufio_write / vfprintf / vprintf
  ← applog_vprintf @ applog.c:172        (the vprintf at the top of the function)
  ← esp_log_va / esp_log
  ← check_auth @ http_server.c:135       (ESP_LOGW("auth failed for ..."))
  ← config_get @ http_server.c:1019
  ← httpd_thread (stack: 8 KB)
```

Coredump confirmed the crashing TCB's stack pointer was invalid (0x80376ec4 — way above DRAM range 0x3fc...), meaning the httpd task's stack had overflowed into its own TCB. The FreeRTOS assertion fired on garbage `uxMutexesHeld` data.

### Why V2.4.18 didn't show it but V2.4.19 did

V2.4.18 had the same 1 KB stack allocation, but V2.4.19's added code paths (`net_arp.c` + `periodic.c`) shifted the linker layout enough to move the httpd task's TCB into the corruption zone. V2.4.18 was probably overflowing too but the corruption was landing on benign neighbour memory (no assertion). Either way, the 1 KB-on-stack-in-a-logger was a latent bug; this version fixes it.

### What changed

One-line semantic change in `main/applog.c::applog_vprintf()`:

```c
- char line[LOG_LINE_MAX];                     // 1024 B on stack per call
+ static char line[LOG_LINE_MAX];              // V2.4.20: BSS, protected by s_mtx
```

Plus a fat comment block explaining the why + the V2.4.16 precedent. Safe because:

- `s_mtx` is taken at line 166 and released at line 196 — the entire `line[]` usage window is inside the critical section. Only one task touches `line` at a time.
- Same proven pattern as `syslog_emit`'s `s_emit_buf[600]` (V2.4.16) — that fix has been stable in production since 2026-05-18.
- BSS cost: 1024 bytes once, replacing 1024 bytes off every httpd / TX worker / main / etc. stack frame on every log call.

### Files touched

- `main/applog.c` (~14 LOC delta: `static` keyword + a 12-line WHY comment block)
- `main/version.h`: V2.4.19 → V2.4.20
- `CHANGELOG.md`: this entry

No partition change. No `sdkconfig` change. Drop-in OTA upgrade from V2.4.19, V2.4.18, or earlier post-coredump-migration builds.

---

## V2.4.19

**Gratuitous ARP after every WiFi reconnect + a 24h safety-net.** Closes the failure mode where a sensor's MQTT loop gets permanently stuck in `esp-tls: select() timeout` retries after a WiFi roam, because the upstream AP/mesh bridge's CAM entry for the sensor's MAC has aged out and re-learned the wrong forwarding path.

### Motivation

2026-05-21 incident on bench sensor esp32-5963724 @10.11.12.193. WiFi dropped overnight at `19:32:11` with `reason=34`, sensor reconnected cleanly within 13 s with a fresh DHCP lease. Madavi HTTPS uploads continued to succeed every 150 s for the next ~14 h. But MQTT to the LAN broker at `10.11.12.150:8883` was stuck in an endless retry loop — `sock=N select() timeout` every 20-25 s, surviving the FTP-time MQTT teardown+restart at `09:14:14` unchanged.

Diagnosis via `tcpdump` on the broker showed the sensor's SYNs arriving and the broker's SYN-ACKs leaving within 100 µs — but the sensor's repeated SYNs all carried the same TCP sequence number, proving the SYN-ACKs never reached the sensor's radio. `ping`, `curl`, all broker-initiated traffic to the sensor was 100 % loss while the broker's ARP cache showed the correct sensor MAC as REACHABLE.

Broker and sensor were on different APs in the same mesh SSID (Pi on BSSID `46:78:95:83:d3:34` ch 10, sensor on BSSID `42:78:95:83:d2:80` ch 2). The mesh's per-AP bridge forwarding table for the sensor's MAC had aged out during the long idle gap before the WiFi drop and re-learned wrong after the roam — blackholing every frame the broker addressed to the sensor. Fix at the time was a soft reboot of the sensor (the resulting re-association burst retrained every bridge); the underlying mesh behaviour is upstream of firmware. Full post-mortem in `[[reference_mqtt_one_way_loss_after_wifi_roam]]`.

### What changed

New `main/net_arp.c` + `main/net_arp.h` — single function `net_arp_send_gratuitous()` that calls `etharp_gratuitous()` on the STA netif under `LOCK_TCPIP_CORE`. Cost = one 60-byte ARP broadcast on-air, single µs-level lwIP call. Module is tiny on purpose — shared by the two call sites without coupling them.

Two call sites:

1. **After each WiFi reconnect** (`main/main.c`, in the main loop's `bits & EV_GOT_IP` block). On every GOT_IP event a `s_arp_after_reconnect_pending` flag is set; the actual send is deferred to a later tick when `tx_is_idle()` AND `wifi_up()` hold. The deferral matters for two reasons:
   - lwIP already emits one gratuitous ARP on GOT_IP automatically. Firing ours on the very next tick (1 s typical) gives the upstream AP a second, well-separated chance to learn the path — better than two back-to-back ARPs that could both vanish in the same airtime collision.
   - The TX worker (CPU1) may be mid-handshake with Madavi/sensor.community; deferring coalesces with the worker's natural idle window.

2. **24h safety-net** (`main/log_ftp.c`, piggy-backed on the existing PSA crypto refresh). One call site, runs on the same 24h schedule with the same `tx_is_idle()` gate. Catches the rare case where a sensor holds a single WiFi association for days without a reconnect — the per-reconnect ARP wouldn't fire and the bridge entry could age out anyway.

**FTP gating is structural for both call sites.** Both run on the main task, where FTP also runs blocking. No `log_ftp_is_idle()` API added — would just be extra surface area for the same guarantee.

Default lwIP behaviour is to emit a single gratuitous ARP on `GOT_IP` only. That's hours-to-days apart on a stable link, far longer than any sane bridge aging timer (typically 5 min). V2.4.19 doesn't try to match the aging timer with periodic ARPs — instead, the reconnect-driven fire addresses the failure-mode that actually broke things (path stale across a WiFi drop), and the 24h fire covers the never-reconnects edge case.

### Not changed

- No firmware-side detection or recovery for an *already-broken* bridge path. If a sensor is in the stuck state at the moment V2.4.19 boots, the first WiFi reconnect (or the 24h safety-net fire) should retrain the bridges and let MQTT recover automatically — but if a path is broken hard (mesh node down), a manual `/reboot` or AP power-cycle is still the fix.
- No periodic 5-min ARP cadence. Considered and rejected — chatty for a problem that's really about reconnect-induced path confusion, not slow bridge aging.
- Option B from the analysis (auto-reconnect WiFi after N consecutive MQTT timeouts) deliberately deferred — false-positive risk during legitimate broker downtime causes unnecessary WiFi flap. Revisit if the same stuck-bridge symptom repeats with V2.4.19's prevention in place.

### Files touched

- `main/net_arp.c` + `main/net_arp.h` (NEW, ~60 LOC combined): shared gratuitous-ARP helper
- `main/periodic.c` + `main/periodic.h` (NEW, ~115 LOC combined): main-task periodic housekeeping. Hosts the 24h PSA refresh (extracted from `log_ftp.c::log_ftp_loop()`) and the 24h safety-net ARP. New module exists because the chores aren't conceptually owned by any single feature — pre-V2.4.19 they squatted in `log_ftp.c` for convenience, which made them undiscoverable by file name.
- `main/main.c` (~10 LOC delta): include, EV_GOT_IP-driven flag, deferred-fire block, `periodic_loop()` call in the main tick
- `main/log_ftp.c`: deleted the ~40-LOC PSA-refresh-plus-comment block from the top of `log_ftp_loop()`; added 3-LOC `log_ftp_note_psa_refreshed()` export so `periodic.c` can reset the FTP-side consecutive-OOM streak counter after a successful refresh (preserves the pre-V2.4.19 "healthy refresh blesses the OOM streak" behaviour exactly). The FTP-failure-driven PSA refresh in the upload-error-recovery path stays in `log_ftp.c` because it's intrinsically tied to FTP upload state.
- `main/log_ftp.h`: doc + declaration for `log_ftp_note_psa_refreshed()`
- `main/CMakeLists.txt`: `net_arp.c` + `periodic.c` added to SRCS
- `main/version.h`: V2.4.18 → V2.4.19
- `CHANGELOG.md`: this entry

No partition layout change. No `sdkconfig` change. Drop-in OTA upgrade across all 4 board targets.

### Display detection — probe 0x3D as well as 0x3C

`display.c::try_oled_on_bus()` now probes both standard SSD1306 slave addresses (0x3C default + 0x3D alternate via SA0). Adafruit 326 STEMMA QT OLED, SparkFun and most other I²C OLED breakouts have a solder jumper on the back that flips the SA0 line — some board revisions ship with the jumper closed at 0x3D. Pre-V2.4.19 firmware probed 0x3C only, so those units silently reported "no display found on STEMMA1 or STEMMA2 — display disabled" with no further diagnostic.

The boot log line now shows the actual address the panel was bound at (`display backend: SSD1309 at 0x3D on STEMMA1 ...`), so you can read it back from `/log` and confirm which jumper position the breakout came from. The reset-pulse block on Heltec runs once per call (before the probe loop), not once per candidate — same behaviour as before for the onboard panel.

`s_dev` is bound with the working address baked into the I²C device handle, so all subsequent display I/O routes correctly with no further code changes. SSD1306 / SSD1309 are register-compatible in the init sequence and bitmap path we use, so no per-chip branching needed beyond the address.

---

## V2.4.18

**Panic dumps are now recoverable over the air.** Enables ESP-IDF's coredump-to-flash, exposes the dump via `GET /coredump.elf`, and surfaces an inline summary (panicking task name + PC) on the status page so you can see at a glance whether a deployed sensor has a backtrace waiting to be downloaded.

### Motivation

2026-05-20 morning: esp32-5963724 (bench FeatherS3-D) syslog showed a clean `mqtt: CONNECTED` at 10:14:04, then ~3.5 minutes of silence before the post-reboot resume. Status page reported `Reset reason: PANIC`. That's all we had — no backtrace, no exception cause, no task name. The panic handler had printed it to UART, but no serial logger was running.

For deployed sensors (sealed enclosure, no USB), this is a permanent diagnostic blind spot. ESP-IDF's existing coredump-to-flash + `esp_core_dump_get_summary()` API fixes it cleanly — pay the one-time cable-reflash to add the partition, and every future panic is recoverable over the air, forever.

### What changed

#### Partition layout (CABLE REFLASH REQUIRED ONCE)

Added a 64 KB `coredump` partition at the tail of both `partitions.csv` (8 MB / 16 MB layout) and `partitions_4mb.csv` (4 MB Heltec knock-offs). NVS stays at `0x9000` unchanged, so user config + WiFi creds survive the migration *as long as you do NOT `erase_flash` first* — just write the new merged-bin image at `0x0`. Same constraint that applied to V2.3.16's 4 MB factory-removal.

| Board | Flash chip | Free tail after | Coredump partition |
|---|---|---|---|
| FeatherS3-D | 16 MB | ~9.8 MB still spare | 0x620000 + 64 KB |
| Heltec 8 MB | 8 MB | 1.825 MB still spare | 0x620000 + 64 KB |
| Heltec 4 MB | 4 MB | 64 KB still spare | 0x3E0000 + 64 KB |

#### sdkconfig

All four per-board sdkconfigs flip `CONFIG_ESP_COREDUMP_ENABLE_TO_NONE=y` → `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y` with ELF format + CRC32 checksum + `CHECK_BOOT=y` (invalidates corrupted dumps automatically).

#### New `main/coredump.c` + `main/coredump.h`

Lifecycle:

- `coredump_init()` runs once at boot (right after `applog_init()`). Probes the partition via `esp_core_dump_image_check()`, calls `esp_core_dump_get_summary()` + `esp_core_dump_get_panic_reason()`, caches the task name + exception PC + reason string in static BSS (~230 bytes).
- `coredump_have_dump()` / `coredump_get_size()` / `coredump_get_summary_html()` — cheap status-page reads.
- `coredump_stream_to_http()` — reads the partition in 4 KB chunks and streams via `httpd_resp_send_chunk()`, stopping at the dump's actual size (not the full 64 KB partition).
- `coredump_erase()` — wraps `esp_core_dump_image_erase()` and clears the cache.

The one-time 700 B `malloc(esp_core_dump_summary_t)` at boot is freed within `coredump_init()`. Steady-state heap delta is zero.

#### Status page

New row in the System block:

- **No dump:** `Core dump: none`
- **Dump present:** `Core dump: yes · 38240 bytes · task=mqtt_task PC=0x400d12a4 · Guru Meditation Error: Core 1 panic'ed (LoadStoreError) · [download .elf] [erase]`

The "download .elf" link points at `/coredump.elf`. The "erase" button POSTs to `/coredump_erase` (CSRF + basic-auth gated) and redirects back to `/`.

#### Two new HTTP endpoints

- **`GET /coredump.elf`** — basic-auth gated. Sets `Content-Disposition: attachment; filename="coredump_<chip-id>.elf"` and streams the partition bytes. The filename includes the chip-id so multi-device captures don't collide on your download folder.
- **`POST /coredump_erase`** — basic-auth + CSRF (Origin header) gated, like `/config`, `/update`, `/reboot`. Erases the partition and returns `303 See Other → /`.

### Off-device decode

Once downloaded, decode with the ESP-IDF coredump tool:

```bash
espcoredump.py info_corefile -t elf -c coredump_esp32-5963724.elf build_feathers3_d/geiger_v2.elf
```

The same ELF used for the build is needed — keep a copy alongside the released binaries (GitHub release artefacts already include `geiger_v2.elf` per board). Output shows the panic exception, the full backtrace of the crashing task, and stack snapshots of every other task at the moment of the panic.

### Deployment

Per-device migration is a one-time event:

1. Build V2.4.18 for the board (`_build.cmd <board>`).
2. Cable-flash the merged-bin image at `0x0` (do NOT `erase_flash` first — preserves NVS).
3. Subsequent OTAs work normally; every future panic is recoverable via `GET /coredump.elf`.

The three currently-deployed sensors (Heltec V2 bench, FeatherS3-D bench, FeatherS3-D prod at Oatlands) all need the one-time cable flash. After that the coredump partition stays put forever — no further partition-table changes anticipated.

### RAM cost

| Cost | Amount | Notes |
|---|---|---|
| Static BSS (coredump.c cache) | ~230 bytes | always present |
| Heap at boot | 700 bytes transient | malloc → free within `coredump_init()` |
| Stack on `/status` render | +800 bytes | `cd_summary[320]` + `cd_line[480]` locals in `format_system` — peak chain still ~3 KB of the httpd task's 8 KB stack |
| IDF coredump component | ~0 steady-state | panic-handler-only writers, no background tasks |

Heltec 4 MB min_free headroom impact: ~1.7 %.

### Files touched

- `partitions.csv`, `partitions_4mb.csv` — new `coredump` partition row + header doc
- `sdkconfig.feathers3_d`, `sdkconfig.heltec_v2`, `sdkconfig.heltec_v2_4mb`, `sdkconfig.adafruit_qtpy_esp32_pico` — `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y` + dependent options
- `main/coredump.c` + `main/coredump.h` — new module (~210 LOC)
- `main/CMakeLists.txt` — add `coredump.c` to SRCS, `espcoredump` to REQUIRES
- `main/main.c` — include `coredump.h`, call `coredump_init()` after `applog_init()`
- `main/http_server.c` — include `coredump.h`, new "Core dump" row in `format_system()`, `coredump_get` + `coredump_erase_post` handlers, register the two routes, bump `max_uri_handlers` 10 → 12

---

## V2.4.17

**Two fixes:** OTA teardown stays sticky during the receive loop; CI cppcheck is now a hard gate before release publish.

### 1. OTA teardown was undone within ~1 s of completion

Observed 2026-05-19 on esp32-176432 V2.4.16 → V2.4.16 OTA. The teardown in `update_post` correctly stopped MQTT + syslog + paused FTPS at `22:53:29.499`. But the main-loop poll re-armed both within seconds:

```
22:53:29.523 mqtt: stop: client destroyed
22:53:31.260 syslog: started — ...        ← back after 1.7 s
22:53:35.184 mqtt: CONNECTED to broker     ← back after 5.6 s
22:53:49.340 esp_image: segment 0: ...     ← bulk of OTA recv was here
22:53:51.343 OTA written (1260256 bytes) — restart flagged
```

So during the ~20 s OTA receive/write loop, MQTT and syslog were running again — defeating V2.4.13's intent of freeing ~25 KB of TLS state for esp_ota_write's scratch buffers. The OTA succeeded anyway because heap state at the buffer-alloc moments happened to be OK, but the teardown was effectively theatrical.

#### Root cause

`mqtt_stop()` and `syslog_stop()` were designed as **transient** stops — main.c's poll loop deliberately re-inits both when their initialized-state flips false. The auto-restart was a V2.4.13 feature so that if an OTA *fails*, services come back automatically.

For the OTA *success path* we want services to stay stopped until the device reboots. `log_ftp_pause()` already had this sticky semantic via its `s_paused` flag — MQTT and syslog needed the equivalent.

#### Fix

Added `main_suspend_services()` / `main_services_suspended()` in `main.c`:

```c
static volatile bool g_services_suspended = false;

void main_suspend_services(void) { g_services_suspended = true; }
bool main_services_suspended(void) { return g_services_suspended; }
```

Both auto-restart polls now check the flag:

```c
if (!mqtt_is_initialized() && !main_services_suspended() && ...) mqtt_init(...);
if (!syslog_is_initialized() && !main_services_suspended() && ...) syslog_init(...);
```

`update_post` calls it right after the existing teardown:

```c
log_ftp_pause();
mqtt_stop();
syslog_stop();
main_suspend_services();   // NEW — sticky until reboot
```

On OTA success the device reboots, services come back fresh on the new firmware. On OTA failure, services stay down until manual `/reboot` — matching `log_ftp_pause()`'s existing semantics.

**V2.4.14's FTPS teardown deliberately does NOT call `main_suspend_services()`** — that path wants MQTT to auto-restart between FTPS uploads so per-cycle publishes resume on the next cadence.

### 2. CI: cppcheck is now a hard gate inside release.yml

#### Problem

Pre-V2.4.17, `build.yml` (triggered by commit push) and `release.yml` (triggered by tag push) ran in parallel on a tag-push event. `build.yml` has a `cppcheck` job; `release.yml` did not. So `release.yml` could publish a release even if `build.yml`'s cppcheck failed on the same commit.

Observed during the V2.4.15 ship: cppcheck flagged a `constVariablePointer` style warning in `syslog.c::gethostbyname`, the build workflow went red, but the release workflow already published V2.4.15 successfully. The follow-up `const` fix shipped as a no-tag commit.

#### Fix

Added a `cppcheck` job at the top of `release.yml`, mirroring `build.yml`'s flags and suppressions exactly. The `build` matrix job now has `needs: cppcheck` so no board build runs on a dirty cppcheck. The release-creation job already has `needs: build`, so it inherits the gate transitively.

If cppcheck fails on a tag push:
- The tag exists on the remote (no way to retract from CI)
- No release is created
- Recovery: fix the cppcheck issue, force-tag back to the new commit (or just bump to the next version)

Note: `build.yml`'s `host-test` (with valgrind) is **not** mirrored into `release.yml` — leaving that as a future enhancement. The release path now has cppcheck + build + version-string verification gates, which covers the common-case static issues.

### Code changes

- **`main/main.c`** — new `g_services_suspended` static + `main_suspend_services()` / `main_services_suspended()` functions; both poll-loop re-init conditions gated on the new flag.
- **`main/main_status.h`** — declarations for the two new functions.
- **`main/http_server.c`** — one new call `main_suspend_services();` in `update_post` right after the existing teardown.
- **`.github/workflows/release.yml`** — new `cppcheck` job at the top; `build` job gains `needs: cppcheck`.

### Recommendation

**Safe to flash as a routine OTA update.** Behaviour change is localized to the OTA POST path (services now actually stay down during the OTA receive/write loop, as V2.4.13 originally intended). Other code paths unchanged.

---

## V2.4.16

**Two V2.4.15 syslog issues fixed.** Both bugs were introduced together in V2.4.15.

### Issue 1: `/config` page caused a hard PANIC

#### Symptom

Reported 2026-05-19 on esp32-5963724 + esp32-5965048 (both FeatherS3-D): with syslog enabled, loading `/config` in a browser caused an immediate reset with reason `PANIC`. Other pages worked fine. Disabling syslog avoided the panic.

#### Confirmed via serial backtrace

Serial capture on the Heltec V2 4MB bench (esp32-176432, also running V2.4.15) reproduced the same panic on `/config`:

```
Guru Meditation Error: Core 1 panic'ed (LoadStoreError). Exception was unhandled.
PC      : 0x4008b14a   EXCVADDR: 0x400835c0   A1: 0x3ffe0770
Backtrace: 0x4008b147 → 0x4008b004 → CORRUPTED
```

addr2line decoded all three frames into `vPortYieldFromInt`, `_frxt_int_exit`, `_frxt_int_enter` — FreeRTOS context-switch assembly. The classic Xtensa stack-overflow signature: a tick or yield interrupt fires while a task's stack is past its allocated boundary, and the context-save attempts to write registers into invalid memory.

#### Root cause

V2.4.15's `syslog_emit()` declared a 600-byte send buffer **on the stack**. The httpd task's 8 KB stack was already tight when rendering `/config` (which has the largest body builder in the firmware — ~14 KB HTML worst case, with ~4 KB of local `html_esc[]` arrays in `config_get`'s function frame). Every `ESP_LOG` call mid-rendering (e.g. `log_access(req, "GET /config")`) added another ~700 B of stack frame via `applog_vprintf` → `syslog_emit`. The combination pushed config_get's prologue past the 8 KB canary.

#### Fix: move syslog_emit's buf to BSS

```c
// V2.4.15: char buf[600];                  ← stack, ~700 B per call
// V2.4.16:
static char s_emit_buf[600];                ← BSS, 0 stack pressure
```

`applog_vprintf` holds its mutex while calling syslog_emit, so concurrent emits are structurally impossible. The static is safe without its own lock. Cost: 600 B of permanent BSS. Benefit: zero stack contribution from the syslog forward path.

**httpd stack remains at 8 KB** (an earlier draft of this hotfix bumped it to 12 KB defensively, but the static-buf fix alone restores pre-V2.4.15 stack usage and the +4 KB heap cost isn't justified on Heltec V2's tight budget).

### Issue 2: every ESP_LOG produced 2-3 syslog packets

#### Symptom

The rsyslog server saw each logical log line split across multiple rows:

```
2026-05-19T22:03:54 MultiGeiger5965048 geiger: I (26-05-19 22:03:54.750) tx:
2026-05-19T22:03:54 MultiGeiger5965048 geiger: sensor.community: ok (rc=201)
```

— annoying to grep, hard to scroll through.

#### Root cause

ESP-IDF v6.0's `esp_log_writev` calls our vprintf hook **multiple times per ESP_LOG**: once for the prefix (`I (timestamp) tag: `), once for the user format body, and once for the trailing `\n`. V2.4.15's `syslog_emit` sent one UDP packet per fragment, producing one syslog row per fragment.

#### Fix: accumulate fragments, emit on newline

`syslog_emit` now appends each fragment to a static 768-byte accumulator and only `sendto`s a UDP packet when the accumulator hits `\n`. Single-threaded by construction (applog_vprintf's mutex). Pathological inputs (single fragment > 768 B) bypass the accumulator and emit standalone.

After: one syslog row per ESP_LOG call.

### Recovery for V2.4.15 devices

OTA from V2.4.15 → V2.4.16 works because the V2.4.13 OTA teardown path in `update_post` runs BEFORE the syslog forward gets a chance to fire on the new request — so even if you can't reach `/config` to disable syslog, you can still reach `/update` to push the V2.4.16 image. Just open `http://<device-ip>/update` directly, pick the new binary, click Upload. The teardown calls `syslog_stop()` (V2.4.15) and `mqtt_stop()` (V2.4.13) before the recv loop, leaving plenty of heap for OTA. USB cable reflash also works as a fallback.

### Code changes

- **`main/syslog.c`** — `syslog_emit` refactored: extracted RFC 3164 frame builder + sendto into a new internal `emit_packet()` helper; moved the 600-byte send buffer to static BSS; added 768-byte static accumulator for fragment coalescing.

### Recommendation

**Immediate flash recommended** for anyone running V2.4.15 with syslog enabled. V2.4.15 without syslog (the default for unconfigured devices) is unaffected — but everyone should flash to V2.4.16 to enable syslog safely going forward.

---

## V2.4.15

**UDP syslog client (RFC 3164) — opt-in per-line log shipping.**

Every ESP_LOG line emitted after STA connect is now optionally forwarded as a UDP syslog packet to a LAN syslog server (rsyslog / syslog-ng). Cheapest log-shipping path on the firmware:

- **0 KB persistent heap** — one UDP socket, no TLS, no retry buffers, no per-connection state
- **~50-100 µs CPU per line** — non-blocking `sendto()` with `MSG_DONTWAIT`
- **No broker / no server software outside what's already installed on the Pi** — rsyslog ships with Raspberry Pi OS

Pairs with [[reference_syslog_pi_setup]] (~10 lines of rsyslog config on the Pi: enable `imudp` module, route by `$hostname startswith "MultiGeiger"` to a dynaFile per-device).

### What it ships

Every line that goes through `applog_vprintf` — i.e., every `ESP_LOGI/W/E/D/V` call. Severity is parsed from the formatted output's level prefix ('E' → error, 'W' → warning, 'D'/'V' → debug, else info). Facility hardcoded to local0 (16). Timestamp included if the device's wall clock is post-NTP-or-RTC-carryover; rsyslog re-stamps from receive-time either way.

### Boot-time logs not shipped

`syslog_init()` runs from the main-loop poll once `n_got_ip > 0`. Anything emitted before STA connect (boot config dump, sensor probes, WiFi events) is captured in the applog ring + UART but NOT in syslog. View it via `/log` after the device comes up. Same constraint as MQTT.

### Config fields

- `syslog_enable` — master switch, off by default (opt-in)
- `syslog_host` — server FQDN or IPv4 (empty also disables defensively)
- `syslog_port` — UDP port, default 514

New `/config` section under "Syslog (UDP)" with the three fields. Schema-driven POST handling — no per-field plumbing.

### Per-board impact

| Board | Heap impact when disabled | Heap impact when enabled (idle) | Heap impact during emit |
|---|---|---|---|
| Heltec V2 | 0 KB | ~1 KB lwIP socket | ~0 KB (mbuf cycles through stack) |
| Heltec V2 4MB | 0 KB | ~1 KB | ~0 KB |
| FeatherS3-D | 0 KB | ~1 KB | ~0 KB |
| QT Py | 0 KB | ~1 KB | ~0 KB |

### How rsyslog differentiates clients

- `$fromhost-ip` — every UDP packet has the sender's IP
- `$hostname` — the firmware embeds the device's WiFi hostname (e.g., `MultiGeiger176432`) in the RFC 3164 header
- `$programname` — always `"geiger"`

The example rsyslog config in the new memory uses `$hostname startswith "MultiGeiger"` and a dynaFile template `/var/log/geiger/%HOSTNAME%.log` to give each device its own log file.

### Teardown integration

`syslog_stop()` is called from the V2.4.13 OTA teardown alongside `mqtt_stop()`. Frees the UDP socket (~1 KB) — small absolute saving but consistent with the teardown pattern. NOT called from V2.4.14's FTPS teardown — losing syslog visibility during FTPS is worse than the ~1 KB heap recovery.

### Code changes

- **`main/syslog.h` / `syslog.c`** (NEW) — ~140 LOC. Public surface: `syslog_init`, `syslog_stop`, `syslog_is_initialized`, `syslog_emit`. Re-entrancy guard (`s_in_emit` flag) and a deliberate "never ESP_LOG from emit path" rule prevent vprintf recursion through applog's mutex.
- **`main/applog.c`** — one new call (`syslog_emit(line, len)`) right after `ring_append()`. `#include "syslog.h"`.
- **`main/main.c`** — new poll branch alongside the MQTT one, gated on `n_got_ip > 0` (no NTP gate — rsyslog tolerates missing timestamps). `#include "syslog.h"`.
- **`main/http_server.c`** — new "Syslog (UDP)" form section + format args + `syslog_stop()` in the OTA teardown. `#include "syslog.h"`.
- **`main/config.c`** — boot dump gets a `syslog:` line under the existing mqtt rows.
- **`main/config.h`** — `CFG_SYSLOG_HOST_MAX = 63`.
- **`main/config_fields.def`** — three new schema entries: `syslog_enable`/`syslog_host`/`syslog_port`. POST handling auto-generated.
- **`main/CMakeLists.txt`** — adds `syslog.c` to SRCS.

### Recommendation

**Opt-in, safe to flash as a routine OTA update.** Off by default — devices already on V2.4.14 with no config change show identical behaviour after upgrade. Enable per-device via `/config` once the rsyslog receiver is set up.

---

## V2.4.14

**Extend the V2.4.13 OTA-teardown pattern to FTPS uploads.** Same fix, different trigger — stop MQTT before the upload to free its TLS session state.

### Observation

Production logs from both Heltec V2 boards on V2.4.13 (2026-05-19 evening soak) showed the FTPS upload was the dominant heap consumer:

```
esp32-176432 (4MB) cycle #6:
  Pre-upload:  free=68500 min_free=15572 largest=59392
  Post-upload: free=68288 min_free=1112  largest=59392   <-- 14 KB peak dip
```

```
esp32-12276328 (8MB) cycle #30 (FTPS server-side stall):
  Pre-upload:  free=68340 min_free=17080
  → 32 s TLS stall, write got 4096/43186 bytes through
  → PSA preemptive reset
  Post-upload: free=68876 min_free=3600
  → mqtt: errno=11 (EAGAIN), DISCONNECTED
  → 30 min of cascading MQTT handshake failures (-0x2700)
```

Even on successful uploads the min_free briefly dipped into single-digit-KB territory — one bad allocation away from OOM. On failed uploads (server stall) the MQTT TLS write paths hit EAGAIN as the kernel ran out of buffers, taking MQTT down with the FTPS attempt.

### Fix

In `log_ftp_loop`, right before `do_ftp_upload()`:

```c
bool mqtt_was_running = mqtt_is_initialized();
if (mqtt_was_running) {
    ESP_LOGI(TAG, "FTPS prep: stopping MQTT to free TLS state");
    mqtt_stop();
}
bool ok = do_ftp_upload();
```

Main loop poll (`!mqtt_is_initialized() && n_got_ip > 0 && ntp_time_valid()`) re-inits MQTT within ~1 s of FTPS completion.

### Expected impact on Heltec V2

- min_free during FTPS upload: **~1 KB → ~40 KB**
- MQTT keep-alive failures during FTPS: **eliminated**
- MQTT cascading reconnect storms after failed FTPS: **eliminated**

### HA / broker behaviour during the FTPS window

`mqtt_stop()` sends a clean MQTT DISCONNECT packet, so the broker does NOT fire LWT (LWT only fires on abrupt disconnect / keep-alive timeout). The retained "online" availability message stays in place. HA sees the device as online continuously; subscribers just miss state publishes for the 5–30 s upload window (acceptable on a 150 s TX cadence). After upload, mqtt_init() reconnects, publishes "online" again (idempotent — same retained value), and republishes HA Discovery configs (idempotent — broker stores latest retained).

### Frequency / retry interaction

FTPS schedule is 15 min by default (configurable). On failure, V2.3.15's 4-retry sequence ~3 min apart applies. Each retry teardown-and-restart-MQTT cycle:

- ~3-5 s of MQTT publish silence per FTPS attempt
- Worst case (1 + 4 retries = 5 attempts): 25 s cumulative MQTT silence over 15 min — still well below the 60 s keep-alive

For users who find this objectionable on FeatherS3-D (where the teardown isn't load-bearing), a future tag could heap-gate the teardown. Skipping for now — keeping the code path uniform across boards.

### Code changes

- **`main/log_ftp.c`** — `#include "mqtt.h"`; ~10 LOC teardown block before the `do_ftp_upload()` call. No other changes.

### Recommendation

**Safe to flash as a routine OTA update.** Behaviour change is localised to FTPS upload start. No NVS / config-form / per-cycle TX path delta.

---

## V2.4.13

**OTA memory-freeing teardown + weak-WiFi recv resilience** — two independent OTA-path improvements, both shipping in the same tag because they touch the same handler.

### 1. OTA memory-freeing teardown

The V2.4.11+ MQTT TLS client + WiFi stack + esp_crt_bundle leaves Heltec V2 (4MB) at min_free ≈ 13 KB at steady state. That's not enough headroom for `esp_ota_write`'s internal scratch buffers, causing OTA to OOM mid-flash. Observed 2026-05-19 on esp32-176432 trying to upgrade V2.4.11 → V2.4.12:

```
W (08:54:03.914) httpd_txrx: httpd_resp_send_err: 500 Internal Server Error - oom
```

Device kept running but could not accept any OTA upload until physically power-cycled and USB-reflashed. Unacceptable as a long-term path — every Heltec V2 OTA from now on would hit this.

### Fix

`POST /update` now runs a three-step teardown BEFORE the OTA receive/write loop, lifting min_free back to ~50 KB:

1. **Drain the TX worker** (`tx_is_idle()` poll, up to 10 s) — prevents a Madavi HTTPS POST in flight from competing with the OTA recv loop on TLS state.
2. **Pause FTPS scheduling** (`log_ftp_pause()`) — sticky-flag prevents the next due upload from firing mid-OTA. Holds ~15-20 KB of mbedTLS state during its handshake otherwise.
3. **Stop+destroy MQTT** (`mqtt_stop()`) — frees ~18-25 KB of TLS session + esp-mqtt state.

After the teardown, the OTA receive loop runs with comfortable headroom and the existing reboot-on-success flow handles the rest.

### Scoped to POST only

GET `/update` (rendering the upload form) is untouched. Just visiting the page does NOT impact MQTT or FTPS — only clicking "Upload" (which submits a POST with the binary) triggers the teardown. Standard HTTP method semantics give us this for free.

### Failure-mode handling

- **OTA succeeds → device reboots** into new firmware, MQTT reconnects on V2.4.X+1, FTPS schedule restarts. No user action.
- **OTA fails** (network drop, bad file, etc.) → device keeps running, but MQTT and FTPS are now down. `mqtt_is_initialized()` (new V2.4.13 internal accessor) makes the main-loop poll re-init MQTT within ~1 s. FTPS stays paused until reboot — user can `/reboot` manually if they want it back without retrying OTA. Re-uploading would re-run the teardown harmlessly (mqtt_stop is idempotent).

### 2. Weak-WiFi OTA recv resilience

OTA over a marginal WiFi link sometimes failed with `500 recv failed` partway through the upload. Two contributors:

1. **Per-call recv timeout was 5 s** (esp_http_server default) — a single TCP gap longer than 5 s killed the entire OTA, even if 90% of the binary had already been received.
2. **First `HTTPD_SOCK_ERR_TIMEOUT` was treated as fatal** — no retry logic, even though the underlying TCP connection is often still alive after a brief stall.

Fix:

- **Bumped `recv_wait_timeout` from 5 s → 30 s** at server-config time. Only ticks during active recv calls (request body reads), so idle keep-alive connections aren't affected. Send-side keeps the default (responses are tiny).
- **Added retry-on-timeout** in the OTA recv loop — up to 5 retries on `HTTPD_SOCK_ERR_TIMEOUT`, resetting the counter on every successful recv. Combined with the 30 s timeout, that's **up to 150 s of TCP outage tolerance per chunk** before the OTA gives up.
- `HTTPD_SOCK_ERR_FAIL` (-1) and peer-close (0) are still fatal — there's no recovering from those.

New log line when a transient timeout happens mid-OTA:

```
W: recv timeout at 524288/1256352 — retry 1/5
```

If the link recovers, the next recv succeeds, the counter resets, and the OTA continues silently. If all 5 retries hit timeout:

```
E: recv timed out 5× at 524288/1256352 — giving up
```

### Code changes

- **`main/mqtt.h` / `mqtt.c`** — new `mqtt_stop()` + `mqtt_is_initialized()`. Internal `s_initialized` sticky flag set on first init (whether enabled or disabled-by-config), cleared by `mqtt_stop()`.
- **`main/log_ftp.h` / `log_ftp.c`** — new `log_ftp_pause()`. Internal `s_paused` flag checked in `log_ftp_loop()` after the due/retry computation, before the upload runs.
- **`main/http_server.c`** — three-step teardown + heap diagnostic log at the top of `update_post()`, after auth/CSRF checks; recv_wait_timeout bumped to 30 s; OTA recv loop gains `HTTPD_SOCK_ERR_TIMEOUT` retry-with-budget logic.
- **`main/main.c`** — dropped local `mqtt_started` static; replaced with `mqtt_is_initialized()` so `mqtt_stop()` is visible to the re-init poll.

### Heap diagnostic log

The teardown emits a one-line snapshot of post-teardown heap so future OOMs are easier to diagnose:

```
http: OTA prep: waiting for TX worker idle
log_ftp: paused — no scheduled uploads until reboot
mqtt: stopping client to free TLS state
mqtt: stop: client destroyed
http: OTA prep: heap free=119344 min=13668 largest=98432
http: OTA: 1256352 bytes -> partition ota_0 @ 0x20000
```

### Recommendation

**Safe to flash as a routine OTA update.** Behaviour change is localised to the `/update` POST path + the server-wide recv timeout bump (which only affects in-flight body reads). Normal operation (cycles, MQTT publishing, FTPS uploads, Madavi/SC/OSM uploads) is byte-identical to V2.4.12. The teardown is harmless on FeatherS3-D / QT Py (heap was never tight) but a Heltec V2 lifesaver, and the recv resilience helps every board on a marginal link.

---

## V2.4.12

**Three small follow-ups to V2.4.11's MQTT work**: start-gate fix, publish reorder, and per-field env-block gating.

### 1. MQTT start gate fix

V2.4.11's `ntp_time_valid()`-only gate didn't actually defer MQTT on soft reboots, because ESP32's RTC retains wall-clock time across `esp_restart` / OTA / watchdog. After the first cold boot, the clock is already past 2025-01-01 every subsequent boot, so `ntp_time_valid()` returns true ~1 second after `app_main` starts — well before WiFi STA has come up, while still in the 2-min AP boot window. Net result: MQTT fires immediately, spams five `esp-tls: select() timeout` cycles per 25 s for the whole AP window (the LAN broker is unreachable from AP mode), then finally connects when the STA finally gets an IP.

Observed in the V2.4.11 boot log on `esp32-176432` (Heltec V2, soft-reboot after OTA):

```
21:48:37.686 v2_main: MQTT deferred until NTP sync (broker=10.11.12.150:8883)
21:48:38.885 v2_main: NTP synced — starting MQTT client     <-- 1.2 s later, no STA yet
21:48:48.905 esp-tls: [sock=57] select() timeout
21:49:13.936 esp-tls: [sock=57] select() timeout
21:49:38.965 esp-tls: [sock=57] select() timeout
21:50:03.995 esp-tls: [sock=57] select() timeout
21:50:29.025 esp-tls: [sock=57] select() timeout
21:50:37.895 v2_main: AP window closed — stopping AP and switching to STA
07:50:41.668 v2_main: GOT_IP #1: 10.11.12.197 ...           <-- 2 min wasted
07:50:45.069 mqtt: CONNECTED to broker
```

### Fix

Gate on **both** conditions, not just clock-sane:

```c
// main.c
if (!mqtt_started && n_got_ip > 0 && ntp_time_valid()) {
    ESP_LOGI(TAG, "STA has IP + clock sane — starting MQTT client");
    mqtt_init(&g_cfg, g_chip_id);
    mqtt_started = true;
}
```

- `n_got_ip > 0` — already maintained by `on_ip_event` at `main.c:285`. Sticky (only ever increments), exactly the "STA has reached the LAN at least once this boot" semantics we need.
- `ntp_time_valid()` — kept as-is. RTC carryover after a soft reboot IS genuinely fine for TLS cert validation; the bug was using only this half of the gate, not the predicate itself.

### Behaviour matrix

| Scenario | `n_got_ip > 0` | `ntp_time_valid()` | Gate fires |
|---|---|---|---|
| Cold boot, AP window | false | false | waits |
| Cold boot, STA up, pre-SNTP | true | false | waits |
| Cold boot, STA up, SNTP done | true | true | ✅ ~4 s after STA GOT_IP |
| Soft reboot, AP window | false | true (RTC) | **waits** (this was the V2.4.11 bug) |
| Soft reboot, STA up | true | true | ✅ immediately on STA GOT_IP |

### Code changes

- **`main/main.c`** — `mqtt_started` poll-loop predicate gains the `n_got_ip > 0` clause; boot-section comment + log line updated to say "STA has IP + NTP synced" instead of just "NTP sync".

No `ntp.c` / `ntp.h` changes. The other two `ntp_time_valid()` callers (`log_ftp.c:750`, `http_server.c:451`) want the "wall clock is sane right now" predicate regardless of source, which is correctly what `ntp_time_valid()` provides — RTC carryover is fine for those.

### 2. MQTT publish reordered before tx_transmit

Pre-V2.4.12 the per-cycle call order was `tx_transmit()` then `mqtt_publish_state()`. Both are non-blocking enqueues to different tasks (CPU1 worker vs esp-mqtt task), so the two log streams raced once both were queued. The MQTT publish — which talks to the LAN broker and ACKs in ~5 ms — would land in the middle of Madavi's TLS handshake log (~1.7 s for cert validation alone), producing visually confusing /log output:

```
tx: free heap before TX: 4308368 bytes / ...
tx: Sending to Madavi (https)
mqtt: publish ok: 110 bytes (#2)              <-- mid-Madavi
esp-x509-crt-bundle: Certificate validated
tx: Madavi: env rc=200 (tube disabled)
tx: Madavi: ok (rc=200)
```

V2.4.12 swaps the call order. The on-LAN broker still wins the race almost every time, but now the call site reflects the intended log shape:

```
tx: free heap before TX: 4308368 bytes / ...
mqtt: publish ok: 110 bytes (#2)              <-- clean header
tx: Sending to Madavi (https)
esp-x509-crt-bundle: Certificate validated
tx: Madavi: env rc=200 (tube disabled)
tx: Madavi: ok (rc=200)
```

No semantic change — same snapshot data (g_last_* is populated at `main.c:385`, well before either call), same non-blocking enqueue, same drop-on-disconnect behaviour. Pure log-clarity refactor.

### 3. Per-field env-block gating in MQTT + HA Discovery

`env_sensor_read()` was returning a single "any field valid" success flag, which mqtt.c and mqtt_discovery.c both treated as "publish/register all three of env_t, env_h, env_p." Result: any setup whose sensor cascade can't produce a full T+H+P trio published phantom zero values for the missing fields, and HA registered phantom entities for them.

Concrete failure observed when only an SHT45 was connected (no Bosch pressure chip):

```
mqtt: publish ok: ... "env_t":18.60,"env_h":81.35,"env_p":0.0 ...
```

HA then created a Pressure entity displaying **0.00 hPa** every cycle forever.

V2.3.29 had already fixed the same class of bug for Madavi (sentinel detection in `build_madavi_env_body`), but the MQTT path went in fresh in V2.4.2 and was never carried over.

**Fix:**

- `env_sensor_read()` API extended with three `bool *out_have_t/h/p` per-field validity out-params. The single existing caller (`main.c`) plumbs them through.
- `env_sensor.h` adds three boot-time driver-presence helpers: `env_t_present()`, `env_h_present()`, `env_p_present()` — the union of cascade chips that can produce each measurement type.
- `main_status_t` gains `have_env_t / have_env_h / have_env_p` alongside the existing `have_env` (which is now the OR of the three, kept for `http_server.c` and `transmission.c` callers that just need "render env block at all?").
- `mqtt.c` gates each field independently: `if (st->have_env_t) APPEND(...)`, etc.
- `mqtt_discovery.c` replaces the shared `env_present_` predicate on env_t/env_h/env_p with `env_t_present_` / `env_h_present_` / `env_p_present_`.

**Behaviour matrix:**

| Setup | env_t | env_h | env_p |
|---|---|---|---|
| SHT45 only | ✅ | ✅ | — (was 0.00 hPa entity) |
| BMP581 only | ✅ | — (was 0.00 % entity) | ✅ |
| SHT45 + BMP581 | ✅ | ✅ | ✅ |
| Nothing | — | — | — |

Pure HA-side observability fix; no upload-path behaviour change. Madavi/SC/OSM/aqi.eco still use the V2.3.29 value-sentinel logic and are unaffected.

### Code changes (combined)

- **`main/main.c`** — gate predicate gains `n_got_ip > 0` (fix 1); MQTT publish block moves above `tx_transmit()` call (fix 2); new `g_last_bme_{t,h,p}_valid` caches + populated from extended `env_sensor_read` (fix 3).
- **`main/env_sensor.h` / `.c`** — new per-field driver-presence helpers; `env_sensor_read()` API gains three `bool *out_have_*` out-params.
- **`main/main_status.h`** — new `have_env_t / have_env_h / have_env_p` fields; `have_env` documented as the OR.
- **`main/mqtt.c`** — three-way per-field env gating.
- **`main/mqtt_discovery.c`** — three per-field predicates wired into the env entity rows.

### Recommendation

V2.4.12 is **safe to flash** as a routine OTA update. Same scope-of-change as V2.4.11; no NVS / config-form / behaviour delta outside the boot window + log ordering + HA entity catalog (existing HA installs may need to manually delete the now-orphaned 0 hPa pressure / 0 %RH humidity entities on previously-misconfigured nodes — they'll just stop receiving updates, the entities themselves persist in HA until removed via UI).

---

## V2.4.11

**Boot diagnostics + NTP-gated MQTT start** — two small, related quality-of-life tweaks that surface in the boot log.

### 1. Boot-time config dump now covers V2.4.x additions

The `config: ` block printed at boot was missing the fields added since V2.4.2. Three new lines now appear:

```
config:   display:          mode=0(auto)
config:   mqtt:             enabled=1 broker=10.11.12.150:8883 user=geiger pw=<set>
config:   mqtt:             topic_prefix=geiger ha_discovery=1
config:   mqtt.tls:         enabled=1 mode=1(B:custom) ca=<set>
```

Covers:
- `display_mode` (V2.4.9) — both numeric value and the auto/radiation/rotation legend.
- MQTT broker / port / user / password / topic_prefix / ha_discovery (V2.4.2–V2.4.5).
- MQTT TLS enable / mode / CA-set flag (V2.4.6) — mode prints as `0(A:bundle)` / `1(B:custom)` / `2(D:skip)` so the log is self-documenting without grepping `config_fields.def`.

Passwords + the CA PEM body are masked the same way as the existing wifi/ftp/radmon credentials (`<set>` / `<empty>`).

### 2. MQTT client deferred until NTP sync

Pre-V2.4.11 `mqtt_init()` was called during the AP boot window, before any STA connection existed. The result was five `esp-tls: connect() error` / `select() timeout` cycles per boot (~25 s of log noise) until the AP window closed and the STA came up. Worse, on first boot without a battery-backed RTC the wall clock reads ~1970-01-01, which makes TLS cert validation fail with `NotBefore` errors — a different failure mode mixed in with the route-unreachable errors.

V2.4.11 holds back the `mqtt_init()` call until `ntp_time_valid()` returns true (wall clock > 2025-01-01). New boot log shape:

```
v2_main: MQTT deferred until NTP sync (broker=10.11.12.150:8883)
...
ntp: sync OK: 2026-05-19T00:04:21 AEST
v2_main: NTP synced — starting MQTT client
mqtt: started — uri=mqtts://10.11.12.150:8883 tls_mode=B (custom CA cert) ...
mqtt: CONNECTED to broker
```

Exception preserved: if MQTT is **disabled** or the broker field is empty, `mqtt_init()` is still called at boot so the existing `mqtt: disabled (...)` log line appears in the boot trace at the same point as before. The deferred-start path only kicks in when MQTT is configured.

### Code changes

- **`main/config.c`** — three new `ESP_LOGI` lines + small inline legend lookups for `display_mode` and `mqtt_tls_mode`.
- **`main/main.c`** — new `mqtt_started` static flag; boot section split on `mqtt_enable` + `mqtt_broker[0]`; main loop polls `ntp_time_valid()` after `ntp_poll()` and fires `mqtt_init()` once.

### Failure modes

- **NTP never syncs** (network unreachable, all servers blocked) → MQTT never starts. Acceptable trade — without a clock TLS would fail anyway, and the FTPS uploads also fail in that state.
- **`mqtt_init()` accidentally called twice** → existing guard at `mqtt.c:127` logs `init called twice — ignoring`; `mqtt_started` flag is belt-and-braces on top.

### Recommendation

V2.4.11 is **safe to flash** as a routine OTA update. No NVS schema changes, no config-form changes, behaviour change only at boot.

---

## V2.4.10

**Cppcheck hotfix** for V2.4.9 — three `variableScope` style warnings in `display.c` flagged by the CI cppcheck step (build CI exit 1, blocked future PR merges). Pure style fix; zero runtime behaviour change. Functionally identical binaries to V2.4.9.

### Warnings fixed

```
display.c:796: char line[16];   // render_oled_pm_mass — moved inside if (s_snap.pm_valid)
display.c:834: char line[16];   // render_oled_pm_number — moved inside if (s_snap.pm_valid)
display.c:835: char val[8];     // render_oled_pm_number — moved inside if (s_snap.pm_valid)
```

In both `render_oled_pm_mass()` and `render_oled_pm_number()` the scratch buffers were only used in the `if (s_snap.pm_valid)` branch; the `else` branch passes string literals directly. Declarations moved inside the `if` block.

### Why bump a version for this

Build CI is treated as authoritative — a red main blocks the next PR. Bumping a tag (rather than just pushing to main untagged) keeps the rule "every tag = unique source state" intact, so `git checkout V2.4.X` always reproduces the exact binary in the corresponding release artefacts. The alternative (commit to main without bump) would leave the V2.4.9 tag pointing at the cppcheck-failing source forever.

### Recommendation

V2.4.10 is **not required to flash** if you're already on V2.4.9 — the binaries are functionally identical. Pull V2.4.10 into the next OTA window for cleanliness.

---

## V2.4.9

**Runtime-configurable display layout** — replaces the compile-time `HAL_MULTIPAGE_ROTATION` macro. New `display_mode` config field with three options: `auto` (default), `radiation` (single-page Heltec-style), `rotation` (5-page Env / PM / Number / Uploads / System). User picks via `/config` dropdown; resolution happens at `display_setup()` time.

### The auto rule (panel-based)

| Backend / chip | Auto resolves to |
|---|---|
| SerLCD | rotation (4-line character LCD has room for multi-page content) |
| Big OLED (SSD1309, 2.42") | rotation (Core Electronics CE09964 on FeatherS3-D) |
| Small OLED (SSD1306, 0.96") | radiation (Heltec V2 onboard, Adafruit 326 on STEMMA QT) |

The SSD1306-vs-SSD1309 distinction uses the per-board compile-time hint (`BOARD_FEATHERS3_D` ships with SSD1309; everything else with SSD1306) because the two controllers are register-compatible at the I²C level and don't expose a chip-ID register that reliably differs. The SerLCD discrimination is runtime via `s_backend == BACKEND_SERLCD`.

### Edge case + escape hatch

Plugging an Adafruit 326 (small SSD1306) into a FeatherS3-D's STEMMA1 in place of the SSD1309 would mis-auto-pick `rotation` because the per-board hint can't see the smaller panel. Workaround: select `radiation` explicitly in `/config` — explicit modes bypass the auto rule entirely.

### What changed in the code

- **`config_fields.def`** — new `X_U32(display_mode, "disp_mode", 0, 0, 2)` field
- **`display.h`** — new `display_mode_t` enum + `display_setup()` signature gains `mode` parameter + new `display_is_multipage()` / `display_mode_str()` getters
- **`display.c`** — auto resolution at `display_setup()`'s `task_spawn` label (after panel is known); all `#if HAL_MULTIPAGE_ROTATION` guards removed; multipage task always compiled-in, spawn decision now runtime
- **`main.c`** — `display_setup()` call updated to pass `g_cfg.display_mode`; per-cycle radiation render and snapshot push gated by `display_is_multipage()` instead of `#if`
- **`http_server.c`** — new `/config` dropdown row between "Enable Display" and "Display brightness"; `/status` System block gains a "Display layout" row showing the resolved mode (e.g. `auto (resolved: rotation)`)
- **`hal.h`** — `HAL_MULTIPAGE_ROTATION` removed from all three board branches + the required-flags doc-comment list (now dead, replaced by runtime decision)

### Migration notes

- **Existing devices on V2.4.8 or earlier:** NVS `disp_mode` key won't exist → `config_load` falls back to compile-time default `0` (auto). FeatherS3-D continues to show rotation; Heltec V2 + QT Py continue to show radiation. No user action required.
- **V2.4.8's QT Py hardcoding (radiation) is now redundant** — auto-mode delivers the same outcome based on the SSD1306 chip type. No behaviour change for users who never opened `/config`.
- **Reboot-required** — display task lifecycle decision happens at `display_setup()` only. Field marked with red `*` in the form.

### Binary size

QT Py reabsorbs the ~3.8 KB of multipage code that V2.4.8 gated out (now compiled into all boards because the decision is runtime). Heltec binaries grow by ~5 KB similarly. Acceptable on all boards; far from any partition limit.

---

## V2.4.8

**QT Py ESP32-PICO: radiation-only single display page** (matches Heltec V2 OLED layout). User paired a QT Py with an Adafruit 326 OLED (Monochrome 0.96" 128×64 SSD1306 STEMMA QT) and wants the Heltec-style radiation page rather than the 5-page rotation that's been the QT Py default since V2.3.29.

### Zero new driver code

The Adafruit 326 is an SSD1306 at I²C 0x3C — identical controller to the Heltec V2's onboard panel. `display.c`'s STEMMA-bus auto-probe (V2.3.29) already covers SSD1306-compatibles and just works when the panel is plugged in via Qwiic cable. Nothing to detect, no new driver, no new dependency.

### One-line change in `hal.h`

```c
// QT Py branch
- #define HAL_MULTIPAGE_ROTATION  1   // 5-page display task (same as FeatherS3-D)
+ #define HAL_MULTIPAGE_ROTATION  0   // V2.4.8: Heltec-style radiation-only single page
```

`HAL_MULTIPAGE_ROTATION=0` makes `main.c` call `display_running()` per TX cycle (line 413 in main.c) — the Heltec radiation layout: `Xs/m/h    <nSv/h>` header, big 5-digit CPM centred, status line at page 7. No display task spawned; no page rotation. The multi-page code stays compiled out via the existing `#if HAL_MULTIPAGE_ROTATION` guards in `display.c` (lines 526, 639, 685).

### Why this matters

QT Py deployments tend to be smaller / sealed-tube-style installations where the radiation reading is the only thing worth showing on a tiny panel. The 5-page rotation was inherited from the FeatherS3-D dust-sensor build context where there's much more to display (env / PM / noise / uploads / system). For radiation-only QT Py builds, the rotation just hides the one number you actually want.

### Reverting if you change your mind

Flip back to `HAL_MULTIPAGE_ROTATION 1` in `hal.h` and rebuild. The display_task code and per-page render functions are still in the binary (`#if`-gated, not deleted).

### FeatherS3-D / Heltec / 4 MB Heltec — unchanged

Only the QT Py overlay changed. FeatherS3-D keeps its 5-page rotation. Heltec was already on the radiation-only page (and unchanged since V2.0).

### Other

- New reference memory captured for the Adafruit 326 panel — see `reference_adafruit_326_oled_stemma.md` in the auto-memory store. Documents the chip, pinout, plug-and-play story, and the V2.4.8 ship.

---

## V2.4.7

**Hotfix — revert `CONFIG_MBEDTLS_DYNAMIC_BUFFER=y` on Heltec.** V2.4.5 enabled this option for ~15-20 KB of transient heap headroom during TLS handshakes. Bench testing of V2.4.6 (which carried the V2.4.5 change forward) revealed a regression: **every FTPS upload triggers a TLSF heap-corruption panic** on the Heltec V2.

### The crash

```
assert failed: block_next tlsf_block_functions.h:161 (!block_is_last(block))

Backtrace (decoded):
  log_ftp.c:287  io_close()
    → mbedtls_ssl_session_reset()
      → mbedtls_ssl_session_reset_int()
        → mbedtls_ssl_session_reset_msg_layer()  ← frees msg-layer buffers
          → mbedtls_free()  → heap_caps_free()  → tlsf assert
```

Pre-upload heap was healthy (113 KB free / 85 KB min / 96 KB largest) — this is heap **corruption**, not OOM. With dynamic-buffer enabled, the session-reset path's free/alloc pattern changes; `log_ftp.c`'s hand-rolled mbedTLS session lifecycle (the only place we call raw `mbedtls_ssl_*` instead of going through `esp_tls` / `esp_http_client`) doesn't tolerate the new free order — likely a double-free or use-after-free in the message-layer buffer chain.

esp_tls-based clients (HTTPS uploads to Madavi / SC / Radmon / OSM / aqi.eco + MQTT TLS) were unaffected because they don't expose mbedTLS internals.

### Resolution

- Removed `CONFIG_MBEDTLS_DYNAMIC_BUFFER=y` from `sdkconfig.defaults.heltec_v2` + `.heltec_v2_4mb`, replaced with explanatory comment about the FTPS conflict
- Cached per-board sdkconfigs deleted to force re-derivation (per `[[feedback_sdkconfig_defaults_only_fills_missing]]` — defaults only fill missing keys, can't override cached y → not-set)
- FeatherS3-D / QT Py unaffected (they never had this option enabled — V2.4.5 was Heltec-only)

### What stays from V2.4.5

The `HAL_LOG_RING_BYTES` 60 → 45 KB trim (the *other* V2.4.5 change) is independent and unaffected. Heltec V2 keeps the +15 KB permanent heap headroom from that change.

### Heap budget on Heltec V2 after V2.4.7

| Metric | Pre-V2.4.5 | V2.4.5+V2.4.6 | V2.4.7 (now) |
|---|---|---|---|
| Free heap (idle) | ~102 KB | ~117 KB | ~117 KB |
| Min free heap (TLS handshake) | 5.6 KB | ~35-40 KB | ~20 KB |
| FTPS upload | ✅ works | ❌ panics | ✅ works |

V2.4.7 is still 4× better than the pre-V2.4.5 baseline on min-free during TLS (just less generous than the broken V2.4.6).

### Future work

The dynamic-buffer feature could be re-enabled in a future release if `log_ftp.c`'s `io_close` + session-reset path is audited and adjusted to handle dynamic-buffer free semantics. Likely a missing buffer-pointer reset or a double-free trigger. Not urgent — the heap budget is healthy without it.

### Validation status

V2.4.7 fixes the regression. Smoke-test FTPS post-flash; expect Heltec V2 to upload normally without panic.

---

## V2.4.6

**MQTT TLS to broker — three configurable trust modes.** Implements `mqtts://` transport for the publish-only MQTT client introduced in V2.4.2-V2.4.4. The previous releases were plain-MQTT-only (port 1883); this lands the long-deferred TLS support with a user-selectable trust model so users can match their broker setup (public-CA, self-signed, or trusted-LAN).

### Trust modes

| Mode | Use case | Wiring |
|---|---|---|
| **A — Mozilla CA bundle** (default) | Broker fronted by Let's Encrypt or another public CA | `verification.crt_bundle_attach = esp_crt_bundle_attach` (reuses the same bundle already baked in for HTTPS uploads) |
| **B — Custom CA cert** | Self-signed Mosquitto (the common HA add-on case) | User pastes the broker's CA cert PEM into a new `/config` textarea; stored verbatim in NVS as `mqtt_tls_ca`; passed to esp-mqtt via `verification.certificate` |
| **D — Skip server verification** | Trusted-LAN deployments where you want encryption-on-the-wire without cert plumbing | `verification.skip_cert_common_name_check = true`, no cert/bundle attached. TLS still negotiated (anyone-on-LAN-MITM remains a risk; do not use facing the open internet) |

Mode C (server fingerprint pinning) was scoped out — added significant complexity (custom mbedTLS verify-callback wiring) for marginal LAN-broker value where rotating a cert means re-pasting either way.

Defensive: if Mode B is selected but `mqtt_tls_ca` is empty, the client degrades to skip-verify (log warning) rather than connect-looping. The CSRF-protected `/config` POST is the only mutation path.

### Config schema additions (V2.4.6 in `config_fields.def`)

```c
X_BOOL(mqtt_tls_enable,        "mqtt_tls",    false)
X_U32 (mqtt_tls_mode,          "mqtt_tls_m",  0,        0,       2)   // 0=A, 1=B, 2=D
X_STR (mqtt_tls_ca,        CFG_MQTT_CA_CERT_MAX + 1, "mqtt_tls_ca", "")
```

`CFG_MQTT_CA_CERT_MAX = 2400` — sized for typical RSA-4096 self-signed CA (~2.2-2.6 KB PEM) with comfortable headroom.

### Form buffer — per-board sizing (`hal.h`)

`CFG_FORM_BUF_SIZE` was previously a fixed 16 KB in `http_server.c`. Adding the PEM textarea pushed the worst-case rendered form length close to that limit. V2.4.6 moves it to `hal.h` as `HAL_CFG_FORM_BUF_SIZE` per-board:

- **Heltec V2 / V2_4MB:** 16 KB (unchanged — tight internal-DRAM budget, the V2.4.5 dynamic-buffer + log-ring trims gave back enough headroom to absorb the new textarea)
- **FeatherS3-D / QT Py ESP32-PICO:** 32 KB (PSRAM-backed; the transient cost is negligible there)

POST body cap also raised from 4 KB → 12 KB to fit the URL-encoded PEM payload (~5 KB) plus all other form fields (~5-6 KB).

### `/status` row

New `<b>TLS:</b>` line in the existing MQTT block. Shows mode in plain language (`on — Mode B (custom CA cert)` / `off (plain MQTT)` etc.) so the trust posture is visible without opening `/config`.

### Validation status

Implementation complete + builds clean across all four boards. **End-to-end validation against a real broker is deferred** — user does not yet have Mosquitto/HA broker running; that landing will smoke-test all three modes. Code review confirms the esp-mqtt verification fields are the documented API for v6.0; the same `esp_crt_bundle_attach` pattern is in production use in `transmission.c` for HTTPS uploads since V2.3.x.

### Files changed

- `main/version.h` — V2.4.5 → V2.4.6
- `main/config_fields.def` — 3 new MQTT TLS rows
- `main/config.h` — `CFG_MQTT_CA_CERT_MAX` constant
- `main/hal.h` — `HAL_CFG_FORM_BUF_SIZE` per board
- `main/http_server.c` — form rows + format_mqtt TLS line + per-board buffer + POST cap bump
- `main/mqtt.c` — `mqtts://` URI + verification switch on mode
- `CHANGELOG.md` — this entry

---

## V2.4.5

**Heap headroom on the Heltec V2.** Two independent tweaks targeting the constrained-DRAM build. Lifetime min-free-heap on the Heltec V2 was sitting at ~5.6 KB during TLS-handshake transients (against ~100 KB idle) — close to the edge for future feature growth. This release lifts that floor by ~30 KB without changing any externally-visible behaviour.

Also rewords the MQTT LWT description on `/config` (carried over from the post-V2.4.4 commit `e7e467f`) — plain-language explanation of what the broker-published "offline" message means for HA.

### `CONFIG_MBEDTLS_DYNAMIC_BUFFER=y` (Heltec V2 only)

mbedTLS now allocates the SSL IN/OUT record buffers on demand and frees the handshake scratch (~16 KB) once the handshake completes. Previously these sat in heap for the lifetime of every `mbedtls_ssl_context`, even between TX cycles.

- **Heltec V2 transient gain: ~15-20 KB.** Directly addresses the 5.6 KB min-free-heap floor.
- **Scope:** enabled in `sdkconfig.defaults.heltec_v2` and `.heltec_v2_4mb` overlays only — the FeatherS3-D / QT Py have abundant PSRAM heap (4+ MB free) and don't need to pay the per-record alloc/free CPU overhead this introduces.
- **Cost:** small per-record alloc/free CPU overhead, ~4 KB flash. Long-standing IDF kconfig — documented in the "Minimizing RAM Usage" guide. Conservative enable: the more aggressive `DYNAMIC_FREE_CONFIG_DATA` / `DYNAMIC_FREE_CA_CERT` sub-options that interact with the cert bundle are left off.

### `HAL_LOG_RING_BYTES` 60 KB → 45 KB (Heltec V2 only)

The applog ring buffer is the largest single permanent heap allocation on the Heltec V2, living in internal SRAM for the device's lifetime. Trimming 15 KB returns that to free heap without meaningfully degrading the `/log` debug experience (~500 lines → ~380 lines of scrollback — still plenty for diagnosing a TX cycle). FeatherS3-D / QT Py rings live in PSRAM and are unaffected.

### Net effect on Heltec V2

| Metric | Before (V2.4.4) | After (V2.4.5, expected) |
|---|---|---|
| Free heap (idle) | ~102 KB | ~117 KB |
| Min free heap (TLS-handshake transient) | ~5.6 KB | ~35-40 KB |
| `/log` scrollback | ~500 lines | ~380 lines |

### Other

- LWT description on `/config` reworded to a plain-language explanation (carried from post-V2.4.4 commit `e7e467f`).

---

## V2.4.4

**MQTT — Phase 3 (UI integration).** Completes the V2.4.2 → V2.4.4 MQTT phased rollout. Adds `/config` form rows for every MQTT field + a `/status` row showing live broker state. No new MQTT functionality — purely makes the existing client configurable + observable through the web UI rather than requiring NVS-tool access.

### `/config` form rows

New "MQTT (Home Assistant / Mosquitto)" section between "FTP log upload" and "Tick, LED and display". Six inputs:

- `mqtt_en` checkbox — enable / disable the publisher (requires reboot, marked with `*`).
- `mqtt_brk` text — broker hostname or IP. Required.
- `mqtt_port` numeric (text + `inputmode=numeric` to dodge the wheel-decrement trap fixed in V2.3.33). Default 1883.
- `mqtt_user` / `mqtt_pw` — optional creds. Password field uses `type=password` for shoulder-surfing resistance only (the value still lives in plaintext NVS).
- `mqtt_pfx` text — topic prefix used as `<prefix>/<chip>/state`. Default `geiger`. Max 31 chars per `CFG_MQTT_PFX_MAX`.
- `mqtt_ha` checkbox — publish HA Discovery on connect (default on; reboot to apply).

Form POST handling falls through the existing `config_post_apply_field()` X-macro dispatcher (added in V2.4.1 A1) — no per-field special-case code needed.

### `/status` row

New "MQTT" block between Uploads and the page-foot links. Skipped entirely when `mqtt_enable=false` (same convention as the Noise / PM / FTPS blocks). Shows:

- Broker `host:port` (or `(not set)` if empty)
- Live state (green `connected` / red `disconnected`)
- Cumulative `mqtt_publish_count()` since boot
- Topic prefix as `<code>`
- HA Discovery on/off

State read is lock-free from `mqtt.c`'s flag + counter — no I/O from the HTTP-handler context.

### Other

- `CFG_FORM_BUF_SIZE` (16 KB since V2.3.33) absorbs the new section comfortably — actual form is ~6 KB rendered, plenty of headroom.
- Binary growth feathers3_d: +3 KB (1265152 → 1268208).

---

## V2.4.3

**MQTT — Phase 2 (Home Assistant Discovery)** + two latent compile-warning fixes + IDF v6.0 PSA-config plumbing.

### HA Discovery (Phase 2)

Auto-registers each present sensor as a Home Assistant entity the first time the device connects to the broker. No manual HA YAML required.

- New `main/mqtt_discovery.[ch]` builds + publishes one retained QoS-1 config payload per entity on every `MQTT_EVENT_CONNECTED`. Topic format: `homeassistant/sensor/geiger_<chip>/<object_id>/config`.
- Entity catalog covers 24 readings: system (cycles, reconnects, uptime), Geiger (cpm, dose rate, HV pulses — gated on `cfg.tube_enabled`), env (T/H/P), PM (PM1/2.5/4/10 + NC05/1/25/4/10 + typical size), noise (LAeq/min/max), illuminance (VEML7700 OR ALS-PT19).
- Each entity gated by its driver's `*_present()` — Heltec V2 (tube only) gets 6 entities; FeatherS3-D with full sensor stack gets the full ~22.
- HA short-form keys (`uniq_id` not `unique_id`, `stat_t` not `state_topic`, etc.) — ~50 % payload reduction over long form. Pressure value template converts Pa → hPa for cleaner HA charts.
- All entities grouped under one HA device card via the `dev` block (identifiers, name, model, sw_version, manufacturer).
- Republished on every reconnect — broker just overwrites identical retained payloads, so the cost is one MQTT packet per entity per reconnect and zero broker storage growth. Resilient against broker wipe / HA reinstall.
- Gated by the `mqtt_ha_discovery` cfg flag (default true) added in V2.4.2.

### Latent compile-warning fixes (surfaced by CI)

Both were pre-existing — V2.4.2 didn't introduce them, CI's full build log just made them visible:

- `i2c_bus.c:14` — `s_bus_secondary` declared at file scope but only read inside `#if defined(BOARD_FEATHERS3_D)`. Wrapped the declaration in the same `#if` so non-FeatherS3-D builds no longer trip `-Wunused-variable`.
- `display.c:98` — `s_oled_inverted` (anti-burn-in invert toggle) declared inside `#if HAL_HAS_OLED` but only used in the nested `#if HAL_MULTIPAGE_ROTATION` block. Wrapped declaration in `#if HAL_MULTIPAGE_ROTATION` so single-page OLED boards (heltec_v2 / heltec_v2_4mb) compile clean.

### IDF v6.0 PSA-config plumbing

`CONFIG_MBEDTLS_PSA_KEY_SLOT_COUNT=128` in `sdkconfig.defaults` was silently ignored after the IDF v5 → v6 upgrade — the kconfig symbol was removed in mbedtls 4.x (PSA configuration moved into `psa/crypto_config.h`). The kconfig setting still has to be made — restored via `idf_build_set_property(COMPILE_DEFINITIONS "MBEDTLS_PSA_KEY_SLOT_COUNT=128" APPEND)` in the top-level CMakeLists. Pre-existing comment block in `sdkconfig.defaults` left as historical context with a pointer to the new injection site.

Risk this exposed: between the kconfig removal and this fix, the PSA slot count was at the upstream mbedtls 4.x default (likely 32, our load needs ~30 peak). Could have manifested as `PSA_ERROR_INSUFFICIENT_MEMORY` (-141) under concurrent FTPS+HTTPS bursts. We didn't see this in production logs — either headroom was tight-but-OK, or the V2.3.22 TLS bidirectional-close fix reduced peak slot pressure more than estimated.

---

## V2.4.2

**MQTT 3.1.1 publish-only client — Phase 1 (skeleton)** + the post-V2.4.1 CI infrastructure batch shipped together.

### MQTT 3.1.1 publish-only client (Phase 1)

Generic MQTT publish path so a Home Assistant / Mosquitto setup on the LAN can subscribe to live sensor readings. Phases 2 (HA Discovery) and 3 (config UI + status row) follow in V2.4.3 / V2.4.4.

- **Build-system change: first managed component dependency.** ESP-MQTT was removed from bundled IDF components in v6.0 and is now distributed via the IDF Component Manager at `components.espressif.com/components/espressif/mqtt` (per `docs/en/migration-guides/release-6.x/6.0/protocols.rst:151`). Declared in new `main/idf_component.yml` (`espressif/mqtt: "^1.0.0"`). Resolved version (currently 1.0.0) pinned in new `dependencies.lock` at the repo root — **now committed to git** (previously gitignored as a defensive default before any managed components were in use). The fetched component sources land under `managed_components/` (still gitignored — regenerated on every clean build). First build on any new machine / CI runner needs internet access to fetch from the registry. The `mqtt` keyword is intentionally NOT in `main/CMakeLists.txt::REQUIRES` — the Component Manager auto-injects it.
- New `main/mqtt.c` + `main/mqtt.h` wrapping ESP-IDF's `esp-mqtt` (`mqtt_client.h`). Singleton client; one event handler updates a `s_connected` flag and re-publishes the "online" availability message on each reconnect.
- New config fields (X-macro one-liners in `config_fields.def`): `mqtt_enable` (default false), `mqtt_broker`, `mqtt_port` (default 1883), `mqtt_user`, `mqtt_password`, `mqtt_topic_prefix` (default `geiger`), `mqtt_ha_discovery` (default true, used in V2.4.3).
- Two new size constants in `config.h`: `CFG_MQTT_HOST_MAX` (63) and `CFG_MQTT_PFX_MAX` (31).
- Topic layout:
  - `<prefix>/<chip-id>/state` — JSON object, published QoS 0 / retain false once per TX cycle (~150 s).
  - `<prefix>/<chip-id>/availability` — `online` (QoS 1, retain) on connect; LWT `offline` registered so abrupt power loss flips HA to unavailable within keepalive×1.5 s.
- JSON payload skips absent sensors entirely (Geiger-only Heltec build sends just radiation/RSSI/uptime). Fields use the natural HA Discovery attribute names (`pm25`, `env_t`, `usvph`, `noise_laeq`, `lux`) so Phase 2's `value_template` is trivial.
- `mqtt_init()` is a no-op when disabled OR when the broker field is empty — defensive against DNS-bombing the LAN with empty-host lookups if a user enables before configuring.
- Publish is non-blocking; if the broker isn't currently connected the publish is silently dropped (debug log) so MQTT can never disturb the existing Madavi / sensor.community TX path.
- HA Discovery payloads, `/config` UI rows, and `/status` row land in V2.4.3 (Phase 2) and V2.4.4 (Phase 3).
- TLS-to-broker (`mqtts://` on port 8883) deferred — one URI scheme + cert handling away, ~3 h.

**No `/config` UI yet for the new fields** — V2.4.2 just adds the schema + plumbing. To exercise the publish path before V2.4.4, set `mqtt_enable=1` and `mqtt_brk=<host>` via NVS tool, or wait.

### Post-V2.4.1 CI batch (T1 / T3 / T4-lite / valgrind / release.yml)

- (T1) **Host-side unit tests.** Five pure helpers (`safe_strcpy`, `ct_memcmp`, `hex_nibble`, `url_decode`, `html_esc`) moved from inline `static` in `http_server.c` to `static inline` in `main/util.h` so they're host-includable. New `test/test_main.c` with 32 test cases (plain-C, no Unity dependency); new `_test.cmd` Windows runner that checks for gcc and exits with install hints if absent; new `host-test` job in `.github/workflows/build.yml` running on Ubuntu via system gcc. Catches regressions in the small pure-C surface (where most recent bugs lived — B6 url_decode, C1 strncpy). No binary-size change — `static inline` produces equivalent machine code to the old `static` functions.
- (T3) **cppcheck static analysis** in CI. New `static-analysis` job runs cppcheck on `main/` with `--enable=warning,style,performance,portability` and IDF-aware suppressions (`unusedFunction` for callback entry points, `missingIncludeSystem` since we don't ship IDF headers, `unknownMacro` for IDF macros like `MACSTR`). Landed informational, **promoted to a hard CI gate** once round-2 triage took the baseline to 0 findings. New findings now fail the job — fix at source or add an inline `// cppcheck-suppress <id>` comment with a reason. Output also uploaded as a 14-day artifact (`if: always()` so it survives failure too).
- (T4-lite) **Property-based fuzz for `url_decode`.** New `test_url_decode_fuzz_invariants` test runs 10000 random inputs (deterministic seed `0xDEADBEEF` for reproducibility) biased toward `%` and `+` chars, asserts (1) output length ≤ input length, (2) guard bytes immediately before/after the working buffer are untouched (no out-of-bounds write), (3) decoder terminates (implicit via CI timeout). ~50 ms per CI run. Catches the "weird input crashes the parser" class without the libFuzzer ceremony.
- (T-valgrind) **Valgrind sweep on host tests.** `host-test` job now runs the unit-test binary twice: once natively (fast pass/fail), once under `valgrind --leak-check=full --track-origins=yes --error-exitcode=2`. Compiled with `-g -O1` so valgrind has line numbers without the optimiser hiding bugs. Catches leak / use-after-free / uninit-read in any of the 5 helpers for free.
- (CI deprecation) Workflow `env: FORCE_JAVASCRIPT_ACTIONS_TO_NODE24=true` — opts every JS-based action (`checkout`, `upload-artifact`, `esp-idf-ci-action`) into Node.js 24 before the 2026-06-02 forced-default cutoff. Then bumped `actions/checkout@v4 → @v5` and `actions/upload-artifact@v4 → @v5` (both Node-24-native majors) to eliminate the deprecation warnings entirely. `actions/download-artifact@v4` left as-is since release.yml hasn't yet run to surface a warning; bump when/if it does.
- (release automation) New `.github/workflows/release.yml` fires on `git push --tags` for any `V*.*.*` tag. Builds all 4 boards in parallel via `espressif/esp-idf-ci-action`, runs `idf.py merge-bin -o geiger_v2_merged_<board>.bin`, stages 5 artefacts per board with the board-suffixed naming convention, verifies `VERSION_STR` matches the tag (catches "forgot to bump version.h" before the release exists), then a follow-on `release` job downloads all 20 artefacts, extracts the matching section from `CHANGELOG.md` as the release notes, and creates the GitHub Release. Manual `workflow_dispatch` also accepts an existing-tag input for re-runs. Replaces the manual ceremony of 4 local `_build.cmd` + 4 `_merge.cmd` + 20-file `gh release create`.
- (cppcheck baseline triage) First cppcheck run produced 11 findings; cleaned up:
  - `hal.h` `#error "No board defined"` — cppcheck was running with no `-DBOARD_*`, hitting the fallback branch. Added `-DBOARD_HELTEC_V2=1` to the cppcheck invocation (only one branch is checked per run, but all four branches share the same pattern so heltec coverage is sufficient).
  - `ntp.c` two `struct tm *ti = localtime(&t)` sites changed to `const struct tm *ti` — we don't mutate `*ti`.
  - `ntp.c` `sync_cb(struct timeval *tv)` — inline `cppcheck-suppress constParameterCallback`; IDF's `sntp_set_time_sync_notification_cb_t` mandates the non-const signature.
  - `transmission.c` two trailing `n += snprintf(...)` in `build_madavi_env_body` and `build_luftdaten_body` — value was never read after, dropped the `n +=` (just `snprintf(...)`).
  - `transmission.c` five `*_fail_streak` function-statics in `tx_run` — inline `cppcheck-suppress variableScope`; comment explains the visual-grouping vs technically-can-narrow trade-off.
- (cppcheck round-2 triage) Second cppcheck run after the round-1 baseline cleanup produced 7 more findings; cleaned up:
  - `display_serlcd.c` two `char line[32]` declarations at function-top moved inside the `if (snap->pm_valid)` branch (only used there).
  - `display_serlcd.c` three `uint8_t buf[N]` arrays declared `const` (passed to I²C write but never mutated).
  - `http_server.c` `size_t sizes[3]` declared `const` (alongside the existing `const char *segs[3]` peer).
  - `main.c` `MACSTR` cppcheck warning suppressed via workflow-level `--suppress=unknownMacro` (IDF-defined macro, cppcheck has no IDF headers).

---

## V2.4.1

Code-review refactor batch — A1 / A4 / A9 + B1 / B2 / B3 / B4 / B5 / B6 + C1 / C3 / C5 / C6 / C9 + T2. No user-visible functional change.

- (A1) schema-driven config (X-macro) — eliminates 5-way hand-duplication
- (B1) atomic 64-bit timestamps — fixes torn-read on cross-task int64_t
- (A4) single `main_status_snapshot()` — replaces 13 hand-extern'd getters
- (C1) `safe_strcpy()` helper — replaces 8 sites of `strncpy(.., n-1) + null`
- (C6) merged `record_outcome()` — one lock instead of attempt+success pair
- (B4) tube pulse callback write under critical section (ISR consistency)
- (B5) tx_run fail_streak statics documented (single-worker assumption)
- (B6) `url_decode` rejects malformed `%XY` (RFC 3986 strict)
- (A9) deferred-restart is now event-driven (`EV_RESTART` bit, not a poll)
- (B3) red asterisks added to `speaker_tick` / `led_tick` (form contract honest about reboot requirement)
- (C3) buffer-size named constants (`CFG_WIFI_SSID_MAX` etc.) replace magic `[33]` / `[65]` / `[26]`
- (C5) `int total = req->content_len` → `size_t` throughout `/config` and `/update` POST handlers
- (B2) `config_load` logs `ESP_LOGW` on unexpected NVS load failures (e.g. `ESP_ERR_NVS_INVALID_LENGTH` after a downgrade), instead of silently reverting to compile-time defaults
- (T2) GitHub Actions workflow `.github/workflows/build.yml` — builds all 4 boards on every push + PR, verifies `VERSION_STR` is embedded, uploads binaries as artifacts
- (C9) Madavi / sensor.community / Radmon URL constants moved from `main.c::build_tx_context` to `transmission.c` (new `tx_target_configure()` helper). main.c no longer hardcodes endpoint URLs.
- Documentation: this file. `main/version.h` shrunk from 862 LOC to ~20.

**No user-visible functional change.** Same NVS keys, same defaults, same `/config` form behaviour, no NVS migration. Soak for a few cycles after flashing to confirm no regression.

### (A4) Status-accessor collapse

`main.c` previously exposed 13 individual functions (`main_status_cycles()`, `main_status_last_cpm()`, `main_status_env_t()`, etc.), each declared via raw `extern` at the top of `http_server.c` and `display.c`. Replaced with one `main_status_t` struct + `main_status_snapshot(&out)` getter declared in the new `main_status.h` header. Reader gets every field in one call; the `int64_t last_cycle_at` takes the spinlock from B1 for atomic read. Other scalars are still torn-tolerant 32-bit reads (no per-field lock). Snapshot is NOT inter-field consistent — `cpm` could be from cycle N and `env_t` from cycle N+1 — but that's the same behaviour as the old independent extern getters, so no semantic change.

### (C1) safe_strcpy() helper

The `strncpy(dst, src, sizeof(dst) - 1); dst[sizeof(dst) - 1] = 0;` two-liner was repeated across ~8 sites in `main.c`, `log_ftp.c`, and the in-A1 `cfg_assign_str` private. Consolidated into a static-inline `safe_strcpy(dst, src, dstsz)` in the new `util.h` header (BSD strlcpy-style argument order — strlcpy itself isn't available in ESP-IDF's newlib). The `X_STR` macro in `config.c` now expands to a single `safe_strcpy` call. Zero behaviour change — same truncate-and-null semantics.

### (C6) record_outcome() merge

The TX orchestrator called `record_attempt()` then `if (ok) record_success()` per target — two separate spinlock cycles, with a brief window where a concurrent reader could see `attempted` incremented but `succeeded` not yet updated. Merged into one `record_outcome(id, rc, ok)` taking one spinlock for the entire field group. Closes the inconsistency window the B1 spinlock alone couldn't fix.

### (B4) Tube pulse callback under critical section

`tube.c` already guards every ISR-shared variable with `portENTER_CRITICAL_ISR`. The `s_pulse_cb` pointer write was the lone exception — set-once-at-boot + 32-bit-aligned-store-atomicity made it safe in practice, but a post-boot reassignment would have been a latent race. Brought into line with the rest of the file's locking convention.

### (B5) tx_run fail_streak documentation

Function-static counters `madavi_fail_streak` etc. survive across `tx_run` invocations but are safe only because `TX_QUEUE_DEPTH == 1` and a single worker task. Added a comment so a future deeper queue or second worker won't silently reintroduce races.

### (B6) url_decode RFC strictness

Pre-V2.4.1 a malformed `%XY` (XY not two hex digits, or `%` at end of string with insufficient bytes) was passed through as a literal `%` plus whatever followed. Per RFC 3986 §2.1 that's invalid encoding. Now the lone `%` is dropped and the trailing chars walk through as plain bytes — `%G5` → `G5`, `%2` (at end) → `2`. Browsers always encode properly so legitimate POST bodies are byte-identical to before; only hand-crafted / corrupt input changes behaviour, in the direction of cleaner stored config values.

### (A9) Event-driven deferred restart

Pre-V2.4.1, `/config`, `/update` and `/reboot` set a polled `s_restart_requested` flag in `http_server.c`; the main loop checked it every iteration (≤ 1 s because of the wait-cap), so "click Save and restart" → actual reboot took up to a second. Now handlers call `main_request_restart()` from `main_status.h`, which both sets the persistent flag AND sets `EV_RESTART` in the main event group — `xEventGroupWaitBits` returns within FreeRTOS context-switch time (~µs). The 1 s wait-cap stays because the other periodic checks (NTP poll, AP-window timer, STA startup watchdog, FTP loop) still need it. Bonus: deletes `http_server_restart_requested()` public API, no more cross-module polled flag.

### (B1) Atomic 64-bit timestamps — torn-read fix

The Xtensa LX6 (Heltec, QT Py) and LX7 (FeatherS3-D) cores are 32-bit natively. A C-level `int64_t` store compiles to TWO 32-bit stores. A reader on a different task can see the high half of one write and the low half of the next — manifesting as a momentary "year 2038" garbage timestamp on the `/status` page around each TX cycle and FTPS upload. Three i64 fields affected (all wall-clock unix-epoch timestamps for `/status` display):

- `main.c::g_last_cycle_at`     (last TX-cycle timestamp)
- `transmission.c::s_stats[].last_at` (per-target last attempt)
- `log_ftp.c::s_last_at`        (last FTP upload completion)

Each is now guarded by a dedicated `portMUX_TYPE` spinlock around both the write site (single-task writer) and the read site (HTTP server task). The transmission and log_ftp stats locks also wrap the surrounding scalar fields so the reader gets a CONSISTENT snapshot (attempted/succeeded/last_rc/last_at all from one record event) instead of a mixed read across two events. Spinlock cost is ~10ns per access — negligible vs the seconds-scale network operations nearby.

### (A1) Schema-driven config — X-macro refactor

**Motivation.** The `config_t` field set used to be hand-duplicated across SIX sites: struct declaration in `config.h`, defaults block in `config.c`, NVS load loop in `config.c`, NVS save loop in `config.c`, POST pre-clear-bools loop in `http_server.c`, POST per-field dispatch in `http_server.c`. Adding one config field meant touching all six. V2.3.32's work touched five of those sites for trivial renames. Code review flagged this as the single highest-ROI cleanup in the codebase.

**Approach.** New file `main/config_fields.def` declares every field once via X-macros (`X_STR`, `X_BOOL`, `X_U32`, `X_F32`, `X_U8`). The file is included multiple times with different expansions to derive:

- the `config_t` struct in `config.h`
- `config_defaults()` in `config.c`
- `config_load()` NVS-read loop in `config.c`
- `config_save()` NVS-write loop in `config.c`
- `config_post_preclear_bools()` helper in `config.c`
- `config_post_apply_field()` dispatcher in `config.c`

`http_server.c::config_post` is now a thin wrapper that slurps the POST body, calls the schema helpers, and applies the cross-field invariants that DON'T fit a flat schema (wifi_11bg → ht20 link, antenna force-clear, ftp_ps preserve).

**What stays hand-written (deliberately):**

- Boot-time log dump in `config.c` — bespoke grouping + password masking, much more readable than a per-field loop would produce.
- Form rendering in `http_server.c::config_get` — too much custom layout (headings, scripts, antenna gating, brightness dropdown).
- `oled_bright` step validator — schema accepts `[0, 100]`, the POST handler enforces the tighter "0 or 10..100 step 10" inline before the generic dispatch sees the key.

**Form-field rename.** The NTP-1 input was historically `name="ntp1"` while its NVS key is `"ntp"`. To collapse to a single schema column, the form input is now `name="ntp"`. Browser-internal change — no user-visible effect (the form is not deep-linkable).

**Files touched:** `config.h` (rewrite, 47 LOC), `config.c` (rewrite, 235 LOC), `http_server.c` (POST handler shrunk ~150 → ~70 LOC; form field rename; `assign_str` helper deleted as it became unused), `config_fields.def` (new, 100 LOC).

**Adding a config field after this:** ONE line in `config_fields.def` (struct + defaults + NVS load + NVS save + POST pre-clear + POST dispatch all derived). HTML form rendering remains a hand-written `snprintf`-template-arg edit — that's the next refactor opportunity if the form grows further.

### (B3) Red asterisks on speaker/LED tick

`speaker_setup()` reads `play_sound` / `led_tick` / `speaker_tick` by value at boot, but the `/config` form previously had no reboot-required marker on `sp_tick` or `led_tick`. Added the red `*` marker so the form's "live vs reboot" promise is honest. Code unchanged — same boot-time capture as V2.3.x.

### (C3) Buffer-size named constants

Pre-V2.4.1 each string field in `config_t` was declared with an inline magic number (`char wifi_ssid[33];`, `char osm_box_id[26];`, etc.) with the rationale only in adjacent comments. V2.4.1 introduces `CFG_WIFI_SSID_MAX` / `CFG_WIFI_PSK_MAX` / `CFG_OSM_BOX_MAX` / `CFG_TOKEN_MAX` / etc. in `config.h`, derived from real spec/protocol limits (IEEE 802.11 SSID max, WPA2 PSK max, MongoDB ObjectId length, etc.). The `X_STR` entries in `config_fields.def` use `CFG_*_MAX + 1` so the buffer always has room for content + NUL. No size or NVS change.

### (C5) size_t for content_len

`req->content_len` is `size_t` in esp_http_server. Pre-V2.4.1 we assigned to `int total` and used `int received` in the recv loop — silent narrowing on hostile values, sign-mixing in size arithmetic. V2.4.1 holds everything as `size_t`. The OTA partition-size clamp from V2.3.33 means a malicious oversized claim is rejected up front regardless, but the type cleanup removes the sign-mixing class of bug.

### (B2) Loud NVS load failures

Pre-V2.4.1 (and pre-A1) `config_load` called `nvs_get_str` / `nvs_get_u8` / `nvs_get_u32` and silently ignored every non-OK return — the buffer/field kept its compile-time default. Correct behaviour for `ESP_ERR_NVS_NOT_FOUND` (first boot or a newly-added schema field) but surprising for `ESP_ERR_NVS_INVALID_LENGTH` (stored value larger than the current buffer — happens if a future version grew a field and the user then downgrades). New `CFG_LOAD_LOG` helper inside `config_load` emits `ESP_LOGW(TAG, "load 'key': %s (kept default)", esp_err_to_name(r))` on any non-OK / non-NOT_FOUND return. Pure visibility fix — buffer behaviour unchanged (IDF nvs_get_str leaves the buffer untouched on every failure path; verified).

### (T2) CI build matrix on GitHub Actions

New `.github/workflows/build.yml` runs on every push to `main`, every PR, and on manual dispatch. Matrix of 4 board jobs running in parallel (`fail-fast: false` so one breakage doesn't cancel the others). Each job uses `espressif/esp-idf-ci-action@v1` pinned to IDF v6.0 (matching the local dev toolchain). Per-job steps: build via `idf.py -DBOARD=<board> build`, report `geiger_v2.bin` size as a notice + step summary, verify `VERSION_STR` from `main/version.h` is embedded in the binary (catches the V2.3.10/V2.3.11 `App version` cache-skew bug class), upload bootloader/partition-table/ota-data/firmware as a 14-day artifact named `geiger_v2_<board>_<sha>`.

Paths filter: only builds when `main/**`, `CMakeLists.txt`, `sdkconfig.defaults*`, `partitions*.csv`, or `.github/workflows/**` change — README / CHANGELOG edits don't burn CI time. Closes the "broke board X while testing on Y" gap that was previously caught only by manual cross-board builds before each release.

**Compatibility:** no NVS key changes, no sdkconfig changes, no partition changes. OTA-safe from V2.3.33. Binary size delta is in the noise (X-macro expansion produces the same machine code as the old hand-written paths). Programmatic POSTs from outside a browser still require the V2.3.33-introduced `Origin` / `Referer` header.

---

## V2.3.33 — 2026-05-17 — /config truncation + scroll-drift + /log unauth + security hardening

Four unrelated web-UI changes shipping together.

### (A) Silent page truncation under longer field values

*Symptom:* the `/config` page rendered partially on some boards — different devices were cut at different points (one feather stopped after the bold "Reboot" header, the Heltec stopped just before the "Firmware update" link, the other feather rendered fully). Replicable across reboots, fixed config — no transient state involved.

*Root cause:* `config_get` builds the whole page into a single 8 KB heap buffer via one big `snprintf`, then `httpd_resp_send`s it. The static template alone is ~7 KB; once you add field values (escaped SSID, FTP host/user/pw/path, three NTP servers, TZ string, openSenseMap box+token, aqi token), longer configs blow past 8192. `snprintf` silently truncates and returns the would-have-been length. Different boards have different field values → different cut points; same board always cuts at the same place. Latent since V2.3.3 (the buffer was last sized then for the openSenseMap + aqi.eco additions).

*Fix:*

1. `CFG_FORM_BUF_SIZE` 8192 → 16384 in `http_server.c`. Heap-only during the few ms of request handling, freed immediately. Gives ~8 KB headroom over the current worst case, plenty for many more releases of form growth.
2. New `ESP_LOGE` after the `snprintf` if `n >= CFG_FORM_BUF_SIZE` so any future truncation surfaces in serial instead of producing another batch of silently-broken pages. Also clamps the send length to the buffer so we never read past it.

*Why not chunked sends:* the natural "robust" fix would be to convert `config_get` to chunked `httpd_resp_send_chunk` like `status_get` already does, but the cost/benefit didn't justify it: ~100 LOC of mechanical refactor across a critical UI page, multiplied retest matrix, and the actual problem here was "we had no alarm on truncation" — fixed by the one-line `ESP_LOGE`. The 16 KB ceiling is plenty until form growth or a future fragmentation issue forces the hand.

### (B) Number inputs converted to text-with-inputmode to disable the wheel-decrements-value trap

*Symptom:* `station_altitude_m` values drifting by small multiples of the field's `step` between saves — Heltec set to 63.0 m reading back as 62.8 m (2 × 0.1 m), prior incident on 66.0 m → 65.8 m. Same UX trap caused, but never identified for years.

*Root cause:* `<input type="number">` on both Chrome and Firefox treats the focused element as a mouse-wheel target — scrolling decrements or increments by `step`. User edits the field, finishes typing, then scrolls the page to reach the Save button; the focused number input eats the wheel events, value silently ticks down, Save persists the wrong value. Affected the three number fields on the form (`alt_m`, `ftp_int`, `tx_int_ms`).

*Fix:* converted all three to `type="text" inputmode="decimal|numeric"`. No spinner arrows; no wheel-value behaviour; mobile keyboards still pop the numeric/decimal layout via the `inputmode` hint. Loses HTML5 `min`/`max` live validation, but the POST handler already enforces the same bounds server-side (alt_m `[-500, 9000]`, ftp_int `[1, 1440]`, tx_int_ms `[10000, 3600000]`) — so functionally identical, just no in-browser red outline.

### (C) /log no longer auth-gated

One-click view from the unauth'd `/status` page. The ring buffer is diagnostic output (boot banner, WiFi/upload status, sensor cycle summaries) — same class of information the device already publishes to Madavi / sensor.community / Grafana publicly, so the password prompt added friction without protecting anything sensitive. `/config`, `/update`, `/reboot` remain password-protected.

### (D) Web-UI security hardening — 4 fixes from the /config + /update audit

All in `http_server.c`. Closes the realistic attack surface for a LAN sensor without pulling in HTTPS / signed-boot / NVS encryption (deferred: see audit notes).

1. **CSRF protection on POST handlers.** Basic-auth credentials are cached per-origin by browsers and auto-attached to any subsequent same-origin request — including cross-origin form POSTs from a malicious page the admin happens to visit in the same browser session. Without protection, attacker.com can submit `<form action="http://device/config" method=POST>` with arbitrary fields and the browser attaches cached `Authorization`. New `check_same_origin()` helper requires the request's `Origin` header (or `Referer` as fallback) to match `Host`. Applied to `/config`, `/update`, `/reboot` POST handlers. **Programmatic clients (curl/scripts) need `-H "Origin: http://<device>:<port>"`.**
2. **Constant-time credential compare.** `check_auth` previously used `strcmp` to compare the received Authorization header against the expected base64, which short-circuits on the first differing byte — leaks position via response-time variance, enables byte-at-a-time brute force on a timing-attack-capable adversary. New `ct_memcmp()` inline helper runs over the full buffer regardless of position of the mismatch. Length check is still early-exit (length is non-secret).
3. **OTA `content_len` clamp to partition size.** `update_post` previously trusted the client-claimed `content_len` as the recv loop bound. Auth'd attacker could claim a giant size and dribble bytes (slowloris) to hold the OTA partition open. Now rejected with HTTP 400 before any erase happens.
4. **`X-Frame-Options: DENY` on /config, /update, /reboot.** Free clickjacking protection — blocks framing from any origin (the device never frames itself either). New `set_security_headers()` helper called on success-response paths.

**Deferred** (called out in audit, not addressed in this release): HTTPS migration (LAN-only, not worth heap+UX cost), signed-app OTA (production-hardening, key-management overhead), flash + NVS encryption (same), auth-failure rate limit / lockout (mostly informational without HTTPS).

**Compatibility:** no NVS key changes, no sdkconfig changes, no partition changes. OTA-safe from V2.3.32. +8 KB transient heap during `/config` render only. Programmatic POSTs from outside a browser now require an explicit `Origin` or `Referer` header.

---

## V2.3.32 — 2026-05-15 — config-page polish + status-page Madavi link + display OFF

Three small UX changes — all in `http_server.c` + `display.c`:

1. Config page label rename. "Drive OLED display" → "Enable Display". "OLED brightness" → "Display brightness". The OLED-only labels were misleading on FeatherS3-D + Heltec where the field also drives the SerLCD backlight (V2.3.28 added SerLCD support but didn't relabel the form). NVS key stays `oled_bright` — no migration needed.
2. Brightness dropdown gains an "OFF" entry (value 0). Lets the user put the display fully dark without unchecking "Enable Display" (which would also disable the multi-page render task). On OLED, `0xAE` puts the panel into sleep mode — segment + common drivers off, charge pump retained, RAM contents preserved → instant re-enable on the next non-zero brightness write. On SerLCD, the RGB backlight goes to (0,0,0); the LCD glass is still being driven so a strong external light would show faint text, but in a sealed-tube deployment that's invisible. `display_set_contrast()` previously clamped `pct < 10` to 10; that clamp is removed so 0 reaches the backend.
3. Status page: per-chip Madavi link in the bottom links block. Only emitted when `send_madavi=1` in `/config` — no point linking to a graph page with no data behind it. Resolved at HTML render time (no JavaScript), so toggling Madavi in `/config` only changes the link's visibility on the next status-page load. URL pattern: `api-rrd.madavi.de:3000/grafana/d/q87EBfWGk/temperature-humidity-pressure?var-chipID={chip}`.

**Compatibility:** no NVS key changes, no sdkconfig changes, no partition changes. OTA-safe from V2.3.31.

---

## V2.3.31 — 2026-05-15 — fix sub-tick `vTaskDelay` timing in I²C drivers (SHT45 H=0% root cause)

**Headline:** at the ESP-IDF default `CONFIG_FREERTOS_HZ=100` (10 ms tick), `vTaskDelay(pdMS_TO_TICKS(N))` for N ≤ 10 evaluates to `vTaskDelay(1)` — a 1-tick yield that actually sleeps **0..10 ms** depending on the call's phase relative to the next tick boundary. Several driver post-command waits assumed millisecond precision and were silently shorter than the chip's conversion / response time, causing intermittent failures that looked like flaky hardware.

Most visible symptom: SHT45 returning H=0.00% with valid T (cycles 1358 + 1409 on esp32-5965048), or post-measure NACK `ESP_ERR_INVALID_RESPONSE`. SHT45 measures T first then H; if the read happens at 4-9 ms the T register is fresh but the H register is still 0x0000, with CRC byte 0x81 (CRC-8 of `00 00`) which **passes** the integrity check. Looked like a chip fault for V2.3.0..V2.3.30. The other SHT45 in the user's stock had the same behaviour, just less often → diagnostic clue that proved it wasn't a chip fault.

**Fix pattern:** for sub-20 ms timing-critical waits, replace `vTaskDelay(pdMS_TO_TICKS(N))` with `esp_rom_delay_us(N * 1000)` — a precise busy-wait via the system RTC.

**Files touched:**

1. `sht45.c` (4 sites) — post-measurement wait in `sht45_read()` (THE cycle-bug fix), post-measurement wait in `try_init_pass()`, serial-read wait in `sht45_read_serial()`, post-soft-reset wait in `try_init_pass()`.
2. `bmp581.c` (1 site) — forced-mode 12 ms post-conversion wait. Chip needs 11.4 ms; 1-tick `vTaskDelay` was 0..10 ms = below spec.
3. `bme280.c` (1 site) — bumped 55 → 70 ms target on post-measurement wait.
4. `sps30.c` (4 sites) — Sensirion-style 5 ms inter-command waits in serial read, data-ready poll, measurement read, status read.
5. `veml7700.c` (1 site) — post-wake 5 ms wait. Datasheet tWAKE = 2.5 ms.

OTA-safe from V2.3.30.

---

## V2.3.30 — 2026-05-15 — sensor serials at boot + VEML7700 ambient-light driver

Small additive release — diagnostic-friendly serial-number logging for two sensors that have factory-burned unique IDs, plus a new I²C ambient-light sensor driver. No architecture changes, no removed APIs. OTA-safe from V2.3.29.

1. **SHT45 serial logged at init** (`sht45.c`) — read via cmd 0x89, logged as 8-char hex.
2. **SPS30 serial logged at init** (`sps30.c`) — read via cmd 0xD033, 32-char ASCII.
3. **New Vishay VEML7700 driver** (`veml7700.c/.h`) — I²C ambient-light sensor at fixed address 0x10. Auto-detected via the existing `PROBE_ON_BOTH_BUSES` chain. Returns lux (computed from raw ALS × resolution with polynomial non-linearity correction), raw 16-bit ALS count, raw 16-bit white-channel count.
4. **`/status` "Ambient light" block extended** to render BOTH sensors when present — VEML7700 (lux + raw ALS + raw white) and ALS-PT19 (mV + approx lux).

---

## V2.3.29 — 2026-05-15 — multi-page display + dual-bus auto-detect + brightness control + ALS-PT19 + i2c_bus refactor

Major V2 architectural cleanup plus four user-facing additions, all OTA-safe from V2.3.28:

1. **Multi-page display rotation** (5 pages, 7 s each) on FeatherS3-D and QT Py — replaces V2.3.28's single Env page. Single binary auto-detects which display is fitted (SerLCD at 0x72 OR SSD1309/6 at 0x3C) AND which I²C bus it lives on. Page set adapts to the sensors actually present.
2. **Dual-bus auto-detect** for both displays AND sensors. Probes STEMMA1 first, falls through to STEMMA2 (FeatherS3-D only). LDO2 is enabled lazily and torn down at end-of-init if no consumer bound to STEMMA2 — saves ~5–10 mA quiescent + NeoPixel idle.
3. **OLED contrast / SerLCD backlight brightness dropdown** in `/config` — 10 % steps (10 % to 100 %). Live-applied on Save (no reboot). Default 80 % matches the V2.3.28 hardcoded register.
4. **Onboard ALS-PT19 ambient-light sensor** exposed (FeatherS3-D only — GPIO 4 = ADC1_CH3). New `/status` row "Ambient light: 245 mV (~127 lux, indoor lit)". Two-layer calibrated (eFuse mV + nominal 1.6 mV/lux, ±50 % typical accuracy).

**Pages:** Env (T/RH/P + Noise) / PM Mass / PM Number / Uploads / System.

**Architectural changes:** new `i2c_bus.c/.h` module owning both buses (lazy + sheddable secondary), `env_sensor_init(bus)` API change, `PROBE_ON_BOTH_BUSES` macro in main.c, new `HAL_MULTIPAGE_ROTATION` + `HAL_HAS_ALS` flags, `display_snapshot_t` for cross-task push, new `display_task` FreeRTOS task, OLED invert anti-burn-in, `display_set_contrast()` backend-aware live brightness, `main_target_enabled()` accessor.

**Madavi compatibility fix:** sentinel-based field selection in `build_madavi_env_body()` — emits full `BME280_*` trio only when pressure is real (>1 hPa); falls back to DHT-style "temperature"/"humidity" for SHT45-only setups, instead of sending fake `BME280_pressure: 0.00`.

OTA-safe from V2.3.28.

---

## V2.3.28 — 2026-05-13 — external SSD1309 OLED on FeatherS3-D STEMMA2 (MVP, single page)

Add support for an external 2.42" SSD1309 128x64 OLED (Core Electronics CE09964) on the FeatherS3-D's SECOND STEMMA QT connector (STEMMA2 — IO15 SCL / IO16 SDA, powered from LDO2 / 3V3.2). FeatherS3-D ONLY; Heltec / QT Py builds bit-identical to V2.3.27.

**What's new:**

1. `hal.h` FeatherS3-D: `HAL_HAS_OLED` 0 → 1.
2. `display.c::display_setup()`: brings up STEMMA2 (IO39 HIGH = LDO2 enable + second I²C bus on I2C_NUM_1). Reset block now `#ifdef PIN_OLED_RESET` (external breakout has no reset line). SSD1306 init sequence reused (SSD1309 is register-compatible).
3. New `display_environment()` renders a single-page T/RH/P (+ optional LAeq) view.

**NOT in this release** (deferred): sensors on STEMMA2, page rotation, radiation page on FeatherS3-D.

**Side effect:** enabling LDO2 also powers the onboard WS2812 NeoPixel on IO40. We never drive its data line, so internal POR keeps it dark.

OTA-safe from V2.3.27.

---

## V2.3.27 — 2026-05-13 — FeatherS3-D pin map: HV_FET + speaker moved off A2..A4

PCB harness rev: HV_FET_OUT moved from A2 (IO14) to A5 (IO5); SPEAKER_P moved from A3 (IO12) to D10 (IO3); SPEAKER_N moved from A4 (IO6) to D9 (IO1). Frees the contiguous A2..A4 trio for future analog expansion.

Notes: IO3 (now PIN_SPEAKER_P) is an ESP32-S3 boot strap (JTAG vs USB-Serial-JTAG select). Chip's internal pull-up holds it HIGH at boot → default USB-Serial-JTAG mode. We only drive it post-boot in `speaker_setup()`, so the strap reads correctly.

FeatherS3-D ONLY. FeatherS3-D installs MUST be flashed BEFORE re-attaching the new PCB harness.

OTA-safe from V2.3.26.

---

## V2.3.26 — 2026-05-13 — env-sensor diagnostics (no behaviour change)

Make a flaky SHT45 (or any silently-failing env sensor) visible in the logs.

1. `sht45_read()`'s two silent I²C error paths now log `ESP_LOGW` mirroring the init path's verbosity. Triggered by esp32-176432: SHT45 ACK'd at init then on every cycle reported H=0.00 % silently → cascade quietly fell through to BMP390.
2. `env_sensor_read()` gained an optional `(char *raw_log, size_t cap)` tail so the caller sees per-sensor reads alongside the fused result. `main.c` cycle log line now reads e.g.: `SHT45: T=18.86°C  H=0.00%, BMP390: T=18.88°C P=1026.60hPa  SHT45+BMP390: T=18.86°C  H=0.00%  P=1026.60hPa`.

No I²C transaction changes, same fallback cascade, OTA-safe from V2.3.25.

---

## V2.3.25 — 2026-05-12 — aqi.eco compatibility fix + body trim

**Headline fix:** aqi.eco's `devices.esp8266_id` column is `bigint`. We were emitting `"esp8266id": "esp32-5965048"` (string). MySQL string→bigint coerce failed, PHP unhandled exception, nginx 500. Fix: in `build_luftdaten_body`, when `prefix_aqi_id == true`, strip the leading `"esp32-"` before emitting the `esp8266id` field.

**Other aqi.eco body trims:** drop `Si22G_*` radiation block (no destination column), drop `SPS30_TS`, drop `DNMS_noise_LA_min`/`_max` — only LAeq is mapped.

**NAMF spray-and-pray additions:** add `SHT3X_temperature`/`SHT3X_humidity` alongside `BME280_*`, add `BMP_pressure` alongside `BME280_pressure`. Same numeric value, multiple aliases — server picks whichever it prefers.

Net body ~30 % smaller per aqi.eco POST. openSenseMap path unchanged. OTA-safe from V2.3.24.

---

## V2.3.24 — 2026-05-12 — the wrap-corruption fix + UX polish

**Headline fix:** the V2.3.16-era streaming snapshot in `applog_stream_begin` had a wrap-specific pointer aliasing bug. In the wrapped case seg_a started at exactly where the writer's next `ring_append()` lands — releasing the mutex exposed the segment to in-flight overwrites. Visible from V2.3.16 onwards as torn lines at the head of every uploaded FTPS file. Only manifested on Heltec because PSRAM boards' giant 4 MB ring barely wraps in production lifetime.

Fix: `applog_stream_begin` now copies the danger zone (first `HAL_LOG_SNAP_SCRATCH_BYTES`) into a pre-allocated scratch buffer under the mutex; callers stream `scratch + ring-tail-remainder + newer-pre-wrap-half`. Per-board scratch sizing: Heltec 4 KB internal DRAM, FeatherS3-D/QT Py 16 KB PSRAM.

**FTPS-internal log lines downgraded INFO→DEBUG.** Six lines per upload (~660 B).

**Config page UX rework — two submit buttons:** Save (no reboot, live-apply) vs Save and restart (V2.3.23 behaviour). Reboot-required fields marked with red `*` + legend.

OTA-safe from V2.3.23.

---

## V2.3.23 — 2026-05-11 — closes out the V2.3.5 → V2.3.22 FTPS+TLS 1.3 arc + slow-fragmentation prevention

1. **`ftp_tls12_only` default flipped from true to false.** TLS 1.3 is now the FTPS default after V2.3.22's bidirectional-close fix landed and was production-validated. Existing devices upgrading keep their saved value via NVS — only fresh / NVS-erased devices get the new default.
2. **Scheduled PSA crypto refresh every 24h.** Third trigger for the existing PSA-reset machinery (already fires on 5 consecutive OOMs and on TCP write stalls). Time-based — flushes slow heap fragmentation across hundreds of TLS handshakes. Implemented in `log_ftp.c::log_ftp_loop` BEFORE the `ftp_enabled` early return so it fires on every device. Gated on `tx_is_idle()`.

OTA-safe from V2.3.22.

---

## V2.3.22 — 2026-05-11 — Fix FTPS+TLS 1.3 "426 Connection reset" (bidirectional close)

## V2.3.21 — 2026-05-11 — V1 boot melody + OLED TX-status wiring (V1 parity)

## V2.3.20 — 2026-05-11 — 4th board target (Adafruit QT Py ESP32-PICO) + NeoPixel driver

## V2.3.19 — 2026-05-10 — Fix FTPS+TLS 1.3 "426 Connection reset" (close_notify flush)

## V2.3.18 — 2026-05-10 — Status page expansion + PSRAM 4MB ring + FTP log polish

## V2.3.17 — 2026-05-10 — stream /log + min_free in TX log

## V2.3.16 — 2026-05-10 — streaming FTPS + BMP390 cal fix + 4MB partition + polish

## V2.3.15 — 2026-05-10 — TLS 1.3 NewSessionTicket fix + SPS30 SC body fix + PSA infrastructure + 4MB knock-off support

## V2.3.14 — 2026-05-09 — OTA upload form: board-specific prompt with bold-red board name

## V2.3.13 — 2026-05-09 — OTA target-board chip-ID validation

## V2.3.12 — 2026-05-09 — BMP3xx + BMP5xx 10-sample priming after init

## V2.3.11 — 2026-05-09 — SHT45 init wait fix + CMakeLists PROJECT_VER auto-rebuild

## V2.3.10 — 2026-05-09 — FTPS TLS 1.3 regression fix + diagnostic logging

## V2.3.9 — 2026-05-09 — sensor.community SPS30 X-PIN fix (12 → 1)

## V2.3.8 — 2026-05-09 — Madavi SPS30 visibility hack (SDS_P1 / SDS_P2 alias)

## V2.3.7 — 2026-05-08 — DNMS noise sensor support (LAeq/min/max)

## V2.3.6 — 2026-05-08 — BMP581 pressure sensor driver

## V2.3.5 — 2026-05-08 — TLS 1.3 + session ticket support + mbedTLS -O2

## V2.3.4 — 2026-05-08 — SPS30 device-status surfacing

## V2.3.3 — 2026-05-08 — openSenseMap and aqi.eco upload targets

## V2.3.2 — 2026-05-08 — SPS30 → Madavi + sensor.community

## V2.3.1 — 2026-05-08 — Sensirion SPS30 driver

## V2.3.0 — 2026-05-08 — Tube optional

## V2.2.1 — 2026-05-04 — FeatherS3-D bring-up prep

## V2.2.0 — 2026-05-03 — dual-board scaffolding (Heltec WiFi Kit 32 V2 + UM FeatherS3-D)

## V2.1.22 — 2026-05-02

## V2.1.21 — 2026-05-01

## V2.1.20 — 2026-04-29

## V2.1.19 — 2026-04-26

## V2.1.18 — 2026-04-26

## V2.1.17 — 2026-04-25

## V2.1.16 — 2026-04-23 — FTP WiFi PS override + retry on failure

## V2.1.15 — 2026-04-23 — SHT45 + BMP390 sensor support

## V2.1.14 — 2026-04-23

## V2.1.13 — 2026-04-23

## V2.1.12 — 2026-04-23

## V2.1.11 — 2026-04-22 — first public release
