# MultiGeiger-V2 — Changelog

Per-release WHAT/WHY notes. Extracted from `main/version.h` in **V2.4.1** so the release archaeology doesn't bloat every build's parse cycle.

- **V2.4.1 down to V2.3.23**: full notes preserved from the pre-V2.4.1 `version.h` header.
- **V2.3.22 down to V2.1.11**: one-line headlines only — full bodies live in the GitHub release for each tag at <https://github.com/MMBytes/MultiGeiger-V2/releases>.

For build / flash / release workflow see `README.md` and the `_build.cmd` / `_merge.cmd` / `_flash.cmd` helpers at the repo root.

---

## Unreleased

Accumulating for the next tag. Bump + ship when ready.

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
