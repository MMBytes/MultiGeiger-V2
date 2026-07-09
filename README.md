# MultiGeiger V2 (ESP-IDF native)

A ground-up C rewrite of the [MultiGeiger](https://github.com/ecocurious2/MultiGeiger) radiation sensor firmware, ported from Arduino / PlatformIO to **native ESP-IDF 6.0**. Runs on **seven** ESP32 / ESP32-S3 board variants with a wide selection of optional environmental, particulate, noise, and ambient-light sensors. Uploads to **nine** public back-ends and publishes to MQTT (with Home Assistant Discovery) and remote syslog.

See the [releases page](https://github.com/MMBytes/MultiGeiger-V2/releases) for the latest build and per-release notes.

## What it does

- Counts Geiger pulses and computes CPM and µSv/h with a **runtime-selectable tube type** (Si22G / SBM-20 / SBM-19 / Unknown) configured on `/config` — no rebuild required.
- Drives the HV flyback boost converter from on-chip GPIO timing (no Arduino dependency).
- Optional **environmental sensing**: temperature, humidity, pressure, sea-level-adjusted pressure.
- Optional **PM (particulate matter)**: PM1 / PM2.5 / PM4 / PM10 plus number concentrations and typical particle size.
- Optional **noise**: LAeq / LAmin / LAmax dB(A).
- Optional **ambient light**: lux (two sensor families supported).
- Optional **GNSS position display** — auto-detected PA1010D or MAX-M10S receiver shows fix, satellites, HDOP, lat/lon (OpenStreetMap link), and altitude on `/status`; display-only (NTP is the sole time source).
- Per-cycle upload (default 150 s, configurable 10 s – 1 h) to any subset of **nine** public back-ends.
- **MQTT publish** with 24 Home Assistant Discovery entities, three TLS modes including custom CA.
- **Remote syslog** (UDP 514, RFC 5424) for centralised log aggregation.
- **Hourly FTP/FTPS** upload of the on-device log ring buffer.
- **CPM history graph** on `/status` — 60-minute and 24-hour inline SVG with gridlines and a `HH:MM` time axis; rolling **cpm5 / cpm15** 5- and 15-minute averages fed to GMCMap and ThingSpeak.
- **Web UI** at `http://<device>/` — `/config` (settings), `/status` (live metrics + per-target TX stats + per-sensor presence + CPM graph), `/update` (OTA), `/log` (in-memory log ring).
- **Crash recovery**: ESP-IDF panic handler writes a coredump to a dedicated flash partition; downloadable post-reboot via `GET /coredump.elf` (no USB required).
- **OLED status display** (SSD1306 128×64, also supports SparkFun SerLCD I²C and NeoPixel tick on supported boards).
- **NTP** with up to three configurable servers and POSIX TZ string.

## Supported hardware

### Boards (seven build targets)

| Build target | MCU / module | Flash | Notes |
|---|---|---|---|
| `heltec_v2` | Heltec WiFi Kit 32 V2 (ESP32-D0WDQ6) | 8 MB | Onboard SSD1306 OLED. The original target — most production deployments. |
| `heltec_v2_4mb` | Heltec WiFi Kit 32 V2 clone | 4 MB | Same module silicon, smaller flash. Tight on heap during OTA — see V2.4.13 teardown logic in `main.c`. |
| `feathers3_d` | Unexpected Maker FeatherS3 with display (ESP32-S3) | 8 MB | Two STEMMA QT connectors (STEMMA1 on IO8/IO9, STEMMA2 LDO-gated on IO15/IO16). External I²C OLED via STEMMA. PSRAM — WiFi/lwIP/mbedTLS offloaded to PSRAM for sustained heap headroom. |
| `adafruit_qtpy_esp32_pico` | Adafruit QT Py ESP32-PICO | 8 MB | Compact form factor. Optional NeoPixel tick on pulse. PSRAM. |
| `seeed_xiao_esp32s3` | Seeed Studio XIAO ESP32-S3 | 8 MB | Tiny 21×17.5 mm — shares QT Py form factor + Geiger pin map (A0 / A1 / SCK), so one PCB design works for both. Onboard user LED blinks per pulse. PSRAM. |
| `heltec_wifi_lora32_v4_r2` | Heltec WiFi LoRa 32 V4 — base/R2 variant (ESP32-S3R2) | 16 MB | Third-party PCB: the standard Multigeiger V2 mainboard populated with this module instead of the Heltec V2. Onboard SSD1315 OLED on a dedicated, always-on I²C bus separate from the env-sensor bus. 2 MB in-package PSRAM. GPIO 7-14 reserved (undriven) for future LoRaWAN/Meshtastic work — not implemented yet. |
| `sparkfun_thing_plus_esp32s3` | SparkFun Thing Plus ESP32-S3, WRL-24408 (ESP32-S3-MINI-1) | 4 MB | Second-source MCU for the FeatherS3-D-format carrier PCB — same Feather footprint, drop-in alternative to `feathers3_d` on the same board. External I²C OLED via Qwiic, onboard MAX17048 fuel gauge, onboard WS2812 NeoPixel (always-on rail, no power-gate GPIO). 2 MB in-package PSRAM. |

Build/flash invocation takes a board argument — see `_build.cmd` / `_flash.cmd` / `_merge.cmd` helpers. All boards share the same `main/` source tree; differences are isolated in per-board `sdkconfig.defaults.<board>` and HAL pin map. PSRAM boards additionally include `sdkconfig.defaults.psram` (WiFi roaming app + PSRAM offload knobs).

### Geiger tube

The tube type is a runtime **`/config` dropdown** — no rebuild required to switch tubes:

| Option | cps → µSv/h | Notes |
|---|---|---|
| **Unknown** | — (dose suppressed) | CPM reported; µSv/h field omitted from all uploads |
| **SBM-20** | 1 / 2.47 | Energy-compensated cylindrical tube |
| **SBM-19** | 1 / 9.81888 | Large pancake, lower background |
| **Si22G** | 1 / 12.2792 | **Default** — empirical vs. odlinfo.bfs.de reference |

Calibration factors from the upstream MultiGeiger (`ecocurious2 / t-pi tube.cpp`). The selected tube is shown on the `/status` Radiation card and in the boot config dump. Changing it in `/config` applies live (no reboot). CPM counting, HV timing, and dead-time gating are all tube-agnostic.

### Environmental sensors (auto-detected on I²C)

| Sensor | Address | Provides | Notes |
|---|---|---|---|
| **SHT45** | 0x44 | Temperature, humidity | Sensirion successor to SHT3x. PTFE filter variant supported. |
| **BME280** | 0x76 / 0x77 | Temperature, humidity, pressure | Bosch — original upstream sensor. |
| **BME688** | 0x76 / 0x77 | Temperature, humidity, pressure | Bosch — gas channel currently unused. |
| **BMP390** | 0x76 / 0x77 | Temperature, pressure | Bosch — high-accuracy barometric. |
| **BMP581** | 0x46 / 0x47 | Temperature, pressure | Bosch latest-gen — sub-Pa noise floor. |

The driver mix lets a node combine, e.g., SHT45 (best RH) + BMP581 (best pressure) on the same bus. The firmware picks the right reading per measurand from whichever sensors responded at boot.

### Specialty sensors (auto-detected on I²C)

| Sensor | Address | Provides | Notes |
|---|---|---|---|
| **Sensirion SPS30** | 0x69 | PM1 / PM2.5 / PM4 / PM10 mass, N05–N10 number concentrations, typical particle size | Adds the SPS30_* field group to all upload targets. |
| **hbitter DNMS** | 0x55 | LAeq / LAmin / LAmax dB(A) | Nettigo NAM-style Teensy-based noise module. Adds DNMS_noise_* fields. |
| **ALS-PT19** | (analog) | Ambient light (lux) | Onboard FeatherS3-D photodiode on ADC1_CH3. |
| **VEML7700** | 0x10 | Ambient light (lux) | Vishay I²C, alternative to ALS-PT19. |
| **PA1010D** | 0x10 | GNSS position + time (display-only) | Adafruit 4415 (MediaTek MT3333). Detected by live NMEA sniff — disambiguates from a VEML7700 at the same address. |
| **MAX-M10S** | 0x42 | GNSS position + time (display-only) | SparkFun Qwiic (u-blox DDC). Factory chip serial queried via UBX-SEC-UNIQID and shown on `/status`. |

### Displays

- **SSD1306 OLED** 128×64, I²C 0x3C or 0x3D — auto-probe both addresses (V2.4.19).
- **SparkFun SerLCD** (I²C variant) — alternative line-based display.
- **NeoPixel** tick-per-pulse on supported boards (configurable colour / brightness via `display_mode`).
- Built-in **speaker** click on pulse (configurable, off by default for sealed-tube installs).

## Upload targets

All **nine** targets are fully optional and independently configurable. Each has per-target circuit-breaker logic (V2.3.x) — 3 consecutive all-retry failures suspend that target for 20 cycles to prevent failed TLS handshakes from fragmenting heap.

| Target | Endpoint | Auth | Fields posted |
|---|---|---|---|
| **Madavi** | `api-rrd.madavi.de/data.php` (HTTP / HTTPS) | none | T/H/P; PM and noise are POSTed but the legacy server only graphs T/H/P |
| **sensor.community** | `api.sensor.community/v1/push-sensor-data/` (HTTP / HTTPS) | X-Sensor = chip ID | Radiation (X-PIN 19), T/H/P (11), PM (1), noise (15) — split bodies per pin |
| **Radmon** | `radmon.org/radmon.php?function=submit` (HTTP / HTTPS) | basic auth user/pw | Radiation CPM only |
| **openSenseMap** | `ingress.opensensemap.org/boxes/<BOX_ID>/data?luftdaten=1` (HTTPS) | Box ID in URL + optional raw access token (no `Bearer ` prefix) | Combined Luftdaten body |
| **aqi.eco** | `api.aqi.eco/update/<TOKEN>` (HTTPS) | token in URL | Combined Luftdaten body with `esp8266id` field |
| **GMCMap** | `gmcmap.com/log2.asp` (HTTP only) | Account ID + Geiger counter ID in URL | Radiation CPM + ACPM (5-min rolling average). Skipped when tube disabled. |
| **ThingSpeak** | `api.thingspeak.com/update` (HTTP / HTTPS) | Write API key | field1=CPM, field2=µSv/h, field3=cpm5, field4=cpm15, field5–7=T/H/P. Skipped when tube disabled. |
| **ThingSpeak PM** | `api.thingspeak.com/update` (HTTP / HTTPS) | Separate write API key | field1–4=PM1/PM2.5/PM4/PM10, field5–7=T/H/P, field8=typical particle size. Skipped when no PM sensor. |
| **openSenseMap STAGING** | `upload.staging.opensensemap.org/boxes/<BOX_ID>/data?luftdaten=1` (HTTPS only) | Independent Box ID + access token | Same Luftdaten body as production OSM — lets a node mirror to prod + staging simultaneously. |

Each target shows live per-target stats on `/status` — attempted / succeeded counters, last HTTP status, last attempt time, current breaker state.

Cert verification uses the Mozilla CA bundle baked into flash. Per-target "insecure mode" available for self-hosted instances. TLS 1.2 and TLS 1.3 both supported.

## Observability

### MQTT (V2.4.2–V2.4.6)

Publish-only MQTT 3.1.1 client with three TLS modes:

- **Mode 0** — plain TCP (broker on LAN)
- **Mode 1** — TLS via Mozilla CA bundle (public broker)
- **Mode 2** — TLS via user-supplied custom CA PEM (self-signed broker, e.g. a Pi running Mosquitto on the LAN)

When `mqtt_ha_discovery` is enabled (default), the firmware publishes 24 Home Assistant Discovery entities at boot — radiation, all detected sensors, WiFi RSSI, per-target TX stats, etc. — auto-gated on driver presence. HA picks them up without manual configuration.

Topic layout: `<prefix>/<chip-id>/state` (full state JSON every cycle) plus `<prefix>/<chip-id>/availability` for the LWT.

### Syslog (V2.4.15)

UDP syslog client (port 514, configurable). **RFC 5424** framing with `local0` facility. Every applog line mirrors to syslog when enabled. Before the wall clock is valid (pre-NTP sync), the timestamp is the NILVALUE `-` — a stock rsyslog collector falls back to its own receive time, correctly dating boot-banner and config-dump lines. Once NTP has synced, the timestamp is local RFC 3339 with a numeric UTC offset (e.g. `2026-06-13T20:45:50+10:00`), matching the in-message device time shown on `/status`. The per-cycle log line reports `syslog udp: tx=N drops=M` so device-side UDP packet loss is visible (V2.5.29).

### FTP / FTPS log upload (V2.0.x +)

Configurable interval (1 min – 10 090 min, ~7 days; default hourly) upload of the on-device log ring buffer to an FTP server. TLS supported (FTPS explicit-mode, AUTH TLS, TLS 1.2 and 1.3 both work). Cleared after successful upload; carried over across reboot via NVS.

### CPM history graph (V2.5.6)

Two-tier in-RAM ring: 60 one-minute samples + 24 one-hour samples (~250 bytes, all boards). Sampled from a monotonic pulse counter independent of the destructive per-cycle `tube_read()`, so the TX interval has no effect on graph resolution. Displayed as an **inline SVG** on `/status` with horizontal CPM gridlines and a `HH:MM` time axis. Also drives the **cpm5 / cpm15** rolling averages (means of the last 5 / 15 minute samples), fed to GMCMap as ACPM and to ThingSpeak as field3 / field4.

### Coredump on panic (V2.4.18)

64 KB dedicated coredump partition. The ESP-IDF panic handler writes the panicking task's register state + stack snapshot before reset. Recoverable post-reboot via `GET /coredump.elf` (basic auth gated) — no USB needed once flashed. Decode with `espcoredump.py`.

## Security

V2.3.33 web security audit + V2.4.x follow-ups (see CHANGELOG):

- **Basic auth** on `/config`, `/update`, `/reboot`, `/coredump.elf` (user `admin`, password = configured AP password).
- **Constant-time password comparison** to defeat timing oracles.
- **CSRF protection**: POSTs to `/config` / `/update` / `/reboot` require an `Origin` header matching the device's URL — protects against drive-by browser POSTs. **Note for curl users**: add `-H "Origin: http://device:port"` or you'll get a 403.
- **OTA size clamp**: rejects images larger than the OTA partition (2 MB) before writing.
- **X-Frame-Options: DENY** on all responses.
- **TX is paused during OTA** (V2.4.24): scheduled cycles skip while an OTA upload is in progress, giving the OTA the full WiFi airtime.

Deferred for future work: HTTPS for the device UI itself, signed OTA images, NVS encryption, rate limiting.

## Installation

### Flashing a prebuilt release (no build environment needed)

Every [release](https://github.com/MMBytes/MultiGeiger-V2/releases) attaches per-board artefacts. Pick the bundle that matches your board — see the release notes for the file naming.

**Browser-based first flash (no tools needed):** The [MultiGeiger-V2 web flasher](https://mmbytes.github.io/MultiGeiger-V2/) supports Chrome / Edge via the WebSerial API — connect the board over USB, select your board, and flash the latest release directly from the browser.

**`esptool` first flash:** Install [`esptool`](https://github.com/espressif/esptool) (`pip install esptool`); replace `COM3` with your actual serial port.

**First flash of a blank / fresh / bricked device — use `geiger_v2_merged_<board>-<version>.bin`:**

```
esptool --chip esp32 --port COM3 write-flash 0x0 geiger_v2_merged_heltec_v2-<version>.bin
```

(For ESP32-S3 boards substitute `--chip esp32s3`. FeatherS3-D additionally needs `--before no-reset --after no-reset` and the BOOT+RST button dance.)

The merged image bundles the bootloader, partition table, OTA slot pointer, and the app at their correct flash offsets — a single-file factory flash.

**Upgrading a device already running this firmware:**

- **OTA (recommended)** — browse to `http://<device-ip>/update`, log in (`admin` / your configured AP password), pick the `geiger_v2.bin` artefact for your board. No cable, keeps your NVS (WiFi credentials, MQTT broker, etc.) intact.

### First boot

The device comes up as an open WiFi AP named after its chip ID (derived from the MAC). Connect to it, browse to `http://192.168.4.1/config`, and set WiFi credentials, admin password, and back-end choices. After the 2-minute boot window or a manual reboot, it joins your network in STA mode.

Default web credentials (change these on first login):

| Field | Default |
|---|---|
| Username | `admin` |
| Password | `ESP32Geiger` |

## Build from source

Needed only if you want to modify the firmware.

### Requirements

- [ESP-IDF v6.0](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) — pure IDF, not arduino-esp32, not PlatformIO
- A Windows / Linux / macOS host with the IDF tools installed
- USB cable with data lines

### Commands

With the IDF environment sourced (`export.ps1` on Windows, `export.sh` on Linux/macOS):

```
idf.py -B build_heltec_v2 -D SDKCONFIG_DEFAULTS=sdkconfig.defaults.heltec_v2 build
idf.py -B build_heltec_v2 -p <PORT> flash monitor
```

Substitute `heltec_v2_4mb`, `feathers3_d`, `adafruit_qtpy_esp32_pico`, `seeed_xiao_esp32s3`, `heltec_wifi_lora32_v4_r2`, or `sparkfun_thing_plus_esp32s3` for other boards. Per-board build/cache directories prevent cross-board sdkconfig pollution.

The repo includes `_build.cmd <board>`, `_flash.cmd <board>`, `_merge.cmd <board>` helpers that wrap the above.

### Release workflow

`git tag V2.X.Y && git push --tags` is the entire release ceremony — GitHub Actions `release.yml` builds all seven boards in parallel and creates the GitHub Release with bundled artefacts + CHANGELOG body. Manual fallback documented in `_merge.cmd`.

## Repository layout

```
main/                       firmware C sources
  main.c                    entry point, WiFi state machine, cycle loop
  tube.c                    pulse ISR + HV driver timing
  tube_types.c / .h         tube characteristics table (cps→µSv/h factors per tube type)
  tube_pcnt.c / .h          optional PCNT width-filter for narrow-pulse characterisation
  tube_logic.h              pure inline helpers: clamp_u32, gmc_classify, guard policy
  config.c / config_fields.def
                            NVS-backed config, X-macro schema
  http_server.c             web UI (/, /config, /status, /update, /log, /coredump.elf)
  transmission.c            upload pipeline: Madavi / SC / Radmon / OSM / aqi / GMC / ThingSpeak
  mqtt.c / mqtt_discovery.c MQTT 3.1.1 publish + HA Discovery
  syslog.c                  RFC 5424 UDP syslog client
  log_ftp.c                 FTP/FTPS log upload
  net_arp.c                 gratuitous ARP (mesh-AP blackhole prevention)
  periodic.c                housekeeping (PSA refresh + ARP safety-net)
  coredump.c                /coredump.elf streaming from partition
  ntp.c                     SNTP with configurable POSIX TZ string
  history.c / .h            in-RAM CPM history ring + cpm5/cpm15 rolling averages
  gnss.c / .h               GNSS driver: PA1010D + MAX-M10S auto-detected, display-only
  env_sensor.c              umbrella for T/H/P drivers
  sht45.c bmp581.c bmp390.c bme688.c bme280.c
                            individual T/H/P drivers
  pm_sensor.c sps30.c       particulate matter
  noise_sensor.c dnms.c     noise (DNMS / NAM)
  als.c veml7700.c          ambient light
  display.c display_serlcd.c neopixel.c led.c
                            display drivers + LED pulse-tick (XIAO onboard LED)
  speaker.c                 pulse tick (boards with speaker)
  i2c_bus.c                 shared I²C bus handle + mutex
  applog.c                  in-memory log ring buffer
  diag.c / .h               I²C error counter + per-cycle heap split logging
  util.h                    clamped string helpers, URL encoding
  hal.h                     per-board pin map + capability flags
  sysinfo.h                 chip model string, reset-reason string
  main_status.h             request-restart IPC (main loop ← HTTP/TX workers)
partitions.csv              factory + dual-OTA (2 MB each) + coredump (64 KB) on 8 MB flash
partitions_4mb.csv          tighter layout for 4 MB-flash boards (heltec_v2_4mb, sparkfun_thing_plus_esp32s3)
sdkconfig.defaults.<board>  per-board IDF configuration
sdkconfig.defaults.psram    shared WiFi-roaming app + PSRAM-offload knobs (PSRAM boards)
CHANGELOG.md                per-release WHAT/WHY notes

Hardware/                   PCB design files (KiCad)
  Revision_B/               FeatherS3-D host PCB (single-module design)
    geiger.kicad_pro / .kicad_pcb / .kicad_sch
                            KiCad 8 project sources
    0_Custom_Library.pretty / geiger.pretty
                            custom symbol + footprint libraries
    3d-Files/               3D STEP models for non-trivial parts
    Deliverables/
      Gerber Files/         fab-ready Gerbers
      Pick And Place Files/ assembly CSVs
      BOM/                  bill of materials
      Schematic/            schematic PDF
      Renderings/           3D rendered previews
      Supporting Files/     STEP export
  Revision_C/               shared-footprint PCB: QT Py ESP32-PICO or XIAO ESP32-S3
    (same sub-structure as Revision_B)
```

## PCB design files

Two PCB revisions are included in the `Hardware/` directory.

### Revision B — FeatherS3-D host

[`Hardware/Revision_B/`](Hardware/Revision_B/) is the full KiCad 8 project for the single-module FeatherS3-D PCB, plus fabrication-ready Gerbers, pick-and-place CSVs, schematic PDF, BOM, and 3D STEP exports. Si22G tube; full BOM uses metal-film resistors and polypropylene HV capacitors throughout.

- **Browse the schematic**: [`Hardware/Revision_B/Deliverables/Schematic/geiger.pdf`](Hardware/Revision_B/Deliverables/Schematic/geiger.pdf)
- **3D renderings**: [`Hardware/Revision_B/Deliverables/Renderings/`](Hardware/Revision_B/Deliverables/Renderings/)
- **Re-fabricate at JLCPCB or similar**: upload the Gerbers + PnP CSV from `Deliverables/` directly, or open the `.kicad_pro` to regenerate

### Revision C — shared QT Py / XIAO footprint

[`Hardware/Revision_C/`](Hardware/Revision_C/) is a shared-footprint PCB that accepts either the Adafruit QT Py ESP32-PICO or the Seeed XIAO ESP32-S3 in the same U1 socket — both use the same Geiger pin map (HV_FET / GMC / interrupt on A0 / A1 / SCK), so one board runs both `adafruit_qtpy_esp32_pico` and `seeed_xiao_esp32s3` firmware builds. Same HV circuit as Revision B; no piezo (sealed-tube form factor). On-board Qwiic / STEMMA QT connector with on-board pull-ups.

- **Browse the schematic**: [`Hardware/Revision_C/Deliverables/Schematic/MultiGeiger 2.0 Rev C.pdf`](<Hardware/Revision_C/Deliverables/Schematic/MultiGeiger 2.0 Rev C.pdf>)
- **3D renderings**: [`Hardware/Revision_C/Deliverables/Renderings/`](Hardware/Revision_C/Deliverables/Renderings/)
- **Re-fabricate**: upload Gerbers + PnP CSV from `Revision_C/Deliverables/`

**License** for both revisions: same GPL-3.0-or-later as the firmware.

## Documentation

- **Per-release notes**: [`CHANGELOG.md`](CHANGELOG.md) — full WHAT/WHY for every release since V2.3.23, headlines for older. GitHub release pages have the rest.
- **In-firmware**: `/status` shows live state including which drivers responded at boot, per-target TX stats, FTP / MQTT / syslog status, CPM history graph, and GNSS position if a receiver is detected.

## Attribution

This firmware is an **independent rewrite** of [MultiGeiger](https://github.com/ecocurious2/MultiGeiger) by the ecocurious2 project. Hardware design, sensor-community protocol compatibility, the Si22G calibration constant, and the upload-target conventions come from upstream; the C source in this repository shares no commits with it.

The MQTT / HA Discovery, MQTT TLS modes, openSenseMap support, aqi.eco support, GMCMap support, ThingSpeak support, syslog (RFC 5424), coredump-to-flash, multi-board build (FeatherS3 / QT Py / XIAO), runtime-selectable tube type, PCNT width filter, dead-time guard, CPM history graph and rolling averages, GNSS display, and most of the additional sensor drivers (SHT45, BMP581, BMP390, BME688, SPS30, DNMS, ALS-PT19, VEML7700, PA1010D, MAX-M10S) are net-new in V2.

## License

**GPL-3.0-or-later** — same as upstream MultiGeiger. See [LICENSE](LICENSE).
