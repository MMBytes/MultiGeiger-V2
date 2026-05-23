# MultiGeiger V2 (ESP-IDF native)

A ground-up C rewrite of the [MultiGeiger](https://github.com/ecocurious2/MultiGeiger) radiation sensor firmware, ported from Arduino / PlatformIO to **native ESP-IDF 6.0**. Runs on four ESP32 / ESP32-S3 board variants with a wide selection of optional environmental, particulate, noise, and ambient-light sensors. Uploads to five public back-ends and publishes to MQTT (with Home Assistant Discovery) and remote syslog.

See the [releases page](https://github.com/MMBytes/MultiGeiger-V2/releases) for the latest build and per-release notes.

## What it does

- Counts Geiger pulses from a Si22G tube (or any tube — calibration constant configurable), computes CPM and µSv/h.
- Drives the HV flyback boost converter from on-chip GPIO timing (no Arduino dependency).
- Optional **environmental sensing**: temperature, humidity, pressure, sea-level-adjusted pressure.
- Optional **PM (particulate matter)**: PM1 / PM2.5 / PM4 / PM10 plus number concentrations and typical particle size.
- Optional **noise**: LAeq / LAmin / LAmax dB(A).
- Optional **ambient light**: lux (two sensor families supported).
- Per-cycle upload (default 150 s, configurable 10 s – 1 h) to any subset of five public back-ends.
- **MQTT publish** with 24 Home Assistant Discovery entities, three TLS modes including custom CA.
- **Remote syslog** (UDP 514) for centralised log aggregation.
- **Hourly FTP/FTPS** upload of the on-device log ring buffer.
- **Web UI** at `http://<device>/` — `/config` (settings), `/status` (live metrics + per-target TX stats + per-sensor presence), `/update` (OTA), `/log` (in-memory log ring).
- **Crash recovery**: ESP-IDF panic handler writes a coredump to a dedicated flash partition; downloadable post-reboot via `GET /coredump.elf` (no USB required).
- **OLED status display** (SSD1306 128×64, also supports SparkFun SerLCD I²C and NeoPixel tick on supported boards).
- **NTP** with up to three configurable servers and POSIX TZ string.

## Supported hardware

### Boards (four build targets)

| Build target | MCU / module | Flash | Notes |
|---|---|---|---|
| `heltec_v2` | Heltec WiFi Kit 32 V2 (ESP32-D0WDQ6) | 8 MB | Onboard SSD1306 OLED. The original target — most production deployments. |
| `heltec_v2_4mb` | Heltec WiFi Kit 32 V2 clone | 4 MB | Same module silicon, smaller flash. Tight on heap during OTA — see V2.4.13 teardown logic in `main.c`. |
| `feathers3_d` | Unexpected Maker FeatherS3 with display (ESP32-S3) | 8 MB | Two STEMMA QT connectors (STEMMA1 on IO8/IO9, STEMMA2 LDO-gated on IO15/IO16). External I²C OLED via STEMMA. |
| `adafruit_qtpy_esp32_pico` | Adafruit QT Py ESP32-PICO | 8 MB | Compact form factor. Optional NeoPixel tick on pulse. |

Build/flash invocation takes a board argument — see `_build.cmd` / `_flash.cmd` / `_merge.cmd` helpers. All four boards share the same `main/` source tree; differences are isolated in per-board `sdkconfig.defaults.<board>` and HAL pin map.

### Geiger tube

- **Si22G** pancake tube (default calibration: `µSv/h = cps / 12.2792`, empirical vs. odlinfo.bfs.de reference).
- Other tubes are supported by changing the calibration constant — the firmware doesn't care which tube produces the pulses.

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
| **Sensirion SPS30** | 0x69 | PM1 / PM2.5 / PM4 / PM10 mass, N05–N10 number concentrations, typical particle size | Optional. Adds the SPS30_* field group to all upload targets. |
| **hbitter DNMS** | 0x55 | LAeq / LAmin / LAmax dB(A) | Nettigo NAM-style Teensy-based noise module. Adds DNMS_noise_* fields. |
| **ALS-PT19** | (analog) | Ambient light (lux) | Onboard FeatherS3-D photodiode on ADC1_CH3. |
| **VEML7700** | 0x10 | Ambient light (lux) | Vishay I²C, alternative to ALS-PT19. |

### Displays

- **SSD1306 OLED** 128×64, I²C 0x3C or 0x3D — auto-probe both addresses (V2.4.19).
- **SparkFun SerLCD** (I²C variant) — alternative line-based display.
- **NeoPixel** tick-per-pulse on supported boards (configurable colour / brightness via `display_mode`).
- Built-in **speaker** click on pulse (configurable, off by default for sealed-tube installs).

## Upload targets

All five targets are fully optional and independently configurable. Each has per-target circuit-breaker logic (V2.3.x) — 3 consecutive all-retry failures suspend that target for 20 cycles to prevent failed TLS handshakes from fragmenting heap.

| Target | Endpoint | Auth | Fields posted |
|---|---|---|---|
| **Madavi** | `api-rrd.madavi.de/data.php` (HTTP / HTTPS) | none | T/H/P; PM and noise are POSTed but the legacy server only graphs T/H/P |
| **sensor.community** | `api.sensor.community/v1/push-sensor-data/` (HTTP / HTTPS) | none, X-Sensor = chip ID | Radiation (X-PIN 19), T/H/P (11), PM (1), noise (15) — split bodies per pin |
| **Radmon** | `radmon.org/radmon.php?function=submit` (HTTP / HTTPS) | basic auth user/pw | Radiation CPM only |
| **openSenseMap** | `ingress.opensensemap.org/boxes/<BOX_ID>/data?luftdaten=1` (HTTPS) | Box ID in URL + optional raw access token in Authorization header (no `Bearer ` prefix) | Combined Luftdaten body |
| **aqi.eco** | `api.aqi.eco/update/<TOKEN>` (HTTPS) | token in URL | Combined Luftdaten body wrapped with `esp8266id` field |

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

UDP syslog client (port 514, configurable). RFC 3164 BSD-format messages with `<13>` facility/severity. Every applog line mirrors to syslog when enabled. Useful for centralised log aggregation across a fleet — point all nodes at the same rsyslog server on your network.

### FTP / FTPS log upload (V2.0.x +)

Hourly (configurable 1 min – 24 h) upload of the on-device log ring buffer to an FTP server. TLS supported (FTPS explicit-mode, AUTH TLS, TLS 1.2 and 1.3 both work). Cleared after successful upload; carried over across reboot via NVS.

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

Install [`esptool`](https://github.com/espressif/esptool) first (`pip install esptool`); replace `COM3` with your actual serial port.

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

Substitute `heltec_v2_4mb`, `feathers3_d`, or `adafruit_qtpy_esp32_pico` for other boards. Per-board build/cache directories prevent cross-board sdkconfig pollution.

The repo includes `_build.cmd <board>`, `_flash.cmd <board>`, `_merge.cmd <board>` helpers that wrap the above.

### Release workflow

`git push --tags V2.X.Y` is the entire release ceremony — GitHub Actions `release.yml` builds all four boards in parallel and creates the GitHub Release with bundled artefacts + CHANGELOG body. Manual fallback documented in `_merge.cmd`.

## Repository layout

```
main/                       firmware C sources
  main.c                    entry point, WiFi state machine, cycle loop
  tube.c                    pulse ISR + HV driver timing
  config.c / config_fields.def
                            NVS-backed config, X-macro schema
  http_server.c             web UI (/, /config, /status, /update, /log, /coredump.elf)
  transmission.c            Madavi / sensor.community / Radmon / OSM / aqi.eco uploads
  mqtt.c / mqtt_discovery.c MQTT 3.1.1 publish + HA Discovery
  syslog.c                  RFC 3164 UDP syslog client
  log_ftp.c                 hourly FTP/FTPS log upload
  net_arp.c                 gratuitous ARP (mesh-AP blackhole prevention)
  periodic.c                housekeeping (PSA refresh + ARP safety-net)
  coredump.c                /coredump.elf streaming from partition
  ntp.c                     SNTP with configurable POSIX TZ
  env_sensor.c              umbrella for T/H/P drivers
  sht45.c bmp581.c bmp390.c bme688.c bme280.c
                            individual T/H/P drivers
  pm_sensor.c sps30.c       particulate matter
  noise_sensor.c dnms.c     noise (DNMS / NAM)
  als.c veml7700.c          ambient light
  display.c display_serlcd.c neopixel.c
                            display drivers
  speaker.c                 pulse tick + LED
  i2c_bus.c                 shared I²C bus handle + mutex
  applog.c                  in-memory log ring buffer
partitions.csv              factory + dual-OTA (2 MB each) + coredump (64 KB) on 8 MB flash
partitions_4mb.csv          tighter layout for heltec_v2_4mb
sdkconfig.defaults.<board>  per-board IDF configuration
CHANGELOG.md                per-release WHAT/WHY notes
```

## Documentation

- **Per-release notes**: [`CHANGELOG.md`](CHANGELOG.md) — full WHAT/WHY for every release since V2.3.23, headlines for older. GitHub release pages have the rest.
- **In-firmware**: `/status` shows live state including which drivers responded at boot, per-target TX stats, FTP / MQTT / syslog status.

## Attribution

This firmware is an **independent rewrite** of [MultiGeiger](https://github.com/ecocurious2/MultiGeiger) by the ecocurious2 project. Hardware design, sensor-community protocol compatibility, the Si22G calibration constant, and the upload-target conventions come from upstream; the C source in this repository shares no commits with it.

The MQTT / HA Discovery, MQTT TLS modes, openSenseMap support, aqi.eco support, syslog, coredump-to-flash, multi-board build (FeatherS3 / QT Py), and most of the additional sensor drivers (SHT45, BMP581, BMP390, BME688, SPS30, DNMS, ALS-PT19, VEML7700) are net-new in V2.

## License

**GPL-3.0-or-later** — same as upstream MultiGeiger. See [LICENSE](LICENSE).
