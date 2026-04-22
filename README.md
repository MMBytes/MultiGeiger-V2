# MultiGeiger V2 (ESP-IDF native)

A ground-up C rewrite of the [MultiGeiger](https://github.com/ecocurious2/MultiGeiger) radiation sensor firmware, ported from Arduino / PlatformIO to **native ESP-IDF 6.0** for the Heltec Wireless Stick V2 (ESP32) with a Si22G pancake tube.

## Status

Running. First overnight soak passed without crashes, uploading to all three default back-ends. API compatibility with the upstream sensor.community / Madavi / Radmon submission endpoints is preserved.

## What it does

- Counts Geiger pulses from a Si22G tube, computes CPM and µSv/h (calibration: `µSv/h = cps / 12.2792`, empirical vs. odlinfo.bfs.de reference).
- Uploads every 150 s to [Madavi](https://api-rrd.madavi.de), [sensor.community](https://api.sensor.community), and [Radmon](https://radmon.org) (any subset; all three optional, HTTPS or HTTP, with Mozilla CA bundle baked into flash).
- Optional BME280 environmental sensor (temperature, humidity, pressure — sea-level-adjusted if station altitude is configured).
- OLED status display (SSD1306, 128×64) with running µSv/h, CPM, uptime.
- Built-in 1.3" speaker and LED tick-per-pulse (configurable).
- WiFi STA with a 2-minute AP window at boot for first-time configuration.
- Web UI at `http://<device>/config` (basic auth, configurable password).
- OTA firmware upload at `/update`.
- Access log and per-minute TX stats at `/log`.
- Hourly FTP upload of the on-device log ring buffer (optional; FTPS supported).
- Configurable NTP servers (up to three) with POSIX TZ string (UTC by default).

## Target hardware

- **MCU:** Heltec Wireless Stick V2 (ESP32-D0WDQ6, 8 MB flash)
- **Tube:** Si22G (high-sensitivity pancake; other tubes need a calibration constant change)
- **Optional:** BME280 on I²C (Wire pins per Heltec V2 default)
- **HV driver:** as per upstream MultiGeiger hardware

This firmware does **not** currently target the Heltec Wireless Stick Lite / V3 / V4 or other ESP32 variants. The pin map and HV timing are specific to V2.

## Installation

### Flashing a prebuilt release (no build environment needed)

Every [release](https://github.com/MMBytes/MultiGeiger-V2/releases) attaches five `.bin` artefacts. Which one you use depends on what you're doing. Install [`esptool`](https://github.com/espressif/esptool) first (`pip install esptool`); replace `COM3` with your actual serial port.

**First flash of a blank / fresh / bricked device — use `merged_<VERSION>.bin`:**

```
esptool.py --chip esp32 --port COM3 write-flash 0x0 merged_V2.1.11.bin
```

The merged image bundles the bootloader, partition table, OTA slot pointer, and the app at their correct flash offsets — a single-file factory flash. Use this after `esptool.py erase-flash` or for a device that's never run this firmware before.

**Upgrading a device already running this firmware:**

- **OTA (recommended)** — browse to `http://<device-ip>/update`, log in (user `admin`, password is your configured AP password), pick `geiger_v2.bin`. No cable, keeps your configuration.
- **Wired app-only upgrade** — keeps NVS (WiFi credentials, etc.) intact:
  ```
  esptool.py --chip esp32 --port COM3 write-flash 0x20000 geiger_v2.bin
  ```

**Flashing components separately** (advanced — if the merged image doesn't suit your toolchain):

| File | Offset | Purpose |
|---|---|---|
| `bootloader.bin` | `0x1000` | Second-stage bootloader |
| `partition-table.bin` | `0x8000` | Partition table |
| `ota_data_initial.bin` | `0xf000` | OTA slot pointer (factory) |
| `geiger_v2.bin` | `0x20000` | App image |

```
esptool.py --chip esp32 --port COM3 write-flash \
  0x1000 bootloader.bin \
  0x8000 partition-table.bin \
  0xf000 ota_data_initial.bin \
  0x20000 geiger_v2.bin
```

### First boot

The device comes up as an open WiFi AP named after its chip ID (derived from the MAC). Connect to it, browse to `http://192.168.4.1/config`, and set WiFi credentials, admin password, and back-end choices. After the 2-minute boot window or a manual reboot, it joins your network in STA mode.

## Build from source

Needed only if you want to modify the firmware.

### Requirements

- [ESP-IDF v6.0](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) (pure IDF — not arduino-esp32, not PlatformIO)
- A Windows / Linux / macOS host with the IDF tools installed
- USB cable with data lines

### Commands

With the IDF environment sourced (`export.ps1` on Windows, `export.sh` on Linux/macOS):

```
idf.py build
idf.py -p <PORT> flash
idf.py -p <PORT> monitor
```

To produce the merged single-file image like the ones attached to releases:

```
idf.py merge-bin -o merged_<VERSION>.bin
```

## Repository layout

```
main/              firmware C sources + CMakeLists
  main.c           entry point, WiFi state machine, cycle loop
  tube.c           pulse ISR, counts accumulation
  transmission.c   Madavi / sensor.community / Radmon uploads
  http_server.c    web UI (config, OTA, log)
  ntp.c            SNTP with configurable POSIX TZ
  log_ftp.c        hourly FTP upload of the log ring buffer
  display.c        SSD1306 driver and screens
  speaker.c        pulse tick + LED
  bme280.c         I²C BME280 driver
  applog.c         in-memory log ring buffer
  config.c         typed Preferences-equivalent in NVS
partitions.csv     factory + dual-OTA (2 MB each) on 8 MB flash
sdkconfig.defaults ESP-IDF configuration
```

## Attribution

This firmware is an **independent rewrite** of [MultiGeiger](https://github.com/ecocurious2/MultiGeiger) by the ecocurious2 project. Hardware design, sensor-community protocol compatibility, and the Si22G calibration constant come from upstream; the C source in this repository shares no commits with it.

## License

**GPL-3.0-or-later** — same as upstream MultiGeiger. See [LICENSE](LICENSE).
