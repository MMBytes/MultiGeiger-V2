# Heltec WiFi LoRa 32 V4 (R2) Board Port — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a sixth build target, `heltec_wifi_lora32_v4_r2`, so the existing MultiGeiger V2 firmware runs on a third-party PCB (the standard Multigeiger V2 mainboard populated with a Heltec WiFi LoRa 32 V4 base/R2 module, ESP32-S3R2) instead of the previously-supported Heltec V2 module.

**Architecture:** Follow the codebase's existing per-board HAL pattern exactly — one new `#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)` branch in `main/hal.h` defining the same `PIN_*`/`HAL_HAS_*` macros every other board defines, one new `elseif(BOARD STREQUAL "heltec_wifi_lora32_v4_r2")` branch in `CMakeLists.txt`, and one new `sdkconfig.defaults.heltec_wifi_lora32_v4_r2` overlay. The one structural wrinkle: this board's OLED lives on a fixed module-internal I²C bus (GPIO17/18) that is completely separate from the external env-sensor bus (GPIO47/48) exposed on the mainboard's J2 header — every other board shares one bus between OLED and sensor. This reuses the existing dual-bus abstraction already built for FeatherS3-D's STEMMA1/STEMMA2 split (`main/i2c_bus.c`'s `i2c_bus_get_secondary()`), extended with a new branch that brings up `I2C_NUM_1` on GPIO17/18 unconditionally (no LDO gating — this bus is always on, unlike STEMMA2).

**Tech Stack:** ESP-IDF v6.0.1, C, CMake, GitHub Actions (reusable workflow `_build-boards.yml`).

## Global Constraints

- Follow the authoritative design spec exactly: `docs/superpowers/specs/2026-07-06-heltec-wifi-lora32-v4-r2-board-port-design.md` (as corrected 2026-07-06 — SDA=GPIO48/SCL=GPIO47, and `PIN_SPEAKER_N`=GPIO5 was added; see that file's §2 correction note). Every pin value in this plan is taken from that corrected spec, itself re-verified against `Multigeiger_V1.9/Pin-Matrix_Heltec_MG_neu-V1.9.pdf`'s "Heltec WiFi LoRa 32 V4" column.
- No physical board is in hand in this session (a separate team owns the PCB) — **no flashing or bench verification is possible as part of this plan.** Every task's "verification" step is a `_build.cmd` compile — this proves the firmware builds correctly for the new target, not that it runs correctly on real silicon. Three assumptions are flagged in the spec/hal.h comments as bench-verify-on-first-flash: LED polarity (GPIO35, expected active-HIGH), speaker P/N pin assignment (arbitrary — the source matrix doesn't distinguish, and it's functionally inconsequential for a piezo), and PSRAM speed (80 MHz assumed for the in-package quad PSRAM; back off to 40 MHz in the overlay if the boot log shows PSRAM init failure). Do not attempt to "resolve" these by guessing further — they're correctly deferred.
- Do **not** add any LoRaWAN/Meshtastic code, config fields, or a `HAL_HAS_LORA` flag. Per spec §6, this port only documents (in comments) that GPIO 7-14 are reserved for that future work — nothing else changes.
- Do **not** touch VBAT ADC (GPIO1) or GNSS (GPIO38-42) — both explicitly out of scope per spec §9.
- Build verification commands use the project's canonical invocation (`& .\_build.cmd <board> 2>&1 | Select-Object -Last 25` from PowerShell) — not raw `idf.py` via the Bash tool's `cmd.exe` bridge, which has a known silent-skip failure mode. After every build, confirm the version string actually landed with `strings build_<board>/geiger_v2.bin | grep -E "V2\.[0-9]+\.[0-9]+" | sort -u` (also PowerShell-invokable via `Select-String` if `strings` isn't on PATH — see Task 3 Step 4 for the exact fallback).
- Version: current `main/version.h` is `V2.6.6`. Per this project's own convention (every prior new-board addition — e.g. V2.4.25's `seeed_xiao_esp32s3` — got its own version bump + CHANGELOG entry), bump to `V2.6.7` once the board builds cleanly (Task 6), not before.
- A fourth bench-verify item (alongside LED polarity, speaker P/N, and PSRAM speed): `PIN_GMC_COUNT_INPUT` reuses GPIO3, an ESP32-S3 JTAG-source boot-strap pin. Unlike `BOARD_FEATHERS3_D`'s reuse of the same pin for `PIN_SPEAKER_P` (safe because the speaker driver stays hi-Z until code drives it post-boot), this pin is an always-connected external input from the tube pulse-conditioning circuit — its level during the ROM bootloader's strap-sampling window is not under firmware control. Flagged in `hal.h`, not resolved by this plan; first-flash bench verification must confirm the board still enumerates over USB-Serial-JTAG.
- A fifth bench-verify item: the onboard SSD1315's physical panel size (128x64 vs. larger) is unconfirmed — the design spec and pin-matrix source don't state it. This plan assumes the same small single-page 128x64 footprint as Heltec V2's SSD1306 (not FeatherS3-D's 2.42" SSD1309), since Heltec WiFi LoRa 32 modules ship with the same compact onboard OLED across the V2/V3/V4 line. Flagged explicitly in Task 5 rather than left as a silent default.

## File Structure

| File | Change |
|---|---|
| `sdkconfig.defaults.heltec_wifi_lora32_v4_r2` | **New.** S3R2-specific overlay: 16 MB flash, 2 MB in-package quad PSRAM, UART console. |
| `CMakeLists.txt` | Modify. New `elseif(BOARD STREQUAL "heltec_wifi_lora32_v4_r2")` branch + update the 3 board-list mentions (usage comment, cache-string help text, `else()` error message). |
| `main/hal.h` | Modify. New `#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)` branch (pin map + feature flags, incl. a GPIO3 strap-safety caveat on `PIN_GMC_COUNT_INPUT`); update the final `#error` message and the stale `@file` board-enumeration comment. |
| `main/i2c_bus.c` | Modify. Widen `s_bus_secondary`'s board guard; add a new branch inside `i2c_bus_get_secondary()` that brings up `I2C_NUM_1` on GPIO17/18 (OLED bus), always-on, no gating GPIO; widen `i2c_bus_finalize()` so it no longer silently no-ops on this board. |
| `main/i2c_bus.h` | Modify. Add a "Heltec WiFi LoRa 32 V4-R2" paragraph to the `@file` per-board doc comment AND update the separate stale function-level doc comment above `i2c_bus_get_secondary()`. |
| `main/main.c` | Modify. Guard the `PROBE_ON_BOTH_BUSES` macro's secondary-bus fallback so sensor drivers (env/PM/noise/GNSS/VEML7700) never probe this board's OLED-only secondary bus. |
| `main/display.c` | Modify. Extend `OLED_CHIP_NAME` chain with `"SSD1315"`; update the top-of-file per-board comment (with an anchor-text fix vs. the file's real current text); make the `"STEMMA1"`/`"STEMMA2"` log-label strings board-aware; give `DISPLAY_MODE_AUTO` an explicit branch for this board instead of relying on the `#else` catch-all; skip the primary-bus probe and the SerLCD-wake delay for this board's always-secondary-only OLED. |
| `main/display.h` | Modify. Update the stale top-of-file per-board panel/bus doc comment (currently only documents Heltec V2 and FeatherS3-D). |
| `main/http_server.c` | Modify. Add an `#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)` arm to the OTA-page `UPLOAD_PROMPT_BOARD` chain (mirrors the V2.5.19 XIAO fix — without this, the OTA page shows "(unknown board)"). |
| `.github/workflows/_build-boards.yml` | Modify. Add `heltec_wifi_lora32_v4_r2` to `matrix.board`; extend the `target:` ternary so it maps to `esp32s3` (not the `esp32` default); fix the stale "5-board list" line-1 comment. |
| `.github/workflows/release.yml` | Modify. Bump `EXPECTED_BOARDS` from `5` to `6`; fix 4 other stale "5 boards"/"25 artefacts" comment lines. |
| `docs/manifests/heltec_wifi_lora32_v4_r2.json` | **New.** ESP Web Tools manifest for the 6th board, modeled on the existing 5 manifests. |
| `docs/index.html` | Modify. Add a 6th `<option>` to the board `<select>` so the web flasher can actually target this board. |
| `main/version.h` | Modify. `V2.6.6` → `V2.6.7`. |
| `CHANGELOG.md` | Modify. New `## V2.6.7` entry. |
| `README.md` | Modify. New board-table row; extend the "Substitute ... for other boards" sentence; fix 2 other stale "five boards" mentions (intro paragraph, release-workflow section). |

No new C source files are needed (unlike, say, adding a whole new sensor driver) — this is a pure HAL/config addition, matching how every previous board was added.

---

### Task 1: `sdkconfig.defaults.heltec_wifi_lora32_v4_r2` overlay

**Files:**
- Create: `sdkconfig.defaults.heltec_wifi_lora32_v4_r2`

**Interfaces:**
- Consumes: nothing (static config file).
- Produces: an overlay filename that Task 2's `CMakeLists.txt` branch references verbatim in its `SDKCONFIG_DEFAULTS` string. Must be spelled exactly `sdkconfig.defaults.heltec_wifi_lora32_v4_r2`.

- [ ] **Step 1: Create the overlay file**

Modeled on `sdkconfig.defaults.feathers3_d`'s structure (closest S3 precedent) but for S3R2 in-package PSRAM (2 MB, quad — not FeatherS3-D's 8 MB external QSPI), no native USB (like `sdkconfig.defaults.adafruit_qtpy_esp32_pico`'s UART console path).

```
# ============================================================================
# Heltec WiFi LoRa 32 V4 (R2) overlay — ESP32-S3R2, 16 MB flash, 2 MB
# in-package QSPI PSRAM (quad mode), USB-UART bridge console (no native USB).
#
# "R2" in the board name refers to the ESP32-S3R2 chip variant (2 MB
# in-package PSRAM) — disambiguates from a possible future ESP32-S3R8 "V4-R8"
# port, which has different GPIOs and 8 MB octal PSRAM. Pin map and feature
# flags live in main/hal.h under BOARD_HELTEC_WIFI_LORA32_V4_R2. See
# docs/superpowers/specs/2026-07-06-heltec-wifi-lora32-v4-r2-board-port-design.md
# for the full pin-sourcing rationale.
# ============================================================================

CONFIG_IDF_TARGET="esp32s3"

# 16 MB flash.
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_ESPTOOLPY_FLASHSIZE="16MB"

# In-package PSRAM, 2 MB, quad mode (S3R2 — NOT octal; the R8 variant is
# octal and would need CONFIG_SPIRAM_MODE_OCT instead, see the FeatherS3-D
# overlay's comment on that same distinction). 80 MHz is the standard speed
# for Espressif's in-package R2/R8 PSRAM variants — UNVERIFIED against this
# specific board's boot log (no hardware in hand this session). If the boot
# log shows a PSRAM init failure, drop to CONFIG_SPIRAM_SPEED_40M as the
# first fallback.
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_QUAD=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_SPIRAM_USE_MALLOC=y
# Heap-spill policy and the V2.5.33 network/TLS PSRAM-offload knobs are
# identical across all PSRAM boards and live in the shared sdkconfig.defaults
# ("PSRAM heap policy" section).

# Console / flashing path: UART0 via the module's USB-UART bridge (this SKU
# has no native USB peripheral wired out — see hal.h HAL_HAS_NATIVE_USB=0).
CONFIG_ESP_CONSOLE_UART_DEFAULT=y

# Wi-Fi roaming app + other shared PSRAM-board config: see sdkconfig.defaults.psram
# (added to this board's SDKCONFIG_DEFAULTS in CMakeLists.txt).
```

- [ ] **Step 2: Verify the file was written correctly**

Run (PowerShell):
```powershell
Get-Content "sdkconfig.defaults.heltec_wifi_lora32_v4_r2"
```
Expected: the exact content above, byte-for-byte (compare against `sdkconfig.defaults.feathers3_d` for structural similarity — both should have `CONFIG_IDF_TARGET`, flash size, `CONFIG_SPIRAM*` block, and a console line).

- [ ] **Step 3: Commit**

```powershell
git add sdkconfig.defaults.heltec_wifi_lora32_v4_r2
git commit -m "Add sdkconfig overlay for Heltec WiFi LoRa 32 V4 (R2)"
```

---

### Task 2: `CMakeLists.txt` board selector branch

**Files:**
- Modify: `CMakeLists.txt:7-11` (usage comment), `CMakeLists.txt:18-20` (`if(NOT DEFINED BOARD)` cache-string help text), `CMakeLists.txt:47-57` (new `elseif` branch + `else()` error message)

**Interfaces:**
- Consumes: `sdkconfig.defaults.heltec_wifi_lora32_v4_r2` (Task 1) by exact filename.
- Produces: the compile define `BOARD_HELTEC_WIFI_LORA32_V4_R2=1`, which Task 3's `hal.h` branch and Task 4's `i2c_bus.c` branch key off of. Also produces `MQTT_RICH_STATE=1` (every PSRAM board defines this — matches `feathers3_d`/`adafruit_qtpy_esp32_pico`/`seeed_xiao_esp32s3`).

- [ ] **Step 1: Attempt a reconfigure with the not-yet-defined board name — verify it fails**

Run (PowerShell):
```powershell
cd "C:\Users\matth\OneDrive\Claude_Code\Geiger\Git_Repository_Geiger_V2"
$env:MSYSTEM=$null; $env:OSTYPE=$null; $env:MINGW_PREFIX=$null
. 'C:\esp\v6.0\esp-idf\export.ps1' 2>&1 | Out-Null
idf.py -B build_heltec_wifi_lora32_v4_r2 -DBOARD=heltec_wifi_lora32_v4_r2 reconfigure 2>&1 | Select-Object -Last 15
```
Expected: `CMake Error` containing `Unknown BOARD 'heltec_wifi_lora32_v4_r2'. Valid: heltec_v2 | heltec_v2_4mb | feathers3_d | adafruit_qtpy_esp32_pico | seeed_xiao_esp32s3` — proving the board name isn't recognized yet.

- [ ] **Step 2: Update the usage comment (lines 7-11)**

In `CMakeLists.txt`, find:
```cmake
#     idf.py -DBOARD=heltec_v2             build   (genuine Heltec WiFi Kit 32 V2, 8 MB)
#     idf.py -DBOARD=heltec_v2_4mb         build   (knock-off variant with 4 MB flash)
#     idf.py -DBOARD=feathers3_d           build   (UM FeatherS3-D, ESP32-S3 + 8 MB QSPI PSRAM)
#     idf.py -DBOARD=adafruit_qtpy_esp32_pico  build (ESP32-PICO SiP, 2 MB QSPI PSRAM)
#     idf.py -DBOARD=seeed_xiao_esp32s3    build   (Seeed XIAO ESP32-S3, 8 MB OPI PSRAM — I²C sensor host)
```
Replace with (adds one line):
```cmake
#     idf.py -DBOARD=heltec_v2             build   (genuine Heltec WiFi Kit 32 V2, 8 MB)
#     idf.py -DBOARD=heltec_v2_4mb         build   (knock-off variant with 4 MB flash)
#     idf.py -DBOARD=feathers3_d           build   (UM FeatherS3-D, ESP32-S3 + 8 MB QSPI PSRAM)
#     idf.py -DBOARD=adafruit_qtpy_esp32_pico  build (ESP32-PICO SiP, 2 MB QSPI PSRAM)
#     idf.py -DBOARD=seeed_xiao_esp32s3    build   (Seeed XIAO ESP32-S3, 8 MB OPI PSRAM — I²C sensor host)
#     idf.py -DBOARD=heltec_wifi_lora32_v4_r2  build (Heltec WiFi LoRa 32 V4 base/R2, ESP32-S3R2 + 2 MB PSRAM)
```

- [ ] **Step 3: Update the default-BOARD cache-string help text (line 19)**

Find:
```cmake
    set(BOARD "heltec_v2" CACHE STRING "Target board (heltec_v2 | heltec_v2_4mb | feathers3_d | adafruit_qtpy_esp32_pico | seeed_xiao_esp32s3)")
```
Replace with:
```cmake
    set(BOARD "heltec_v2" CACHE STRING "Target board (heltec_v2 | heltec_v2_4mb | feathers3_d | adafruit_qtpy_esp32_pico | seeed_xiao_esp32s3 | heltec_wifi_lora32_v4_r2)")
```

- [ ] **Step 4: Add the new `elseif` branch**

Find the `elseif(BOARD STREQUAL "seeed_xiao_esp32s3")` branch (ends just before `else()`):
```cmake
elseif(BOARD STREQUAL "seeed_xiao_esp32s3")
    # Seeed Studio XIAO ESP32-S3 (SKU 113991054). ESP32-S3 LX7 dual-core, 8 MB
    # QSPI flash + 8 MB OPI (octal-mode) PSRAM, native USB-C. Intended as an
    # I²C-only sensor host — Geiger tube not wired. See hal.h block under
    # BOARD_SEEED_XIAO_ESP32S3 for pin map and the "intended use" rationale.
    set(IDF_TARGET "esp32s3" CACHE STRING "")
    add_compile_definitions(BOARD_SEEED_XIAO_ESP32S3=1 MQTT_RICH_STATE=1)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.psram;sdkconfig.defaults.seeed_xiao_esp32s3")
else()
    message(FATAL_ERROR "Unknown BOARD '${BOARD}'. Valid: heltec_v2 | heltec_v2_4mb | feathers3_d | adafruit_qtpy_esp32_pico | seeed_xiao_esp32s3")
endif()
```
Replace with (inserts a new `elseif` before `else()`, and updates the error message):
```cmake
elseif(BOARD STREQUAL "seeed_xiao_esp32s3")
    # Seeed Studio XIAO ESP32-S3 (SKU 113991054). ESP32-S3 LX7 dual-core, 8 MB
    # QSPI flash + 8 MB OPI (octal-mode) PSRAM, native USB-C. Intended as an
    # I²C-only sensor host — Geiger tube not wired. See hal.h block under
    # BOARD_SEEED_XIAO_ESP32S3 for pin map and the "intended use" rationale.
    set(IDF_TARGET "esp32s3" CACHE STRING "")
    add_compile_definitions(BOARD_SEEED_XIAO_ESP32S3=1 MQTT_RICH_STATE=1)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.psram;sdkconfig.defaults.seeed_xiao_esp32s3")
elseif(BOARD STREQUAL "heltec_wifi_lora32_v4_r2")
    # Heltec WiFi LoRa 32 V4 — base/R2 variant (ESP32-S3R2, 2 MB in-package
    # PSRAM). NOT the V4-R8 variant (ESP32-S3R8, 8 MB PSRAM, different
    # GPIOs) — "_r2" in the board name permanently disambiguates. Runs on a
    # third-party PCB: the standard Multigeiger V2 mainboard populated with
    # this module instead of the Heltec V2. See hal.h under
    # BOARD_HELTEC_WIFI_LORA32_V4_R2 for the pin map and
    # docs/superpowers/specs/2026-07-06-heltec-wifi-lora32-v4-r2-board-port-design.md
    # for the full design rationale.
    set(IDF_TARGET "esp32s3" CACHE STRING "")
    add_compile_definitions(BOARD_HELTEC_WIFI_LORA32_V4_R2=1 MQTT_RICH_STATE=1)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.psram;sdkconfig.defaults.heltec_wifi_lora32_v4_r2")
else()
    message(FATAL_ERROR "Unknown BOARD '${BOARD}'. Valid: heltec_v2 | heltec_v2_4mb | feathers3_d | adafruit_qtpy_esp32_pico | seeed_xiao_esp32s3 | heltec_wifi_lora32_v4_r2")
endif()
```

- [ ] **Step 5: Verify reconfigure now succeeds**

Run:
```powershell
idf.py -B build_heltec_wifi_lora32_v4_r2 -DBOARD=heltec_wifi_lora32_v4_r2 reconfigure 2>&1 | Select-Object -Last 15
```
Expected: no `CMake Error`; output ends with something like `-- Configuring done` / `-- Generating done` / `-- Build files have been written to: ...build_heltec_wifi_lora32_v4_r2`. (A full `build` at this point will still fail — hal.h has no matching branch yet. That's expected; Task 3 fixes it.)

- [ ] **Step 6: Commit**

```powershell
git add CMakeLists.txt
git commit -m "Wire up heltec_wifi_lora32_v4_r2 in the CMake board selector"
```

---

### Task 3: `main/hal.h` board branch

**Files:**
- Modify: `main/hal.h:338-424` (insert new branch between the `BOARD_SEEED_XIAO_ESP32S3` block and the final `#else`)

**Interfaces:**
- Consumes: `BOARD_HELTEC_WIFI_LORA32_V4_R2` (from Task 2's `add_compile_definitions`).
- Produces every `PIN_*`/`HAL_HAS_*` macro that `main/i2c_bus.c` (Task 4), `main/display.c` (Task 5), `main/speaker.c` (unchanged — consumes `PIN_SPEAKER_P`/`PIN_SPEAKER_N`/`PIN_LED_BUILTIN`/`HAL_HAS_SPEAKER`), and every other existing `.c` file in `main/` already consume by macro name. In particular: `PIN_I2C_SDA=48`, `PIN_I2C_SCL=47` (primary/env-sensor bus), `PIN_OLED_SDA=17`, `PIN_OLED_SCL=18`, `PIN_OLED_RESET=21` (new pins, consumed by Task 4's `i2c_bus.c` branch — `PIN_OLED_RESET` is also consumed by `display.c`'s existing `#ifdef PIN_OLED_RESET` → `PIN_OLED_RST` alias, no `display.c` change needed for that part).

- [ ] **Step 1: Attempt a full build — verify it fails at the `#error`**

Run:
```powershell
& .\_build.cmd heltec_wifi_lora32_v4_r2 2>&1 | Select-Object -Last 25
```
Expected: compilation fails on the first `.c` file that includes `hal.h`, with a preprocessor error:
```
#error "No board defined. Set -DBOARD_HELTEC_V2=1 / -DBOARD_FEATHERS3_D=1 / -DBOARD_ADAFRUIT_QTPY_ESP32_PICO=1 / -DBOARD_SEEED_XIAO_ESP32S3=1 via CMake."
```
This confirms `BOARD_HELTEC_WIFI_LORA32_V4_R2` is being defined (Task 2 worked) but `hal.h` has no branch for it yet.

- [ ] **Step 2: Insert the new `hal.h` branch**

In `main/hal.h`, find the end of the `BOARD_SEEED_XIAO_ESP32S3` block and the start of `#else`:
```c
    // RESERVED / strap pins — never repurpose:
    //   GPIO0   BOOT button (strap)
    //   GPIO45  flash voltage select (strap)
    //   GPIO46  boot mode (strap)
    //   GPIO19/20  native USB D-/D+ (USB-Serial-JTAG)
    //   GPIO26-32, 33-37  internal flash / PSRAM (not broken out)
    //   GPIO43/44  default UART0 TX/RX (D6/D7 on header — re-purposable
    //              post-boot but UART0 console is disabled in our sdkconfig
    //              in favour of USB-Serial-JTAG, so they're free)

#else
    #error "No board defined. Set -DBOARD_HELTEC_V2=1 / -DBOARD_FEATHERS3_D=1 / -DBOARD_ADAFRUIT_QTPY_ESP32_PICO=1 / -DBOARD_SEEED_XIAO_ESP32S3=1 via CMake."
#endif
```
Replace with (inserts a new `#elif` branch before `#else`, and updates the `#error` message):
```c
    // RESERVED / strap pins — never repurpose:
    //   GPIO0   BOOT button (strap)
    //   GPIO45  flash voltage select (strap)
    //   GPIO46  boot mode (strap)
    //   GPIO19/20  native USB D-/D+ (USB-Serial-JTAG)
    //   GPIO26-32, 33-37  internal flash / PSRAM (not broken out)
    //   GPIO43/44  default UART0 TX/RX (D6/D7 on header — re-purposable
    //              post-boot but UART0 console is disabled in our sdkconfig
    //              in favour of USB-Serial-JTAG, so they're free)

#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)

    // Heltec WiFi LoRa 32 V4 — base/R2 variant (ESP32-S3R2 chip, 2 MB
    // in-package quad PSRAM). NOT the V4-R8 variant (ESP32-S3R8, 8 MB octal
    // PSRAM, different GPIOs for Vext/VGNSS/LED/PA_CTX) — confirmed via
    // heltec.org's own datasheet and the HelTecAutomation/Heltec_ESP32
    // Arduino library, which defines WIFI_LORA_32_V4 and WIFI_LORA_32_V4_R8
    // as distinct targets. "_r2" in the board name permanently disambiguates
    // from a possible future R8 port.
    //
    // Third-party PCB, not our own design: a separate team populated the
    // standard Multigeiger V2 mainboard
    // (Multigeiger_V1.9/Hardware/Eagle/projects/Multigeiger_V2/) with this
    // Heltec module instead of the Heltec V2. Pin map sourced from
    // Multigeiger_V1.9/Pin-Matrix_Heltec_MG_neu-V1.9.ods/.pdf, cross-validated
    // pin-for-pin against Heltec's V4 datasheet §2.2.1/2.2.2/2.2.3 — zero
    // contradictions. Full rationale (incl. the Vext/GPIO2/GPIO46
    // non-conflict analysis and the future-LoRaWAN pin reservations) in
    // docs/superpowers/specs/2026-07-06-heltec-wifi-lora32-v4-r2-board-port-design.md.
    #define BOARD_NAME              "heltec_wifi_lora32_v4_r2"
    #define HAL_HAS_OLED              1   // SSD1315 on a dedicated I2C_NUM_1 bus (GPIO17/18) — see PIN_OLED_SDA/SCL below
    #define HAL_HAS_ALS               0   // No onboard ambient-light sensor
    #define HAL_HAS_FUEL_GAUGE        0   // No onboard fuel gauge
    #define HAL_HAS_PSRAM             1   // 2 MB in-package, quad
    #define HAL_HAS_NATIVE_USB        0   // Console via USB-UART bridge on GPIO43/44 (TX/RX), not native USB
    // GPIO36 (Vext_Ctrl) only gates the external "Ve" header pin (peripheral
    // power, 500 mA max, per the datasheet's §3.3 "Power Output") — it does
    // NOT power the OLED or either I²C bus on this board, unlike Heltec V2
    // (where Vext gates the shared OLED+sensor rail and MUST be driven for
    // the bus to work — see i2c_bus.c's HAL_HAS_VEXT_GATE block). Deliberately
    // left undriven here. See spec §2.
    #define HAL_HAS_VEXT_GATE         0
    #define HAL_HAS_ANTENNA_SWITCH    0   // PCB antenna only (no u.FL / no RF switch)
    #define HAL_HAS_I2C_PINOUT_SWITCH 0   // Single fixed route per bus
    #define HAL_HAS_SPEAKER           1   // Piezo, GPIO26 (P) / GPIO5 (N)
    #define HAL_HAS_NEOPIXEL          0   // No onboard NeoPixel

    // Ring/scratch/form-buffer sizes modeled on BOARD_ADAFRUIT_QTPY_ESP32_PICO
    // (closest analog: 2 MB in-package PSRAM, not FeatherS3-D's 8 MB external).
    #define HAL_LOG_RING_BYTES      (1 * 1024 * 1024)   // 1 MB of 2 MB PSRAM (50% headroom)
    #define HAL_LOG_SNAP_SCRATCH_BYTES  (16 * 1024)
    #define HAL_CFG_FORM_BUF_SIZE   (32 * 1024)

    // Geiger / HV / speaker pins — from the Multigeiger mainboard's J2/J3
    // wiring intent (Pin-Matrix_Heltec_MG_neu-V1.9.ods/.pdf), cross-validated
    // against the V4 datasheet.
    #define PIN_HV_FET_OUTPUT       33   // J2 pin12 — HV MOSFET gate
    #define PIN_HV_CAP_FULL_INPUT    2   // J3 pin13 — ADC1_CH1/TOUCH2, plain GPIO on base V4 (see spec §5: no LoRa-PA conflict on this SKU)
    // IO3 is an ESP32-S3 boot strap (JTAG vs USB-Serial-JTAG select), same
    // strap BOARD_FEATHERS3_D reuses for PIN_SPEAKER_P below — but that reuse
    // is safe ONLY because the speaker driver stays hi-Z until code drives it
    // post-boot. This pin is different: it's an always-connected external
    // input from the tube pulse-conditioning circuit, whose level during the
    // ROM bootloader's strap-sampling window is NOT under firmware control.
    // Not resolvable without hardware in hand — flagged as a first-flash
    // bench-verify item (see Global Constraints): confirm the board still
    // enumerates over USB-Serial-JTAG after flashing.
    #define PIN_GMC_COUNT_INPUT      3   // J3 pin14 — Geiger tube pulse

    // Piezo pins. The pin-matrix marks BOTH GPIO26 (J2 pin15) and GPIO5 (J3
    // pin16) as "SPK" without distinguishing P/N — a piezo isn't polarity
    // sensitive in a way that matters for tone generation, so this
    // assignment is arbitrary (bench-verify perceived loudness on first
    // flash; worst case if reversed is a quieter click, not broken audio).
    #define PIN_SPEAKER_P           26   // J2 pin15
    #define PIN_SPEAKER_N            5   // J3 pin16 ("Touch5 – SPK")

    // Onboard LED. GPIO35, ACTIVE-HIGH — confirmed via the independent
    // DN9KGB/rMesh project's hal_HELTEC_WiFi_LoRa_32_V4.c (whose other pin
    // values match this exact base-V4 pinout): LOW at boot = off, HIGH =
    // on. Not Heltec's own datasheet text, so bench-verify polarity on
    // first flash (same standard applied to the XIAO ESP32-S3's LED).
    // HAL_LED_ACTIVE_LOW omitted (led.c defaults to active-high) — but
    // led.c's own driver compiles OUT on this board anyway
    // (HAL_HAS_SPEAKER=1), so speaker.c owns PIN_LED_BUILTIN instead; its
    // hardcoded gpio_set_level(PIN_LED_BUILTIN, 1)==on already assumes
    // active-high and needs no changes here.
    #define PIN_LED_BUILTIN         35   // J2 pin10

    // I2C bus (env sensor — the bus every sensor driver targets by default).
    // From the Multigeiger mainboard's J2 pins 13/14, NOT the OLED's bus
    // (see PIN_OLED_SDA/SCL below and spec §3 for why this board needs two
    // separate I²C controllers).
    #define PIN_I2C_SDA             48   // J2 pin14
    #define PIN_I2C_SCL             47   // J2 pin13

    // OLED bus — fixed module-internal bus, NOT on the J2/J3 header at all.
    // Dedicated permanently to the onboard SSD1315; i2c_bus.c brings this up
    // as I2C_NUM_1 directly (no probe/fallback — this board always has the
    // OLED, unlike FeatherS3-D's optional STEMMA2 plug-in). See spec §3.
    #define PIN_OLED_SDA            17
    #define PIN_OLED_SCL            18
    #define PIN_OLED_RESET          21   // J2 pin16

    // RESERVED for future LoRaWAN/Meshtastic work (hardware-reservation
    // only per spec §6 — no radio driver, no HAL_HAS_LORA flag, nothing
    // here should need to change when that spec is eventually written):
    //   GPIO 8/9/10/11/12/13/14 — SX1262 LoRa radio's dedicated internal SPI
    //     bus (NSS/SCK/MOSI/MISO/DIO1/RST/BUSY). Confirmed via the datasheet
    //     ("LoRa and Flash have each utilized a separate SPI interface") and
    //     never appears in the J2/J3 header tables — nothing else could
    //     claim these pins anyway.
    //   GPIO 7 — VFEM_Control (LoRa front-end enable per the datasheet's own
    //     J3 pin table, J3 pin18). Left undriven — not assigned to any
    //     Multigeiger function.
    //
    // RESERVED / never repurpose (out of scope or module-internal — see
    // spec §2 and §9):
    //   GPIO36        Vext_Ctrl — deliberately undriven, see HAL_HAS_VEXT_GATE above
    //   GPIO19/20     native USB D-/D+ (module-internal strap pair; this
    //                 board doesn't use native USB but the pins are still
    //                 module wiring, not free GPIO)
    //   GPIO1         VBAT_Read — out of scope (no battery ADC support)
    //   GPIO38-42     GNSS connector (RST/PPS/Wakeup/TX/RX) — out of scope
    //   GPIO45/46     "DIP0/DIP1" in the Multigeiger matrix — DIP-switch
    //                 inputs unused by V2 firmware on every board
    //   GPIO6         "DIP3" in the Multigeiger matrix — same as above

#else
    #error "No board defined. Set -DBOARD_HELTEC_V2=1 / -DBOARD_FEATHERS3_D=1 / -DBOARD_ADAFRUIT_QTPY_ESP32_PICO=1 / -DBOARD_SEEED_XIAO_ESP32S3=1 / -DBOARD_HELTEC_WIFI_LORA32_V4_R2=1 via CMake."
#endif
```

- [ ] **Step 3: Verify the build now progresses past `hal.h`**

Run:
```powershell
& .\_build.cmd heltec_wifi_lora32_v4_r2 2>&1 | Select-Object -Last 40
```
Expected: no `#error`. The build will very likely complete fully at this point too (Task 4/5 aren't strictly required for compilation to succeed — `i2c_bus_get_secondary()` simply falls to its `#else return NULL;` branch for this board until Task 4 adds a case for it, and `display.c` compiles unconditionally). Confirm the tail of the output ends with:
```
Project build complete.
```
If it fails, read the actual compiler error (don't guess) — most likely cause at this stage would be a typo'd macro name colliding with something else already defined elsewhere in `main/`.

- [ ] **Step 4: Confirm the binary actually rebuilt (silent-skip check)**

```powershell
Get-Content build_heltec_wifi_lora32_v4_r2\geiger_v2.bin -Raw -Encoding Byte | ForEach-Object { [System.Text.Encoding]::ASCII.GetString($_) } | Select-String -Pattern "V2\.\d+\.\d+" -AllMatches | ForEach-Object { $_.Matches.Value } | Sort-Object -Unique
```
Expected: `V2.6.6` (version not bumped yet — that's Task 6). Confirms the binary was freshly produced, not stale.

- [ ] **Step 5: Fix the stale `@file` top-of-header board enumeration**

The header's own top-of-file doc comment enumerates `BOARD_*` macros and is already stale (it omits `BOARD_SEEED_XIAO_ESP32S3`, added in a prior board port). Fix it while adding the 6th board rather than perpetuating the drift.

Find:
```c
/** @file
 *  @brief Board-level hardware abstraction — pin map and feature flags.
 *
 *  One of `BOARD_HELTEC_V2`, `BOARD_FEATHERS3_D`, or
 *  `BOARD_ADAFRUIT_QTPY_ESP32_PICO` is defined by the top-level CMakeLists.txt
 *  based on the `BOARD` variable (default `heltec_v2`). All module .c/.h files
 *  include this header and reference pins / features by the macros below —
 *  never by raw GPIO numbers.
```
Replace with:
```c
/** @file
 *  @brief Board-level hardware abstraction — pin map and feature flags.
 *
 *  One of `BOARD_HELTEC_V2`, `BOARD_FEATHERS3_D`,
 *  `BOARD_ADAFRUIT_QTPY_ESP32_PICO`, `BOARD_SEEED_XIAO_ESP32S3`, or
 *  `BOARD_HELTEC_WIFI_LORA32_V4_R2` is defined by the top-level
 *  CMakeLists.txt based on the `BOARD` variable (default `heltec_v2`). All
 *  module .c/.h files include this header and reference pins / features by
 *  the macros below — never by raw GPIO numbers.
```

- [ ] **Step 6: Verify the build still succeeds**

```powershell
& .\_build.cmd heltec_wifi_lora32_v4_r2 2>&1 | Select-Object -Last 10
```
Expected: `Project build complete.` (Doc-comment-only change — this is a regression check, not a new behavior test.)

- [ ] **Step 7: Commit**

```powershell
git add main/hal.h
git commit -m "Add BOARD_HELTEC_WIFI_LORA32_V4_R2 pin map to hal.h"
```

---

### Task 4: `main/i2c_bus.c` / `main/i2c_bus.h` / `main/main.c` — dedicated OLED bus

**Files:**
- Modify: `main/i2c_bus.c:13-19` (widen the `s_bus_secondary` guard), `main/i2c_bus.c:90-122` (`i2c_bus_get_secondary()` — add a new branch), `main/i2c_bus.c:128-140` (`i2c_bus_finalize()` — stop silently no-opping on this board)
- Modify: `main/i2c_bus.h:11-33` (`@file` per-board doc comment — add a paragraph), `main/i2c_bus.h:64-79` (separate function-level doc comment above `i2c_bus_get_secondary()`)
- Modify: `main/main.c:1006-1016` (`PROBE_ON_BOTH_BUSES` macro — guard the secondary-bus fallback)

**Interfaces:**
- Consumes: `PIN_OLED_SDA=17`, `PIN_OLED_SCL=18` from Task 3's `hal.h`. Also `BOARD_HELTEC_WIFI_LORA32_V4_R2` for the `#if`/`#elif` guards.
- Produces: `i2c_bus_get_secondary()` now returns a valid, always-on `i2c_master_bus_handle_t` on `I2C_NUM_1` for this board (previously it unconditionally returned `NULL` for every board except FeatherS3-D). No signature change — `main/display.c` (Task 5) and any future consumer keep calling it exactly as before. `PROBE_ON_BOTH_BUSES` keeps its existing macro signature (`init_fn, present_fn, bus1`) — only its internal expansion changes for this board.

- [ ] **Step 1: Widen the `s_bus_secondary` static's board guard**

In `main/i2c_bus.c`, find:
```c
static i2c_master_bus_handle_t s_bus_primary    = NULL;
#if defined(BOARD_FEATHERS3_D)
// Only the FeatherS3-D ever brings the second STEMMA QT bus online; on the
// other boards the declaration would trip -Wunused-variable. Keep the
// declaration scoped to the boards that actually read/write it.
static i2c_master_bus_handle_t s_bus_secondary  = NULL;
#endif
```
Replace with:
```c
static i2c_master_bus_handle_t s_bus_primary    = NULL;
#if defined(BOARD_FEATHERS3_D) || defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
// Only boards with a real second I²C controller ever bring this online; on
// the other boards the declaration would trip -Wunused-variable. Keep the
// declaration scoped to the boards that actually read/write it.
static i2c_master_bus_handle_t s_bus_secondary  = NULL;
#endif
```

- [ ] **Step 2: Add the new branch inside `i2c_bus_get_secondary()`**

Find:
```c
i2c_master_bus_handle_t i2c_bus_get_secondary(void) {
#if defined(BOARD_FEATHERS3_D)
    if (s_bus_secondary) return s_bus_secondary;

    // Enable LDO2 (powers 3V3.2 → STEMMA2 + onboard NeoPixel rail).
    // Default state of IO39 is hi-Z + internal pull-down → LDO2 OFF →
    // STEMMA2 V+ pin dead, no device can ACK there. Drive HIGH to wake.
    gpio_reset_pin(GPIO_NUM_39);
    gpio_set_direction(GPIO_NUM_39, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_39, 1);
    vTaskDelay(pdMS_TO_TICKS(10));                     // 3V3.2 rail settle (LDO2 turn-on ~1 ms)

    i2c_master_bus_config_t cfg = {
        .i2c_port             = I2C_NUM_1,
        .sda_io_num           = 16,                    // FeatherS3-D STEMMA2 SDA
        .scl_io_num           = 15,                    // FeatherS3-D STEMMA2 SCL
        .clk_source           = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt    = 7,
        .flags.enable_internal_pullup = true,          // STEMMA2 has no PCB pull-ups
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus_secondary);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "secondary bus init failed: %s", esp_err_to_name(err));
        s_bus_secondary = NULL;
        gpio_set_level(GPIO_NUM_39, 0);                // back off — don't burn LDO2 for nothing
        return NULL;
    }
    ESP_LOGI(TAG, "secondary bus up (I2C_NUM_1, SDA=16 SCL=15) — LDO2 enabled");
    return s_bus_secondary;
#else
    return NULL;   // Heltec / QT Py: no second bus on this board
#endif
}
```
Replace with:
```c
i2c_master_bus_handle_t i2c_bus_get_secondary(void) {
#if defined(BOARD_FEATHERS3_D)
    if (s_bus_secondary) return s_bus_secondary;

    // Enable LDO2 (powers 3V3.2 → STEMMA2 + onboard NeoPixel rail).
    // Default state of IO39 is hi-Z + internal pull-down → LDO2 OFF →
    // STEMMA2 V+ pin dead, no device can ACK there. Drive HIGH to wake.
    gpio_reset_pin(GPIO_NUM_39);
    gpio_set_direction(GPIO_NUM_39, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_39, 1);
    vTaskDelay(pdMS_TO_TICKS(10));                     // 3V3.2 rail settle (LDO2 turn-on ~1 ms)

    i2c_master_bus_config_t cfg = {
        .i2c_port             = I2C_NUM_1,
        .sda_io_num           = 16,                    // FeatherS3-D STEMMA2 SDA
        .scl_io_num           = 15,                    // FeatherS3-D STEMMA2 SCL
        .clk_source           = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt    = 7,
        .flags.enable_internal_pullup = true,          // STEMMA2 has no PCB pull-ups
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus_secondary);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "secondary bus init failed: %s", esp_err_to_name(err));
        s_bus_secondary = NULL;
        gpio_set_level(GPIO_NUM_39, 0);                // back off — don't burn LDO2 for nothing
        return NULL;
    }
    ESP_LOGI(TAG, "secondary bus up (I2C_NUM_1, SDA=16 SCL=15) — LDO2 enabled");
    return s_bus_secondary;
#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    if (s_bus_secondary) return s_bus_secondary;

    // No gating GPIO on this board — the OLED's bus (PIN_OLED_SDA/SCL) is a
    // fixed module-internal bus, always powered from the module's own
    // always-on 3.3V rail (NOT the switchable Vext/"Ve" pin — see hal.h's
    // HAL_HAS_VEXT_GATE comment). Bring the controller up directly.
    i2c_master_bus_config_t cfg = {
        .i2c_port             = I2C_NUM_1,
        .sda_io_num           = PIN_OLED_SDA,
        .scl_io_num           = PIN_OLED_SCL,
        .clk_source           = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt    = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus_secondary);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OLED bus init failed: %s", esp_err_to_name(err));
        s_bus_secondary = NULL;
        return NULL;
    }
    ESP_LOGI(TAG, "secondary bus up (I2C_NUM_1, SDA=%d SCL=%d) — onboard OLED, always-on",
             PIN_OLED_SDA, PIN_OLED_SCL);
    return s_bus_secondary;
#else
    return NULL;   // Heltec / QT Py: no second bus on this board
#endif
}
```

- [ ] **Step 3: Widen `i2c_bus_finalize()` so it doesn't silently no-op on this board**

`i2c_bus_finalize()`'s entire body is scoped to `#if defined(BOARD_FEATHERS3_D)` with no `#else` — it compiles to an empty function for every other board. That's fine for boards with no secondary bus (nothing to finalize), but on this board `i2c_bus_get_secondary()` always creates a real, always-on handle — if the OLED fails to probe on a given boot, `i2c_bus_secondary_keep_alive()` is never called, and this board's `I2C_NUM_1` handle would be silently leaked with zero log trace. This board's bus is never torn down (it has no gating GPIO to drop), so the fix is a log-only branch — but an explicit one, not a silent no-op.

Find:
```c
void i2c_bus_finalize(void) {
#if defined(BOARD_FEATHERS3_D)
    if (s_bus_secondary && !s_secondary_kept) {
        i2c_del_master_bus(s_bus_secondary);
        s_bus_secondary = NULL;
        gpio_set_level(GPIO_NUM_39, 0);                // LDO2 off — saves ~5–10 mA quiescent
        ESP_LOGI(TAG, "secondary bus torn down — no consumer (LDO2 off)");
    } else if (s_bus_secondary) {
        ESP_LOGI(TAG, "secondary bus kept alive — at least one consumer is using it");
    }
    // s_bus_secondary == NULL && !kept: secondary was never requested, nothing to do.
#endif
}
```
Replace with:
```c
void i2c_bus_finalize(void) {
#if defined(BOARD_FEATHERS3_D)
    if (s_bus_secondary && !s_secondary_kept) {
        i2c_del_master_bus(s_bus_secondary);
        s_bus_secondary = NULL;
        gpio_set_level(GPIO_NUM_39, 0);                // LDO2 off — saves ~5–10 mA quiescent
        ESP_LOGI(TAG, "secondary bus torn down — no consumer (LDO2 off)");
    } else if (s_bus_secondary) {
        ESP_LOGI(TAG, "secondary bus kept alive — at least one consumer is using it");
    }
    // s_bus_secondary == NULL && !kept: secondary was never requested, nothing to do.
#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    // No gating GPIO on this board — the bus is never a teardown candidate,
    // it's dedicated permanently to the onboard OLED. This branch exists
    // only so an OLED probe failure leaves a log trace instead of silently
    // leaking the I2C_NUM_1 handle with zero diagnostic trail.
    if (s_bus_secondary && !s_secondary_kept) {
        ESP_LOGW(TAG, "OLED bus is up but no consumer claimed it — "
                      "OLED probe likely failed this boot");
    }
#endif
}
```

- [ ] **Step 4: Update the `i2c_bus.h` `@file` doc comment**

In `main/i2c_bus.h`, find:
```c
 *    QT Py ESP32-PICO: primary bus on STEMMA QT (IO22 SDA / IO19 SCL).
 *      Secondary bus: not available.
 *
 *  Lazy + sheddable secondary lets the multi-page display task and any
 *  future STEMMA2-attached sensor opt into bus 2 without forcing it
 *  always-on for deployments that only use STEMMA1.
 */
```
Replace with:
```c
 *    QT Py ESP32-PICO: primary bus on STEMMA QT (IO22 SDA / IO19 SCL).
 *      Secondary bus: not available.
 *
 *    Heltec WiFi LoRa 32 V4-R2: primary bus on the external env-sensor
 *      header (IO48 SDA / IO47 SCL). Secondary bus on the module's fixed
 *      internal OLED bus (IO17 SDA / IO18 SCL) — always-on (no gating
 *      GPIO, unlike FeatherS3-D's LDO2-gated STEMMA2), created eagerly on
 *      first call and never torn down. Unlike every other board's
 *      secondary bus, this one can only ever host the onboard OLED — it
 *      isn't wired to anything else, so there's no "no consumer, shed it"
 *      case to handle.
 *
 *  Lazy + sheddable secondary lets the multi-page display task and any
 *  future STEMMA2-attached sensor opt into bus 2 without forcing it
 *  always-on for deployments that only use STEMMA1. (Heltec V4-R2 is the
 *  exception — see above.)
 */
```

- [ ] **Step 5: Fix the separate, stale function-level doc comment above `i2c_bus_get_secondary()`**

`i2c_bus.h` has a SECOND, separate doc comment directly above the `i2c_bus_get_secondary()` declaration (distinct from the `@file` block just updated in Step 4) that still says the secondary bus is FeatherS3-D-only and always returns NULL elsewhere.

Find:
```c
/** @brief Get the secondary I²C bus handle (FeatherS3-D STEMMA2 only).
 *
 *  On FeatherS3-D: lazy init on first call — drives IO39 HIGH (LDO2
 *  enable), waits 10 ms for the rail to settle, then creates the
 *  I²C controller on I²C_NUM_1 / IO15 / IO16. Subsequent calls return
 *  the cached handle.
 *
 *  On Heltec / QT Py / any board without a second bus: always returns
 *  NULL. Callers should treat NULL as "no second bus on this board"
 *  and skip secondary-bus probing gracefully.
 *
 *  Calling this enables LDO2 — adds ~5–10 mA continuous draw (LDO2
 *  quiescent + onboard NeoPixel idle on FeatherS3-D). If no consumer
 *  subsequently calls i2c_bus_secondary_keep_alive(), i2c_bus_finalize()
 *  will tear it back down.
 */
i2c_master_bus_handle_t i2c_bus_get_secondary(void);
```
Replace with:
```c
/** @brief Get the secondary I²C bus handle (FeatherS3-D STEMMA2, or the
 *  Heltec WiFi LoRa 32 V4-R2's dedicated OLED bus).
 *
 *  On FeatherS3-D: lazy init on first call — drives IO39 HIGH (LDO2
 *  enable), waits 10 ms for the rail to settle, then creates the
 *  I²C controller on I²C_NUM_1 / IO15 / IO16. Subsequent calls return
 *  the cached handle.
 *
 *  On Heltec WiFi LoRa 32 V4-R2: lazy init on first call — no gating GPIO
 *  (this board's OLED bus is always powered), creates the I²C controller
 *  on I²C_NUM_1 / IO17 / IO18. Always non-NULL once created; never torn
 *  down (see i2c_bus_finalize()'s log-only branch for this board).
 *
 *  On Heltec V2 / QT Py / any board without a second bus: always returns
 *  NULL. Callers should treat NULL as "no second bus on this board"
 *  and skip secondary-bus probing gracefully.
 *
 *  Calling this on FeatherS3-D enables LDO2 — adds ~5–10 mA continuous draw
 *  (LDO2 quiescent + onboard NeoPixel idle). If no consumer subsequently
 *  calls i2c_bus_secondary_keep_alive(), i2c_bus_finalize() will tear it
 *  back down. On Heltec WiFi LoRa 32 V4-R2 there is no LDO to drop — the
 *  bus stays up regardless.
 */
i2c_master_bus_handle_t i2c_bus_get_secondary(void);
```

- [ ] **Step 6: Guard `main/main.c`'s `PROBE_ON_BOTH_BUSES` macro so sensor drivers never probe the OLED-only bus**

`main.c`'s `PROBE_ON_BOTH_BUSES` macro (used for env/PM/noise sensors, GNSS, and VEML7700) has no per-board guard — it falls back to `i2c_bus_get_secondary()` unconditionally whenever the primary-bus probe fails. On every other board, that's correct (the secondary bus is either NULL or a general-purpose STEMMA2 connector any sensor might occupy). On this board, once Task 4 lands, `i2c_bus_get_secondary()` always returns a valid handle — but it's permanently and exclusively the onboard SSD1315's bus. Every sensor driver would spuriously probe the OLED on every boot.

In `main/main.c`, find:
```c
    // V2.3.29: dual-bus device probing.
    //
    // i2c_bus.c owns both buses (primary always-on, secondary lazy +
    // sheddable). For each sensor module we try the primary bus first;
    // if no device was found, we ask for the secondary bus (which
    // lazily enables LDO2 on FeatherS3-D, returns NULL on Heltec / QT Py)
    // and probe again. On a hit, mark the secondary bus as kept-alive
    // so i2c_bus_finalize() below doesn't tear it down.
    //
    // Display does its own dual-bus auto-detect inside display_setup()
    // (and calls i2c_bus_secondary_keep_alive() itself if it lands on
    // the secondary). After all init, i2c_bus_finalize() drops LDO2 if
    // nothing — sensor or display — ended up on STEMMA2.
    //
    // V2.5.19: select the primary-bus pin route from config BEFORE the first
    // i2c_bus_get_primary() below caches the bus. No-op except on QT Py
    // (HAL_HAS_I2C_PINOUT_SWITCH); reboot-required by construction.
    i2c_bus_set_primary_pinout(g_cfg.i2c_pinout);
    i2c_master_bus_handle_t bus1 = i2c_bus_get_primary();

    // V2.6.6: MAX17048 fuel gauge is a fixed onboard part on STEMMA1/
    // primary — unlike the pluggable env/PM/noise sensors below, it never
    // needs the secondary-bus fallback probe.
    fuel_gauge_init(bus1);

    // Helper macro: try a sensor's init on bus 1; if no device bound,
    // try bus 2; if a device was found there, keep the bus alive.
    #define PROBE_ON_BOTH_BUSES(init_fn, present_fn, bus1)                  \
        do {                                                                \
            init_fn(bus1);                                                  \
            if (!present_fn()) {                                            \
                i2c_master_bus_handle_t _b2 = i2c_bus_get_secondary();      \
                if (_b2) {                                                  \
                    init_fn(_b2);                                           \
                    if (present_fn()) i2c_bus_secondary_keep_alive();       \
                }                                                           \
            }                                                               \
        } while (0)
```
Replace with:
```c
    // V2.3.29: dual-bus device probing.
    //
    // i2c_bus.c owns both buses (primary always-on, secondary lazy +
    // sheddable). For each sensor module we try the primary bus first;
    // if no device was found, we ask for the secondary bus (which
    // lazily enables LDO2 on FeatherS3-D, returns NULL on Heltec / QT Py)
    // and probe again. On a hit, mark the secondary bus as kept-alive
    // so i2c_bus_finalize() below doesn't tear it down.
    //
    // Display does its own dual-bus auto-detect inside display_setup()
    // (and calls i2c_bus_secondary_keep_alive() itself if it lands on
    // the secondary). After all init, i2c_bus_finalize() drops LDO2 if
    // nothing — sensor or display — ended up on STEMMA2.
    //
    // Heltec WiFi LoRa 32 V4-R2 is the one exception: its secondary bus is
    // permanently and exclusively the onboard OLED, never a general-purpose
    // STEMMA-style connector — see PROBE_ON_BOTH_BUSES below, which skips
    // the secondary-bus fallback entirely on this board.
    //
    // V2.5.19: select the primary-bus pin route from config BEFORE the first
    // i2c_bus_get_primary() below caches the bus. No-op except on QT Py
    // (HAL_HAS_I2C_PINOUT_SWITCH); reboot-required by construction.
    i2c_bus_set_primary_pinout(g_cfg.i2c_pinout);
    i2c_master_bus_handle_t bus1 = i2c_bus_get_primary();

    // V2.6.6: MAX17048 fuel gauge is a fixed onboard part on STEMMA1/
    // primary — unlike the pluggable env/PM/noise sensors below, it never
    // needs the secondary-bus fallback probe.
    fuel_gauge_init(bus1);

    // Helper macro: try a sensor's init on bus 1; if no device bound,
    // try bus 2; if a device was found there, keep the bus alive.
    //
    // Heltec WiFi LoRa 32 V4-R2: the secondary bus is the onboard OLED,
    // never a pluggable sensor connector, so the fallback probe is
    // compiled out entirely for this board — it would otherwise probe the
    // display's bus on every sensor driver, every boot, for no reason.
#if defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    #define PROBE_ON_BOTH_BUSES(init_fn, present_fn, bus1)                  \
        do {                                                                \
            init_fn(bus1);                                                  \
        } while (0)
#else
    #define PROBE_ON_BOTH_BUSES(init_fn, present_fn, bus1)                  \
        do {                                                                \
            init_fn(bus1);                                                  \
            if (!present_fn()) {                                            \
                i2c_master_bus_handle_t _b2 = i2c_bus_get_secondary();      \
                if (_b2) {                                                  \
                    init_fn(_b2);                                           \
                    if (present_fn()) i2c_bus_secondary_keep_alive();       \
                }                                                           \
            }                                                               \
        } while (0)
#endif
```

- [ ] **Step 7: Verify the build still succeeds**

```powershell
& .\_build.cmd heltec_wifi_lora32_v4_r2 2>&1 | Select-Object -Last 25
```
Expected: `Project build complete.` (This is a regression check — no functional test is possible without hardware. The i2c_bus.c change is inert until Task 5's `display.c` actually calls `i2c_bus_get_secondary()` and finds a display.)

- [ ] **Step 8: Structural check — confirm the new branch compiled in, not silently skipped**

```powershell
Select-String -Path "build_heltec_wifi_lora32_v4_r2\CMakeFiles\geiger_v2.elf.dir\i2c_bus.c.obj" -Pattern "onboard OLED" -SimpleMatch -ErrorAction SilentlyContinue
```
This is a weak check (object files aren't reliably greppable text); the authoritative check is Step 7's successful `Project build complete.` combined with re-reading the diffs in Steps 2, 3, and 6. Skip this step if the object-file grep finds nothing — that's expected (compiled/optimized object files don't usually retain source comments as literal text); do not treat a miss here as a build failure.

- [ ] **Step 9: Regression-check that other boards' sensor probing is unaffected**

```powershell
& .\_build.cmd feathers3_d 2>&1 | Select-Object -Last 10
```
Expected: `Project build complete.` (Confirms the `#if defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)` guard around `PROBE_ON_BOTH_BUSES` didn't disturb FeatherS3-D's `#else` branch, which is textually adjacent and must keep its full dual-bus fallback behavior.)

- [ ] **Step 10: Commit**

```powershell
git add main/i2c_bus.c main/i2c_bus.h main/main.c
git commit -m "Add dedicated always-on OLED bus for Heltec WiFi LoRa 32 V4-R2"
```

---

### Task 5: `main/display.c` / `main/display.h` / `main/http_server.c` — OLED support, panel heuristic, and board-name consistency

**Files:**
- Modify: `main/display.c:1-19` (top-of-file per-board comment), `main/display.c:70-74` (`OLED_CHIP_NAME` chain), `main/display.c:497-536` (probe order — skip the primary-bus probe and SerLCD-wake delay for this board), `main/display.c:580-592` (`DISPLAY_MODE_AUTO` heuristic — explicit branch), `main/display.c` bus-label strings (inside `display_setup()`)
- Modify: `main/display.h:3-30` (top-of-file per-board panel/bus doc comment)
- Modify: `main/http_server.c:2158-2173` (`UPLOAD_PROMPT_BOARD` OTA-page board-name chain)

**Interfaces:**
- Consumes: `BOARD_HELTEC_WIFI_LORA32_V4_R2`, and (indirectly, via `i2c_bus_get_secondary()`) Task 4's new bus branch.
- Produces: nothing new consumed elsewhere — this task changes boot-log/status-log text accuracy, the boot-time probe order for this board, the multipage-vs-single-page display decision, and the OTA page's board-name label. Behavioral change to the probe cascade for THIS BOARD ONLY: it now tries the secondary (OLED) bus first and skips the SerLCD-wake delay, instead of always probing the primary bus first and unconditionally sleeping 500 ms before it can ever reach the always-present onboard OLED.

- [ ] **Step 1: Update the top-of-file per-board comment**

Find:
```c
// OLED display — SSD1306 / SSD1309 128x64 over I2C.
// Hand-rolled driver (page-addressing mode) — no U8g2 dependency. SSD1306
// and SSD1309 are register-compatible (init sequence + command set), so one
// driver handles both. Per-board chip name comes from OLED_CHIP_NAME below
// for accurate boot-log identification.
//
//   Heltec V2 / Heltec V2 4MB : onboard SSD1306 on the shared env_sensor bus
//                               (SDA=GPIO4, SCL=GPIO15, RST=GPIO16).
//   FeatherS3-D               : external SSD1309 breakout (Core Electronics
//                               CE09964) on STEMMA2 (SDA=IO16, SCL=IO15),
//                               powered from LDO2 — see bring_up_stemma2_bus.
//
// Layout: boot splash, then either the radiation-focused running screen
// (time + nSv/h on top, big CPM in the middle, status line at the bottom)
// or the Environment view (T/RH/P at 2x font, no status line).
//
// On boards without an OLED (HAL_HAS_OLED == 0), the entire driver compiles
// down to no-op stubs at the bottom of this file; callers in main.c don't
// need to know whether a display is fitted.
```
Replace with:
```c
// OLED display — SSD1306 / SSD1309 / SSD1315 128x64 over I2C.
// Hand-rolled driver (page-addressing mode) — no U8g2 dependency. SSD1306,
// SSD1309, and SSD1315 are register-compatible (init sequence + command
// set), so one driver handles all three. Per-board chip name comes from
// OLED_CHIP_NAME below for accurate boot-log identification.
//
//   Heltec V2 / Heltec V2 4MB : onboard SSD1306 on the shared env_sensor bus
//                               (SDA=GPIO4, SCL=GPIO15, RST=GPIO16).
//   FeatherS3-D               : external SSD1309 breakout (Core Electronics
//                               CE09964) on STEMMA2 (SDA=IO16, SCL=IO15),
//                               powered from LDO2 — see bring_up_stemma2_bus.
//   Heltec WiFi LoRa 32 V4-R2 : onboard SSD1315 on a bus dedicated
//                               permanently to the display (SDA=GPIO17,
//                               SCL=GPIO18, RST=GPIO21) — module-internal,
//                               not shared with the env-sensor bus at all
//                               (unlike every board above). Always present
//                               (on-module, not a plug-in breakout) — this
//                               board probes the secondary bus FIRST (see
//                               Step 6 below), since the primary bus can
//                               never have a display on it.
//
// Layout: boot splash, then either the radiation-focused running screen
// (time + nSv/h on top, big CPM in the middle, status line at the bottom)
// or the Environment view (T/RH/P at 2x font, no status line).
//
// On boards without an OLED (HAL_HAS_OLED == 0), the entire driver compiles
// down to no-op stubs at the bottom of this file; callers in main.c don't
// need to know whether a display is fitted.
```

- [ ] **Step 2: Extend the `OLED_CHIP_NAME` chain**

Find:
```c
#if defined(BOARD_FEATHERS3_D)
    #define OLED_CHIP_NAME  "SSD1309"
#else
    #define OLED_CHIP_NAME  "SSD1306"
#endif
```
Replace with:
```c
#if defined(BOARD_FEATHERS3_D)
    #define OLED_CHIP_NAME  "SSD1309"
#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    #define OLED_CHIP_NAME  "SSD1315"
#else
    #define OLED_CHIP_NAME  "SSD1306"
#endif
```

- [ ] **Step 3: Make the bus-label strings board-aware**

Find (inside `display_setup()`):
```c
    const char *bus_label = "STEMMA1";
```
Replace with:
```c
#if defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    const char *bus_label = "env-sensor-bus";
#else
    const char *bus_label = "STEMMA1";
#endif
```

Find:
```c
    // --- Fall through to secondary bus ---------------------------------
    bus = i2c_bus_get_secondary();
    if (!bus) {
        ESP_LOGW(TAG, "no display on STEMMA1, no secondary bus on this board — "
                      "display disabled");
        return false;
    }
    bus_label = "STEMMA2";
```
Replace with:
```c
    // --- Fall through to secondary bus ---------------------------------
    bus = i2c_bus_get_secondary();
    if (!bus) {
        ESP_LOGW(TAG, "no display on %s, no secondary bus on this board — "
                      "display disabled", bus_label);
        return false;
    }
#if defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    bus_label = "onboard-OLED-bus";
#else
    bus_label = "STEMMA2";
#endif
```

Find (the final fallback warning, after both probes on the secondary bus fail):
```c
    ESP_LOGW(TAG, "no display found on STEMMA1 or STEMMA2 — display disabled");
```
Replace with:
```c
    ESP_LOGW(TAG, "no display found on either I2C bus — display disabled");
```

- [ ] **Step 4: Skip the primary-bus probe and SerLCD-wake delay for this board**

`display_setup()` always probes the primary bus first (correct for every other board — it's the cheapest way to find a display if one happens to be there). On this board, no display can EVER be on the primary/env-sensor bus — the OLED is wired exclusively to the secondary bus. Probing primary first wastes a full probe-timeout, and then the code unconditionally sleeps 500 ms (meant only to wake an occasionally-present SerLCD's bootloader) before it can even reach the always-present onboard OLED. Fix: for this board only, skip straight to the secondary bus and skip the SerLCD delay (it's an OLED, never a SerLCD).

Find (this is the state of the function after Steps 1-3 above have been applied):
```c
    // V2.3.29: auto-detect display across BOTH I²C buses. Probe primary
    // first since LDO2 is off until the secondary call enables it — lets
    // the dual-bus probe terminate quickly + cheaply if the display lives
    // on bus 1. The SerLCD ATmega bootloader's ~500 ms wait only fires
    // before the SECONDARY probe (bus 1 has been powered since boot, by
    // display_setup time the SerLCD has had hundreds of ms to wake).
    //
    // Boards without a secondary bus (Heltec, QT Py): i2c_bus_get_secondary()
    // returns NULL and the bus-2 branch is silently skipped.

    i2c_master_bus_handle_t bus = i2c_bus_get_primary();
    if (!bus) {
        ESP_LOGW(TAG, "no primary I²C bus — display disabled");
        return false;
    }

#if defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    const char *bus_label = "env-sensor-bus";
#else
    const char *bus_label = "STEMMA1";
#endif

    // --- Try primary bus -----------------------------------------------
    if (try_serlcd_on_bus(bus, show_display, brightness_pct)) {
        ESP_LOGI(TAG, "display backend: SerLCD at 0x72 on %s (show=%d brightness=%d%%)",
                 bus_label, show_display, brightness_pct);
        goto task_spawn;
    }
    {
        uint8_t oled_addr = 0;
        if (try_oled_on_bus(bus, &oled_addr)) {
            apply_oled_init_sequence();
            s_show    = show_display;
            s_backend = BACKEND_OLED;
            display_set_contrast(brightness_pct);
            ESP_LOGI(TAG, "display backend: %s at 0x%02X on %s (show=%d brightness=%d%%)",
                     OLED_CHIP_NAME, oled_addr, bus_label,
                     show_display, brightness_pct);
            goto task_spawn;
        }
    }

    // --- Fall through to secondary bus ---------------------------------
    bus = i2c_bus_get_secondary();
    if (!bus) {
        ESP_LOGW(TAG, "no display on %s, no secondary bus on this board — "
                      "display disabled", bus_label);
        return false;
    }
#if defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    bus_label = "onboard-OLED-bus";
#else
    bus_label = "STEMMA2";
#endif

    // SerLCD ATmega bootloader needs ~500 ms after LDO2 enable. (Bus 1
    // probes above didn't need this — bus 1 has been powered since boot.)
    vTaskDelay(pdMS_TO_TICKS(500));

    if (try_serlcd_on_bus(bus, show_display, brightness_pct)) {
        i2c_bus_secondary_keep_alive();
        ESP_LOGI(TAG, "display backend: SerLCD at 0x72 on %s (show=%d brightness=%d%%)",
                 bus_label, show_display, brightness_pct);
        goto task_spawn;
    }
    {
        uint8_t oled_addr = 0;
        if (try_oled_on_bus(bus, &oled_addr)) {
            apply_oled_init_sequence();
            s_show    = show_display;
            s_backend = BACKEND_OLED;
            display_set_contrast(brightness_pct);
            i2c_bus_secondary_keep_alive();
            ESP_LOGI(TAG, "display backend: %s at 0x%02X on %s (show=%d brightness=%d%%)",
                     OLED_CHIP_NAME, oled_addr, bus_label,
                     show_display, brightness_pct);
            goto task_spawn;
        }
    }

    ESP_LOGW(TAG, "no display found on either I2C bus — display disabled");
    return false;
```
Replace with:
```c
    // V2.3.29: auto-detect display across BOTH I²C buses. Probe primary
    // first since LDO2 is off until the secondary call enables it — lets
    // the dual-bus probe terminate quickly + cheaply if the display lives
    // on bus 1. The SerLCD ATmega bootloader's ~500 ms wait only fires
    // before the SECONDARY probe (bus 1 has been powered since boot, by
    // display_setup time the SerLCD has had hundreds of ms to wake).
    //
    // Boards without a secondary bus (Heltec, QT Py): i2c_bus_get_secondary()
    // returns NULL and the bus-2 branch is silently skipped.
    //
    // Heltec WiFi LoRa 32 V4-R2 is the one exception to "probe primary
    // first": its OLED lives EXCLUSIVELY on the secondary bus — the
    // primary/env-sensor bus never has a display on it — so probing
    // primary first would pay a guaranteed probe-timeout plus the SerLCD
    // 500 ms wake delay on every single boot for no possible benefit. This
    // board skips straight to the secondary bus and skips the SerLCD
    // delay too (it's an OLED, never a SerLCD).

    i2c_master_bus_handle_t bus;
    const char *bus_label;

#if defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    bus = i2c_bus_get_secondary();
    if (!bus) {
        ESP_LOGW(TAG, "no secondary I²C bus — display disabled");
        return false;
    }
    bus_label = "onboard-OLED-bus";
#else
    bus = i2c_bus_get_primary();
    if (!bus) {
        ESP_LOGW(TAG, "no primary I²C bus — display disabled");
        return false;
    }
    bus_label = "STEMMA1";

    // --- Try primary bus -----------------------------------------------
    if (try_serlcd_on_bus(bus, show_display, brightness_pct)) {
        ESP_LOGI(TAG, "display backend: SerLCD at 0x72 on %s (show=%d brightness=%d%%)",
                 bus_label, show_display, brightness_pct);
        goto task_spawn;
    }
    {
        uint8_t oled_addr = 0;
        if (try_oled_on_bus(bus, &oled_addr)) {
            apply_oled_init_sequence();
            s_show    = show_display;
            s_backend = BACKEND_OLED;
            display_set_contrast(brightness_pct);
            ESP_LOGI(TAG, "display backend: %s at 0x%02X on %s (show=%d brightness=%d%%)",
                     OLED_CHIP_NAME, oled_addr, bus_label,
                     show_display, brightness_pct);
            goto task_spawn;
        }
    }

    // --- Fall through to secondary bus ---------------------------------
    bus = i2c_bus_get_secondary();
    if (!bus) {
        ESP_LOGW(TAG, "no display on %s, no secondary bus on this board — "
                      "display disabled", bus_label);
        return false;
    }
    bus_label = "STEMMA2";

    // SerLCD ATmega bootloader needs ~500 ms after LDO2 enable. (Bus 1
    // probes above didn't need this — bus 1 has been powered since boot.)
    vTaskDelay(pdMS_TO_TICKS(500));
#endif

    if (try_serlcd_on_bus(bus, show_display, brightness_pct)) {
        i2c_bus_secondary_keep_alive();
        ESP_LOGI(TAG, "display backend: SerLCD at 0x72 on %s (show=%d brightness=%d%%)",
                 bus_label, show_display, brightness_pct);
        goto task_spawn;
    }
    {
        uint8_t oled_addr = 0;
        if (try_oled_on_bus(bus, &oled_addr)) {
            apply_oled_init_sequence();
            s_show    = show_display;
            s_backend = BACKEND_OLED;
            display_set_contrast(brightness_pct);
            i2c_bus_secondary_keep_alive();
            ESP_LOGI(TAG, "display backend: %s at 0x%02X on %s (show=%d brightness=%d%%)",
                     OLED_CHIP_NAME, oled_addr, bus_label,
                     show_display, brightness_pct);
            goto task_spawn;
        }
    }

    ESP_LOGW(TAG, "no display found on either I2C bus — display disabled");
    return false;
```

- [ ] **Step 5: Give `DISPLAY_MODE_AUTO` an explicit branch for this board's panel size**

The multipage-vs-single-page heuristic only special-cases `BOARD_FEATHERS3_D`; every other board — including this new one — silently gets `resolved_multipage=false` by falling into the `#else`, with no per-board statement of *why* that's the right answer. Make the decision explicit rather than implicit-by-elimination.

Find:
```c
task_spawn:;
    // V2.4.9: finalise AUTO mode now that the panel has been detected.
    // The rule is panel-based:
    //   - SerLCD backend       → rotation (typical 20x4 char display has
    //                            room for the multi-page content)
    //   - Big OLED (SSD1309)   → rotation (per-board compile-time hint
    //                            — BOARD_FEATHERS3_D ships with a 2.42"
    //                            SSD1309 from Core Electronics)
    //   - Small OLED (SSD1306) → radiation (Heltec onboard 0.96" or
    //                            Adafruit 326 plugged into STEMMA QT)
    //
    // Caveat: SSD1306 and SSD1309 are register-compatible (we drive both
    // with the same init sequence) and there's no reliable chip-ID
    // register that differs. So the OLED size discrimination uses the
    // compile-time per-board hint. The edge case — Adafruit 326 (small)
    // plugged into a FeatherS3-D's STEMMA1 in place of the SSD1309 —
    // would auto-pick rotation incorrectly; the user can override via
    // display_mode = RADIATION in /config.
    if (mode == DISPLAY_MODE_AUTO) {
        bool resolved_multipage;
        if (s_backend == BACKEND_SERLCD) {
            resolved_multipage = true;
        } else {
#if defined(BOARD_FEATHERS3_D)
            resolved_multipage = true;   // SSD1309 2.42" — big enough for rotation
#else
            resolved_multipage = false;  // SSD1306 0.96" — radiation single page
#endif
        }
        s_is_multipage = resolved_multipage;
    }
```
Replace with:
```c
task_spawn:;
    // V2.4.9: finalise AUTO mode now that the panel has been detected.
    // The rule is panel-based:
    //   - SerLCD backend       → rotation (typical 20x4 char display has
    //                            room for the multi-page content)
    //   - Big OLED (SSD1309)   → rotation (per-board compile-time hint
    //                            — BOARD_FEATHERS3_D ships with a 2.42"
    //                            SSD1309 from Core Electronics)
    //   - Small OLED (SSD1306 / SSD1315) → radiation (Heltec onboard
    //                            0.96", Adafruit 326 plugged into STEMMA
    //                            QT, or Heltec WiFi LoRa 32 V4-R2's onboard
    //                            SSD1315 — assumed same compact 0.96"
    //                            footprint Heltec ships across its
    //                            V2/V3/V4 module line; NOT bench-verified
    //                            for this exact SKU, see Global Constraints)
    //
    // Caveat: SSD1306, SSD1309, and SSD1315 are register-compatible (we
    // drive all three with the same init sequence) and there's no reliable
    // chip-ID register that differs. So the OLED size discrimination uses
    // the compile-time per-board hint. The edge case — Adafruit 326 (small)
    // plugged into a FeatherS3-D's STEMMA1 in place of the SSD1309 —
    // would auto-pick rotation incorrectly; the user can override via
    // display_mode = RADIATION in /config.
    if (mode == DISPLAY_MODE_AUTO) {
        bool resolved_multipage;
        if (s_backend == BACKEND_SERLCD) {
            resolved_multipage = true;
        } else {
#if defined(BOARD_FEATHERS3_D)
            resolved_multipage = true;   // SSD1309 2.42" — big enough for rotation
#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
            resolved_multipage = false;  // SSD1315 0.96" (assumed) — radiation single page
#else
            resolved_multipage = false;  // SSD1306 0.96" — radiation single page
#endif
        }
        s_is_multipage = resolved_multipage;
    }
```

- [ ] **Step 6: Update `main/display.h`'s stale per-board doc comment**

`display.h`'s top-of-file comment enumerates panels/buses for only Heltec V2 and FeatherS3-D — this file is separate from `display.c` and wasn't touched by Steps 1-5 above.

Find:
```c
/** @file
 *  @brief OLED display driver — SSD1306 / SSD1309 128x64 over I2C.
 *
 *  Per-board panel + bus:
 *    Heltec V2 (+ 4MB clone) : onboard SSD1306 on the env_sensor I2C bus
 *                              (SDA=GPIO4, SCL=GPIO15, dedicated reset=GPIO16).
 *    FeatherS3-D             : external SSD1309 breakout on STEMMA2
 *                              (SDA=IO16, SCL=IO15), powered from LDO2;
 *                              4-pin module — no reset line, chip POR only.
 *
 *  SSD1306 and SSD1309 are register-compatible — the same init sequence and
 *  command set drive both. Boot log distinguishes them via OLED_CHIP_NAME
 *  in display.c (per-board #define) so a glance at /log shows which silicon
 *  is fitted without needing to know the board variant.
```
Replace with:
```c
/** @file
 *  @brief OLED display driver — SSD1306 / SSD1309 / SSD1315 128x64 over I2C.
 *
 *  Per-board panel + bus:
 *    Heltec V2 (+ 4MB clone)   : onboard SSD1306 on the env_sensor I2C bus
 *                                (SDA=GPIO4, SCL=GPIO15, dedicated reset=GPIO16).
 *    FeatherS3-D               : external SSD1309 breakout on STEMMA2
 *                                (SDA=IO16, SCL=IO15), powered from LDO2;
 *                                4-pin module — no reset line, chip POR only.
 *    Heltec WiFi LoRa 32 V4-R2 : onboard SSD1315 on a bus dedicated
 *                                permanently to the display (SDA=GPIO17,
 *                                SCL=GPIO18, RST=GPIO21) — module-internal,
 *                                not the env-sensor bus. See i2c_bus.h for
 *                                why this board needs two I²C controllers.
 *
 *  SSD1306, SSD1309, and SSD1315 are register-compatible — the same init
 *  sequence and command set drive all three. Boot log distinguishes them
 *  via OLED_CHIP_NAME in display.c (per-board #define) so a glance at /log
 *  shows which silicon is fitted without needing to know the board variant.
```

- [ ] **Step 7: Add the OTA-page board-name arm in `main/http_server.c`**

The OTA update page's `UPLOAD_PROMPT_BOARD` chain has an arm for every existing board — without one for this board, it falls through to `#else` and shows "(unknown board)", exactly reproducing the V2.5.19 XIAO regression documented in this file's own comment.

Find:
```c
#elif BOARD_SEEED_XIAO_ESP32S3
    // V2.5.19: the XIAO target shipped in V2.4.25 but was never given a label
    // branch here, so it fell through to "(unknown board)" on the OTA page —
    // which, by elimination, was the only way to identify a XIAO build.
    #define UPLOAD_PROMPT_BOARD "<b style=\"color:red\">Seeed XIAO ESP32-S3</b>"
#else
    #define UPLOAD_PROMPT_BOARD "<b style=\"color:red\">(unknown board)</b>"
#endif
```
Replace with:
```c
#elif BOARD_SEEED_XIAO_ESP32S3
    // V2.5.19: the XIAO target shipped in V2.4.25 but was never given a label
    // branch here, so it fell through to "(unknown board)" on the OTA page —
    // which, by elimination, was the only way to identify a XIAO build.
    #define UPLOAD_PROMPT_BOARD "<b style=\"color:red\">Seeed XIAO ESP32-S3</b>"
#elif BOARD_HELTEC_WIFI_LORA32_V4_R2
    #define UPLOAD_PROMPT_BOARD "<b style=\"color:red\">Heltec WiFi LoRa 32 V4 (R2)</b>"
#else
    #define UPLOAD_PROMPT_BOARD "<b style=\"color:red\">(unknown board)</b>"
#endif
```

- [ ] **Step 8: Verify the build succeeds**

```powershell
& .\_build.cmd heltec_wifi_lora32_v4_r2 2>&1 | Select-Object -Last 25
```
Expected: `Project build complete.` This is the last source-code task — the firmware now builds cleanly for `heltec_wifi_lora32_v4_r2` with all HAL/display/I²C wiring in place.

- [ ] **Step 9: Regression-check the other five boards still build**

The `#if`/`#elif` chains touched in Tasks 3-5 are additive (new branches only), but verify no existing board's branch was accidentally altered:
```powershell
& .\_build.cmd heltec_v2 2>&1 | Select-Object -Last 10
& .\_build.cmd feathers3_d 2>&1 | Select-Object -Last 10
```
Expected: both end with `Project build complete.` (Running all 5 pre-existing boards is thorough but slow; these two cover both chip targets — `esp32` and `esp32s3` — and are the two boards whose `hal.h`/`i2c_bus.c`/`display.c`/`http_server.c` branches sit textually adjacent to the new one, so they're the most likely to show an accidental edit.)

- [ ] **Step 10: Commit**

```powershell
git add main/display.c main/display.h main/http_server.c
git commit -m "Add SSD1315 support, board-aware bus labels, and OTA board name for Heltec WiFi LoRa 32 V4-R2"
```

---

### Task 6: Version bump + CHANGELOG

**Files:**
- Modify: `main/version.h`
- Modify: `CHANGELOG.md`

**Interfaces:**
- Consumes: nothing.
- Produces: `VERSION_STR "V2.6.7"`, embedded in every board's next build via the existing `CMAKE_CONFIGURE_DEPENDS` mechanism (no `CMakeLists.txt` change needed — this is already wired).

- [ ] **Step 1: Bump the version**

In `main/version.h`, find:
```c
#define VERSION_STR "V2.6.6"
```
Replace with:
```c
#define VERSION_STR "V2.6.7"
```

- [ ] **Step 2: Add the CHANGELOG entry**

In `CHANGELOG.md`, find:
```markdown
---

## V2.6.6 — MAX17048 battery fuel gauge (FeatherS3-D): /status, MQTT, HA discovery, per-cycle log
```
Replace with:
```markdown
---

## V2.6.7 — New `heltec_wifi_lora32_v4_r2` board target (6th build target)

- **New board**: Heltec WiFi LoRa 32 V4 — base/R2 variant (ESP32-S3R2, 2 MB
  in-package quad PSRAM). Runs on a third-party PCB: the standard Multigeiger
  V2 mainboard populated with this Heltec module instead of the Heltec V2.
  NOT the V4-R8 variant (ESP32-S3R8, 8 MB PSRAM, different GPIOs) — the
  `_r2` suffix permanently disambiguates from a possible future R8 port.
- Two-I²C-bus architecture: the onboard SSD1315 OLED lives on a
  module-internal bus (GPIO17 SDA / GPIO18 SCL) completely separate from the
  external env-sensor bus exposed on the mainboard header (GPIO48 SDA /
  GPIO47 SCL) — every other supported board shares one bus between the two.
  Reuses the dual-bus abstraction built for FeatherS3-D's STEMMA1/STEMMA2
  split (`i2c_bus.c`'s `i2c_bus_get_secondary()`), extended with an
  always-on (no LDO gating) branch for this board's fixed OLED bus.
  `display.c` gains `SSD1315` as a third register-compatible OLED chip
  identity alongside SSD1306/SSD1309.
- Onboard LED (GPIO35) is active-HIGH per independent third-party firmware
  cross-reference (Heltec's own Arduino library defines no LED pin for this
  module at all). Piezo speaker uses GPIO26/GPIO5 — the source pin matrix
  marks both as "SPK" without distinguishing P/N; assignment is arbitrary
  and functionally inconsequential for a piezo.
- Hardware-reservation only for future LoRaWAN/Meshtastic work: GPIO
  7-14 (the SX1262 radio's dedicated internal SPI bus + front-end enable)
  are documented as reserved in `hal.h` but not driven by any code — no
  `HAL_HAS_LORA` flag, no radio driver, no config fields. LoRaWAN is a
  parallel connectivity mode (own OTAA/ABP join flow, no WiFi/internet
  dependency), not another TX-dispatch-table target, so it needs its own
  dedicated design work once scoped.
- CI matrix (`_build-boards.yml`) and release artefact count
  (`release.yml`'s `EXPECTED_BOARDS`) updated for the 6th board.
- Design spec:
  `docs/superpowers/specs/2026-07-06-heltec-wifi-lora32-v4-r2-board-port-design.md`.
- **Not bench-verified** — no hardware in hand this session. LED polarity,
  speaker P/N assignment, and PSRAM speed (80 MHz assumed) are flagged in
  `hal.h`/the sdkconfig overlay as first-flash verification items.

## V2.6.6 — MAX17048 battery fuel gauge (FeatherS3-D): /status, MQTT, HA discovery, per-cycle log
```

- [ ] **Step 3: Rebuild and verify the new version string is embedded**

```powershell
& .\_build.cmd heltec_wifi_lora32_v4_r2 2>&1 | Select-Object -Last 25
```
Expected: `Project build complete.`

```powershell
Get-Content build_heltec_wifi_lora32_v4_r2\geiger_v2.bin -Raw -Encoding Byte | ForEach-Object { [System.Text.Encoding]::ASCII.GetString($_) } | Select-String -Pattern "V2\.\d+\.\d+" -AllMatches | ForEach-Object { $_.Matches.Value } | Sort-Object -Unique
```
Expected: `V2.6.7` only (not `V2.6.6`) — confirms the version bump actually took effect in the binary, not just the source file.

- [ ] **Step 4: Commit**

```powershell
git add main/version.h CHANGELOG.md
git commit -m "Bump to V2.6.7 — Heltec WiFi LoRa 32 V4 (R2) board support"
```

---

### Task 7: CI workflow, web flasher manifest — board matrix, stale board-count comments, and install page

**Files:**
- Modify: `.github/workflows/_build-boards.yml:1-5` (header comment), `:52-57` (matrix), `:83-85` (target ternary)
- Modify: `.github/workflows/release.yml:1-2` (header comment), `:42-44` (changelog-check comment), `:107-114` (`EXPECTED_BOARDS` + artefact-count comment), `:180-181` (Pages-publish comment)
- Create: `docs/manifests/heltec_wifi_lora32_v4_r2.json`
- Modify: `docs/index.html` (board `<select>`)

**Interfaces:**
- Consumes: nothing (CI-only config + static web assets, not exercised locally).
- Produces: the 6-board CI matrix that both `build.yml` and `release.yml` call into via `_build-boards.yml`'s `workflow_call`; the 6th `<option>`/manifest pair the ESP Web Tools flasher at `docs/index.html` needs to offer this board.

- [ ] **Step 1: Add the board to the matrix**

In `.github/workflows/_build-boards.yml`, find:
```yaml
      matrix:
        board:
          - heltec_v2
          - heltec_v2_4mb
          - feathers3_d
          - adafruit_qtpy_esp32_pico
          - seeed_xiao_esp32s3
```
Replace with:
```yaml
      matrix:
        board:
          - heltec_v2
          - heltec_v2_4mb
          - feathers3_d
          - adafruit_qtpy_esp32_pico
          - seeed_xiao_esp32s3
          - heltec_wifi_lora32_v4_r2
```

- [ ] **Step 2: Fix the stale "5-board list" header comment in `_build-boards.yml`**

This file's own top-of-file comment calls itself "the SINGLE source of the 5-board list" — the kind of claim that becomes actively misleading the moment Step 1 above lands.

Find:
```yaml
# Reusable board-build matrix — the SINGLE source of the 5-board list, called
# by build.yml (release:false → CI build) and release.yml (release:true → build
# + merge-bin + version==tag check + staged artefacts). Adding/removing a board
# now means editing the matrix HERE only (plus EXPECTED_BOARDS in release.yml's
# count check, which is deliberately kept as an independent guard).
```
Replace with:
```yaml
# Reusable board-build matrix — the SINGLE source of the board list, called
# by build.yml (release:false → CI build) and release.yml (release:true → build
# + merge-bin + version==tag check + staged artefacts). Adding/removing a board
# now means editing the matrix HERE only (plus EXPECTED_BOARDS in release.yml's
# count check, which is deliberately kept as an independent guard).
```

- [ ] **Step 3: Extend the `target:` ternary**

Find:
```yaml
          # Mirrors CMakeLists.txt's IDF_TARGET selection (kept in sync manually):
          # ESP32-S3 for feathers3_d + seeed_xiao_esp32s3, esp32 for the rest.
          target: ${{ (matrix.board == 'feathers3_d' || matrix.board == 'seeed_xiao_esp32s3') && 'esp32s3' || 'esp32' }}
```
Replace with:
```yaml
          # Mirrors CMakeLists.txt's IDF_TARGET selection (kept in sync manually):
          # ESP32-S3 for feathers3_d + seeed_xiao_esp32s3 + heltec_wifi_lora32_v4_r2,
          # esp32 for the rest.
          target: ${{ (matrix.board == 'feathers3_d' || matrix.board == 'seeed_xiao_esp32s3' || matrix.board == 'heltec_wifi_lora32_v4_r2') && 'esp32s3' || 'esp32' }}
```

- [ ] **Step 4: Fix the three stale board-count comments in `release.yml`**

`release.yml` states "5 boards" / "25 artefacts" in three separate comments that don't get touched by the `EXPECTED_BOARDS` bump in Step 6 below — each needs its own fix or the file will contradict its own guard.

Find:
```yaml
# Release workflow — fire on tag push, build all 5 boards, create the GitHub
# Release with all 25 artefacts attached, and publish the web flasher to Pages.
```
Replace with:
```yaml
# Release workflow — fire on tag push, build all 6 boards, create the GitHub
# Release with all 30 artefacts attached, and publish the web flasher to Pages.
```

Find:
```yaml
  # V2.5.31: fail FAST if CHANGELOG.md has no section for this tag. Pre-V2.5.31
  # this only surfaced as a warning in the release job AFTER all 5 boards built
  # (minutes), shipping a placeholder release body (bit us on V2.4.2/.3). This
  # cheap check mirrors the heading match the extract step uses below.
```
Replace with:
```yaml
  # V2.5.31: fail FAST if CHANGELOG.md has no section for this tag. Pre-V2.5.31
  # this only surfaced as a warning in the release job AFTER all boards built
  # (minutes), shipping a placeholder release body (bit us on V2.4.2/.3). This
  # cheap check mirrors the heading match the extract step uses below.
```

Find:
```yaml
          # KEEP IN SYNC with the matrix in _build-boards.yml. Each board
          # contributes exactly 5 .bin files: geiger_v2, _merged, bootloader,
          # partition-table, ota_data_initial. Independent guard on the count.
```
Replace with:
```yaml
          # KEEP IN SYNC with the matrix in _build-boards.yml. Each board
          # contributes exactly 5 .bin files: geiger_v2, _merged, bootloader,
          # partition-table, ota_data_initial. Independent guard on the count.
          # (The per-board count of 5 is unrelated to and does not need to
          # change with the number of boards — see EXPECTED_BOARDS below.)
```

- [ ] **Step 5: Fix the stale Pages-publish comment in `release.yml`**

Find:
```yaml
  # Publish the browser "web flasher" (docs/) + the 5 merged firmware images to
  # GitHub Pages, so https://<owner>.github.io/<repo>/ always serves an install
  # page wired to THIS release's binaries.
```
Replace with:
```yaml
  # Publish the browser "web flasher" (docs/) + the 6 merged firmware images to
  # GitHub Pages, so https://<owner>.github.io/<repo>/ always serves an install
  # page wired to THIS release's binaries.
```

- [ ] **Step 6: Bump `EXPECTED_BOARDS` in `release.yml`**

Find:
```bash
          EXPECTED_BOARDS=5
```
Replace with:
```bash
          EXPECTED_BOARDS=6
```

- [ ] **Step 7: Create the web flasher manifest for this board**

The ESP Web Tools install page at `docs/index.html` picks a board by loading a per-board manifest JSON — every existing board has one under `docs/manifests/`; this board needs one too or it can never appear as a flashable option.

Create `docs/manifests/heltec_wifi_lora32_v4_r2.json`:
```json
{
  "name": "MultiGeiger V2 — Heltec WiFi LoRa 32 V4 (R2)",
  "version": "latest",
  "new_install_prompt_erase": true,
  "builds": [
    {
      "chipFamily": "ESP32-S3",
      "parts": [
        {
          "path": "../firmware/geiger_v2_merged_heltec_wifi_lora32_v4_r2.bin",
          "offset": 0
        }
      ]
    }
  ]
}
```
(Modeled directly on `docs/manifests/seeed_xiao_esp32s3.json`, the other ESP32-S3 board's manifest — same `chipFamily`, same single-part layout, filename following the `geiger_v2_merged_<board>.bin` convention the release workflow produces.)

- [ ] **Step 8: Add the board to `docs/index.html`'s dropdown**

Find:
```html
          <option value="manifests/seeed_xiao_esp32s3.json">Seeed XIAO ESP32-S3</option>
        </select>
```
Replace with:
```html
          <option value="manifests/seeed_xiao_esp32s3.json">Seeed XIAO ESP32-S3</option>
          <option value="manifests/heltec_wifi_lora32_v4_r2.json">Heltec WiFi LoRa 32 V4 (R2)</option>
        </select>
```

- [ ] **Step 9: Verify via grep (no local CI execution possible)**

```powershell
Select-String -Path ".github\workflows\_build-boards.yml" -Pattern "heltec_wifi_lora32_v4_r2"
Select-String -Path ".github\workflows\release.yml" -Pattern "EXPECTED_BOARDS=6"
Select-String -Path "docs\index.html" -Pattern "heltec_wifi_lora32_v4_r2"
Test-Path "docs\manifests\heltec_wifi_lora32_v4_r2.json"
```
Expected: all four checks return a match / `True`. This is a structural check only — the actual CI matrix, artefact-count guard, and web flasher page are exercised for real on the next push / tag / Pages deploy, not in this session.

- [ ] **Step 10: Commit**

```powershell
git add .github/workflows/_build-boards.yml .github/workflows/release.yml docs/manifests/heltec_wifi_lora32_v4_r2.json docs/index.html
git commit -m "CI + web flasher: add heltec_wifi_lora32_v4_r2 as the 6th board"
```

---

### Task 8: `README.md` — board count, board table, and build-command reference

**Files:**
- Modify: `README.md:3` (intro paragraph board count)
- Modify: `README.md:28-38` (board table + `SDKCONFIG_DEFAULTS` sentence)
- Modify: `README.md:200` (`idf.py` "Substitute ... for other boards" sentence)
- Modify: `README.md:206` (release workflow "five boards" sentence)

**Interfaces:**
- Consumes: nothing.
- Produces: nothing consumed by code — documentation only.

- [ ] **Step 1: Fix the stale board count in the intro paragraph**

Find:
```markdown
A ground-up C rewrite of the [MultiGeiger](https://github.com/ecocurious2/MultiGeiger) radiation sensor firmware, ported from Arduino / PlatformIO to **native ESP-IDF 6.0**. Runs on **five** ESP32 / ESP32-S3 board variants with a wide selection of optional environmental, particulate, noise, and ambient-light sensors. Uploads to **nine** public back-ends and publishes to MQTT (with Home Assistant Discovery) and remote syslog.
```
Replace with:
```markdown
A ground-up C rewrite of the [MultiGeiger](https://github.com/ecocurious2/MultiGeiger) radiation sensor firmware, ported from Arduino / PlatformIO to **native ESP-IDF 6.0**. Runs on **six** ESP32 / ESP32-S3 board variants with a wide selection of optional environmental, particulate, noise, and ambient-light sensors. Uploads to **nine** public back-ends and publishes to MQTT (with Home Assistant Discovery) and remote syslog.
```

- [ ] **Step 2: Update the board table header and add a row**

Find:
```markdown
### Boards (five build targets)

| Build target | MCU / module | Flash | Notes |
|---|---|---|---|
| `heltec_v2` | Heltec WiFi Kit 32 V2 (ESP32-D0WDQ6) | 8 MB | Onboard SSD1306 OLED. The original target — most production deployments. |
| `heltec_v2_4mb` | Heltec WiFi Kit 32 V2 clone | 4 MB | Same module silicon, smaller flash. Tight on heap during OTA — see V2.4.13 teardown logic in `main.c`. |
| `feathers3_d` | Unexpected Maker FeatherS3 with display (ESP32-S3) | 8 MB | Two STEMMA QT connectors (STEMMA1 on IO8/IO9, STEMMA2 LDO-gated on IO15/IO16). External I²C OLED via STEMMA. PSRAM — WiFi/lwIP/mbedTLS offloaded to PSRAM for sustained heap headroom. |
| `adafruit_qtpy_esp32_pico` | Adafruit QT Py ESP32-PICO | 8 MB | Compact form factor. Optional NeoPixel tick on pulse. PSRAM. |
| `seeed_xiao_esp32s3` | Seeed Studio XIAO ESP32-S3 | 8 MB | Tiny 21×17.5 mm — shares QT Py form factor + Geiger pin map (A0 / A1 / SCK), so one PCB design works for both. Onboard user LED blinks per pulse. PSRAM. |

Build/flash invocation takes a board argument — see `_build.cmd` / `_flash.cmd` / `_merge.cmd` helpers. All boards share the same `main/` source tree; differences are isolated in per-board `sdkconfig.defaults.<board>` and HAL pin map. PSRAM boards additionally include `sdkconfig.defaults.psram` (WiFi roaming app + PSRAM offload knobs).
```
Replace with:
```markdown
### Boards (six build targets)

| Build target | MCU / module | Flash | Notes |
|---|---|---|---|
| `heltec_v2` | Heltec WiFi Kit 32 V2 (ESP32-D0WDQ6) | 8 MB | Onboard SSD1306 OLED. The original target — most production deployments. |
| `heltec_v2_4mb` | Heltec WiFi Kit 32 V2 clone | 4 MB | Same module silicon, smaller flash. Tight on heap during OTA — see V2.4.13 teardown logic in `main.c`. |
| `feathers3_d` | Unexpected Maker FeatherS3 with display (ESP32-S3) | 8 MB | Two STEMMA QT connectors (STEMMA1 on IO8/IO9, STEMMA2 LDO-gated on IO15/IO16). External I²C OLED via STEMMA. PSRAM — WiFi/lwIP/mbedTLS offloaded to PSRAM for sustained heap headroom. |
| `adafruit_qtpy_esp32_pico` | Adafruit QT Py ESP32-PICO | 8 MB | Compact form factor. Optional NeoPixel tick on pulse. PSRAM. |
| `seeed_xiao_esp32s3` | Seeed Studio XIAO ESP32-S3 | 8 MB | Tiny 21×17.5 mm — shares QT Py form factor + Geiger pin map (A0 / A1 / SCK), so one PCB design works for both. Onboard user LED blinks per pulse. PSRAM. |
| `heltec_wifi_lora32_v4_r2` | Heltec WiFi LoRa 32 V4 — base/R2 variant (ESP32-S3R2) | 16 MB | Third-party PCB: the standard Multigeiger V2 mainboard populated with this module instead of the Heltec V2. Onboard SSD1315 OLED on a dedicated, always-on I²C bus separate from the env-sensor bus. 2 MB in-package PSRAM. GPIO 7-14 reserved (undriven) for future LoRaWAN/Meshtastic work — not implemented yet. |

Build/flash invocation takes a board argument — see `_build.cmd` / `_flash.cmd` / `_merge.cmd` helpers. All boards share the same `main/` source tree; differences are isolated in per-board `sdkconfig.defaults.<board>` and HAL pin map. PSRAM boards additionally include `sdkconfig.defaults.psram` (WiFi roaming app + PSRAM offload knobs).
```

- [ ] **Step 3: Update the "Substitute ... for other boards" sentence**

Find:
```markdown
Substitute `heltec_v2_4mb`, `feathers3_d`, `adafruit_qtpy_esp32_pico`, or `seeed_xiao_esp32s3` for other boards. Per-board build/cache directories prevent cross-board sdkconfig pollution.
```
Replace with:
```markdown
Substitute `heltec_v2_4mb`, `feathers3_d`, `adafruit_qtpy_esp32_pico`, `seeed_xiao_esp32s3`, or `heltec_wifi_lora32_v4_r2` for other boards. Per-board build/cache directories prevent cross-board sdkconfig pollution.
```

- [ ] **Step 4: Fix the stale "five boards" mention in the release-workflow section**

Find:
```markdown
`git tag V2.X.Y && git push --tags` is the entire release ceremony — GitHub Actions `release.yml` builds all five boards in parallel and creates the GitHub Release with bundled artefacts + CHANGELOG body. Manual fallback documented in `_merge.cmd`.
```
Replace with:
```markdown
`git tag V2.X.Y && git push --tags` is the entire release ceremony — GitHub Actions `release.yml` builds all six boards in parallel and creates the GitHub Release with bundled artefacts + CHANGELOG body. Manual fallback documented in `_merge.cmd`.
```

- [ ] **Step 5: Verify**

```powershell
Select-String -Path "README.md" -Pattern "heltec_wifi_lora32_v4_r2"
Select-String -Path "README.md" -Pattern "\*\*six\*\*|six boards"
```
Expected: first command returns 3 matches (table row + substitute-boards sentence + any other literal mentions added above); second returns 2 matches (intro paragraph + release-workflow sentence).

- [ ] **Step 6: Commit**

```powershell
git add README.md
git commit -m "docs: add heltec_wifi_lora32_v4_r2 to the README board table"
```

---

## Self-Review

**Spec coverage:**
- §1 (naming) → Task 2 (`BOARD_HELTEC_WIFI_LORA32_V4_R2`, `heltec_wifi_lora32_v4_r2`), Task 3 (`BOARD_NAME`). ✅
- §2 (pin map, as corrected) → Task 3's `hal.h` branch — every pin in the corrected table is present (HV_FET=33, HV_CAP_FULL=2, GMC_COUNT=3, SPEAKER P=26/N=5, SDA=48/SCL=47, USER_SW=0 documented, LED=35), plus the OLED pins and the full reserved-pin list. ✅
- §3 (two-I²C-bus architecture) → Task 4 (`i2c_bus.c` new secondary-bus branch) + Task 5 (`display.c` label/comment updates). ✅
- §4 (LED polarity) → Task 3's `PIN_LED_BUILTIN` comment; flagged as bench-verify in Global Constraints and the CHANGELOG entry. ✅
- §5 (GPIO2/GPIO46 non-conflict) → Task 3's `PIN_HV_CAP_FULL_INPUT` comment. ✅
- §6 (future LoRaWAN readiness) → Task 3's "RESERVED for future LoRaWAN/Meshtastic work" comment block; explicitly no `HAL_HAS_LORA` flag anywhere in the plan. ✅
- §7 (`hal.h` feature flags) → Task 3, values match exactly. ✅
- §8 (`CMakeLists.txt` + sdkconfig overlay) → Task 1 + Task 2, values match exactly. ✅
- §9 (out of scope) → no VBAT/GNSS/LoRaWAN code anywhere in the plan; explicitly called out in Global Constraints. ✅

**Placeholder scan:** No "TBD"/"TODO"/"handle appropriately" anywhere above — every step shows the literal before/after code, exact file paths, and exact verification commands with expected output.

**Type consistency:** `i2c_bus_get_secondary()` keeps its existing `i2c_master_bus_handle_t` return type and zero-argument signature across Task 4 (producer) and Task 5 (consumer — unchanged call site). `PIN_OLED_SDA`/`PIN_OLED_SCL` are defined once in Task 3 and consumed only in Task 4 — no naming drift. `BOARD_NAME` string literal (`"heltec_wifi_lora32_v4_r2"`) matches the CMake `BOARD` value and the CI matrix entry exactly in every task that mentions it.

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-06-heltec-wifi-lora32-v4-r2-board-port.md`. Two execution options:

**1. Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.

**2. Inline Execution** — Execute tasks in this session using executing-plans, batch execution with checkpoints.

Which approach?
