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
- Version: current `main/version.h` is `V2.6.6`. Per this project's own convention (every prior new-board addition — e.g. V2.4.25's `seeed_xiao_esp32s3` — got its own version bump + CHANGELOG entry), bump to `V2.6.7` once the board builds cleanly (Task 8), not before.

## File Structure

| File | Change |
|---|---|
| `sdkconfig.defaults.heltec_wifi_lora32_v4_r2` | **New.** S3R2-specific overlay: 16 MB flash, 2 MB in-package quad PSRAM, UART console. |
| `CMakeLists.txt` | Modify. New `elseif(BOARD STREQUAL "heltec_wifi_lora32_v4_r2")` branch + update the 3 board-list mentions (usage comment, cache-string help text, `else()` error message). |
| `main/hal.h` | Modify. New `#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)` branch (pin map + feature flags) + update the final `#error` message. |
| `main/i2c_bus.c` | Modify. Widen `s_bus_secondary`'s board guard; add a new branch inside `i2c_bus_get_secondary()` that brings up `I2C_NUM_1` on GPIO17/18 (OLED bus), always-on, no gating GPIO. |
| `main/i2c_bus.h` | Modify. Add a "Heltec WiFi LoRa 32 V4-R2" paragraph to the per-board doc comment. |
| `main/display.c` | Modify. Extend `OLED_CHIP_NAME` chain with `"SSD1315"`; update the top-of-file per-board comment; make the `"STEMMA1"`/`"STEMMA2"` log-label strings board-aware (this board has no STEMMA connector). |
| `.github/workflows/_build-boards.yml` | Modify. Add `heltec_wifi_lora32_v4_r2` to `matrix.board`; extend the `target:` ternary so it maps to `esp32s3` (not the `esp32` default). |
| `.github/workflows/release.yml` | Modify. Bump `EXPECTED_BOARDS` from `5` to `6`. |
| `main/version.h` | Modify. `V2.6.6` → `V2.6.7`. |
| `CHANGELOG.md` | Modify. New `## V2.6.7` entry. |
| `README.md` | Modify. New board-table row; extend the "Substitute ... for other boards" sentence. |

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
Expected: `V2.6.6` (version not bumped yet — that's Task 8). Confirms the binary was freshly produced, not stale.

- [ ] **Step 5: Commit**

```powershell
git add main/hal.h
git commit -m "Add BOARD_HELTEC_WIFI_LORA32_V4_R2 pin map to hal.h"
```

---

### Task 4: `main/i2c_bus.c` / `main/i2c_bus.h` — dedicated OLED bus

**Files:**
- Modify: `main/i2c_bus.c:13-19` (widen the `s_bus_secondary` guard), `main/i2c_bus.c:90-122` (`i2c_bus_get_secondary()` — add a new branch)
- Modify: `main/i2c_bus.h:11-33` (per-board doc comment — add a paragraph)

**Interfaces:**
- Consumes: `PIN_OLED_SDA=17`, `PIN_OLED_SCL=18` from Task 3's `hal.h`. Also `BOARD_HELTEC_WIFI_LORA32_V4_R2` for the `#if`/`#elif` guards.
- Produces: `i2c_bus_get_secondary()` now returns a valid, always-on `i2c_master_bus_handle_t` on `I2C_NUM_1` for this board (previously it unconditionally returned `NULL` for every board except FeatherS3-D). No signature change — `main/display.c` (Task 5) and any future consumer keep calling it exactly as before.

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

Note: `i2c_bus_finalize()` needs no change — its `#if defined(BOARD_FEATHERS3_D)` teardown/LDO-drop logic is FeatherS3-D-specific (nothing to un-gate on this board), and `display.c` already calls `i2c_bus_secondary_keep_alive()` on a successful OLED bind (Task 5), so this board's bus is never a teardown candidate regardless.

- [ ] **Step 3: Update the `i2c_bus.h` doc comment**

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

- [ ] **Step 4: Verify the build still succeeds**

```powershell
& .\_build.cmd heltec_wifi_lora32_v4_r2 2>&1 | Select-Object -Last 25
```
Expected: `Project build complete.` (This is a regression check — no functional test is possible without hardware. The i2c_bus.c change is inert until Task 5's `display.c` actually calls `i2c_bus_get_secondary()` and finds a display.)

- [ ] **Step 5: Structural check — confirm the new branch compiled in, not silently skipped**

```powershell
Select-String -Path "build_heltec_wifi_lora32_v4_r2\CMakeFiles\geiger_v2.elf.dir\i2c_bus.c.obj" -Pattern "onboard OLED" -SimpleMatch -ErrorAction SilentlyContinue
```
This is a weak check (object files aren't reliably greppable text); the authoritative check is Step 4's successful `Project build complete.` combined with re-reading the diff in Step 2. Skip this step if the object-file grep finds nothing — that's expected (compiled/optimized object files don't usually retain source comments as literal text); do not treat a miss here as a build failure.

- [ ] **Step 6: Commit**

```powershell
git add main/i2c_bus.c main/i2c_bus.h
git commit -m "Add dedicated always-on OLED bus for Heltec WiFi LoRa 32 V4-R2"
```

---

### Task 5: `main/display.c` — OLED chip name + board-aware bus labels

**Files:**
- Modify: `main/display.c:1-19` (top-of-file per-board comment), `main/display.c:70-74` (`OLED_CHIP_NAME` chain), `main/display.c:503,528,532,559` (bus-label strings)

**Interfaces:**
- Consumes: `BOARD_HELTEC_WIFI_LORA32_V4_R2`, and (indirectly, via `i2c_bus_get_secondary()`) Task 4's new bus branch.
- Produces: nothing new consumed elsewhere — this task only changes boot-log/status-log text accuracy. No behavioral change to the probe cascade itself (it already works correctly for this board once Task 4 lands: primary bus has no display, so it falls through to secondary, which always has the SSD1315).

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
//                               CE09964) on STEMMA1 or STEMMA2 (SDA=IO16, SCL=IO15),
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
//                               CE09964) on STEMMA1 or STEMMA2 (SDA=IO16, SCL=IO15),
//                               powered from LDO2 — see bring_up_stemma2_bus.
//   Heltec WiFi LoRa 32 V4-R2 : onboard SSD1315 on a bus dedicated
//                               permanently to the display (SDA=GPIO17,
//                               SCL=GPIO18, RST=GPIO21) — module-internal,
//                               not shared with the env-sensor bus at all
//                               (unlike every board above). Always present
//                               (on-module, not a plug-in breakout), so it's
//                               reached via the same primary→secondary
//                               probe fallback but always resolves on the
//                               secondary bus.
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

- [ ] **Step 4: Verify the build succeeds**

```powershell
& .\_build.cmd heltec_wifi_lora32_v4_r2 2>&1 | Select-Object -Last 25
```
Expected: `Project build complete.` This is the last source-code task — the firmware now builds cleanly for `heltec_wifi_lora32_v4_r2` with all HAL/display/I²C wiring in place.

- [ ] **Step 5: Regression-check the other five boards still build**

The `#if`/`#elif` chains touched in Tasks 3-5 are additive (new branches only), but verify no existing board's branch was accidentally altered:
```powershell
& .\_build.cmd heltec_v2 2>&1 | Select-Object -Last 10
& .\_build.cmd feathers3_d 2>&1 | Select-Object -Last 10
```
Expected: both end with `Project build complete.` (Running all 5 pre-existing boards is thorough but slow; these two cover both chip targets — `esp32` and `esp32s3` — and are the two boards whose `hal.h`/`i2c_bus.c`/`display.c` branches sit textually adjacent to the new one, so they're the most likely to show an accidental edit.)

- [ ] **Step 6: Commit**

```powershell
git add main/display.c
git commit -m "Add SSD1315 support and board-aware bus labels for Heltec WiFi LoRa 32 V4-R2"
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

### Task 7: CI workflow — board matrix + release artefact count

**Files:**
- Modify: `.github/workflows/_build-boards.yml:52-57` (matrix), `.github/workflows/_build-boards.yml:83-85` (target ternary)
- Modify: `.github/workflows/release.yml:107-114` (`EXPECTED_BOARDS`)

**Interfaces:**
- Consumes: nothing (CI-only config, not exercised locally).
- Produces: the 6-board CI matrix that both `build.yml` and `release.yml` call into via `_build-boards.yml`'s `workflow_call`.

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

- [ ] **Step 2: Extend the `target:` ternary**

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

- [ ] **Step 3: Bump `EXPECTED_BOARDS` in `release.yml`**

In `.github/workflows/release.yml`, find:
```bash
          EXPECTED_BOARDS=5
```
Replace with:
```bash
          EXPECTED_BOARDS=6
```

- [ ] **Step 4: Verify via grep (no local CI execution possible)**

```powershell
Select-String -Path ".github\workflows\_build-boards.yml" -Pattern "heltec_wifi_lora32_v4_r2"
Select-String -Path ".github\workflows\release.yml" -Pattern "EXPECTED_BOARDS=6"
```
Expected: both commands return a match. This is a structural check only — the actual CI matrix and artefact-count guard are exercised for real on the next push / tag, not in this session.

- [ ] **Step 5: Commit**

```powershell
git add .github/workflows/_build-boards.yml .github/workflows/release.yml
git commit -m "CI: add heltec_wifi_lora32_v4_r2 to the build matrix (6th board)"
```

---

### Task 8: `README.md` — board table + build-command reference

**Files:**
- Modify: `README.md:28-38` (board table + `SDKCONFIG_DEFAULTS` sentence)
- Modify: `README.md:200` (`idf.py` "Substitute ... for other boards" sentence)

**Interfaces:**
- Consumes: nothing.
- Produces: nothing consumed by code — documentation only.

- [ ] **Step 1: Update the board table header and add a row**

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

- [ ] **Step 2: Update the "Substitute ... for other boards" sentence**

Find:
```markdown
Substitute `heltec_v2_4mb`, `feathers3_d`, `adafruit_qtpy_esp32_pico`, or `seeed_xiao_esp32s3` for other boards. Per-board build/cache directories prevent cross-board sdkconfig pollution.
```
Replace with:
```markdown
Substitute `heltec_v2_4mb`, `feathers3_d`, `adafruit_qtpy_esp32_pico`, `seeed_xiao_esp32s3`, or `heltec_wifi_lora32_v4_r2` for other boards. Per-board build/cache directories prevent cross-board sdkconfig pollution.
```

- [ ] **Step 3: Verify**

```powershell
Select-String -Path "README.md" -Pattern "heltec_wifi_lora32_v4_r2"
```
Expected: 3 matches (table row, board-count-context sentence is prose so won't match the literal string — just the table row + the substitute-boards sentence + any other literal mentions added above).

- [ ] **Step 4: Commit**

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
