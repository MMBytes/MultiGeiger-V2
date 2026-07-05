# Heltec WiFi LoRa 32 V4 (R2) Board Port — Design Spec

Status: approved by user. Not yet implemented.
Board scope: a third-party team's PCB — the standard Multigeiger V2 mainboard
(`Multigeiger_V1.9/Hardware/Eagle/projects/Multigeiger_V2/`) populated with a
Heltec WiFi LoRa 32 V4 module instead of the previously-supported Heltec V2.
Not our own PCB design; this board runs our firmware on someone else's build.

## 1. Board identity — which V4, exactly

Heltec sells two silicon variants under "WiFi LoRa 32 V4" that share a product
name but differ in GPIO assignment:

| | base V4 | V4-R8 |
|---|---|---|
| Chip | ESP32-S3**R2** | ESP32-S3**R8** |
| PSRAM | 2 MB (in-package, quad) | 8 MB (in-package, octal) |
| Vext_Ctrl | GPIO36 | GPIO40 |
| VGNSS_Ctrl | GPIO34 | GPIO42 |
| Onboard LED | GPIO35 | GPIO46 |
| LoRa PA_CTX | not broken out | GPIO5 |

This team's board is the **base V4 (ESP32-S3R2)** — confirmed via
`heltec.org/project/wifi-lora-32-v4/`, the user-supplied datasheet
(`Heltec-Doku/WiFi_LoRa_32_V4.2.0.pdf`), and cross-checked against Heltec's own
`HelTecAutomation/Heltec_ESP32` Arduino library source, which defines
`WIFI_LORA_32_V4` and `WIFI_LORA_32_V4_R8` as distinct preprocessor targets.

Naming (per explicit user direction — the "_R2" suffix permanently disambiguates
from a possible future R8 port):

- Macro: `BOARD_HELTEC_WIFI_LORA32_V4_R2`
- Build: `idf.py -DBOARD=heltec_wifi_lora32_v4_r2 build`
- `BOARD_NAME "heltec_wifi_lora32_v4_r2"` (boot log, `/status`)

## 2. Pin map

Sourced from `Multigeiger_V1.9/Pin-Matrix_Heltec_MG_neu-V1.9.ods` (the
mainboard's own J2/J3 wiring intent) and cross-validated pin-for-pin against
Heltec's official V4 datasheet §2.2.1/2.2.2/2.2.3 — zero contradictions on any
pin both sources document. Two matrix cells (J2 pin18 USB D+, J3 pin18
VFEM_Ctrl) were blank in the spreadsheet's V4 column but confirmed present at
the same GPIO via the datasheet (author simply didn't retype a value that
carried over unchanged from the V3 column).

| Function | GPIO | Notes |
|---|---|---|
| HV_FET_OUTPUT | 33 | HV MOSFET gate |
| HV_CAP_FULL_INPUT | 2 | ADC1_CH1/TOUCH2 — plain GPIO on base V4, see §5 |
| GMC_COUNT_INPUT | 3 | Geiger tube pulse |
| SPEAKER (P) | 26 | Piezo — J2 pin15, "SPK" |
| SPEAKER (N) | 5 | Piezo — J3 pin16, "Touch5 – SPK". The matrix marks both GPIO26 and GPIO5 as "SPK" without distinguishing which leg is which; a piezo isn't polarity-sensitive in a way that matters for tone generation, so the P/N assignment here is arbitrary and functionally inconsequential (worst case: a quieter click if reversed). Bench-verify perceived loudness on first flash, same standard as the LED polarity in §4. |
| SDA (env sensor bus) | 48 | J2 pin14, "SDA" |
| SCL (env sensor bus) | 47 | J2 pin13, "SCL" |
| UART TX / RX (console) | 44 / 43 | USB-UART bridge, not native USB |
| USER_SW | 0 | Boot-strap-shared button, matches every other board's convention |
| LED | 35 | Active-HIGH — see §4 |

**Correction (2026-07-06, caught before implementation):** an earlier draft of
this table had SDA/SCL swapped (GPIO47 as SDA, GPIO48 as SCL) and omitted the
SPEAKER (N) row entirely. Both are fixed above after re-verifying directly
against `Multigeiger_V1.9/Pin-Matrix_Heltec_MG_neu-V1.9.pdf`'s "Heltec WiFi
LoRa 32 V4" column (J2 pin13 = `IO47 – SCL`, J2 pin14 = `IO48 – SDA`, J2
pin15 = `IO26 – SPK`, J3 pin16 = `IO05 Touch5 – SPK`). §3 already had the
SDA/SCL assignment correct in its prose — only this table was wrong.

**Vext_Ctrl (GPIO36) is deliberately NOT driven by this port.** Datasheet
§3.3 "Power Output" documents three output pins: the always-on 3.3V pin,
5V@USB, and a *switchable* "Ve" pin gated by VextCtrl — Ve is for powering
external peripherals plugged into the header, separate from the module's
own always-on 3.3V rail that feeds the OLED and the ESP32-S3 itself. The
Multigeiger matrix marks the J2 pins carrying Ve (pins 3/4) as unused by
this mainboard. Unlike Heltec V2 (where Vext gates the shared OLED+sensor
I²C rail and MUST be driven low for the bus to work at all — see
`i2c_bus.c`'s existing `HAL_HAS_VEXT_GATE` block), nothing in this board's
OLED bus or external sensor bus depends on Vext. Leave GPIO36 undriven;
`HAL_HAS_VEXT_GATE=0`.

OLED (fixed module bus, not on J2/J3 — see §3):
| Function | GPIO |
|---|---|
| OLED SDA | 17 |
| OLED SCL | 18 |
| OLED_RST | 21 (J2 pin16) |

Reserved for future LoRa work (never touched by this port — see §6):
GPIO 7, 8, 9, 10, 11, 12, 13, 14.

Reserved / never repurpose (out of scope per §9, or module-internal):
- GPIO36 — Vext_Ctrl, deliberately undriven; see the note above.
- GPIO19/20 — native USB D-/D+ on the module (JTAG/USB-Serial-JTAG strap
  pair); this board doesn't use native USB (`HAL_HAS_NATIVE_USB=0`) but the
  pins are still module-internal wiring, not free GPIO.
- GPIO1 — VBAT_Read, out of scope per §9.
- GPIO38-42 — GNSS connector (RST/PPS/Wakeup/TX/RX), out of scope per §9.
- GPIO45/46 (J3 pins 5/6, "DIP0/DIP1" in the Multigeiger matrix) and GPIO6
  (J3 pin17, "DIP3") — DIP-switch inputs unused by V2 firmware on every
  board; left undriven here too, consistent with existing boards.

## 3. Two-I²C-bus architecture

The Heltec V4's onboard SSD1315 OLED lives on a **fixed module-internal bus**
(GPIO17 SDA / GPIO18 SCL) that is not part of the J2/J3 header at all. The
external env-sensor bus exposed to the Multigeiger mainboard is a completely
separate pair (GPIO47 SCL / GPIO48 SDA). Every existing board in hal.h shares
one bus between OLED and sensor; this board can't, because the OLED bus isn't
reachable from off-module.

Closest precedent: FeatherS3-D's STEMMA1/STEMMA2 split, which already uses a
second I²C controller (I2C_NUM_1) for a secondary bus. This port reuses that
pattern:
- `PIN_I2C_SDA` / `PIN_I2C_SCL` (primary, `i2c_bus_get_primary()`) = GPIO47/48,
  the env-sensor bus — this is the bus every sensor driver (env/PM/noise)
  already targets by default on every board.
- `PIN_OLED_SDA` / `PIN_OLED_SCL` (new pins, second controller) = GPIO17/18,
  dedicated permanently to the display. Unlike STEMMA2 (which is
  lazily-enabled and can host either a display or a sensor), this bus can
  only ever be the OLED — there's no alternate consumer possible since it
  isn't wired to anything else.
- `display.c` brings up I2C_NUM_1 directly on GPIO17/18 rather than
  probing/falling-through the way it does for FeatherS3-D's STEMMA1→STEMMA2.
  No probe needed: this board always has the OLED (it's on-module, not an
  optional plug-in breakout).

## 4. Onboard LED — GPIO35, active-HIGH

Heltec's own `HelTecAutomation/Heltec_ESP32` Arduino library defines no LED
pin for the V4 at all (it only manages LoRa PA control pins). An independent
third-party project, `DN9KGB/rMesh`, ships a dedicated
`hal_HELTEC_WiFi_LoRa_32_V4.h/.cpp` whose other pin values (LORA_NSS=8,
SCK=9, MOSI=10, MISO=11, DIO1=14, RST=12, BUSY=13, VEXT_CTRL=36,
VGNSS_CTRL=34) match the base V4 pinout exactly — good confidence this is the
same silicon variant, not R8. Its code:

```c
#define PIN_WIFI_LED 35
digitalWrite(PIN_WIFI_LED, 0);              // init: LED off
void setWiFiLED(bool value) { digitalWrite(PIN_WIFI_LED, value); }
```

LOW at boot = off, `true`/HIGH = on → **active-high**. This is independent
corroboration, not Heltec's own datasheet text, so treat it as
bench-verify-on-first-flash, same standard applied to the XIAO ESP32-S3's LED
polarity. `HAL_LED_ACTIVE_LOW` is omitted (matches the codebase's convention:
`led.c` defaults `HAL_LED_ACTIVE_LOW` to 0 when a board doesn't define it —
see `led.c:24-25`).

Because `HAL_HAS_SPEAKER=1` on this board, `led.c`'s own driver stays compiled
out (`#if defined(PIN_LED_BUILTIN) && !HAL_HAS_SPEAKER && !HAL_HAS_NEOPIXEL`)
and `speaker.c` owns `PIN_LED_BUILTIN` directly via hardcoded
`gpio_set_level(PIN_LED_BUILTIN, 1)` = on. That hardcoded polarity already
assumes active-high (matches Heltec V2 GPIO25 and FeatherS3-D GPIO13, both
undocumented-as-active-low and both driven the same way) — no changes needed
in speaker.c for this board.

## 5. GPIO2 / GPIO46 — not a LoRa-PA conflict on this SKU

Some other Heltec V4 SKUs populate a KCT8103L or GC1109 front-end PA chip,
and Heltec's own `board-config.h` reassigns GPIO2/GPIO46 to PA control on
those. This team's board's own datasheet (`WiFi_LoRa_32_V4.2.0.pdf` §2.2.2)
lists GPIO2 plainly as `"GPIO2, ADC1_CH1, TOUCH2"` and GPIO46 plainly as
`"GPIO46"` — no PA annotation. Only **GPIO7** carries an explicit
`"VFEM_Control"` note in this datasheet. Confirmed safe to wire GPIO2 to
`HV_CAP_FULL_INPUT` as the Multigeiger matrix intends; no conflict.

## 6. Future LoRaWAN / Meshtastic readiness (hardware reservation only)

Out of scope for this spec: LoRaWAN is a parallel connectivity mode, not
another TX-dispatch-table target — confirmed via Heltec's own quick-start
docs (OTAA/ABP join, DevEUI/AppEUI/AppKey provisioning, no WiFi/internet
dependency at all in that mode). Meshtastic is a further, separate,
even-larger question (own mesh protocol/crypto, or an entirely separate
firmware image). Both need their own dedicated brainstorming session once
scoped — designing that architecture now, without a concrete second-step
plan, would be guessing.

What this spec *does* do to avoid foreclosing that work:
- Document (but do not drive, do not `#define` as `PIN_*` macros consumed by
  any driver) that GPIO 8/9/10/11/12/13/14 are the SX1262 LoRa radio's
  dedicated internal SPI bus — confirmed via the datasheet ("LoRa and Flash
  have each utilized a separate SPI interface") and never appears in the
  J2/J3 header tables, so nothing else could claim these pins anyway. A code
  comment in the `BOARD_HELTEC_WIFI_LORA32_V4_R2` hal.h block records this
  so a future contributor doesn't have to re-derive it.
- GPIO7 (VFEM_Control, §5) is left undriven — not assigned to any
  Multigeiger function — since it's plainly the LoRa front-end enable per
  Heltec's own datasheet.
- No `HAL_HAS_LORA` flag, no radio driver, no config fields, no build changes
  beyond the comment above. Nothing here should need to change when the
  LoRaWAN/Meshtastic spec is eventually written.

## 7. `hal.h` feature flags

```c
#define HAL_HAS_OLED              1   // SSD1315 on dedicated I2C_NUM_1 bus (GPIO17/18)
#define HAL_HAS_ALS               0
#define HAL_HAS_FUEL_GAUGE        0
#define HAL_HAS_PSRAM             1   // 2 MB in-package, quad
#define HAL_HAS_NATIVE_USB        0   // USB-UART bridge on GPIO43/44
#define HAL_HAS_VEXT_GATE         0   // GPIO36 only gates the external "Ve" header pin, unused on this board — see §2
#define HAL_HAS_ANTENNA_SWITCH    0
#define HAL_HAS_I2C_PINOUT_SWITCH 0
#define HAL_HAS_SPEAKER           1   // GPIO26
#define HAL_HAS_NEOPIXEL          0
```

Ring/scratch/form-buffer sizes modeled on `BOARD_ADAFRUIT_QTPY_ESP32_PICO`
(closest analog: 2 MB in-package PSRAM, not FeatherS3-D's 8 MB external) —
`HAL_LOG_RING_BYTES (1*1024*1024)`, `HAL_LOG_SNAP_SCRATCH_BYTES (16*1024)`,
`HAL_CFG_FORM_BUF_SIZE (32*1024)`.

## 8. `CMakeLists.txt`

New branch:
```cmake
elseif(BOARD STREQUAL "heltec_wifi_lora32_v4_r2")
    set(IDF_TARGET "esp32s3" CACHE STRING "")
    add_compile_definitions(BOARD_HELTEC_WIFI_LORA32_V4_R2=1 MQTT_RICH_STATE=1)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.psram;sdkconfig.defaults.heltec_wifi_lora32_v4_r2")
```
Update the `if(NOT DEFINED BOARD)` comment, the default-BOARD cache-string
help text, and the final `else()` error message to list the new board name.

New file `sdkconfig.defaults.heltec_wifi_lora32_v4_r2`, modeled on
`sdkconfig.defaults.feathers3_d`'s structure but for S3R2 silicon:
`CONFIG_IDF_TARGET="esp32s3"`, 16 MB flash, `CONFIG_SPIRAM_MODE_QUAD=y` (R2 =
quad, not octal — unlike FeatherS3-D/XIAO which are octal), 2 MB PSRAM,
`CONFIG_ESP_CONSOLE_UART_DEFAULT=y` (matches QT Py's console path — this
board has no native USB).

## 9. Out of scope (per earlier user decisions)

- VBAT ADC read (GPIO1) — not wired into firmware.
- GNSS module support (GPIO38-42, GNSS SH1.25-8Pin connector) — pins left
  reserved/undriven, no driver code.
- LoRaWAN / Meshtastic application logic — see §6.
