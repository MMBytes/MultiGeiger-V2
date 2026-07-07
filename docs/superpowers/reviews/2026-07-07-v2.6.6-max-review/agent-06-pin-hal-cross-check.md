# Agent 6: pin/HAL cross-check

**Status: done**

**Scope note:** This review covers the same `main/hal.h` fuel-gauge additions
as agent-05's 2026-07-03 pass (which examined commit `959a39c`, the last
hal.h-touching commit at that time), re-run against the current tip
`86a0273` (`git diff f26f85e..86a0273 -- main/hal.h`). The diff for
`main/hal.h` itself is **unchanged** since agent-05's review — the
intervening commits (`efcdfc5`, `e125c26`, `30d3b1d`, `86a0273`) touched
`fuel_gauge.c`/`.h` and `http_server.c` (diagnostic logging, the
VCELL→config-checkbox rework) but did not add/move/remove any pin or HAL
macro. This pass independently re-derives every finding from repo content
(not copied from agent-05) and additionally verifies the STEMMA1/`bus1`
call-site chain that agent-05's brief didn't explicitly walk.

## 1. Board enumeration (`main/hal.h`)

Board selection is `#if defined(BOARD_...) / #elif ... / #else #error`
(hal.h:46, 114, 234, 338, 422). Four `#elif` blocks define the five build
targets (Heltec V2 shares one block for both flash variants, distinguished
only by `BOARD_HELTEC_V2_4MB` inside it — hal.h:52-56):

| Block | BOARD_NAME(s) | Lines |
|---|---|---|
| `BOARD_HELTEC_V2` | `heltec_v2`, `heltec_v2_4mb` | 46-113 |
| `BOARD_FEATHERS3_D` | `feathers3_d` | 114-233 |
| `BOARD_ADAFRUIT_QTPY_ESP32_PICO` | `adafruit_qtpy_esp32_pico` | 234-337 |
| `BOARD_SEEED_XIAO_ESP32S3` | `seeed_xiao_esp32s3` | 338-421 |

This matches the "5 build targets" from project memory. Every branch
explicitly defines `HAL_HAS_FUEL_GAUGE` (hal.h:66, 125, 275, 369) — no
implicit/missing-default risk, consistent with the file's own documented
convention ("no implicit defaults" — hal.h:16-17).

## 2. New symbols added in V2.6.6 (`git diff f26f85e..86a0273 -- main/hal.h`)

- `HAL_HAS_FUEL_GAUGE` — added to all 4 board blocks: `1` on
  `BOARD_FEATHERS3_D` (hal.h:125), `0` on Heltec V2 (hal.h:66), QT Py
  (hal.h:275), and XIAO (hal.h:369). **Correct fallback value (0) on all
  3 non-target boards — Critical-class miswiring (e.g. accidentally `1`
  elsewhere, or missing on one board) is absent.**
- `PIN_VBUS_DETECT` (hal.h:192, `= 34`) — added **only** inside the
  `BOARD_FEATHERS3_D` block. No other board defines it, and no sentinel
  value is needed on other boards because every reference to it is itself
  compiled out there (see §5). This is the correct pattern used elsewhere
  in this file (e.g. `PIN_OLED_RESET` is simply absent rather than given a
  dummy value — hal.h:204-206).
- No new MAX17048 I2C-address/bus `#define` was added to hal.h — `0x36` is
  defined locally in `main/fuel_gauge.c:15` as `MAX17048_ADDR`, not in
  hal.h. This is consistent with how other on-bus device addresses are
  handled in this codebase (I2C bus pins live in hal.h; device addresses
  live in the driver .c file) — not a defect, just noting it as the only
  "MAX17048-related" symbol not living in hal.h.

## 3. GPIO collision check — full `BOARD_FEATHERS3_D` pin inventory (hal.h:114-233)

| Macro | GPIO | Line |
|---|---|---|
| `PIN_ANTENNA_SELECT` | 41 | 148 |
| `PIN_HV_CAP_FULL_INPUT` | 17 | 174 |
| `PIN_GMC_COUNT_INPUT` | 18 | 175 |
| `PIN_HV_FET_OUTPUT` | 5 | 176 |
| `PIN_SPEAKER_P` | 3 | 179 |
| `PIN_SPEAKER_N` | 1 | 180 |
| `PIN_LED_BUILTIN` | 13 | 184 |
| **`PIN_VBUS_DETECT`** | **34** | **192** |
| `PIN_I2C_SDA` | 8 | 196 |
| `PIN_I2C_SCL` | 9 | 197 |
| `PIN_ALS` | 4 | 202 |

No two macros share a GPIO number. The block's own "RESERVED pins" comment
(hal.h:208-214) independently lists `IO34 VBUS-present detect` alongside
IO0 (boot strap), IO19/20 (USB D-/D+), IO40 (NeoPixel), IO45/46 (straps),
IO2 (fuel-gauge interrupt, not wired to a macro), and IO8/9 (STEMMA1 +
MAX17048 bus) — GPIO34 does not appear against any *other* reserved
function, and the "PINS WE NOW DRIVE despite reservation" list
(hal.h:216-232, covering IO3, IO39, IO15/16) does not include 34 either.
**No collision found.**

**Needs external verification (not verifiable from repo content alone):**
GPIO34 falls in the ESP32-S3's 33-37 range that is dedicated to octal
(OPI) PSRAM/flash D4-D7 wiring on modules configured for octal mode.
hal.h:126 documents this board as "8 MB **QSPI** PSRAM" (quad-mode, only
needs GPIO26-32), which is why GPIO34 would be free — contrast
`BOARD_SEEED_XIAO_ESP32S3` at hal.h:361 which explicitly says "OPI PSRAM
(octal mode)" and separately reserves GPIO26-37 as unavailable (hal.h:417).
This determination hinges entirely on the FeatherS3-D module's actual
silicon being wired for quad (not octal) PSRAM — internally consistent
with this file's own comments, and pre-existing (the reserved-pin
comment already listed IO34 for this exact purpose before this diff), but
I cannot confirm against the physical Unexpected Maker datasheet from
repo content alone. Flag for a hardware-level sanity check, not a code
defect.

## 4. STEMMA1/primary-bus identity check — `fuel_gauge_init(bus1)` call site

Traced the full chain from `main.c` to bus creation:

- `main/fuel_gauge.h:8` documents: "MAX17048 wired to the same pins as the
  STEMMA1 connector (GPIO8 SDA / GPIO9 SCL — the primary I²C bus)".
- `main/hal.h:196-197` (`BOARD_FEATHERS3_D` block): `PIN_I2C_SDA = 8`,
  `PIN_I2C_SCL = 9` — matches exactly.
- `main/main.c:996-1002`: `i2c_bus_set_primary_pinout(g_cfg.i2c_pinout)`
  then `i2c_master_bus_handle_t bus1 = i2c_bus_get_primary();` then
  `fuel_gauge_init(bus1);` — comment at main.c:999-1001 explicitly states
  "MAX17048 fuel gauge is a fixed onboard part on STEMMA1/primary... it
  never needs the secondary-bus fallback probe."
- `main/i2c_bus.c:27,42-51`: `i2c_bus_get_primary()` only honours the
  `i2c_pinout` alternate-route selector when `HAL_HAS_I2C_PINOUT_SWITCH`
  is set. `BOARD_FEATHERS3_D` sets this flag to `0` (hal.h:130 — "Single
  fixed I²C route"), so the `#else` branch at i2c_bus.c:48-50 unconditionally
  applies: `sda = PIN_I2C_SDA` (8), `scl = PIN_I2C_SCL` (9). There is no
  runtime path on this board by which `bus1` could resolve to anything
  other than GPIO8/9.

**Confirmed: `bus1` passed to `fuel_gauge_init()` is definitively the
STEMMA1/GPIO8-9 primary bus, matching fuel_gauge.h's documented wiring.**
No secondary-bus (STEMMA2, IO15/16, LDO2-gated) ambiguity is possible for
this call, unlike the pluggable env/PM/noise sensors elsewhere in
`main.c` which do try both buses (main.c:979-991, 1008-1009).

## 5. Cross-board leakage check

`grep -rn "PIN_VBUS_DETECT"` across `main/` returns exactly:
- `main/hal.h:186,192,210` — all inside the `BOARD_FEATHERS3_D` block only.
- `main/fuel_gauge.c:83,86,94` — all inside `#if HAL_HAS_FUEL_GAUGE`
  (fuel_gauge.c:7) ... `#else` (fuel_gauge.c:159) ... `#endif`
  (fuel_gauge.c:174). The stub branch (`fuel_gauge.c:161`,
  `HAL_HAS_FUEL_GAUGE == 0`) defines `fuel_gauge_init()` as
  `{ (void)bus; return ESP_OK; }` with **no reference to
  `PIN_VBUS_DETECT`** — confirmed this cannot fail to compile on the other
  3 boards where the macro is undefined.
- `main/fuel_gauge.h:53` — a comment only, not code.

No other file references `PIN_VBUS_DETECT`. `main/http_server.c` and
`main/main.c` only ever call the wrapper functions
(`fuel_gauge_vbus_present()`, `fuel_gauge_present()`,
`fuel_gauge_set_user_present()`), never the raw pin macro — these wrappers
are unconditionally declared in `fuel_gauge.h` and always compile (return
`false`/no-op bodies in the `#else` branch), so no board-specific `#ifdef`
is needed at any call site, exactly as documented in fuel_gauge.h:34-36.

`HAL_HAS_FUEL_GAUGE` itself is referenced additionally in
`main/http_server.c:1463` (comment), `:1740` (`#if HAL_HAS_FUEL_GAUGE`),
and `:1981` (`#if !HAL_HAS_FUEL_GAUGE` — force-disables `batt_present` in
the config-save path on non-fuel-gauge boards, http_server.c:1979-1983).
Both directions of the guard are present and correctly paired — no
unguarded reference found on any board.

## 6. `gpio_set_direction(PIN_VBUS_DETECT, ...)` gating (`main/fuel_gauge.c:83`)

Confirmed: `fuel_gauge.c:7` opens `#if HAL_HAS_FUEL_GAUGE`, and the entire
"real" implementation including `fuel_gauge_init()` (containing the
`gpio_set_direction` call at line 83) runs through to the `#else` at line
159. The stub implementation for `HAL_HAS_FUEL_GAUGE == 0` boards
(lines 159-174) never mentions `PIN_VBUS_DETECT`, so on Heltec V2 / QT Py
/ XIAO — where the macro is undefined — the preprocessor never emits code
referencing it. **No possibility of an "undefined symbol" compile error
on the other 3 boards.**

## Findings summary

| # | Severity | Finding |
|---|---|---|
| 1 | Info (needs external verification) | GPIO34 lies in the ESP32-S3's 33-37 octal-PSRAM range; hal.h's own "QSPI not OPI" claim (hal.h:126) makes it free, and this is internally consistent with the file's pre-existing reserved-pin comment, but cannot be verified against the physical FeatherS3-D datasheet from repo content alone. Not a code defect. |
| 2 | Pass | `HAL_HAS_FUEL_GAUGE` correctly `1` on `BOARD_FEATHERS3_D` only, `0` on the other 3 boards, with no implicit defaults. |
| 3 | Pass | `PIN_VBUS_DETECT` = GPIO34 does not collide with any of the other 10 `PIN_*` macros in the `BOARD_FEATHERS3_D` block, nor with any documented reservation (straps, USB D+/-, NeoPixel, ALS, antenna select, STEMMA1/2 buses, fuel-gauge INT). |
| 4 | Pass | GPIO34 is not a legacy-ESP32 input-only pin (that restriction doesn't apply to ESP32-S3) and not a strapping pin — `GPIO_MODE_INPUT` usage in fuel_gauge.c:83 is valid. |
| 5 | Pass | MAX17048 documented bus (GPIO8 SDA/GPIO9 SCL, fuel_gauge.h:8) matches hal.h's `PIN_I2C_SDA`/`PIN_I2C_SCL` for `BOARD_FEATHERS3_D` (hal.h:196-197) exactly. |
| 6 | Pass | `main.c:1002`'s `fuel_gauge_init(bus1)` call site is confirmed, by tracing `i2c_bus_get_primary()` and the board's `HAL_HAS_I2C_PINOUT_SWITCH=0` setting, to always resolve to the STEMMA1/GPIO8-9 bus — no secondary-bus ambiguity possible for this fixed onboard part. |
| 7 | Pass | Zero cross-board leakage: `PIN_VBUS_DETECT` referenced only inside `BOARD_FEATHERS3_D`'s hal.h block and inside `#if HAL_HAS_FUEL_GAUGE` in fuel_gauge.c. The `#else` stub never references the pin macro. |
| 8 | Pass | `gpio_set_direction(PIN_VBUS_DETECT, ...)` at fuel_gauge.c:83 is correctly gated inside `#if HAL_HAS_FUEL_GAUGE` (opens fuel_gauge.c:7, `#else` at 159) — cannot compile-reference an undefined symbol on the other 3 boards. |
| 9 | Pass | `http_server.c`'s `batt_present` config field is force-disabled via `#if !HAL_HAS_FUEL_GAUGE` (http_server.c:1981-1983) on non-fuel-gauge boards — defence-in-depth against a hand-crafted POST, mirrored from the existing `i2c_pinout` pattern (http_server.c:1975-1977). |

No Critical or Important defects found. One Info-level item (GPIO34 vs.
octal-PSRAM range) is flagged for hardware-datasheet spot-check rather
than scored pass/fail, since it depends on a physical-silicon fact this
repo cannot self-verify.

## Overall verdict

**PASS.** The V2.6.6 fuel-gauge additions to `main/hal.h`
(`HAL_HAS_FUEL_GAUGE`, `PIN_VBUS_DETECT`) are correctly scoped to
`BOARD_FEATHERS3_D` only, with safe `0` fallbacks on the other 3 board
blocks and zero leakage of the pin macro outside its `#if HAL_HAS_FUEL_GAUGE`
guard in `fuel_gauge.c`. GPIO34 does not collide with any other pin
assignment on FeatherS3-D. The MAX17048's documented STEMMA1 bus
(GPIO8/9) matches hal.h's `PIN_I2C_SDA`/`PIN_I2C_SCL` for this board, and
tracing `main.c`'s `fuel_gauge_init(bus1)` call through
`i2c_bus_get_primary()` confirms `bus1` is unambiguously the primary
STEMMA1 bus (the board's `HAL_HAS_I2C_PINOUT_SWITCH=0` means no runtime
pin-swap path exists to redirect it). All four non-target boards compile
cleanly with the fuel-gauge code entirely preprocessed out — no
undefined-symbol risk. The only open item — whether GPIO34 truly sits
outside octal-PSRAM silicon wiring on the real FeatherS3-D module — is a
hardware-datasheet fact this review cannot close from source alone and is
flagged as such rather than asserted. `main/hal.h` itself has not changed
since agent-05's 2026-07-03 pass; this independent re-derivation reaches
the same pass verdict and additionally closes the STEMMA1/`bus1`
call-site verification that was implicit but unstated in that earlier
report.
