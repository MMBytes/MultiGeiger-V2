# Agent 5: pin/HAL cross-check

**Status: done**

## Pin/HAL cross-check: BOARD_FEATHERS3_D — PIN_VBUS_DETECT (GPIO34)

**Full GPIO inventory checked, `BOARD_FEATHERS3_D` block (hal.h:114–233):**

| Macro | GPIO | Line |
|---|---|---|
| PIN_ANTENNA_SELECT | 41 | 148 |
| PIN_HV_CAP_FULL_INPUT | 17 | 174 |
| PIN_GMC_COUNT_INPUT | 18 | 175 |
| PIN_HV_FET_OUTPUT | 5 | 176 |
| PIN_SPEAKER_P | 3 | 179 |
| PIN_SPEAKER_N | 1 | 180 |
| PIN_LED_BUILTIN | 13 | 184 |
| **PIN_VBUS_DETECT** | **34** | **192** |
| PIN_I2C_SDA | 8 | 196 |
| PIN_I2C_SCL | 9 | 197 |
| PIN_ALS | 4 | 202 |

No `PIN_OLED_RESET` (intentionally undefined). Comment-only reservations (no macro, not driven): IO0 boot strap, IO2 fuel-gauge INT, IO19/20 native USB D-/D+, IO39 LDO2 enable, IO40 NeoPixel, IO45/46 straps, IO15/16 STEMMA2 bus. **GPIO34 does not collide with any `PIN_*` macro or documented reservation** — good.

> RESOLVED 2026-07-04: independently verified against Unexpected Maker's
> own FeatherS3D datasheet (Mouser-hosted PDF) and product page — confirms
> "8MB QSPI PSRAM" (quad, not octal) and GPIO34 as the vendor's own
> documented 5V/USB-detect pin. No code change; closed by verification.

**Finding 1 (Info, not a defect):** GPIO34 falls in the ESP32-S3's 33–37 range, which is dedicated to octal (OPI) PSRAM/flash D4–D7 lines on modules configured for octal mode. hal.h:126 documents this board as **"8 MB QSPI PSRAM"** (quad, 4-line) rather than octal — contrast with `BOARD_SEEED_XIAO_ESP32S3` at hal.h:361, which explicitly says "OPI PSRAM (octal mode)". Given quad PSRAM only needs GPIO26–32, GPIO33–37 are free general-purpose I/O on this specific module, making GPIO34 legitimate. This determination hinges entirely on the "QSPI not OPI" claim in the comment being accurate to the actual FeatherS3-D module's silicon/wiring — could not independently verify against a datasheet, but it's internally consistent with this file's own documentation and the reserved-pin list at line 210 already listed IO34 for this exact purpose before the code change (**Important — flag for hardware-datasheet spot-check, not a code bug**).

**Finding 2:** GPIO34 is not a strapping pin (0/3/45/46, confirmed absent) and not USB D+/D- (19/20 per hal.h:127 native-USB comment). `GPIO_MODE_INPUT` is valid — nothing here suggests it's a legacy-ESP32-style input-only pin (that restriction was original ESP32 GPIO34-39; ESP32-S3 GPIO34 has full input/output capability). No conflict.

**Finding 3 (cross-board leakage): none found.** `grep -rn "PIN_VBUS_DETECT" main/` shows it's used only in `main/hal.h:186,192,210` (inside `BOARD_FEATHERS3_D`) and `main/fuel_gauge.c:72,83` (inside `#if HAL_HAS_FUEL_GAUGE`, fuel_gauge.c:7). The stub branch at `fuel_gauge.c:135` (`#else // HAL_HAS_FUEL_GAUGE==0`) returns `false` without referencing the pin. `transmission.c:1440` only calls the wrapper `fuel_gauge_vbus_present()`, never the raw macro — safe on all boards since that function is always declared. No compile break for other targets.

**Finding 4:** No in-repo `docs/` wiring file documents GPIO34 for VBUS-detect (grep of `docs/` found nothing relevant). hal.h's own reserved-pin comment (line 210, pre-existing before this diff per the "IO34 VBUS-present detect" text) is self-consistent with the new `#define`. Could not cross-check against `reference_feathers3_d_wiring.md` — that file lives only in the Claude memory system, not this repo.

**Finding 5:** `build_feathers3_d/geiger_v2.bin` and `.elf` are timestamped 2026-07-03 23:23, matching `git log -1 --format=%cI -- main/hal.h` (2026-07-03T23:23:31+10:00) almost to the second — strong corroborating evidence a feathers3_d build succeeded with this pin definition present.

## Overall verdict

GPIO34 is safe to use for `PIN_VBUS_DETECT` on `BOARD_FEATHERS3_D`. It doesn't collide with any other pin macro in the block, isn't a strapping pin, isn't USB D+/D-, and (per this file's own documentation of quad-not-octal PSRAM) isn't consumed by internal PSRAM wiring. No cross-board compile risk exists since all references are properly guarded. The only open item is independently confirming the "QSPI PSRAM" (not octal) claim against the actual FeatherS3-D module datasheet — worth a quick hardware-level sanity check before flashing, not a blocking code issue.
