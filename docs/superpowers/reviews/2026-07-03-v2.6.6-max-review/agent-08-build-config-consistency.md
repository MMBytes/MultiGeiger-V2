# Agent 8: build configuration consistency

**Status: done**

## Build-Configuration Consistency Review

**1. `main/CMakeLists.txt`** — one line added: `"fuel_gauge.c"` (line 22), unconditional, placed right after `"als.c"`/`"veml7700.c"` — same pattern as the other optional-capability drivers. Consistent, no gating needed here because `fuel_gauge.c` self-gates internally via `#if HAL_HAS_FUEL_GAUGE` (lines 7/131 of the file — real driver body vs. no-op stubs returning `false`/`ESP_FAIL`). `fuel_gauge.h` unconditionally includes `driver/i2c_master.h` (line 42) and `fuel_gauge.c`'s stub branch doesn't need it either, but since it's a standard ESP-IDF built-in driver header present in every IDF 6.0 component tree regardless of board, this include is harmless on all 5 targets. **Verdict: safe, no problem.**

**2. `main/version.h`** — `grep -n "VERSION_STR"` → line 20: `#define VERSION_STR "V2.6.6"`. Confirmed correct; no `V2.6.7` stray bump present.

**3. `main/hal.h` — HAL_HAS_FUEL_GAUGE coverage** — found in exactly 4 `#if/#elif` board blocks (the 5th target, `heltec_v2_4mb`, is a sub-variant inside the `BOARD_HELTEC_V2` block via `BOARD_HELTEC_V2_4MB`, sharing the same flag value):
- `BOARD_HELTEC_V2` (line 66): `0`
- `BOARD_FEATHERS3_D` (line 125): `1`
- `BOARD_ADAFRUIT_QTPY_ESP32_PICO` (line 275): `0`
- `BOARD_SEEED_XIAO_ESP32S3` (line 369): `0`

All 4 blocks define it — no board leaves it undefined. Searched root `CMakeLists.txt`, `main/CMakeLists.txt`, and all `sdkconfig.defaults*` for `-Wundef`/`-Werror`/`COMPILER_WARN`/`WERROR` — **none found**, so even an undefined-macro case wouldn't have been fatal. Moot here since coverage is complete anyway. **Verdict: safe.**

**4. Unbuilt boards — actual rebuild verification** — ran `.\_build.cmd adafruit_qtpy_esp32_pico` (incremental build against the existing `build_adafruit_qtpy_esp32_pico` dir, `HAL_HAS_FUEL_GAUGE=0` for this board). Build completed successfully: `fuel_gauge.c` compiled into `esp-idf/main/CMakeFiles/__idf_main.dir/`, linked into `libmain.a`, final image `geiger_v2.bin` generated (0x149130 bytes, 36% partition free), "Project build complete." No errors or warnings surfaced in the tail output. This confirms the stub path (`#else` branch, lines 131-142 of `fuel_gauge.c`) compiles cleanly with zero I²C/GPIO includes pulled in. Did not rebuild `heltec_v2`, `heltec_v2_4mb`, or `seeed_xiao_esp32s3`, but since all three share the identical `HAL_HAS_FUEL_GAUGE=0` code path already verified on the QTPy target, and the CMakeLists change is board-agnostic, risk is low.

**5. sdkconfig impact** — `git diff --stat f26f85e..HEAD` shows no `sdkconfig*` files touched. `fuel_gauge.c`'s real (non-stub) branch uses only `driver/gpio.h` and `driver/i2c_master.h`, both already-required dependencies for the existing FeatherS3-D I²C/GPIO usage (i2c_bus.c, als.c etc.) — no new Kconfig option needed.

## Overall verdict

The build configuration is sound across the full board matrix, not merely appearing to work from single-board testing. Flag coverage is complete (all 4 hal.h blocks define `HAL_HAS_FUEL_GAUGE`), the CMakeLists change is minimal and unconditional by design (matching existing project convention), and one non-feathers3_d target (QTPy, stub path) was independently rebuilt and verified to succeed. No sdkconfig or Kconfig risk. The three remaining un-rebuilt boards (heltec_v2, heltec_v2_4mb, seeed_xiao_esp32s3) share the exact same `HAL_HAS_FUEL_GAUGE=0` stub path just verified, so residual risk is minimal but not zero — a full CI/all-board build pass would be the only way to close it out completely.
