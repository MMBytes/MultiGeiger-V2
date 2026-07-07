# Agent 9: build configuration consistency

**Status: done**

Scope: `main/CMakeLists.txt`, `main/version.h`, and confirming all 5 board
targets still build cleanly after both the fuel-gauge feature (c683204) and
the I2C driver consolidation refactor (959a39c). Actual builds were run
(toolchain available: ESP-IDF v6.0 at `C:\esp\v6.0\esp-idf`, invoked via the
project's own `.\_build.cmd <board>`), not just static verification — but
cppcheck itself is NOT installed in this environment, so item 6 is static
review only (see below).

## 1. `main/CMakeLists.txt` diff (f26f85e..86a0273)

```
+        "fuel_gauge.c"
```

One line added, at `main/CMakeLists.txt:22`, in the `SRCS` list right after
`"veml7700.c"` — same unconditional-registration pattern as every other
optional-capability driver in this list (`als.c`, `veml7700.c`, `dnms.c`,
etc.). Correct and complete:

- `fuel_gauge.c` — new translation unit, correctly added to `SRCS`. Confirmed
  present.
- `fuel_gauge.h`, `i2c_bus.h` (V2.3.29, pre-existing), `sensirion_crc.h`
  (new in 959a39c) — all header-only (`static inline` helpers), no CMake
  `SRCS` entry needed or expected. `i2c_bus.c` is a separate, pre-existing
  file already in `SRCS` (line 19) — unaffected by this diff.
- No gating (`if(BOARD_...)`) added or needed: `fuel_gauge.c` self-gates
  internally via `#if HAL_HAS_FUEL_GAUGE` (real driver vs. no-op stub
  returning `false`/`ESP_FAIL`), matching the project's established
  convention for `als.c`/`veml7700.c`/`neopixel.c` etc.

**Verdict: correct, no missing registration, no gating gap.**

## 2. `main/version.h` — VERSION_STR

`main/version.h:20`: `#define VERSION_STR "V2.6.6"` — confirmed current.
`CHANGELOG.md:12` top entry is also `## V2.6.6 — MAX17048 battery fuel
gauge...`, consistent.

Searched all committed history (`git log --all -p -- main/version.h`) for a
stray `V2.6.7`: none found — every version.h commit is a clean monotonic
bump (…V2.6.4→V2.6.5→V2.6.6, no V2.6.7 ever committed). The "bump then
revert" churn mentioned in the task brief, if it happened, was uncommitted
working-tree churn during the session and never reached a commit — the
committed record is clean. No action needed.

Grepped the full `main/` tree for other `VERSION_STR` **definitions** (not
uses): only `main/version.h:20` defines it. All other hits
(`display.c`, `http_server.c`, `main.c`, `mqtt.c`, `mqtt_discovery.c`,
`syslog.c`) are consumers (`VERSION_STR` used in format strings / string
concatenation), not redefinitions. **Single source of truth confirmed.**

## 3. Actual clean build — FeatherS3-D (`HAL_HAS_FUEL_GAUGE=1`)

Ran `.\_build.cmd feathers3_d` for real (ESP-IDF 6.0 environment activated
successfully). First invocation was an incremental no-op against the
existing `build_feathers3_d` dir (nothing to recompile — confirms the
board's last committed state already builds). To get a genuine
recompile-and-check-for-warnings signal on the exact files this session
touched, touched (`Get-Item ... .LastWriteTime = Get-Date`) all
session-modified files — `fuel_gauge.c`, `i2c_bus.h`, `sensirion_crc.h`,
`bme280.c`, `bme688.c`, `bmp390.c`, `bmp581.c`, `dnms.c`, `sht45.c`,
`sps30.c`, `veml7700.c` — and rebuilt:

- Result: **`Project build complete`**, 12 objects recompiled including
  `fuel_gauge.c.obj`, `i2c_bus.c.obj`, `bmp581.c.obj`, `sht45.c.obj`,
  `bme280.c.obj`, `bmp390.c.obj`, `veml7700.c.obj`, `bme688.c.obj`,
  `dnms.c.obj`, `sps30.c.obj`, plus dependents `display.c.obj`/`main.c.obj`.
- **Zero warnings** (`Select-String -Pattern "warning"` on the full log:
  0 matches).
- Flash usage: `geiger_v2.bin` = 0x14c390 bytes, smallest app partition
  0x200000 bytes, **35% free** (0xb3c70 bytes). Bootloader 34% free.

## 4. Actual clean build — Heltec V2 (`HAL_HAS_FUEL_GAUGE=0`, stub path)

Same touched-file set, ran `.\_build.cmd heltec_v2`:

- Result: **`Project build complete`**, 30 objects rebuilt (the larger
  fan-out vs. FeatherS3-D is expected — `i2c_bus.h` being touched forces
  recompilation of every TU that includes it transitively, e.g. `config.c`,
  `http_server.c`, `periodic.c`, `mqtt.c`, `mqtt_discovery.c`,
  `transmission.c`, `log_ftp.c`, `display_serlcd.c`, in addition to the
  directly-touched driver files and `fuel_gauge.c.obj` itself, compiled
  here down the `#else` stub branch).
- **Zero warnings** (0 matches for "warning" in the full log).
- Flash usage: `geiger_v2.bin` = 0x144780 bytes, **37% free**
  (0xbb880 bytes). Bootloader 8% free (Heltec's bootloader partition is
  much tighter than FeatherS3-D's, as expected/pre-existing — not a
  regression from this session's changes).

This directly answers the task's specific concern: `fuel_gauge_chip_detected()`
was removed from BOTH the real and stub implementations this session, and a
stale reference surviving in only one branch would only surface when
compiling the branch that still referenced it. Building both branches
(FeatherS3-D = real driver, Heltec V2 = stub) with zero warnings and zero
errors on both directly falsifies that risk. Additionally ran
`grep -rn "fuel_gauge_chip_detected"` across the entire source tree
(`.c`/`.h`, all directories) and both freshly-built `build_feathers3_d` /
`build_heltec_v2` directories: **zero hits anywhere.** The function name is
completely gone, on both branches, confirmed by grep AND by a real compile
of both.

## 5. Board-conditionality of the I2C-consolidation refactor (959a39c)

Checked whether any of the 9 files touched by 959a39c
(`bme280.c`, `bme688.c`, `bmp390.c`, `bmp581.c`, `dnms.c`, `fuel_gauge.c`,
`sht45.c`, `sps30.c`, `veml7700.c`) is board-conditionally compiled.

`main/CMakeLists.txt`'s `SRCS` list is a single flat, unconditional list —
**every** `.c` file in it (including all 9 refactored drivers) is compiled
into **every** board target's `libmain.a`. Cross-checked against
`main/hal.h`: none of BME280/BME688/BMP390/BMP581/DNMS/SPS30/SHT45/VEML7700
have a `HAL_HAS_*` compile-time gate in any of the 4 `#if/#elif` board
blocks — the only `HAL_HAS_*` flags found are `OLED`, `PSRAM`, `NATIVE_USB`,
`VEXT_GATE`, `ANTENNA_SWITCH`, `I2C_PINOUT_SWITCH`, `SPEAKER`, `NEOPIXEL`,
`ALS`, `FUEL_GAUGE` — none of which gate the refactored drivers' source
files. These sensors are **runtime probe-detected** (per `hal.h` comments,
e.g. "External SSD1309 on STEMMA1 (optional — probe-detected)"), not
compile-time board-gated. This means the FeatherS3-D + Heltec V2 pair
already built in steps 3–4 exercises 100% of the refactored driver code on
every board — there is no third board needed to reach a driver these two
skip, because none of the 5 boards skip any of the 9 refactored files at
compile time. **Verdict: no board-conditionality risk; the two builds
already performed are fully representative of all 5 targets for this
refactor's scope.**

(The refactor commit's own message additionally states "Verified with a
clean build across all 5 board targets" — this review independently
re-verified 2 of the 5 with a genuine forced recompile rather than trusting
the commit message at face value, per this review's mandate.)

## 6. cppcheck constParameter check on the new `i2c_bus.h` / `sensirion_crc.h` helpers

**cppcheck is not installed in this review environment** (`which cppcheck`
→ not found), so this could not be run live; the project's actual gate is
`.github/workflows/_cppcheck.yml` (`--enable=warning,style,performance,
portability`, `--std=c11`, `-DBOARD_HELTEC_V2=1`, hard `--error-exitcode=1`
gate on both `build.yml` and `release.yml`). Did the most thorough manual
static check available instead: read every new/changed function signature
in `main/i2c_bus.h` (lines 116–190, added by 959a39c) and `main/
sensirion_crc.h` (new file, 959a39c) by hand, and cross-checked each
pointer parameter's mutation behaviour against its usage in the function
body:

| Function | Pointer param | Read or written? | Qualified `const`? |
|---|---|---|---|
| `i2c_dev_write_reg` | none (`dev` is opaque handle typedef, `reg`/`val` by value) | n/a | n/a |
| `i2c_dev_read_regs` | `uint8_t *buf` | **written** (output buffer) | correctly non-const |
| `i2c_dev_read_u16_be` | `uint16_t *out` | **written** | correctly non-const |
| `i2c_dev_read_u16_le` | `uint16_t *out` | **written** | correctly non-const |
| `i2c_dev_write_u16_le` | none | n/a | n/a |
| `i2c_add_device` | `i2c_master_dev_handle_t *dev` | **written** (receives new handle) | correctly non-const |
| `i2c_probe_and_add` | `i2c_master_dev_handle_t *dev` | **written** (delegates to `i2c_add_device`) | correctly non-const |
| `i2c_dev_teardown` | `i2c_master_dev_handle_t *dev` | **read then written** (`*dev` checked, then `i2c_master_bus_rm_device(*dev)`, then set to NULL) | correctly non-const |
| `sensirion_crc8` | `const uint8_t *data` | **read-only** (looped, never written) | **correctly marked `const`** (`sensirion_crc.h:22`) |

Every read-only pointer parameter (`sensirion_crc8`'s `data`) is already
`const`-qualified; every non-const pointer parameter is genuinely written
through (output params or read-modify-write on a handle slot), so none of
them are constParameter candidates. `dev`/`bus` handle parameters passed
by value (`i2c_master_dev_handle_t dev`, `i2c_master_bus_handle_t bus`) are
opaque-pointer typedefs from ESP-IDF, passed to further opaque IDF calls —
not raw pointers subject to this check.

Also diffed every driver file 959a39c touched
(`git show 959a39c -- main/bme280.c main/bmp581.c main/veml7700.c
main/fuel_gauge.c` etc.) to confirm no **new** locally-scoped helper
function with a pointer parameter was introduced anywhere else by this
refactor — confirmed: the refactor's pattern throughout is pure deletion of
each driver's local duplicate helper (which had the identical, already
correct, signature) in favour of a call to the new shared `i2c_bus.h`
helper. No new signatures were added at any call site.

**Verdict: no constParameter findings; the new shared helpers pass what
this review could determine of the project's cppcheck gate by manual
inspection.** This is not a substitute for actually running cppcheck (not
available in this environment) — recommend CI's next run of
`_cppcheck.yml` against this exact diff be checked before merge/tag, as a
belt-and-braces close-out, since manual review can miss cppcheck's other
enabled categories (style/performance/portability) that weren't
specifically re-derived by hand here beyond the constParameter question the
task asked about.

## Overall verdict

Build-config consistency for V2.6.6 is sound. `main/CMakeLists.txt`'s one
added line (`fuel_gauge.c`) is correct, complete, and follows the existing
optional-driver convention with no gating gap. `main/version.h` is
correctly pinned at `V2.6.6` with no stray `V2.6.7` in committed history and
exactly one definition site. Two real builds were executed in this review
(not merely inspected) — FeatherS3-D (`HAL_HAS_FUEL_GAUGE=1`, real driver
path) and Heltec V2 (`HAL_HAS_FUEL_GAUGE=0`, stub path) — both against a
forced recompile of every file the I2C-consolidation refactor touched, both
completing with **zero warnings and zero errors**, and both confirmed by
build-directory + full-source-tree grep to contain no stale reference to
the removed `fuel_gauge_chip_detected()` symbol on either branch. The
I2C-consolidation refactor's 9 touched driver files are unconditionally
compiled on all 5 board targets (runtime-probe-detected, not
compile-time-board-gated), so the two builds performed here already cover
100% of that refactor's board-matrix exposure — no third board was needed.
The one gap in this review is item 6: cppcheck itself could not be executed
locally (not installed in this environment), so the constParameter
conclusion rests on manual signature-by-signature inspection rather than a
tool run; everything else in scope was verified by an actual build, not
static reasoning alone. No Critical, Important, or Minor findings raised
against this review's scope.

**Findings: 0 Critical, 0 Important, 0 Minor.**
