# Agent 10: security review

**Status: done**

## Security Review — V2.6.6 Fuel-Gauge + I2C Consolidation (f26f85e..86a0273)

**Scope covered:** `main/fuel_gauge.c`/`.h` (new), `main/hal.h` (`PIN_VBUS_DETECT`), `main/http_server.c`
(`config_post()`, `format_battery()`), `main/main.c`, `main/mqtt.c` (rich-state JSON `APPEND` block),
`main/mqtt_discovery.c`, `main/transmission.c`, `main/version.h`, `main/CMakeLists.txt`,
`main/config_fields.def`, `main/i2c_bus.h` (new), `main/sensirion_crc.h` (new), and the 8 refactored
I2C drivers (`bmp581.c`, `bmp390.c`, `bme280.c`, `bme688.c`, `veml7700.c`, `sht45.c`, `sps30.c`,
`dnms.c`).

### 1. `/config` POST handling for `batt_present` — no issue found (Minor note only)

`config_post()` (`http_server.c:1847-2043`) requires `check_auth()` (Basic auth,
`http_server.c:109-124`) and `check_same_origin()` (`http_server.c:168-183`) before touching the body
— both are pre-existing, unchanged by this diff, and confirm the LAN-exposed `/config` endpoint is
not unauthenticated as the task brief flagged as a possibility to check.

`batt_present` is declared via the same `X_BOOL(name, key, def)` X-macro as every other checkbox
field (`config_fields.def:245`), so it goes through the identical generic dispatch as
`use_external_antenna` / `i2c_pinout` — no special-cased parsing code was added for this field.
Critically, `X_BOOL`'s POST-apply expansion (`config.c:374-378`) does **not** parse the value string
at all — the mere presence of the `batt_present=` key sets the field to `true` unconditionally,
regardless of content, length, or encoding of the value. There is no `strtol`/`strtof`/`memcpy` on
attacker-controlled bytes for this field — an oversized or malformed value cannot influence parsing
because the value is never read. Bounds/overflow risk: none.

The force-clear block (`http_server.c:1979-1983`):
```c
#if !HAL_HAS_FUEL_GAUGE
    cfg_next.batt_present = false;
#endif
```
mirrors the pre-existing `use_external_antenna` (`http_server.c:1969-1971`) and `i2c_pinout`
(`http_server.c:1975-1977`) defence-in-depth pattern exactly — same ordering (after generic dispatch,
before `config_save`), same compile-time gate, same "UI already greys the checkbox but a hand-crafted
POST could still set it" rationale. A hand-crafted POST with `batt_present=1` to a non-FeatherS3-D
board's `/config` is force-cleared before `config_save()` is called, so it is never persisted to NVS
and never observed by `fuel_gauge_set_user_present()` (whose non-FeatherS3-D stub is a no-op returning
nothing meaningful anyway, `fuel_gauge.c:164`). No crash, no UB, no unintended state change — worst
case is the same silent-no-op behavior as the two pre-existing analogous fields.

### 2. Buffer sizing / format-string / snprintf return-value handling — no issue found

Every new formatting call in the diff goes through an existing bounds-checked helper with a **literal**
format string; no I2C-sourced or user-controlled data is ever passed as a format-string argument itself
— always as a `%s`/`%f`/`%d` *parameter*:
- `mqtt.c:537-539` — the new `APPEND(",\"batt_v\":%.3f", ...)` / `batt_soc` / `batt_rate` lines all use
  the project's `APPEND` macro (`mqtt.c:458-465`), which is the **safe** pattern this project has
  previously flagged the danger of *not* using: `n += (_w < rem) ? _w : (rem - 1)` clamps `n` to never
  exceed `sizeof(buf)-1` even on truncation. This is not the bare `n += snprintf(...)` anti-pattern.
- `http_server.c:725-731` (`format_battery()`) uses `append_safe()`, the same clamped-accumulation
  helper as every other `/status` block, with a literal format string and numeric-only interpolation.
- `transmission.c:1499-1502` — the new battery `ESP_LOGI` line is a literal format string; the
  `"present"`/`"absent"` string is a fixed literal selected by a ternary, not an attacker-influenced
  value, and is passed as `%s`.

**Independent buffer-size re-derivation for `mqtt.c`'s `buf[1792]`** (recomputing worst case rather
than trusting the in-code comment or a prior agent's number):
- `,"batt_v":%.3f` → literal prefix `,"batt_v":` = 10 bytes. `batt_v` is derived from a `uint16_t` raw
  VCELL register (`fuel_gauge.c:115-118`, scale 0.000078125), so its maximum possible value is
  `65535 × 0.000078125 ≈ 5.11996`, formatted `%.3f` → `"5.120"` (5 chars, never negative, never
  scientific notation since the value is bounded and small). Total ≈ 15 bytes.
- `,"batt_soc":%.1f` → prefix `,"batt_soc":` = 12 bytes. `soc` = `raw/256`, raw is `uint16_t` max
  65535 → max ≈ 255.996, `%.1f` → `"256.0"` (5 chars). Total ≈ 17 bytes.
- `,"batt_rate":%.2f` → prefix `,"batt_rate":` = 13 bytes. `rate` = `(int16_t)raw × 0.208`, raw
  reinterpreted as **signed** 16-bit, so range is `-32768..32767 × 0.208` = `-6815.744..6815.736`,
  `%+.2f`-worst-case (project uses `%.2f` here, no explicit sign flag, but a negative value still needs
  the `-`) → `"-6815.74"` (8 chars). Total ≈ 21 bytes.
- Sum of new worst-case bytes: 15 + 17 + 21 = **53 bytes** — matches the in-code comment's own
  claim exactly. Because every source value is bounded by a 16-bit register width (no float can ever
  reach an unbounded/`FLT_MAX`-scale magnitude that would blow out `%f`'s fixed-point expansion — see
  finding 3 below for why this bound is guaranteed even on a corrupted read), this worst case is a hard
  ceiling, not a probabilistic estimate.
- Buffer is 1792 bytes; the pre-V2.6.6 baseline already carried ~379 bytes of slack per the file's own
  historical comment trail (cpm5/cpm15 addition still left "~380 B slack"). 379 − 53 = ~326 bytes
  remaining, which matches the diff's own comment (`mqtt.c:17-21`). Confirmed independently: **no
  overflow risk, margin holds with room to spare.**

### 3. I2C error handling / CRC-8 — no issue found

Reviewed the full diff hunks for all 8 refactored drivers plus `fuel_gauge.c` against the pattern
"read into a buffer, then use before checking the return code":
- Every refactored call site (`bme280.c`, `bme688.c`, `bmp390.c`, `bmp581.c`, `veml7700.c`) is a pure
  mechanical rename from a private `read_regs()`/`reg_read16()` to the shared `i2c_dev_read_regs()`/
  `i2c_dev_read_u16_be()`/`_le()` in `i2c_bus.h` — the `if (err != ESP_OK) return err;` (or equivalent)
  guard immediately following each read is preserved unchanged at every site. No new call site drops
  the error check.
- `fuel_gauge_read()` (`fuel_gauge.c:105-141`) explicitly documents and implements an all-or-nothing
  contract: each of `volts`/`soc_pct`/`rate_pct_per_hr` is read into a **local** (`v`, `s`, `r`) and
  only copied to the caller's output pointers after every requested register read has returned
  `ESP_OK` — a mid-sequence I2C failure returns early without writing any output pointer, so no caller
  (`http_server.c:717-722`, `mqtt.c:535-540`, `transmission.c:1494-1503`) can observe a partially
  populated result. All three callers correctly gate use on the `ESP_OK` return.
- `sensirion_crc8()` (`sensirion_crc.h:19-28`) is a pure mechanical extraction of three previously
  duplicated identical implementations (`sht45.c`, `sps30.c`, `dnms.c`) — same polynomial (0x31), same
  init (0xFF), same MSB-first computation. Every call site's CRC comparison is preserved unchanged and
  is always followed by `return ESP_FAIL` (or equivalent) on mismatch, never a silent continue:
  `sht45.c:1310/1319/1352`, `sps30.c:1404/1437/1446/1451/1460/1465`, `dnms.c:556-558`. No CRC failure
  is ever ignored or downgraded to a warning-only path that still uses the data.
- `sps30_read_serial()`'s incremental `out_idx` write loop (`sps30.c:1401-1407`) writes validated bytes
  only *after* each word's CRC passes, and returns `ESP_FAIL` immediately on the first mismatch without
  null-terminating or otherwise finalizing `out` — callers must (and do, per existing code paths not
  touched by this diff) check the return value before using `out`.

### 4. New GPIO input (`PIN_VBUS_DETECT`) — confirmed read-only, not a trust boundary

`PIN_VBUS_DETECT` (GPIO 34, `hal.h:192`) is configured `GPIO_MODE_INPUT` exactly once
(`fuel_gauge.c:83`) and is only ever read via `gpio_get_level()` inside `fuel_gauge_vbus_present()`
(`fuel_gauge.c:92-95`). Repo-wide search confirms its only consumer is a diagnostic string
(`"present"`/`"absent"`) folded into the per-cycle `ESP_LOGI` battery line in `transmission.c:1439` —
it is never read as a precondition for any other action, never used in an `if` that gates a
privileged/irreversible operation, and no code path ever calls `gpio_set_direction()` on it again to
flip it to output. A manipulated VBUS line (physically injecting a voltage on GPIO 34) could at most
flip a cosmetic "present"/"absent" word in a log line — it is not a trust boundary for anything
security-relevant.

### 5. NVS storage of `batt_present` — type-safe, same mechanism as every other bool

Confirmed via `config.c`'s X-macro expansions:
- **Load** (`config.c:85-91`): `nvs_get_u8(h, key, &_v)` into a stack `uint8_t _v`, then
  `cfg->name = (_v != 0)` — any byte value 0x00-0xFF normalizes cleanly to `false`/`true`. A corrupted
  or garbage NVS blob for this key cannot cause an OOB read: `nvs_get_u8` bounds its own read to
  exactly 1 byte, and the normalization is a simple integer comparison, not a pointer/length
  operation.
- **Save** (`config.c:301-304`, elided but same pattern as load): `nvs_set_u8(h, key, _u)` — a plain
  1-byte write, same as every other `X_BOOL` field.
- No special-case code exists anywhere for `batt_present`'s NVS key (`"batt_present"`,
  `config_fields.def:245`) — it is indistinguishable in the load/save code path from
  `wifi_11bg_only`, `i2c_pinout`, etc. Any hypothetical NVS corruption affecting this key behaves
  identically to corruption affecting any other pre-existing boolean field (already an accepted risk
  profile, not new).

### 6. New attack surface — explicit answer

**No.** This changeset introduces no new attack surface and no new memory-safety risk relative to the
pre-V2.6.6 baseline:
- The new `/config` field (`batt_present`) reuses the existing generic X-macro dispatch verbatim — it
  adds zero new parsing code, zero new buffer operations, and is force-cleared on non-FeatherS3-D
  boards using the identical pattern proven safe for two pre-existing fields.
- The new MQTT/HA/`/status` battery fields use only the project's existing bounds-checked
  accumulation helpers (`APPEND` macro, `append_safe()`) with literal format strings — no new
  injection class is opened.
- The I2C consolidation (`i2c_bus.h`, `sensirion_crc.h`) is a byte-for-byte behavior-preserving
  extraction of previously-duplicated code; every error check and every CRC-failure branch survives
  the refactor unchanged.
- The one genuinely new I/O surface, `PIN_VBUS_DETECT`, is a passive diagnostic input with a single
  consumer that never gates any decision.
- `batt_present`'s NVS persistence uses the same 1-byte `nvs_get_u8`/`nvs_set_u8` path as every other
  config bool, with no bespoke code that could introduce a new corruption/OOB class.

## Overall verdict

**PASS.** No Critical, Important, or Minor security findings. The fuel-gauge feature and the I2C
consolidation refactor are both security-neutral: the former reuses every existing safe pattern
(schema-driven config dispatch, `APPEND`/`append_safe` bounds-checked formatting, all-or-nothing I2C
read contracts) with no new parsing or buffer-management code, and the latter is a mechanical
call-site rename that preserves every pre-existing error check and CRC-validation branch verbatim.
Independent re-derivation of the `mqtt.c` `buf[1792]` worst-case sizing confirms the declared ~326-byte
margin holds (53 bytes of new content against ~379 bytes of pre-existing slack), and every new
battery-derived value is intrinsically bounded by its source register's bit width, so no format-string
expansion or numeric edge case can threaten that margin. This changeset introduces no new attack
surface and is safe to ship from a security standpoint.
