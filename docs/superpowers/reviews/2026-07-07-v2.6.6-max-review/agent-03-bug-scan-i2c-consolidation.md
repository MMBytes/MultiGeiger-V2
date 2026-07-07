# Agent 3: deep bug scan of the I2C driver consolidation refactor (`959a39c`)

**Status: done**

Scope: `959a39c` ("V2.6.6: consolidate I2C driver boilerplate and Sensirion
CRC-8 (no functional change)"). New shared headers `main/i2c_bus.h` and
`main/sensirion_crc.h`, plus the 8 refactored drivers: `main/bmp581.c`,
`main/bmp390.c`, `main/bme280.c`, `main/bme688.c`, `main/veml7700.c`,
`main/sht45.c`, `main/sps30.c`, `main/dnms.c`. Also traced every caller of
`i2c_bus.h`'s shared helpers, including `main/fuel_gauge.c` (added to
`i2c_bus.h` in the same commit; its own driver logic was covered by the prior
review round's Agent 2 and is not re-litigated here except where it
illuminates a helper-level bug).

Method: `git show 959a39c --stat` / `git show 959a39c -- <file>` for a
line-by-line before/after diff of each of the 8 drivers + fuel_gauge.c, then
full reads of the two new headers and every current driver file to check the
diff didn't miss a call site.

## Findings

**1. No issue — byte-order (BE/LE) assignment is correct for every caller.**
`i2c_bus.h:128-146` provides two distinct 16-bit register readers:
`i2c_dev_read_u16_be()` (MSB-first) and `i2c_dev_read_u16_le()` (LSB-first).
Grepped every call site: `fuel_gauge.c:70-73,118,125,133,147` (MAX17048, all
`_be`) and `veml7700.c:73,85,111,116` (VEML7700, all `_le` for both the
16-bit write helper and the two reads). No driver crossed over to the wrong
variant during consolidation — the exact bug class the header's own comment
(`i2c_bus.h:106-109`) says this consolidation was meant to prevent is not
present. `write_u16_le` is used only by VEML7700 (`i2c_dev_write_u16_le`,
`veml7700.c:73,85`), correctly LE, matching its own wire format comment
(now moved into `i2c_bus.h:148-149`, previously in `veml7700.c` itself).
**No issue.**

**2. No issue — the four register-based BMx drivers (bmp581, bmp390, bme280,
bme688) are a pure 1:1 mechanical substitution.** Diffed each in full via
`git show 959a39c -- main/bmp581.c` etc. Every `write_reg(reg, val)` /
`read_regs(reg, buf, n)` call site was replaced with
`i2c_dev_write_reg(s_dev, reg, val)` / `i2c_dev_read_regs(s_dev, reg, buf, n)`
with identical arguments, identical call order, identical error-handling
branches (each `if (err != ESP_OK) { ...; return err; }` block kept its log
message and control flow verbatim). The `i2c_device_config_t` literal
(`dev_addr_length = I2C_ADDR_BIT_LEN_7`, fixed `device_address`,
`scl_speed_hz`) that each driver built by hand is reproduced exactly by
`i2c_add_device()` (`i2c_bus.h:158-167`) with the same speed constants passed
through unchanged (100 kHz for BMP581/390/BME280/688 — confirmed
`bmp581.c` original `100000`, `i2c_bus.h` helper takes it as a parameter, no
hardcoded default substituted). Both 100 ms transaction timeouts
(`i2c_bus.h:118,125` for write_reg/read_regs) match every removed driver's
hardcoded `100` literal. `i2c_dev_teardown()` (`i2c_bus.h:185-190`) reproduces
`i2c_master_bus_rm_device(s_dev); s_dev = NULL;` exactly, at every call site
that previously did the same pair of statements inline. **No issue** — "no
functional change" holds for these four files.

**3. No issue — probe timeouts / retry structure unchanged for every
driver.** Confirmed by grepping all `i2c_master_probe(...)` /
`i2c_probe_and_add(...)` call sites post-refactor and cross-checking each
literal against the pre-refactor value in the diff (or, where the diff didn't
touch the line, confirming the line is untouched — i.e. provably identical):
- `bmp581.c:71,73` — 50 ms for both 0x46/0x47 candidates (unchanged, diff
  didn't touch these lines).
- `bmp390.c:104` — 50 ms (unchanged).
- `bme280.c:79` / `bme688.c:96` — 100 ms per address in their
  multi-candidate loops (unchanged; the commit message explicitly calls out
  that multi-address probers keep calling `i2c_master_probe()` directly
  rather than going through `i2c_probe_and_add()`, to avoid double-probing
  the address they settle on — verified true, neither file calls
  `i2c_probe_and_add`).
- `sht45.c:119` / `sps30.c:101` / `dnms.c:71` — all still call
  `i2c_master_probe()` directly at 50 ms (unchanged) — these three did *not*
  get switched to `i2c_probe_and_add()` even though they only have one
  candidate address; not a bug (behavior is identical either way), just an
  inconsistency in how far the consolidation reached, worth noting for a
  future pass.
- `veml7700.c:62` / `fuel_gauge.c:42` — both now call
  `i2c_probe_and_add(bus, ADDR, speed, 50, &s_dev)` — 50 ms preserved exactly
  from each driver's original standalone `i2c_master_probe(bus, ADDR, 50)`
  call. No retry-count or timeout drift anywhere. **No issue.**

**4. No issue — CRC-8 polynomial/init/xor-out constants are correct and
identical for all three consumers.** `sensirion_crc.h:19-28`: poly `0x31`,
init `0xFF`, MSB-first, no reflection, no XOR-out, computed 2 bytes (one
16-bit word) at a time. Diffed each of the three pre-refactor
implementations being replaced:
  - `sps30.c` pre-refactor `crc8()` — identical algorithm, identical
    constants.
  - `sht45.c` pre-refactor `crc_ok(a, b, crc)` — algorithmically identical
    (init 0xFF, XOR each byte in turn, same shift/poly loop), just fused
    the compare into the function; `sensirion_crc8(&buf[0], 2) != buf[2]`
    (now at `sht45.c:50,97,214`) is mathematically equivalent to the old
    `!crc_ok(buf[0], buf[1], buf[2])`.
  - `dnms.c` pre-refactor `crc8()` — identical algorithm, identical
    constants, identical doc-comment about "same as SHT45/SPS30" (now
    literally true instead of independently-asserted).
  All three vendor lines (Sensirion SHT4x, Sensirion SPS30, and Nettigo's
  Sensirion-derived DNMS protocol) use the *same* CRC-8 variant — there was
  no polynomial divergence to accidentally collapse in the first place, and
  the shared implementation matches all three call patterns exactly (always
  invoked with `n=2`, per-16-bit-word, matching the header's own doc-comment
  at `sensirion_crc.h:16-18`). **No issue.**

**5. Minor — the explicit defensive `s_dev = NULL` on an add-device failure
was silently dropped for the two drivers that route through
`i2c_probe_and_add()` (veml7700, fuel_gauge), narrowing to a real (if very
unlikely) gap not present in the other six drivers.** Before the refactor,
both `veml7700.c` and `fuel_gauge.c` had:
```c
esp_err_t err = i2c_master_bus_add_device(bus, &devcfg, &s_dev);
if (err != ESP_OK) {
    ESP_LOGW(TAG, "i2c_master_bus_add_device: %s", esp_err_to_name(err));
    s_dev = NULL;   // <-- explicit reset, now gone
    return err;
}
```
After the refactor (`veml7700.c:62-69`, `fuel_gauge.c:42-49`):
```c
esp_err_t err = i2c_probe_and_add(bus, ADDR, speed, 50, &s_dev);
if (err == ESP_ERR_NOT_FOUND) { ...; return err; }
else if (err != ESP_OK) { ESP_LOGW(...); return err; }   // no s_dev = NULL
```
`i2c_probe_and_add()` (`i2c_bus.h:176-181`) only short-circuits to
`ESP_ERR_NOT_FOUND` *before* calling `i2c_add_device()` — that path leaves
`s_dev` exactly as it was (still `NULL`, its static initializer, since both
`_init()` functions are only ever called once from `main.c`), which is fine
and was already confirmed safe by the prior review round (2026-07-07 Agent 2,
finding 7). But if the probe succeeds and `i2c_add_device()` itself then
fails (e.g. `ESP_ERR_NO_MEM` — out of I2C device-handle slots, a real
though rare failure mode), the helper returns whatever
`i2c_master_bus_add_device()` returned *without* touching `*dev` on that
specific path, and the caller no longer re-nulls it either. ESP-IDF's
`i2c_master_bus_add_device()` contract does not guarantee it leaves the
output handle untouched on error (this was not verifiable locally — no
vendored IDF driver source found in the repo tree to confirm one way or the
other). Practical impact is contained: `s_ready` is never set until *after*
this call succeeds, and every other function in both files gates on
`s_ready` before touching `s_dev`, so there is no live use-before-init path
today. This is a class-wide regression only in the state-hygiene sense (the
belt was removed, the suspenders — `s_ready` gating — still hold), and it
only affects the two drivers that adopted the new `i2c_probe_and_add()`
convenience wrapper, not the six that call `i2c_add_device()` directly (those
never had the extra reset either, before or after — see finding 2). It would
only bite a future change that calls `_init()` more than once after a
partial add-device failure, or that reads `s_dev` without checking
`s_ready` first. **Minor** — no live symptom, but it is a genuine, if small,
divergence from "no functional change" at the state level, and it is the
kind of thing worth a one-line follow-up (`if (err != ESP_OK) *dev = NULL;`
inside `i2c_probe_and_add()`/`i2c_add_device()` itself, so all 9 callers get
the guarantee for free).

**6. No issue — no public function signatures changed.** Every `_init()`,
`_read()`, `_present()` etc. across all 8 drivers keeps its exact
pre-refactor prototype (confirmed via the diffs above — every changed line
is inside a function body, never a signature). Grepped `main.c` and
`transmission.c` (the two files most likely to call into these drivers) for
any of the 8 drivers' public entry points — no call-site changes needed or
made. **No issue.**

**7. No issue — error-code propagation is identical, not degraded.** Every
driver's distinct error paths (`ESP_ERR_NOT_FOUND` for "not present at this
address", the underlying `esp_err_t` from a failed bind, `ESP_FAIL` for a
CRC or sanity-check failure, the specific `esp_err_t` from a failed register
transaction) are preserved verbatim — the shared helpers are thin pass-
throughs that return exactly what the underlying `i2c_master_*` call
returned, with `i2c_probe_and_add()`'s only added behavior being the
translation of "probe timed out" into `ESP_ERR_NOT_FOUND`, which is exactly
what every pre-refactor driver's own probe-failure branch already did by
hand (e.g. old `veml7700.c`: `if (i2c_master_probe(...) != ESP_OK) { ...;
return ESP_ERR_NOT_FOUND; }`). No driver lost a distinction it used to make
(e.g. nothing collapses NOT_FOUND and TIMEOUT, or CRC-fail and I2C-NAK, into
each other). **No issue.**

**8. Minor — inconsistent adoption of `i2c_probe_and_add()` leaves a small
"why not?" gap for a future maintainer.** As noted in finding 3, `sht45.c`,
`sps30.c`, and `dnms.c` each still hand-roll `probe → i2c_add_device()` as
two separate statements instead of the one-line `i2c_probe_and_add()` call
that `veml7700.c`/`fuel_gauge.c` now use, even though all three are
single-candidate-address drivers (the documented reason for *not* using the
helper — avoiding a double-probe on multi-candidate drivers — doesn't apply
to them). Not a functional bug (behavior is identical either way — see
finding 3), just an incomplete pass that a future contributor might read as
inconsistent style. **Minor.**

## Overall verdict

The "no functional change" claim **holds** for wire behavior: byte order
(BE/LE) is correctly assigned to every one of the 6 register-based drivers +
fuel_gauge with no cross-over, all I2C transaction timeouts (100 ms) and
probe timeouts (50/100 ms per driver, matching each driver's pre-refactor
value) are byte-for-byte preserved, retry/candidate-address logic is
untouched for the multi-address drivers (BMP581, BME280, BME688), the
Sensirion CRC-8 constants (poly 0x31, init 0xFF, no reflection/XOR-out) are
correctly consolidated and were already identical across all three
pre-refactor implementations (SHT45, SPS30, DNMS) so there was no
polynomial-mismatch risk to begin with, no public function signature
changed, and no driver's error-code distinctions were lost or collapsed in
the shared helpers. One **Minor** finding (5): the two drivers that adopted
the new `i2c_probe_and_add()` convenience wrapper (veml7700, fuel_gauge)
silently lost the explicit `s_dev = NULL` defensive reset on the narrow
"probe succeeded but add_device itself failed" path — currently harmless
because `s_ready` gates every subsequent access and both `_init()` functions
are call-once, but it is a genuine (if inert) state-hygiene regression worth
a one-line fix inside the shared helper. One additional **Minor** finding
(8) notes that SHT45/SPS30/DNMS weren't migrated to the same
`i2c_probe_and_add()` convenience call despite being eligible, a style/
completeness gap, not a bug. **Zero Critical and zero Important findings.**
Net: the consolidation is safe to ship as-is; finding 5 is worth folding
into a future one-line hardening pass but does not block anything.

**Severity tally:** Critical: 0. Important: 0. Minor: 2 (findings 5, 8).
