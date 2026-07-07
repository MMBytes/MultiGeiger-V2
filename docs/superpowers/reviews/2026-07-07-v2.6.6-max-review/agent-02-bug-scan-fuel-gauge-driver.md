# Agent 2: fuel_gauge.c/.h deep bug scan (V2.6.6 final state, post-checkbox rework)

**Status: done**

Scope: `main/fuel_gauge.c` + `main/fuel_gauge.h` as they stand after `86a0273`
(manual `batt_present` config-bool rework), not the diff alone. Cross-checked
against `main/hal.h` (PIN_VBUS_DETECT), `main/i2c_bus.h` (shared I2C helpers
introduced in `959a39c`), and every call site (`main.c`, `http_server.c`,
`mqtt.c`, `mqtt_discovery.c`, `transmission.c`).

## Findings

**1. Register-level math — no issue, confirmed correct.** VCELL scale
`0.000078125f` (fuel_gauge.c:118) = 78.125 µV/LSB exactly. SOC scale
`/256.0f` (fuel_gauge.c:125) = 1/256 %/LSB exactly. CRATE's
`(float)(int16_t)raw * 0.208f` (fuel_gauge.c:133) correctly reinterprets the
raw unsigned 16-bit read as two's-complement *before* scaling — the sign
lands on the right operand. Register addresses (0x02/0x04/0x08/0x0A/0x0C/
0x14/0x16/0x18/0x19/0x1A, fuel_gauge.c:18-27) and the STATUS bit layout in
the header doc (fuel_gauge.h:97-98: RI/VH/VL/VR/HD/SC) match the MAX17048/49
datasheet's register map, including the VRESET/ID byte-split (0x18 = high
byte of the VRESET/ID word = threshold+Dis, 0x19 = low byte = chip ID) — this
is genuinely correct byte-addressable auto-increment usage, not a mistake.
Byte order: `i2c_dev_read_u16_be()` (i2c_bus.h:130-136) is unambiguously
big-endian and is the one used throughout this file — no LE/BE mixup.

**2. `fuel_gauge_read()` all-or-nothing contract — correctly implemented.**
fuel_gauge.c:112-140 buffers into locals `v`/`s`/`r` and only writes
`*volts`/`*soc_pct`/`*rate_pct_per_hr` after every requested register read
has succeeded (lines 136-138). A mid-sequence NAK (e.g. VCELL OK, SOC fails)
returns early at line 124 without touching any output pointer. This is the
fix from the prior review round (`30d3b1d`) and it holds. **No issue.**

**3. Important — `fuel_gauge_read_diag()` does NOT get the same
all-or-nothing treatment as its sibling, and its one caller discards the
return code.** fuel_gauge.c:143-157: `version` is written directly to the
caller's pointer at line 147 *before* `status` is attempted at line 152. If
the VERSION read succeeds but the STATUS read then fails, the function
returns a non-OK `err` with `*version` already populated with real data and
`*status` left in whatever state `i2c_dev_read_regs()`'s failed
`i2c_master_transmit_receive()` left the buffer in — which is not guaranteed
by the I2C master driver's contract to be untouched on failure. The only
call site, `transmission.c:1436`, makes this live: it pre-seeds
`version = 0xFFFF; status = 0xFF;` as sentinels (transmission.c:1434-1435)
and then calls `fuel_gauge_read_diag(&version, &status);` **without checking
the return value** (transmission.c:1436), unconditionally logging both
fields at line 1437. In the success/success and fail-on-version paths this
degrades gracefully (sentinels survive or get overwritten correctly), but in
the succeed-on-version/fail-on-status path the logged `status=0x%02X` is
whatever partial/undefined byte the failed transaction left behind, not
provably the `0xFF` "read failed" sentinel the header's diagnostic-sentinel
convention (fuel_gauge.h:94-96, and the analogous comment at
fuel_gauge.c:61-62) implies. This is the exact bug class that was
deliberately fixed in `fuel_gauge_read()` one function above by the prior
review round (finding #7 in the 2026-07-03 review) — the fix was not
propagated to `fuel_gauge_read_diag()`. Impact is limited to a diagnostic
log line (not control flow — `fuel_gauge_present()`/`fuel_gauge_read()`
gate every consumer of real battery data), so it's **Important, not
Critical**, but it's a real, currently-live gap, not a theoretical one.

**4. Important — `fuel_gauge_vbus_present()` is unnecessarily coupled to
MAX17048 I2C-probe success, and the header doc overclaims accuracy for the
case where they diverge.** fuel_gauge.c:92-95 gates on `s_ready`, which is
only set `true` after the MAX17048 successfully ACKs at 0x36
(fuel_gauge.c:38-49, 87). But `PIN_VBUS_DETECT` (hal.h:192, GPIO34) is
documented as "plain digital input, driven by dedicated board circuitry"
(fuel_gauge.c:81-82) — i.e. it is physically and logically independent of
the fuel-gauge chip. If the onboard MAX17048 ever fails to probe (dead
chip, cold solder joint — a real, if low-probability, hardware-fault mode
on a fixed onboard part with no retry path, since `fuel_gauge_init()` is
called exactly once at `main.c:1002`), `gpio_set_direction()` is never
reached (early return at fuel_gauge.c:45) and `fuel_gauge_vbus_present()`
will report `false` for the rest of the session regardless of the pin's
actual electrical state. The header's doc-comment for this function
(fuel_gauge.h:58-63) states an absolute dichotomy — "Always true while
running on USB power... reads false only when running purely off battery
with USB unplugged" — that is not true in this failure mode: VBUS can be
genuinely present while the function reports `false` because the *fuel
gauge*, not VBUS detection, failed. This is a design smell (an unrelated
capability silently disabled by an unrelated dependency's failure) as well
as a doc-accuracy gap. **Important** — no live symptom has been observed
(requires a failed onboard chip), but the doc's unconditional claim is
provably false in a real, reachable code path.

**5. Minor — `fuel_gauge_init()`'s "hard no-op" idempotency claim is
conditioned on the first call having succeeded, but the doc doesn't say
so.** fuel_gauge.h:51-54 states "A second call is a hard no-op — it returns
ESP_OK immediately without re-probing the chip... it does not refresh or
re-validate any state," phrased as an unconditional property of calling the
function twice. In fact this is only true when `s_ready` is already `true`
(fuel_gauge.c:39); if the *first* call failed (`ESP_ERR_NOT_FOUND` or a bind
error), `s_ready` stays `false` and a second call *will* re-probe the bus
end-to-end. There's currently only one call site so this is unreachable in
practice, but the doc-comment's phrasing would mislead a future maintainer
who wants to add a retry-after-failure call, into believing it's a dead end.
**Minor** — doc precision only, not a functional bug.

**6. Stub branch (`HAL_HAS_FUEL_GAUGE == 0`) — verified safe, matches every
call site's expectations.** fuel_gauge.c:161-172: `fuel_gauge_init()`
returns `ESP_OK` without touching `bus` (consistent with the "no board, no
sensor, nothing to fail" contract other stub-able drivers use);
`fuel_gauge_vbus_present()`/`fuel_gauge_present()` both return `false`, which
every call site (`http_server.c:715`, `mqtt.c:544`, `mqtt_discovery.c:59`,
`transmission.c:1431,1439`) treats as "skip this block entirely" — confirmed
by reading each of those call sites, none of them branch on
`fuel_gauge_init()`'s stub return value in a way that would matter (it's
never checked). `fuel_gauge_read()`/`fuel_gauge_read_diag()` return
`ESP_FAIL` with all output params correctly `(void)`-cast and left
untouched — every caller already gates on `== ESP_OK` before reading output
params, so no uninitialized-read path exists through the stub. **No issue.**

**7. `s_dev` / `s_ready` state discipline on init failure — correct.**
Traced `i2c_probe_and_add()` (i2c_bus.h:176-181): on `ESP_ERR_NOT_FOUND` it
returns before calling `i2c_add_device()`, so `s_dev` is left exactly as it
was (`NULL`, its static initializer, since `fuel_gauge_init()` is only ever
called once) — never a dangling/garbage handle. Every subsequent function
(`fuel_gauge_vbus_present`, `fuel_gauge_present`, `fuel_gauge_read`,
`fuel_gauge_read_diag`) gates on `s_ready` before touching `s_dev`
(fuel_gauge.c:93,97-98,106,144), and `s_ready` is only ever set at
fuel_gauge.c:87, strictly after both the I2C bind and the
`gpio_set_direction()` call succeed. No use-before-init or double-init path
found. **No issue** — this is the fix from the prior review round
(`30d3b1d`, init-ordering) holding correctly.

**8. GPIO34 (`PIN_VBUS_DETECT`) — no pin conflict.** hal.h:192 defines it
once; the reserved-pin table at hal.h:208-214 lists "IO34 VBUS-present
detect" as the sole claim on that pin, and nothing else in `main/` sets its
direction or reads its level. GPIO34 is a normal bidirectional GPIO on
ESP32-S3 (unlike the original ESP32 where 34-39 are input-only), and
FeatherS3-D uses quad, not octal, PSRAM/flash, so GPIO33-37 are not
claimed by the memory interface. `gpio_get_level()` is called fresh on every
invocation (fuel_gauge.c:94) with no caching/staleness risk. **No issue.**

**9. Doc-comment accuracy, file header and per-function — otherwise
accurate.** Re-read fuel_gauge.h:1-37 (file header) and fuel_gauge.c:1,33-37
against the actual `86a0273` implementation: the VCELL-threshold-was-wrong
narrative (fuel_gauge.h:14-23) accurately describes the *replaced* behavior
in past tense, and the current `batt_present`-checkbox mechanism description
(fuel_gauge.h:25-32) matches `fuel_gauge_present()`'s actual one-line
implementation (`return s_ready && s_user_present;`, fuel_gauge.c:98).
Grepped the whole repo for `fuel_gauge_ready` / `fuel_gauge_chip_detected` /
`s_batt_present` (the pre-`86a0273` names/state per the git history) — the
only hits are in prior review documents and the original design-plan doc
under `docs/superpowers/plans/`, never in `main/fuel_gauge.c` or
`main/fuel_gauge.h` themselves. **No stale references found in the
reviewed files.**

## Overall verdict

The register-level protocol work (byte order, scaling constants, signed
CRATE reinterpretation, VRESET/ID byte-split) is correct and matches the
MAX17048/49 datasheet, and the init-ordering and `fuel_gauge_read()`
atomicity bugs from the prior review round are genuinely fixed and hold up
under re-inspection. Two **Important** findings remain: (3) the
all-or-nothing buffering fix applied to `fuel_gauge_read()` was not also
applied to its sibling `fuel_gauge_read_diag()`, whose one call site ignores
the return code — a live (if diagnostic-only) partial-write gap; and (4)
`fuel_gauge_vbus_present()` is architecturally coupled to MAX17048 I2C-probe
success even though the VBUS-detect GPIO is independent hardware, which
makes the header's "always true/false" doc-comment provably false in a
reachable (if rare) chip-failure mode. One **Minor** doc-precision nit
on `fuel_gauge_init()`'s idempotency wording. No Critical bugs, no
uninitialized-read path that reaches a real caller, and the stub branch is
a fully safe no-op matching every call site's expectations. Net: solid
after two rounds of fixes, with one real (if low-severity) regression-class
gap (finding 3) that mirrors a bug already fixed in the neighboring
function — worth a follow-up patch, not a blocker.
