# Agent 5: git history / blame context

**Status: done**

Review range: `58ba327..86a0273` (base `f26f85e`, V2.6.5).

## Findings

**1. `main/i2c_bus.h:106` — new I²C helper comment cites the wrong version (Minor)**

Evidence: the consolidation commit `959a39c` is titled "V2.6.6" and `main/version.h`
still reads V2.6.6 for the whole range up to `86a0273`. But the new comment block
above the shared per-device helpers says:

> `// here (V2.6.7) so the wire-level details — byte order, 100 ms transaction`

There is no V2.6.7 anywhere in this range's `version.h` history. Cosmetic, but it's
exactly the kind of stale-version-in-a-comment slip this project's own history has
flagged before as worth catching (`5c3f2ab`'s "stale @file board enumeration" /
"stale doc comment" fixes in the interleaved Heltec V4-R2 plan are the same class
of error, just in a different file).

**2. Sub-tick `vTaskDelay` regression check on the consolidated drivers — audited, no reincarnation found (informational, confirms good practice)**

Evidence: `928decd` (V2.3.31, "fix sub-tick vTaskDelay timing in I2C drivers —
SHT45 H=0% root cause") is a real, previously-shipped, hardware-reproduced bug:
`vTaskDelay(pdMS_TO_TICKS(N))` for N ≤ 10 at `CONFIG_FREERTOS_HZ=100` (10 ms tick)
can complete in 0 ms and race the chip's actual conversion time. The fix replaced
sub-20 ms timing-critical waits in `bmp581.c`, `sht45.c`, `sps30.c`, `veml7700.c`,
and one `bme280.c` bump (55→70 ms, kept as `vTaskDelay`) with `esp_rom_delay_us()`
busy-waits, with an explicit per-file rationale for what was and wasn't touched.

This session's `959a39c` ("consolidate I2C driver boilerplate... no functional
change") touches exactly these five files plus `bme688.c`/`bmp390.c`/`dnms.c`/
`fuel_gauge.c`. Diffed `959a39c` against each: every `esp_rom_delay_us(...)` call
and its accompanying V2.3.31 comment survived byte-for-byte; the refactor only
replaced `write_reg`/`read_regs`/CRC bodies with calls to the new
`i2c_dev_*`/`sensirion_crc8()` helpers, never touched a delay call. The "no
functional change" claim holds specifically for the one class of prior bug this
codebase has been bitten by twice (SHT45 H=0%, `bmp581.c`'s FORCED_WAIT_MS). Good
— this is exactly the kind of check a driver refactor in this repo needs to pass,
and it does.

**3. `sht45.c`/`sps30.c` CRC consolidation — algebraically equivalent to the removed local implementations (informational)**

Evidence: `sht45.c`'s old `crc_ok(a, b, crc)` and `sps30.c`/`dnms.c`'s old
`crc8(p, n)` both computed the identical polynomial (0x31, init 0xFF, MSB-first,
no reflection/XOR-out) over the same byte pairs. `959a39c` replaces all three
with the new `sensirion_crc.h:sensirion_crc8()`, called with the same byte
ranges (`&buf[0], 2` / `&buf[3], 2` etc.) at every call site. No CRC input
range or byte order changed. This project has had CRC-adjacent I2C bugs before
in spirit (V2.3.31's SHT45 issue was a CRC-passing stale-read, not a wrong-CRC
bug, but it's the same fragile-I2C-timing family) — the consolidation itself
introduces no new CRC risk.

**4. `i2c_bus.h:104-105` "prior byte-order mix-up" comment — no such historical incident exists in git history (Minor, phrasing risk only)**

Evidence: the new comment reads: *"A prior byte-order mix-up between VEML7700
(LE) and MAX17048 (BE) is the kind of bug this consolidation is meant to make
harder to reintroduce."* Searched `git log --all -i --grep="byte.order"` and
`git log --all -i --grep="VEML" -- main/fuel_gauge.c main/veml7700.c`: no commit
in this repo's history ever fixed a real LE/BE mix-up between these two drivers
— `fuel_gauge.c` is brand new this session (`c683204`) and its BE choice was
correct from the first commit (it already carried the "get this backwards and
every reading is off by a factor of 256" warning verbatim in `c683204`, before
`959a39c` existed). The comment's wording ("the kind of bug") is technically
hedged as hypothetical, not a factual claim of a past incident, so this is not
a fabricated citation — but it reads easily as "this happened before," and a
future maintainer skimming git blame for that incident will not find one.
Recommend clarifying to "a byte-order mix-up... would be" if revisited.

**5. `main/fuel_gauge.h:14-16` "CHANGELOG V2.4.28" citation — verified accurate, and reveals this is a *repeated* assumption, not just an accurately-cited one (Important)**

Evidence: `CHANGELOG.md:808-813` (V2.4.28 entry, "Why this and not VBUS /
fuel-gauge measurement") states verbatim: *"MAX17048 fuel gauge — measures
battery cell voltage. Without a LiPo attached (current Oatlands deployment),
both VCELL and SoC report ~0 / 0 %. No signal, just clutter on HA. Dropped."*
This is exactly what `fuel_gauge.h`'s new V2.6.6 CORRECTION comment cites it as.
The citation is accurate.

But the deeper finding: this V2.4.28 entry's ~0V assumption was never verified
against real hardware in 2026 either — it's the same reasoning (plausible
inference about charger-IC behavior, not a bench measurement) that this
session's own design spec (`58ba327`, §2) re-used to ground the 2000/1500 mV
thresholds, and that `e125c26`'s bench test (2026-07-03) then found to be
empirically false (VCELL floats 4.2-4.4V on USB power with no LiPo). In other
words: the *same unverified assumption* caused the feature to be shelved once
(V2.4.28) and then, ~2.5 months later, caused a design spec to ship with a
threshold that had to be corrected post-bench-test. The current code (`86a0273`)
now correctly abandons voltage-based detection entirely, so the assumption
class can't cause a third recurrence here — but it's a documented instance of
"same unverified premise, two separate features, both had to learn it the hard
way" and is worth flagging as a project-level pattern for future sensor/threshold
work: bench-verify assumed rail behavior before finalizing a spec, not after.

**6. `main/fuel_gauge.c`/`.h` full lifecycle (`git log --all`) — clean linear history, no abandoned attempts or reverts (informational)**

Evidence: `git log --oneline --all -- main/fuel_gauge.c main/fuel_gauge.h` shows
exactly six commits, all within this review range, all on the current branch,
in a coherent build-bench-correct-redesign arc: `c683204` (add) → `e125c26`
(bench-driven log fix) → `efcdfc5` (diagnostic registers) → `30d3b1d` (MAX
review follow-up, explicitly deferring 4 items) → `959a39c` (I2C consolidation)
→ `86a0273` (redesign: VCELL heuristic replaced by `/config` checkbox). No
orphaned branch tips, no force-push artifacts, nothing pre-dating `58ba327`.
This is a file with a fraught *design* history (wrong assumption, corrected
twice) but a clean *git* history — nothing hidden or lost.

**7. Interleaved Heltec V4-R2 commits — zero file overlap with V2.6.6 work, confirmed (informational, resolves potential rebase-hazard concern)**

Evidence: `git show --stat` on all four interleaved commits (`4905165`,
`0deb5b2`, `566aa8e`, `5c3f2ab`) shows every changed path is under `docs/`
(`docs/superpowers/specs/...board-port-design.md` and
`docs/superpowers/plans/...board-port.md`). None touch any of the 20 V2.6.6
source/header files in scope. No merge/rebase hazard.

**8. `30d3b1d`'s deferred-items list vs. what's actually still open — 3 of 4 resolved by `86a0273`, one demoted not because it was wrong but because the underlying design changed (Important → resolved, not a review-accuracy problem)**

Evidence: `30d3b1d`'s commit message explicitly deferred four items: (a) the
"phantom-battery-on-3-surfaces gating decision," (b) the "ready()/present()
naming ambiguity," (c) "two stale VCELL-threshold doc-comments," and (d) the
"`s_batt_present` last-writer-wins race." Checked each against the current
tree (post-`86a0273`):
  - (a) **Resolved.** `86a0273` replaces `fuel_gauge_present()`'s VCELL
    heuristic with the `batt_present` config bool; `http_server.c`, `mqtt.c`,
    and `mqtt_discovery.c` (via the unchanged `fuel_gauge_present_()` wrapper
    at `mqtt_discovery.c:59`, which inherits the new semantics without needing
    its own edit) all now gate on the checkbox, not a floating-voltage guess.
  - (b) **Resolved.** `fuel_gauge_ready()` no longer exists in `fuel_gauge.c`/
    `.h` — `86a0273`'s message cites this as "MAX-review Agent 10 naming
    finding."
  - (c) **Resolved.** `fuel_gauge.h`'s VCELL-threshold comments now read
    "V2.6.6 CORRECTION" / "V2.6.6 FIX" and describe the checkbox-based design
    consistently; no stale ">2000 mV" language remains outside the historical
    explanation.
  - (d) **Same pattern, not eliminated — carried forward under a new name.**
    `s_batt_present` (the old hysteresis bool) is gone, but `fuel_gauge.c:31`
    now has `static bool s_user_present`, written by `fuel_gauge_set_user_present()`
    from the httpd config-save path and from boot init, and read from
    `fuel_gauge_present()` by the TX/MQTT/status paths — the identical
    "single-writer-owns-a-shared-static-bool-read-by-multiple-tasks" shape,
    just now holding a user's config choice instead of a voltage-derived
    guess. This was correctly identified as benign in the prior review
    (07-03 agent 4/6: last-writer-wins, self-corrects, matches codebase
    convention) and nothing about the redesign makes it worse — flagging only
    so it isn't mistaken for "fixed" just because the variable was renamed.

## Overall verdict

This changeset shows a codebase that is actively learning from its own history
in the place that matters most for firmware: I2C timing. The `959a39c`
boilerplate consolidation touches every sensor driver that was previously
bitten by the sub-tick `vTaskDelay` bug (V2.3.31) and preserves every
`esp_rom_delay_us()` busy-wait untouched — the refactor stayed strictly within
register-plumbing and CRC bodies and never came near a timing-critical wait.
The CRC consolidation is a faithful, verified no-op. The four interleaved
Heltec V4-R2 doc commits carry zero risk of cross-contamination with this
range. The fuel-gauge feature's own arc (`c683204` → `86a0273`) is a clean,
undramatic git history despite a rocky design history — and that design
history is worth naming explicitly: the same "assume VCELL reads ~0V with no
battery" premise sank the feature once in V2.4.28 and had to be
bench-corrected again in V2.6.6 before the final commit abandoned voltage
heuristics altogether. Two Minor documentation nits (a stray "V2.6.7" version
reference, and a "prior byte-order mix-up" comment that reads as historical
but isn't) round out the findings. Nothing here blocks the changeset; the one
Important item is a process lesson (bench-verify rail-behavior assumptions
before they reach a design spec) rather than a defect in the code as shipped.
