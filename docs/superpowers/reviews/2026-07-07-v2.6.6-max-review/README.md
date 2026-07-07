# V2.6.6 local MAX code review (v2) — tracking

**Range:** `f26f85e..86a0273` (V2.6.5 → current HEAD). This supersedes the
2026-07-03 review (`docs/superpowers/reviews/2026-07-03-v2.6.6-max-review/`),
which only covered the fuel-gauge feature commits through `e125c26`. This
pass covers the *entire* V2.6.6 changeset end to end, including everything
already reviewed once before — nothing is assumed still-valid from the prior
pass; each agent re-checks its area against the current code.

Excluded: `4905165`, `0deb5b2`, `566aa8e`, `5c3f2ab` (Heltec V4-R2 board-port
design docs — unrelated plan-only commits interleaved in history, no `main/`
code).

**Commits in scope:**
```
58ba327 docs: add MAX17048 fuel-gauge design spec for FeatherS3-D
c683204 V2.6.6: MAX17048 battery fuel gauge (FeatherS3-D)
e125c26 V2.6.6: fuel gauge log line — honest labelling after bench false-positive
efcdfc5 V2.6.6: fuel gauge diagnostic register logging
30d3b1d V2.6.6: MAX review follow-up — init-order, read() atomicity, doc fixes, buffer audit
9ced6f3 docs: record V2.6.6 fuel-gauge MAX review + resolution status
959a39c V2.6.6: consolidate I2C driver boilerplate and Sensirion CRC-8 (no functional change)
86a0273 V2.6.6: replace unreliable VCELL battery auto-detect with a /config checkbox
```

**Files touched (22 + CHANGELOG.md):**
`main/fuel_gauge.c`, `main/fuel_gauge.h`, `main/hal.h`, `main/http_server.c`,
`main/main.c`, `main/mqtt.c`, `main/mqtt_discovery.c`, `main/transmission.c`,
`main/version.h`, `main/CMakeLists.txt`, `main/config_fields.def`,
`main/i2c_bus.h` (new), `main/sensirion_crc.h` (new), `main/bmp581.c`,
`main/bmp390.c`, `main/bme280.c`, `main/bme688.c`, `main/veml7700.c`,
`main/sht45.c`, `main/sps30.c`, `main/dnms.c`, `CHANGELOG.md`.

Two logically distinct changes are bundled in this one unshipped version:
1. **Fuel-gauge feature** (`c683204`→`86a0273`): MAX17048 driver, VBUS-detect
   pin, `/config` "Battery attached" checkbox (`batt_present`), diagnostic
   register logging, per-TX-cycle log line.
2. **I2C driver consolidation** (`959a39c`, explicitly "no functional
   change"): 8 sensor drivers refactored onto shared `i2c_bus.h` helpers
   (register r/w, probe/add/teardown) and shared `sensirion_crc.h` CRC-8.

Agents run **sequentially, one at a time**, each result written to its own
file in this directory immediately on return — so a credit cutoff mid-review
loses at most the one in-flight agent, not the whole batch. To resume: check
which `agent-NN-*.md` files exist below, re-run only the missing ones.

## Status

| # | Agent | Scope | Status |
|---|-------|-------|--------|
| 1 | claude-md-compliance | House-style / feedback-memory conventions, all touched files | done — 2 Important, 2 Minor |
| 2 | bug-scan-fuel-gauge-driver | `fuel_gauge.c`/`.h` deep bug scan | done — 2 Important, 1 Minor |
| 3 | bug-scan-i2c-consolidation | `i2c_bus.h`, `sensirion_crc.h`, and the 8 refactored drivers — verify truly "no functional change" | done — "no functional change" holds; 0 Critical/Important, 2 Minor |
| 4 | bug-scan-integration-points | `http_server.c`/`mqtt.c`/`mqtt_discovery.c`/`transmission.c`/`main.c`/`config_fields.def` | done — 0 findings; confirms carried-forward "3-of-4 VBUS" item now moot |
| 5 | git-history-context | git blame/log on all touched files, past-bug recurrence | done — 2 Important (process lesson; s_user_present race not actually fixed), 2 Minor |
| 6 | pin-hal-cross-check | `hal.h` pin additions vs. full 5-board pin map | done — 0 findings; 1 info (GPIO34/QSPI needs external verification, previously closed 2026-07-04) |
| 7 | concurrency-thread-safety | Cross-task access to fuel-gauge statics + any shared i2c_bus state | done — still benign, re-derived from scratch; 0 Critical/Important, 2 Minor (volatile hygiene) |
| 8 | comment-accuracy | All new/changed comments vs. actual code behaviour | done — 0 Critical, 6 Important, 2 Minor; confirms all 3 carried-forward items independently |
| 9 | build-config-consistency | CMakeLists.txt/version.h/5-board stub + refactored-driver build paths | done — 0 findings; real feathers3_d + heltec_v2 builds both clean, 0 warnings |
| 10 | security-review | Buffer/overflow/I2C-error-handling across both feature and refactor | done — 0 findings; no new attack surface vs. pre-V2.6.6 baseline |
| 11 | architecture-review | Design coherence; re-verify prior-review's deferred items still hold | done — ready to tag after 1 cleanup commit; 8 file:line items named, all mechanical |

Update the Status column (pending → done / failed) as each agent returns.

## Carried-forward findings to explicitly re-verify (not skip)

From `2026-07-03-v2.6.6-max-review/README.md`'s "Deferred" section — each of
these must get an explicit current-state verdict in this pass, not silence:

1. Phantom battery data on 3 of 4 consumer surfaces (`/status`, MQTT, HA
   discovery) still gated on `fuel_gauge_present()` alone, no VBUS
   cross-check — user chose to leave as-is on 2026-07-04. Re-check: is this
   still true, and does the new `batt_present` checkbox change the risk
   calculus (it should — `fuel_gauge_present()` is no longer VBUS-unreliable,
   it now requires explicit user confirmation)?
2. `fuel_gauge_ready()`/`fuel_gauge_present()` naming ambiguity — was
   deferred 2026-07-04, then addressed this session by renaming to
   `fuel_gauge_chip_detected()` and subsequently **removing it entirely**
   once it became an orphaned function after the TX-log-line gating change.
   Re-check: confirm no remaining callers/references anywhere, confirm the
   rename-then-removal didn't leave any stale comments referring to the old
   names.
3. Two stale doc-comments restating the disproven "~0V no-battery" premise —
   fixed 2026-07-04 (`30d3b1d`). Re-check: confirm no other stale VCELL-
   threshold-heuristic references survived into the current `fuel_gauge.h`/
   `.c` header comments after this session's checkbox rewrite.
4. `s_batt_present`/`s_user_present` last-writer-wins race between the boot
   task and the HTTP-server task — left as-is by explicit choice, judged
   benign. Re-check: does this verdict still hold given `s_user_present` is
   now the sole gate for `fuel_gauge_present()` (previously it only gated a
   secondary confirmation alongside VCELL)?

## Resolution (2026-07-07)

All 11 agents ran clean: **0 Critical findings anywhere.** Importants cluster
entirely in comment/doc accuracy and one diagnostic-only code path; none
reach the counting, HV, or NVS-persistence paths. Real builds (not just
static checks) of `feathers3_d` (real driver) and `heltec_v2` (stub path)
both came back clean, 0 warnings, and a full-tree grep confirmed
`fuel_gauge_chip_detected()` has zero remaining references anywhere.

**Carried-forward items — final verdicts:**
1. Phantom battery data (3-of-4 surfaces on VBUS-unreliable data) — **moot**,
   confirmed by agent 4 and agent 11: `fuel_gauge_present()` no longer means
   "voltage guessed", it means "user confirmed", so the original risk no
   longer exists. Residual (accepted) tradeoff: nothing catches a user who
   ticks the box once then later removes the battery without unticking it —
   same class of trust as `use_external_antenna`, no digital signal exists
   to do better.
2. `fuel_gauge_ready()`/`fuel_gauge_chip_detected()` naming ambiguity —
   **fully resolved**: function renamed then removed entirely this session;
   agent 9's dual-board build + grep sweep is conclusive.
3. Stale "~0V no-battery" doc-comments — **fully resolved** (fixed
   2026-07-04, `30d3b1d`); but agents 1/5/8 independently found 6 *new*
   Important comment-accuracy issues from this session's own edits (see
   below).
4. `s_batt_present`/`s_user_present` race — **still benign**, re-derived
   from scratch by agent 7 (not just carried forward): the flag only ever
   gates a read-only display decision, never NVS/GPIO/counting, so the
   "sole gate now" stakes-change doesn't actually change the blast radius.
   `volatile bool` recommended as hygiene.

**New findings this pass, to fix in one small cleanup commit** (per agent
11's release-readiness verdict — all mechanical, no design changes, ready to
tag once done):
- `main/hal.h:38` and `main/mqtt_discovery.c:167` — stale comments still
  describing the discarded VCELL-auto-detect design instead of the checkbox.
- `main/i2c_bus.h:106` — stray "V2.6.7" version tag (should read V2.6.6).
- `main/i2c_bus.h:107-108` — a comment describing "a prior byte-order
  mix-up between VEML7700/MAX17048" that never happened (fuel_gauge.c used
  correct big-endian reads from its first version) — **fabricates history**,
  fix with priority since it actively misinforms future debugging.
- `CHANGELOG.md:59-60` — credits a `reg_read8()` helper that commit
  `959a39c` deleted.
- `main/fuel_gauge.h:58-63` — `fuel_gauge_vbus_present()`'s doc-comment
  claims "Always true/false" but it silently returns `false` forever if the
  MAX17048 I2C probe ever failed at init; doc doesn't mention this.
- `main/fuel_gauge.c:143-157` — `fuel_gauge_read_diag()` lacks the
  all-or-nothing/no-partial-write contract its sibling `fuel_gauge_read()`
  has (and documents); currently harmless since the sole caller ignores the
  return code, but the gap should either be fixed or documented.
- `main/fuel_gauge.c:30-31` — `s_ready`/`s_user_present` recommended
  `volatile` for cross-task read/write hygiene (not required for
  correctness per agent 7's analysis, but cheap and this version isn't
  shipped yet).

Per-agent files (`agent-01` through `agent-11`) have full detail and
evidence for every item above.

