# V2.6.6 local MAX code review — tracking

**Range:** `f26f85e..HEAD` (V2.6.5 → current, includes fuel-gauge feature commit
`c683204` and the log-line correction commit `e125c26`).

**Files touched:** `main/fuel_gauge.c`, `main/fuel_gauge.h`, `main/hal.h`,
`main/http_server.c`, `main/main.c`, `main/mqtt.c`, `main/mqtt_discovery.c`,
`main/transmission.c`, `main/version.h`, `main/CMakeLists.txt`, `CHANGELOG.md`.

Agents run **sequentially, one at a time**, each result written to its own
file in this directory immediately on return — so a credit cutoff mid-review
loses at most the one in-flight agent, not the whole batch. To resume: check
which `agent-NN-*.md` files exist below, re-run only the missing ones.

## Status

| # | Agent | Scope | Status |
|---|-------|-------|--------|
| 1 | claude-md-compliance | House-style / feedback-memory conventions | done — 0 findings |
| 2 | bug-scan-fuel-gauge-driver | `fuel_gauge.c`/`.h` deep bug scan | done — 0 Critical/Important, 2 Minor |
| 3 | bug-scan-integration-points | `http_server.c`/`mqtt.c`/`mqtt_discovery.c`/`transmission.c` | done — 0 findings |
| 4 | git-history-context | git blame/log on touched files, past-bug recurrence | done — 1 Important (known/scoped), 2 Minor |
| 5 | pin-hal-cross-check | `hal.h` pin additions vs. full 5-board pin map | done — 0 code issues, 1 Important (datasheet spot-check advised) |
| 6 | concurrency-thread-safety | Cross-task access to fuel-gauge static state | done — 0 Critical/Important, 2 Minor |
| 7 | comment-accuracy | New comments/doc-strings vs. actual code behaviour | done — 2 Important, 1 Minor |
| 8 | build-config-consistency | CMakeLists.txt/version.h/5-board stub paths | done — 0 findings, QTPy stub rebuild verified clean |
| 9 | security-review | Buffer/overflow/I2C-error-handling issues | done — 0 findings, no new attack surface or memory-safety risk |
| 10 | architecture-review | Design coherence, `fuel_gauge_present()` left-inconsistent risk | done — recommends (b) small follow-up: rename `ready()`, soften 3 surfaces' labels |

Update the Status column (pending → done / failed) as each agent returns.

## Resolution (2026-07-04, commit `30d3b1d`)

Findings triaged and actioned in a follow-up session. Per-finding disposition
below; see each `agent-NN-*.md` file for a `> RESOLVED` / `> DEFERRED` note
inline at the specific finding.

**Closed (code or verification):**
- Agent 2 / Agent 6, init-order hazard (`s_ready` before `gpio_set_direction`)
  — fixed: `s_ready = true` moved after the GPIO call.
- Agent 2, `fuel_gauge_read()` partial-write-before-error — fixed: reads now
  buffer into locals, output pointers written only on full success (matches
  `veml7700_read()`).
- Agent 2, `fuel_gauge_init()` "safe to call multiple times" doc gap — fixed:
  doc-comment now states a second call is a hard no-op.
- Agent 5, GPIO34/PSRAM datasheet spot-check — closed by verification (not a
  code change): confirmed against Unexpected Maker's own FeatherS3D
  datasheet — QSPI (not OPI) PSRAM, and GPIO34 is the vendor's own documented
  5V/USB-detect pin.
- Agent 4, mqtt.c buffer-sizing audit skipped — fixed: worst-case JSON
  re-derived field-by-field (~1466 B against `buf[1792]`, ~326 B/18% slack);
  sizing comment updated with the V2.6.6 entry.
- Agent 7, garbled/self-contradicting `fuel_gauge_vbus_present()` parenthetical
  — fixed: contradicting clause removed.

**Deferred (explicit scope decision this session, not overlooked):**
- Agent 4 / Agent 10, phantom battery data on `/status`, MQTT, HA discovery
  (3 of 4 consumer surfaces still gate on the VBUS-unreliable
  `fuel_gauge_present()`) — user chose to leave as-is for now.
- Agent 10, `fuel_gauge_ready()`/`fuel_gauge_present()` naming ambiguity —
  deferred; recommended fix is a rename (`chip_detected()`), not yet done.
- Agent 7, two stale doc-comments restating the disproven "~0V no-battery"
  premise (`fuel_gauge.c:19-22`, `fuel_gauge.h:72-79` `present()` doc) — not
  yet corrected.
- Agent 4 / Agent 6, `s_batt_present` last-writer-wins race — left as-is by
  explicit choice: genuinely benign (single-byte bool, self-corrects next
  cycle), matches this codebase's existing convention for every other
  sensor driver's shared static.
