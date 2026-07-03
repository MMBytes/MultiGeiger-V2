# Agent 7: comment/documentation accuracy

**Status: done**

## Findings

> DEFERRED 2026-07-04: not yet corrected, left as-is this session.

**1. `main/fuel_gauge.c:19-22` — Important (stale claim restated as fact)**
```
// Presence hysteresis (mV). See fuel_gauge.h file header for the full
// derivation — this dead zone sits between the empirical ~0V no-battery
// reading and the ~2.4V floor a real cell can reach before its protection
// IC disconnects it.
```
This still asserts "the empirical ~0V no-battery reading" as established fact. Pass 2's own bench testing (documented two lines away in `fuel_gauge.h:14-25`) disproved this for the common case (VCELL floats to 4.2-4.4V with USB power, no LiPo). The comment says "see file header for the full derivation" but doesn't flag that the header now contradicts the premise stated right here. A reader of just this `.c` file comment gets a stale, misleading picture of how solid the threshold's basis is.

> DEFERRED 2026-07-04: not yet corrected, left as-is this session.

**2. `main/fuel_gauge.h:72-79` — Important (`fuel_gauge_present()` doc-comment omits the VBUS caveat)**
The function-level doc-comment describes the hysteresis mechanism and performance characteristics but never mentions that the function is unreliable whenever VBUS is present — that caveat exists only in the file header (lines 14-25). A reader who only reads this one function's doc-comment (not the whole file header) should still come away with an accurate picture — this fails that test; read in isolation it reads as a fully trustworthy presence check.

> RESOLVED 2026-07-04 (commit `30d3b1d`): contradicting clause removed.

**3. `main/fuel_gauge.h:64-69` — Minor (confusing/garbled parenthetical)**
```
Always true while running on USB power (the ESP32-S3 can't
execute code on LiPo-only without it); reads false only when running
purely off battery with USB unplugged.
```
The parenthetical "the ESP32-S3 can't execute code on LiPo-only without it" is self-contradicting — the very next clause describes "running purely off battery" as a real, valid state, so the board plainly *can* execute code LiPo-only. The clause reads as a leftover/garbled edit rather than a coherent justification. The core claim (VBUS pin tracks actual VBUS presence) is a reasonable hardware-behavior assumption and is fine — just the wording of this one aside is broken.

**No issues found on:**
- V2.6.7 stragglers — zero matches, confirmed clean (the version-comment fix from this session was applied completely).
- `fuel_gauge.h` file-header block (lines 3-37) — accurately marks the Pass-1 assumption as superseded, cites CHANGELOG V2.4.28 correctly (verified against `CHANGELOG.md:769-775`, which does say "both VCELL and SoC report ~0/0%"), and correctly frames `fuel_gauge_present()` as unreliable/not-ground-truth.
- `fuel_gauge_ready()` doc-comment — accurate, matches `return s_ready;`.
- `hal.h` diff (HAL_HAS_FUEL_GAUGE doc block, PIN_VBUS_DETECT comment) — accurate and consistent with the corrected understanding.
- `transmission.c` log-block comment (lines 1420-1435) — correctly describes current `fuel_gauge_ready()` gating and explicitly, accurately contrasts it with the old `fuel_gauge_present()` gating and why it was replaced.
- No TODO/unfinished-work markers found anywhere in the diff.
- Callers in `http_server.c`/`mqtt.c`/`mqtt_discovery.c` have no attached comments to go stale (bare `if (fuel_gauge_present())` calls) — out of scope but checked for completeness.

## Overall verdict

The corrected documentation trail is largely trustworthy — the file-header narrative in `fuel_gauge.h` is thorough, honest about the Pass-1 mistake, and cites verifiable evidence. However, a future maintainer skimming only `fuel_gauge.c` (not the header) or only the `fuel_gauge_present()` function doc would encounter the disproven "~0V no-battery" claim presented as settled fact, with no local pointer to the correction — exactly the inconsistent trail the review was worried about. Recommend tightening those two spots before treating this as fully reconciled.
