# Agent 4: git history / blame context

**Status: done**

## Findings

> DEFERRED 2026-07-04: left as-is by explicit user choice — genuinely benign
> (single-byte bool, self-corrects next cycle), matches existing codebase
> convention for every other sensor driver's shared static.

**1. `main/transmission.c` — snprintf/torn-read history (Minor, informational)**

Evidence: `f26f85e` "torn-read fix" and multiple `snprintf`-truncation fixes (`d3a733d`, `9059fce`, `7b2e6a1`) show this codebase has repeatedly been bitten by (a) manual `n += snprintf(...)` accumulation and (b) cross-task shared mutable state read without synchronization.

The new log line in `c683204`/`e125c26` (`main/transmission.c` ~line 1423) uses a single `ESP_LOGI(TAG, "...", ...)` call with all values passed as format args — it does **not** repeat the snprintf-accumulation pattern. Good.

However, `fuel_gauge_present()` (`main/fuel_gauge.c:86-101`) maintains hysteresis state in a plain `static bool s_batt_present`, mutated by a read-modify-write with no lock, and is called concurrently from the httpd task (`format_battery`), the TX/cycle task (`transmission.c` originally, now `mqtt.c`), and the MQTT publish path. This is the same "multiple task owners of one shared resource, no synchronization" shape that produced the `f26f85e` torn-read bug — here the risk is much lower (single-byte bool, not an int64_t), so it's Minor, but it is the same anti-pattern recurring in a new module. (Cross-ref: agent 6's concurrency review reached the same conclusion independently — benign last-writer-wins, not a torn read.)

**2. `main/hal.h` GPIO34 — NOT a repeat of a past mistake**

Evidence: `86b5253` (V2.3.27) and `a854eb8` (V2.2.1) both carry a "RESERVED pins — never repurpose" comment block explicitly listing `IO34 VBUS-present detect` as reserved for exactly this purpose, years before this feature was built. `e125c26`'s `#define PIN_VBUS_DETECT 34` (`main/hal.h`) fulfills that reservation precisely — no conflict, correctly following prior documentation. Good adherence.

> RESOLVED 2026-07-04 (commit `30d3b1d`): worst-case JSON re-derived
> field-by-field — ~1466 B against `buf[1792]`, ~326 B (18%) slack.
> Sizing comment updated with the V2.6.6 entry.

**3. `main/mqtt.c` buffer sizing — Minor process gap, not a live bug**

Evidence: `9059fce` and `8201c5d` established a discipline of recalculating worst-case JSON size and bumping `char buf[N]` (with an updated inline comment) every time fields were added to `mqtt_publish_state`'s rich-state buffer; `9059fce`'s own message states truncation is safe (`APPEND` truncates, no overflow) but silently drops the HA publish.

`c683204` adds 3 new `APPEND(",\"batt_v\"...)` lines to `main/mqtt.c` but does not touch `buf[1792]` or its sizing comment (last touched `15857d0`, V2.5.6). The new fields cost roughly 50-70 bytes worst case, almost certainly within existing slack, so this is not an active overflow — but the audit step itself was skipped, breaking the established discipline. Minor.

**4. `main/version.h` bump discipline — no inconsistency**

`c683204` bumps `VERSION_STR` "V2.6.5"→"V2.6.6" in the same commit that claims the V2.6.6 label — consistent with the project norm (e.g. `d4186e4`, `f112fa4` bump in the feature commit itself). `e125c26`, a same-day follow-up fix before any tag, does not bump further, matching precedent (`3f0e7ff` review-fixup after `d4186e4` also didn't re-bump). No inconsistency found.

> DEFERRED 2026-07-04: left as-is by explicit user choice this session
> (again, after re-review) — /status, mqtt.c, mqtt_discovery.c still gate
> on `fuel_gauge_present()`. Tracked, not overlooked.

**5. Spec vs. shipped code — Important, self-documented but unresolved**

Evidence: `58ba327`'s spec (`docs/superpowers/specs/2026-07-03-fuel-gauge-max17048-design.md` §2) grounds the entire 2000/1500mV threshold on the empirical claim "VCELL/SoC read ~0V/0% with nothing attached," and §9 explicitly scopes the *only* known ambiguity as "protection IC tripped," not VBUS-charger float.

`e125c26`'s own commit message reports this justification is empirically false in the common case (USB power, no LiPo → VCELL floats 4.2-4.4V), but the fix is applied **only** to the per-cycle log line (switched to `fuel_gauge_ready()` + VBUS context). Per the commit message itself: "`fuel_gauge_present()`'s threshold is unreliable whenever VBUS is present and is **left as-is for /status, MQTT, and HA discovery**." `main/http_server.c` (`format_battery`), `main/mqtt.c`, and `main/mqtt_discovery.c` (3 entities) all still gate on `fuel_gauge_present()`. This means those three consumer paths will keep reporting a phantom battery with fabricated SoC/rate on any USB-powered, no-LiPo FeatherS3-D — a known false-positive, documented but not fixed everywhere it appears.

**Note (added when persisting, not by the review agent):** this was a deliberate, explicit maintainer decision this session — the user asked only for the log line to be corrected ("we can just leave it" regarding the other three surfaces) — not an oversight. Recorded here as-is because the review agent's job was to report what it found in the code/history, not to know the conversation that scoped the fix.

## Overall verdict

This change shows good awareness of past mistakes where it counted most: it avoided the classic snprintf-accumulation bug, it reused the already-reserved GPIO34 pin correctly instead of picking a fresh one, and it followed the truncation-safe `append_safe`/`APPEND` pattern. It falls short in two smaller ways — the mqtt.c buffer-sizing audit discipline was skipped (Minor), and a shared-mutable-bool race echoes the torn-read anti-pattern at low severity (Minor) — plus one real, currently-shipped issue: the empirically-falsified presence threshold was patched in only 1 of 4 places that use it, leaving `/status`, MQTT, and HA discovery to show phantom battery data on common USB-powered hardware (Important, but scoped-out by explicit user instruction rather than missed).
