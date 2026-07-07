# Agent 8: comment accuracy (independent pass + carried-forward verification)

**Status: done**

Scope: every new/changed comment and doc-string in `git diff f26f85e..86a0273 -- main/ CHANGELOG.md`, checked against current code behavior in the final commit (`86a0273`).

## Carried-forward items — independently verified

**1. `main/hal.h:38` and `main/mqtt_discovery.c:167` — CONFIRMED (Important each)**
Both still say battery presence/rows appear "once a battery is auto-detected" / "Presence auto-detected." `git show 86a0273 -- main/hal.h main/mqtt_discovery.c` is empty for both files — they were never touched by the final commit that ripped out VCELL auto-detect in favor of the `batt_present` checkbox, so they narrate the discarded `c683204` design. Every other occurrence of "auto-detect" language in this diff (`fuel_gauge.h:28`, `http_server.c:710`, `mqtt.c:539-541`, `main.c:1004`, `config_fields.def:236`) is correctly phrased as "can't be auto-detected" — these two are the only stale survivors.

**2. `main/i2c_bus.h:106` "V2.6.7" — CONFIRMED (Minor)**
`#define VERSION_STR "V2.6.6"` (`main/version.h:20`); `git log -p f26f85e..86a0273 -- main/version.h` shows exactly one bump, V2.6.5→V2.6.6, in `c683204`, with no further version churn anywhere in the range. The "(V2.6.7)" tag on the per-device-helpers comment block is simply wrong — every other new-code version tag in this diff correctly says V2.6.6.

**3. `CHANGELOG.md:59` `reg_read8()` — CONFIRMED (Important)**
`grep -rn "reg_read8" main/ CHANGELOG.md` finds it only in the CHANGELOG. `reg_read8()` was added in `efcdfc5` and deleted in `959a39c` (folded into `i2c_dev_read_regs()`), two commits before `86a0273` hand-rewrote this section. `fuel_gauge_read_diag()` mentioned in the same sentence is fine — only the helper name is stale.

**4. `fuel_gauge_read_diag()` doc gap — CONFIRMED (Minor)**
`fuel_gauge_read_diag()` (`main/fuel_gauge.c:143-157`) writes each output pointer directly and returns early on the first failed read — unlike `fuel_gauge_read()` (`fuel_gauge.c:105-141`), which explicitly reads into locals first and only commits to the output pointers after every read succeeds (and says so in a comment, `fuel_gauge.c:108-111`). If `version` succeeds and `status` then fails, `*version` is left populated while the function returns non-`ESP_OK` — a partial-write hazard `fuel_gauge_read_diag()`'s doc-comment (`fuel_gauge.h:90-100`) never mentions, unlike its sibling. Currently harmless in practice: the sole caller (`transmission.c:1585`) ignores the return value and always has sane 0xFFFF/0xFF sentinel defaults — but the doc's silence on this asymmetry is a real completeness gap, not just a style nit.

**5. `fuel_gauge_vbus_present()` "Always true/false" claim — CONFIRMED, agent 2's characterization is accurate (Important)**
Body (`fuel_gauge.c:92-95`):
```c
bool fuel_gauge_vbus_present(void) {
    if (!s_ready) return false;
    return gpio_get_level(PIN_VBUS_DETECT) != 0;
}
```
`s_ready` is only set `true` at the end of a successful `fuel_gauge_init()` (`fuel_gauge.c:87`) — if the MAX17048 probe fails (`i2c_probe_and_add()` returns `ESP_ERR_NOT_FOUND` or any other error, `fuel_gauge.c:42-49`), `s_ready` stays `false` forever and `fuel_gauge_vbus_present()` returns `false` unconditionally from then on, regardless of the pin's actual level. The doc-comment (`fuel_gauge.h:58-62`) — "Always true while running on USB power... reads false only when running purely off battery with USB unplugged" — presents the function as a direct, unconditional read of the VBUS pin and omits this failure mode entirely. A maintainer reading only the header would not know that "false" can mean either "USB unplugged" or "fuel-gauge chip never came up." Confirmed independently; agent 2's finding stands.

**6. `main/i2c_bus.h:107-108` "prior byte-order mix-up between VEML7700 (LE) and MAX17048 (BE)" — CONFIRMED FABRICATED, no such incident exists (Important)**
`fuel_gauge.c` (containing the MAX17048 driver) did not exist before this same V2.6.6 commit range — `git log --oneline f26f85e..86a0273 -- main/fuel_gauge.c` shows it was created fresh in `c683204`. Its very first version (`git show c683204 -- main/fuel_gauge.c`) already used big-endian reads correctly (`*out = ((uint16_t)in[0] << 8) | (uint16_t)in[1];`). `veml7700.c`'s little-endian reads are unchanged by this refactor (`git log` / diff shows only the wrapper-function replacement, not a byte-order fix). A full-text search of the entire git history (`git log --all -p -- main/fuel_gauge.c main/veml7700.c | grep -i "endian\|byte.order"`) turns up no commit that ever fixed a VEML7700/MAX17048 byte-order bug — the only hits are this same new comment text. The claim is invented narrative, not a real incident. Agent 5's finding is confirmed.

## Additional findings from this pass

**7. `main/fuel_gauge.h:14-37` file-header "V2.6.6 CORRECTION"/"V2.6.6 FIX" paragraphs — accurate, not stale (no finding)**
Cross-checked both paragraphs against the final commit's actual behavior: the CORRECTION paragraph's claim (original threshold assumed ~0V no-battery, bench testing found 4.2-4.4V instead) matches `CHANGELOG.md`'s V2.4.28/V2.6.6 narrative and the code history; the FIX paragraph's claim (`fuel_gauge_present()` now driven by `batt_present`, no digital "battery attached" signal exists on this board) matches `fuel_gauge.c:97-99` (`return s_ready && s_user_present;`) and `hal.h`'s `PIN_VBUS_DETECT` comment (a VBUS-presence pin, not a charger-status pin). This header was NOT left describing an intermediate design — it's the most up-to-date, most careful prose in the whole diff.

**8. `main/mqtt_discovery.c:166-167` "first entities...to use HA's standard 'battery' and 'voltage' device classes" — accurate (no finding)**
Checked the full `ENTITIES[]` table (`mqtt_discovery.c:117-172`): no earlier row uses `device_class` `"voltage"` or `"battery"` (existing classes used are `temperature`, `humidity`, `atmospheric_pressure`, `pm1/pm25/pm10`, `sound_pressure`, `illuminance`, `duration`). The claim is true; only the adjacent "Presence auto-detected" clause (item 1 above) is wrong.

**9. `main/transmission.c:1420-1436` fuel-gauge log-line comment block — accurate, no residue (no finding)**
"gated on `fuel_gauge_present()` — chip detected AND the user has ticked the 'Battery attached' checkbox" matches `fuel_gauge_present()`'s body (`s_ready && s_user_present`) exactly. `grep -rn "fuel_gauge_chip_detected"` across `main/` returns zero hits — the removed function name does not appear anywhere in this comment block or elsewhere in the diff. No stale residue from the mid-session `fuel_gauge_ready()` → `fuel_gauge_chip_detected()` → removal churn documented in the session's memory log.

**10. Refactored sensor drivers (`bmp581.c`, `bmp390.c`, `bme280.c`, `bme688.c`, `veml7700.c`, `sht45.c`, `sps30.c`, `dnms.c`) and new `i2c_bus.h`/`sensirion_crc.h` — no stale narration of the old per-driver implementation found (beyond item 6)**
All comments describing wire-level behavior (byte order, timeouts, chip-ID checks, calibration layout, CRC polynomial/init) are either unchanged carry-overs that still apply verbatim to the new shared helpers, or new comments (`i2c_bus.h`'s per-helper Doxygen blocks) that accurately describe the helper bodies immediately below them — all spot-checked against the actual `static inline` implementations in `i2c_bus.h:115-190` and match.

**11. `CHANGELOG.md` V2.6.6 section — remaining technical claims check out**
Register list ("VCELL, SOC, CRATE"), the "7 readable registers" diagnostic-logging claim (version/hibrt/config/valert/vreset/chip_id/status — matches the 7 reads in `fuel_gauge_init()`, `fuel_gauge.c:70-76`), the driver list for the I2C consolidation (`bmp581.c`/`bmp390.c`/`bme280.c`/`bme688.c`/`veml7700.c`/`fuel_gauge.c` fully switched; `sht45.c`/`sps30.c` adopted only add/teardown) — all verified against the diff. The only factual defect found in this section is the `reg_read8()` mention (item 3).

## Overall verdict

Comment accuracy across this changeset is mixed but narrow: the newest and most heavily-edited prose (`fuel_gauge.h`'s file header, most of `fuel_gauge.c`'s inline comments, the `http_server.c`/`mqtt.c`/`main.c`/`config_fields.def` checkbox comments) is accurate and correctly reflects the final design. The defects cluster in exactly the places you'd expect from a fast multi-commit iteration folded into one final state without a last-pass grep: two files the final commit never touched (`hal.h`, `mqtt_discovery.c`) still narrate the discarded VCELL-auto-detect design; a stray version tag drifted to V2.6.7; the hand-rewritten CHANGELOG carried forward a deleted helper's name; and — the most concerning finding of this pass — a newly-invented "prior byte-order mix-up" incident in `i2c_bus.h` has no basis in git history at all, which risks training a future reader to believe this codebase has a track record of a bug class it has never actually had. None of these are safety-critical; all are Important-or-below documentation-accuracy defects that a single follow-up commit could close.

**Finding count:** 6 Important (`hal.h:38`, `mqtt_discovery.c:167`, `CHANGELOG.md:59` reg_read8, `fuel_gauge_vbus_present()` init-failure omission, `i2c_bus.h:107-108` fabricated byte-order incident, plus the `hal.h`/`mqtt_discovery.c` pair counted individually = 2+1+1+1+1 = 6), 2 Minor (`i2c_bus.h:106` V2.6.7 stray tag, `fuel_gauge_read_diag()` partial-write doc gap). 0 Critical.
