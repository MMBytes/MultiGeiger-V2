# Agent 3: integration points bug scan

**Status: done**

## Review Report

**Files reviewed at HEAD:** `main/http_server.c` (format_battery, lines 708–731; call site 1184), `main/mqtt.c` (lines 534–545), `main/mqtt_discovery.c` (lines 37, 59, 165–172, 274–311), `main/transmission.c` (lines 1423–1442), `main/main.c` (lines 999–1002, boot sequence to 1178+).

**1. Buffer safety (`http_server.c`)** — No issues. `format_battery()` is a byte-for-byte structural match to `format_als()`: uses `append_safe()` throughout, threads `n` correctly across calls, never bare-accumulates with raw `snprintf`. `append_safe()` itself (line 875) clamps `n` and guards against negative `vsnprintf` returns.

**2. MQTT JSON correctness (`mqtt.c`)** — No issues. The `APPEND` macro (line 453) is a safe accumulator (checks `rem`, clamps on truncation). Every appended field is comma-prefixed (`,"batt_v":...`), matching the pattern of every other conditional block in the function (env, pm, noise, lux). No TOCTOU risk: `fuel_gauge_present()` gates once, then a single `fuel_gauge_read()` call populates all three floats atomically from one driver call — the three `APPEND`s use already-captured local variables, not repeated reads, so the three fields can't diverge mid-message.

**3. HA discovery entities (`mqtt_discovery.c`)** — No issues. All three rows are well-formed and consistent with neighboring entities: `batt_v`→`voltage`/`V`, `batt_soc`→`battery`/`%` (both standard HA device classes, correctly paired with their units), `batt_rate`→ no device class (correct, HA has no rate class) with a sensible icon, matching the `pm4`/`nc*` pattern of using `NULL` device_class + icon for non-standard measurements. `json_field` values (`batt_v`, `batt_soc`, `batt_rate`) exactly match the keys emitted in `mqtt.c`, so `value_json.<field>` templates resolve correctly. `object_id`s (`battery_voltage`, `battery`, `battery_rate`) don't collide with any other entity in the table.

**4. Log line format-string safety (`transmission.c:1438–1440`)** — No issues. All three floats are explicitly cast to `(double)` before the `%f`-family specifiers, matching the codebase-wide convention used everywhere else in this file and in `format_als()`/`format_battery()` in `http_server.c`. `fuel_gauge_vbus_present()` returns `bool`, formatted via `?:` into a `const char*` for `%s` — correct.

**5. Call-site / boot-order correctness (`main.c`)** — No issues. `fuel_gauge_init(bus1)` runs at line 1002, immediately after `i2c_bus_get_primary()`, well before `esp_netif_init()`/WiFi bring-up (line 1051+). `http_server_start()` (line 1153) and `mqtt_init()` (lines 1178, 1320) both fire only after WiFi/netif setup, so there is no reachable path where `/status` or an MQTT publish executes before `fuel_gauge_init()` has run. No startup race.

**6. Cross-surface consistency** — `/status`, `mqtt.c`, and `mqtt_discovery.c` each call `fuel_gauge_present()` independently (no shared cache passed between them), but this is not a real risk: `fuel_gauge_present()` reads a single module-static `s_batt_present` maintained by the driver's own hysteresis state machine (per the header's documented design), not a live VCELL sample per call. Three near-simultaneous calls within one TX cycle all read the same static boolean, so they cannot diverge from each other — the hysteresis band is exactly what prevents the boundary-flicker scenario asked about.

## Overall verdict

This integration is clean. All five wiring points follow established codebase conventions exactly (safe-append helpers, double-casts for `%f`, comma-prefixed conditional JSON fields, boot-order sequencing) with no copy-paste residue, no buffer or format-string bugs, and no race conditions. The one deliberate inconsistency (three surfaces on `fuel_gauge_present()`, one surface — the log line — on `fuel_gauge_ready()`+`fuel_gauge_vbus_present()`) is well-documented in-code and does not introduce any secondary correctness problem beyond the already-accepted "wrong number" tradeoff.
