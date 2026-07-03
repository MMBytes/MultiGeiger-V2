# Agent 9: security review

**Status: done**

## Security Review — MAX17048 Fuel Gauge (V2.6.6, f26f85e..HEAD)

**Scope covered:** `main/fuel_gauge.c`/`.h` (new), `main/http_server.c` (`format_battery()` + call site), `main/mqtt.c` (new `APPEND` block), `main/mqtt_discovery.c` (3 new HA entities), `main/transmission.c` (new log line), `main/hal.h` (`PIN_VBUS_DETECT`), `main/main.c` (init call).

**1. Buffer overflow / OOB write — none found.**
`reg_read16()` (`fuel_gauge.c:35-41`) always reads exactly `sizeof(in)` = 2 bytes into a fixed `uint8_t in[2]`; the length is a compile-time constant, never attacker/sensor-influenced. `format_battery()` (`http_server.c:713-731`) uses `append_safe()`, the same clamped-accumulation helper already used by `format_als()` (bounds-checked, `__attribute__((format(printf,4,5)))`, `n` never pushed past `sz`). The shared scratch buffer is `static char buf[1600]` (`http_server.c:1148`); the battery block content is ~200 bytes max, well within margin. `mqtt.c`'s new lines use the existing `APPEND` macro (`mqtt.c:453-460`), which correctly clamps `n` to `rem-1` on truncation (the safe pattern, not the bare `n += snprintf` anti-pattern this project has previously flagged). All three new `APPEND` calls follow this macro — no new call bypasses it.

**2. Format-string injection — none found.**
Every new `ESP_LOGI`/`ESP_LOGW`/`snprintf`/`append_safe`/`APPEND` call in the diff uses a literal format string. Dynamic values (`esp_err_to_name(err)`, `"charging"/"discharging"/"idle"`, `"present"/"absent"`) are always passed as `%s` *arguments*, never as the format string itself. No I2C-sourced data is ever used as a format string.

**3. MQTT/HA JSON injection — none found.**
`mqtt.c:534-540` formats `batt_v`/`batt_soc`/`batt_rate` with `%.3f`/`%.1f`/`%.2f` — pure numeric floats, no `%s`, no string interpolation of any kind. Same for the HA discovery rows (`mqtt_discovery.c:165-167`), whose `json_field`/`device_class`/`unit`/`icon` columns are all compile-time literals. Structurally no injection vector exists here, as expected.

**4. Integer/float edge cases — none with memory-safety consequences.**
The one integer-overflow-prone calculation, `mv = (uint32_t)(((uint64_t)raw * 78125u) / 1000000u)` (`fuel_gauge.c:94`), correctly widens to `uint64_t` before the multiply (`raw` max 65535 × 78125 ≈ 5.12B, overflows `uint32_t` but not `uint64_t`) — this is a correctness fix already present, not a new bug. No battery-derived value (voltage, SoC, rate, or the raw `mv`) is ever used as an array index, buffer-size, or loop bound anywhere in the diff — all are purely display/publish values. A malfunctioning or bus-glitched MAX17048 can at worst cause wrong/garbage numbers or a spurious "battery present" state (this is the known, already-documented VBUS-float false-positive) — not memory corruption.

**5. Information disclosure — low, no new category.**
Battery voltage/SoC/rate is operational telemetry consistent with everything else already exposed on `/status`/MQTT (uptime, RSSI, sensor readings); adds no materially new disclosure risk.

**6. GPIO misuse — confirmed read-only.**
`PIN_VBUS_DETECT` (GPIO34) is configured `GPIO_MODE_INPUT` once at init (`fuel_gauge.c:72`) and only ever read via `gpio_get_level()` (`fuel_gauge.c:83`). No code path (MQTT command, HTTP POST, `/config`) reconfigures its direction or drives it as output.

## Overall verdict

This change introduces no new memory-safety risk and no new injection surface. All new formatting goes through the project's existing bounds-checked helpers with literal format strings and numeric-only interpolation of sensor data. The only prior finding of note (VCELL-threshold false-positive under USB power, causing `fuel_gauge_present()` to report "battery" incorrectly) is a functional/correctness issue already self-documented in the code and CHANGELOG — it has no security consequence (worst case: a misleading battery reading published to `/status`/MQTT, not any memory-unsafe or injectable condition).
