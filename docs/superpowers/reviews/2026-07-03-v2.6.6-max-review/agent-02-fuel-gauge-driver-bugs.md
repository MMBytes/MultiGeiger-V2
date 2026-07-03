# Agent 2: fuel_gauge.c/.h deep bug scan

**Status: done**

## Fuel-gauge driver review — `main/fuel_gauge.c` / `main/fuel_gauge.h`

**1. Register read correctness — no issue.** `reg_read16()` (fuel_gauge.c:35-41) does one `i2c_master_transmit_receive` with a 1-byte register address out and a 2-byte `in[2]` buffer (`sizeof(in)`=2, no off-by-one). Byte order `(in[0]<<8)|in[1]` is genuine big-endian. Cross-checked against `veml7700.c:62-68`, which does `in[0] | (in[1]<<8)` (little-endian) — confirms the two drivers are correctly opposite, not accidentally copy-pasted the same way.

**2. Overflow/scaling math — no issue.** `raw` maxes at 65535; `65535 * 78125 = 5,120,859,375`, which exceeds `UINT32_MAX` (4,294,967,295), so the `uint64_t` widen at fuel_gauge.c:94 is genuinely required, not defensive over-caution. `CRATE`'s `(int16_t)raw` reinterpret-cast (line 125) is correct two's-complement, and the `0.208f` scale is applied to the already-signed value, not the raw unsigned one. Correct.

> RESOLVED 2026-07-04 (commit `30d3b1d`): `s_ready = true` moved to after
> `gpio_set_direction()`.

**3. Init ordering (`s_ready=true` before `gpio_set_direction`) — theoretical only, not reachable.** fuel_gauge.c:67/72. Traced the call site: `fuel_gauge_init(bus1)` is called from `main.c:1002`, inline in `app_main`'s sequential sensor-probe block, before any `xTaskCreate` in the startup path (no task creation found in `main.c`; WiFi/MQTT/HTTP tasks are spawned later in the same sequential function). So no other task can call `fuel_gauge_vbus_present()` before `gpio_set_direction()` returns — **non-issue in practice**, but it's a latent footgun (Minor) if this init is ever moved off the main task or called from an ISR/timer callback later.

> RESOLVED 2026-07-04 (commit `30d3b1d`): doc-comment now states a second
> call is a hard no-op, not a re-check.

**4. Idempotency claim — accurate but under-documented.** fuel_gauge.c:44 (`if (s_ready) return ESP_OK;`) makes a second call a true no-op — it skips `gpio_set_direction()` entirely on a re-init. That's fine functionally (the GPIO direction doesn't change across calls, nothing to reconfigure), but the header's "safe to call multiple times" doc-comment (fuel_gauge.h:51-52) doesn't clarify that a second call is a hard skip rather than a re-check. **Minor** — no correctness bug, just a docs gap.

**5. Error handling on I2C failures — consistent with codebase convention, no bug.** On a `reg_read16()` NAK after successful init, `s_dev`/`s_ready` are untouched — the driver keeps returning errors from `reg_read16` forever with no reset/retry. Compared directly against `veml7700.c:129-134`, which does the exact same thing (`s_ready` never cleared on a failed read). This is the established pattern in this codebase, not a fuel-gauge-specific defect.

**6. `s_batt_present` hysteresis state — no race hazard.** It's a file-static `bool`, and per feedback memory on torn-read history that memory concerned multi-word/64-bit shared state; a single `bool` write is atomic on Xtensa/RISC-V ESP32 architectures (single-byte store), so no torn-read risk here even without synchronization. Not flagged.

> RESOLVED 2026-07-04 (commit `30d3b1d`): reads now buffer into locals;
> output pointers are written only after all requested reads succeed.

**7. `fuel_gauge_read()` output-pointer sequencing — real but benign latent behavior.** volts/soc_pct/rate_pct_per_hr are each correctly NULL-checked individually (fuel_gauge.c:106,113,120). However, on a mid-sequence failure (e.g. VCELL read OK, SOC NAKs), `*volts` is already written but the function returns non-OK. Contrast with `veml7700_read()`, which defers writing `*raw_als`/`*raw_white` until after all reads succeed. **Minor** — checked all three call sites (`http_server.c:717`, `mqtt.c:540`, `transmission.c:1437`); all gate entirely on `== ESP_OK` and never touch the out-params otherwise, so this is not currently exploitable, but it's an inconsistency with the sibling driver's stricter all-or-nothing contract.

**8. Stub branch parity — matches exactly.** fuel_gauge.c:133-140 signatures are identical to the header declarations (types, names, order); `fuel_gauge_init`'s stub has `(void)bus;` present and correct.

## Overall verdict

The core register-level math (byte order, overflow-safe scaling, signed CRATE reinterpretation) is correct and matches the datasheet. No critical or important bugs found. The two Minor items (init-ordering theoretical footgun, and `fuel_gauge_read`'s partial-write-before-error return) are both currently unreachable/harmless given actual call-site behavior, not live bugs.
