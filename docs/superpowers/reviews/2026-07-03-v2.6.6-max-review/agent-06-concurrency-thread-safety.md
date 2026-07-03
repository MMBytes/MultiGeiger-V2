# Agent 6: concurrency / thread-safety

**Status: done**

## Concurrency Review: `main/fuel_gauge.c`

**Task landscape (confirmed via call sites and `xTaskCreate`):**
- **main/init task**: `fuel_gauge_init(bus1)` — `main.c:1002`, called synchronously in `app_main`'s linear setup, **strictly before** `tx_setup()` (`main.c:1152`) and `http_server_start()` (`main.c:1153`). Neither `tx_task` nor the httpd task exist yet at this point.
- **httpd task**: `esp_http_server`'s single worker thread (confirmed serial-only by the explicit comment at `http_server.c:2620-2629` — "ONE httpd thread processes all URI handlers serially"). Calls `fuel_gauge_present()` + `fuel_gauge_read()` from `status_get()` at `http_server.c:714,717`.
- **tx task**: `tx_task`, `xTaskCreatePinnedToCore(..., core 1)` at `transmission.c:239-240`. Calls `fuel_gauge_ready()` + `fuel_gauge_read()` + `fuel_gauge_vbus_present()` at `transmission.c:1435-1440` (per-cycle log line), and `fuel_gauge_present()` + `fuel_gauge_read()` at `mqtt.c:538-544` (rich-state JSON) — both execute inside the same `tx_run()` call chain, same task.

So post-boot there **are** two independent, potentially-concurrent tasks (httpd on its core, tx pinned to core 1) that both call into `fuel_gauge.c`.

**Findings:**

> DEFERRED 2026-07-04: left as-is by explicit user choice — benign,
> matches codebase convention.

1. **`s_batt_present` racy read-modify-write** — `fuel_gauge.c:96-100`. Scenario: httpd task's `status_get()` (`http_server.c:714`) and tx task's `mqtt.c:538` both call `fuel_gauge_present()` at nearly the same instant. Each does its own independent I²C VCELL read + hysteresis compare, then writes `s_batt_present`. If VCELL sits inside the 1500-2000 mV band, whichever write lands last wins. This is **not** a torn read — `bool` is single-byte/word-aligned, atomic on both Xtensa and RISC-V — just a benign "last writer wins" outcome, self-correcting on the next call. **Severity: Minor** (cosmetic, one-cycle-stale at worst).

> RESOLVED 2026-07-04 (commit `30d3b1d`): `s_ready = true` moved to after
> `gpio_set_direction()`, exactly the fix recommended below.

2. **`fuel_gauge_init()` ordering hazard** — `s_ready = true` (`fuel_gauge.c:67`) is set **before** `gpio_set_direction(PIN_VBUS_DETECT, ...)` (`fuel_gauge.c:72`). In isolation this looks like the classic "flag set before the state it guards is ready" bug. **But verified non-exploitable today**: init runs single-threaded in the main task before `tx_task`/httpd even exist (see task landscape above), so no concurrent reader of `s_ready` can observe the gap. **Severity: Minor, currently latent/unreachable** — flag as fragile: it depends on call-site ordering convention rather than the function's own invariant. Recommend moving the `s_ready = true` assignment to after `gpio_set_direction()` as defensive hygiene, since a future refactor (e.g., deferred/async init) could silently reintroduce the window.

3. **`s_ready` steady-state reads** — after the one-time init transition, `s_ready` never flips back to `false`; concurrent reads from httpd/tx tasks are just reads of a stable, unchanging bool. Not the same bug class as the project's historical torn-read fixes, which were about *multi-word* (64-bit) values straddling multiple load/store instructions on a 32-bit core. **Non-issue.**

4. **`s_dev` / concurrent `i2c_master_transmit_receive()`** — no app-level mutex guards this, but this matches the codebase's uniform pattern: `bme280.c`, `bmp581.c`, `bmp390.c`, `sht45.c`, `sps30.c`, `veml7700.c`, `gnss.c`, `dnms.c` all call the I²C driver directly with no wrapping lock, relying on ESP-IDF's i2c_master driver being internally thread-safe for the same bus handle across tasks. `fuel_gauge.c` doesn't introduce a new violation of an established stricter pattern — if this assumption is wrong it's a pre-existing systemic risk across every sensor driver, not something new here. **Non-issue for this diff specifically.**

## Overall verdict

No Critical or Important cross-task bugs. Two Minor items, one of which (#2, init-ordering) is currently unreachable given verified init ordering but worth a defensive one-line fix (move `s_ready = true` after the `gpio_set_direction()` call).
