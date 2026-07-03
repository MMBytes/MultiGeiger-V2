# MAX17048 Fuel Gauge — Design Spec

Status: approved by user + independent review agent. Not yet implemented.
Board scope: **FeatherS3-D only** (only board with the physical chip). All other boards compile a zero-cost stub.

## 1. Hardware

The FeatherS3-D ("[D]" revision — distinct from plain FeatherS3/TinyS3/ProS3, which use an ADC-divider VBAT pin instead) has an onboard MAX17048 fuel-gauge IC at I²C address **0x36**, wired to the same pins as the STEMMA1 connector (GPIO8 SDA / GPIO9 SCL — `hal.h:198`). This is the **primary** I²C bus (`i2c_bus_get_primary()`), always powered from the 3.3V LDO1 rail independent of the battery — confirmed both by the project's own wiring doc and by Unexpected Maker's own ESPHome device page, which labels this bus "always on." The chip therefore ACKs on I²C and free-runs regardless of whether a battery is physically connected — no manual enable, no init register writes required.

Registers used:
| Register | Addr | Width | Scale | Notes |
|---|---|---|---|---|
| VCELL | 0x02 | 16-bit unsigned | 78.125 µV/LSB | cell voltage |
| SOC | 0x04 | 16-bit unsigned | 1/256 %/LSB | state of charge |
| CRATE | 0x16 | 16-bit **signed** | 0.208 %/hr/LSB | +charging / -discharging |

## 2. Battery-presence detection

**Auto-detect via voltage threshold. No config toggle, no NVS field.**

```
present:  VCELL > 2000 mV
absent:   VCELL < 1500 mV
```
(hysteresis band between the two — a reading between 1500-2000mV keeps the previous state, to avoid flapping right at a boundary)

Justification (do not re-derive without new evidence — see `project_fuel_gauge_max17048_design.md` memory for the full research trail):
- A bare `VCELL > 0` check was explicitly rejected: zero noise margin against a single glitched I²C byte or ADC jitter near the rail, which would false-positive "battery present" and start publishing garbage SoC/rate to HA.
- Empirical anchor (`CHANGELOG.md` V2.4.28, real deployed no-battery node): VCELL/SoC read **~0V / 0%** with nothing attached, on this exact chip/board.
- A real physically-connected cell — even one sitting at its protection-IC over-discharge cutoff (~2.4V for common cheap protection ICs) — never reads below ~2.4V; the protection IC disconnects it before that point.
- This leaves a wide dead zone (~0V confirmed-absent to ~2.4V confirmed-lowest-possible-present) in which to place a threshold with margin on both sides. Datasheet's exact open-circuit/leakage electrical spec for the CELL pin was not obtainable (5 fetch attempts across ADI, DigiKey, alldatasheet.com mirrors, and the reference ESPHome component source all failed to surface it — the ESPHome `max17048` component used by Unexpected Maker's own device page has no presence-detection logic at all to borrow from). The 2000/1500mV numbers are reasoned-and-justified, not datasheet-verified to the LSB.

## 3. Cost model (why this doesn't burden the other 99% of nodes)

- All non-FeatherS3-D boards: `HAL_HAS_FUEL_GAUGE` is 0, `fuel_gauge.c` compiles to a no-op stub (`#if HAL_HAS_FUEL_GAUGE ... #else ... #endif`, following the `als.c` pattern — not `veml7700.c`, which has no compile-time gating at all). Zero runtime cost.
- FeatherS3-D with no battery: one cheap I²C VCELL read per check (~sub-ms at 400kHz, same order as the existing VEML7700 reads already done every `/status` load and every TX cycle). No full 3-register read, no SoC/CRATE reads, until presence is confirmed.
- FeatherS3-D with a battery: full VCELL/SOC/CRATE read, same cost class as the existing env/PM/noise sensor reads already happening every cycle.

## 4. Module: `fuel_gauge.c` / `fuel_gauge.h`

Facade API, modeled on `als.h`/`sht45.h`:

```c
esp_err_t fuel_gauge_init(i2c_master_bus_handle_t bus);   // no-op stub if !HAL_HAS_FUEL_GAUGE
bool      fuel_gauge_present(void);                        // true iff chip ACKs AND VCELL > threshold (hysteresis-tracked)
esp_err_t fuel_gauge_read(float *volts, float *soc_pct, float *rate_pct_per_hr);  // any pointer may be NULL
```

`fuel_gauge_present()` owns the hysteresis state (a static `bool s_present` latch, updated each call per the 2000/1500mV band above). `hal.h` gains `HAL_HAS_FUEL_GAUGE`, set 1 only in the `BOARD_FEATHERS3_D` block (next to the existing `HAL_HAS_ALS` flag at `hal.h:117`).

`main.c` init: call `fuel_gauge_init(bus1)` directly (no dual-bus probe macro — this chip is a fixed onboard part on STEMMA1/primary, not a pluggable Qwiic peripheral like env/PM/noise sensors), alongside the existing `als_init()` call at `main.c:1032`.

`main/CMakeLists.txt`: no change needed — all driver `.c` files already compile unconditionally for every board (per-board gating happens inside each file via `#if HAL_HAS_X`).

## 5. `/status` page

New `format_battery()` in `http_server.c`, modeled directly on `format_als()` (`http_server.c:667-705`): returns an empty string if `!fuel_gauge_present()`, otherwise a `<div class="info">` block showing voltage, SoC %, and charge rate (with a "charging"/"discharging" label derived from the sign of `rate`). Wired into the page alongside the existing `format_als(...)` call (`http_server.c:1156`).

## 6. MQTT rich-state JSON

New `APPEND` lines in `mqtt.c`, modeled on the lux block (`mqtt.c:519-531`):

```c
if (fuel_gauge_present() &&
    fuel_gauge_read(&batt_v, &batt_soc, &batt_rate) == ESP_OK) {
    APPEND(",\"batt_v\":%.3f",   batt_v);
    APPEND(",\"batt_soc\":%.1f", batt_soc);
    APPEND(",\"batt_rate\":%.2f",batt_rate);
}
```

Field names: `batt_v` (volts, not `batt_mv` — matches the project's unit-agnostic naming convention), `batt_soc` (%), `batt_rate` (%/hr signed — deliberately not `batt_crate`, to avoid leaking the MAX17048 register mnemonic into the public API).

## 7. Home Assistant discovery

Three new rows in `mqtt_discovery.c`'s `ENTITIES[]` table (alongside the ambient-light row at `mqtt_discovery.c:161`), gated on a new `fuel_gauge_present_` predicate:

```c
{ "batt_v",   "battery_voltage", "Battery voltage", "voltage", "V", "measurement", NULL, NULL, fuel_gauge_present_, NULL },
{ "batt_soc", "battery",         "Battery",         "battery", "%", "measurement", NULL, NULL, fuel_gauge_present_, NULL },
{ "batt_rate","battery_rate",    "Battery charge rate", NULL,  "%/h","measurement", "mdi:battery-clock", NULL, fuel_gauge_present_, NULL },
```

These are the first entities in the codebase to use HA's standard `"battery"` and `"voltage"` device classes — no precedent to reconcile, both are standard HA classes.

## 8. Log line

**Standing requirement, independent of any other design consideration**: emit one log line per TX cycle when a battery is attached (`transmission.c`, alongside the per-target rc pass/fail lines), e.g.:

```
ESP_LOGI(TAG, "battery: %.3fV %.1f%% %+.2f%%/hr", batt_v, batt_soc, batt_rate);
```

Gated on `fuel_gauge_present()` — skipped entirely (no log line, no read) when absent, keeping the per-cycle cost on a no-battery FeatherS3-D down to the single cheap VCELL presence-check read described in §3.

## 9. Out of scope

- No config field, no NVS key, no `/config` UI change — detection is fully automatic.
- No support on any board other than FeatherS3-D.
- No attempt to distinguish "no battery" from "battery whose protection IC has tripped from over-discharge" — both present as ~0V at the chip and are indistinguishable under any detection method; this is a hardware limit, not a design gap.
