# Agent 7: concurrency / thread-safety (re-derivation of the `s_user_present` verdict)

**Status: done**

## Scope

Re-examine the `s_user_present` cross-task race in `main/fuel_gauge.c` from scratch,
given that agent 5 (git-history-context, 2026-07-07) flagged that the 2026-07-03
"benign" verdict was reached against the *old* `s_batt_present` semantics — where
the flag only gated a *secondary* confirmation alongside an (unreliable) VCELL
threshold — and was carried forward unexamined after the V2.6.6 rewrite made the
renamed `s_user_present` the **sole** gate for `fuel_gauge_present()`. Also in
scope: any new shared mutable state introduced by the `i2c_bus.h` consolidation
(commit `959a39c`).

## 1–2. Current implementation + call-site / task map

`main/fuel_gauge.c:29-31`:
```c
static i2c_master_dev_handle_t s_dev            = NULL;
static bool                    s_ready          = false;
static bool                    s_user_present   = false;   // mirrors cfg->batt_present
```

`fuel_gauge.c:97-103`:
```c
bool fuel_gauge_present(void) {
    return s_ready && s_user_present;
}
void fuel_gauge_set_user_present(bool present) {
    s_user_present = present;
}
```
No lock, no `volatile`, no atomic type. Plain read/plain write of a file-static `bool`.

**Writers of `s_user_present`** (via `fuel_gauge_set_user_present()`):
- `main.c:1006` — `app_main`, before `tx_setup()` (`main.c:1156`) and
  `http_server_start()` (`main.c:1157`). Runs single-threaded, no other task
  touching fuel_gauge state exists yet. Not part of the race.
- `http_server.c:2031` (`config_post()`) — runs on the **httpd task**, on
  every `/config` POST Save, live-apply, no reboot.

**Readers of `s_user_present`** (via `fuel_gauge_present()`):
- `http_server.c:715` (`format_battery()`, called from `status_get()`) —
  **httpd task**, on every `/status` page load.
- `transmission.c:1431` — **tx task**, once per TX cycle (battery log line).
- `mqtt.c:544` — **tx task** (same call chain as above: `tx_run()` →
  MQTT rich-state JSON build; confirmed this is not a separate MQTT-library
  task, it's inline code executed by `tx_task`).
- `mqtt_discovery.c:59` (`fuel_gauge_present_()` wrapper) — used for HA
  discovery entity registration; invoked from `mqtt_apply_config()` /
  `mqtt_init()`, both called either from `main.c` (boot, single-threaded) or
  from `http_server.c:2039` (`config_post()`, same httpd task as the writer
  at line 2031 — same-task, not a race with itself).

**Task/core map** (`main.c` task-creation calls + `transmission.c:239-240`):
- `tx_task`: `xTaskCreatePinnedToCore(tx_task, "tx", ..., 1)` — **pinned to
  core 1** (`transmission.c:239-240`).
- httpd task: created internally by `esp_http_server` via `httpd_start()`
  (`http_server.c:2671`) using `HTTPD_DEFAULT_CONFIG()` (`http_server.c:2646`),
  which does **not** override `core_id` — IDF's default is `tskNO_AFFINITY`.
  The scheduler is therefore free to run it on core 0 *or* core 1, and there
  is nothing pinning it away from core 1.

So the httpd task and `tx_task` **can genuinely execute simultaneously on two
different physical cores** on the S3's dual-core Xtensa LX7 — this is not a
single-core interleaving question, it is a true SMP data race in the C11
sense (concurrent unsynchronized access to a non-atomic object, at least one
of them a write).

## 3. Is a torn read physically possible?

No. `bool` is a single byte, naturally aligned; both Xtensa and RISC-V ESP32
variants load/store a single byte in one bus transaction — there is no
"tearing" of a 1-byte value. Confirmed as correct in both the 2026-07-03
review and here; not in dispute.

Hardware note relevant to visibility (not raised in the prior review): the
ESP32-S3's two cores access internal SRAM (where this `.bss` static lives)
directly over the system bus — there is **no per-core data cache for internal
RAM** (the cache/MMU only sits in front of flash and PSRAM). So there is no
hardware cache-coherency lag to worry about here; a write from one core is
visible to the other core after ordinary bus arbitration latency (nanoseconds
to low microseconds), not "stale until the next cache sync."

The only real risk is a **narrower one than the prior review named**: not
memory-system visibility, but **compiler-level reordering/caching**. Because
`s_user_present` is a plain non-`volatile` static, the compiler is in
principle free to hoist/cache a read of it across calls if it can prove
(within its single-threaded abstract machine) that nothing in between could
change it — a purely theoretical risk today because `fuel_gauge_present()`
and `fuel_gauge_set_user_present()` are ordinary external functions called
once per invocation from different translation units (no LTO cross-inlining
observed in this build), so in practice each call is a fresh load. But it is
not *guaranteed* safe by the language rules the way it would be with
`volatile` or `_Atomic`.

## 4. Consequence of a stale read — has the blast radius actually changed?

Worst case: a user unchecks "Battery attached" and hits Save at the exact
instant `tx_task`'s `tx_run()` (core 1) is mid-cycle calling
`fuel_gauge_present()` (`transmission.c:1431` / `mqtt.c:544`) on core 0 (httpd).
Whichever ordering wins, the outcome is: **one battery log line and/or one
MQTT `batt_v`/`batt_soc`/`batt_rate` JSON block is either present when it
should now be absent, or vice versa, for exactly one TX cycle.** The next
cycle reads the settled value and self-corrects.

Checked whether anything downstream *latches* this transient value into
something persistent or destructive:
- **NVS**: `batt_present` is persisted by `config_save(&cfg_next)`
  (`http_server.c:2016`) — a completely separate write path, entirely inside
  the httpd task, using the POST-local `cfg_next`/`s_cfg`, never touching
  `s_user_present`. The race cannot corrupt NVS.
- **GPIO**: `fuel_gauge_present()` does not drive any GPIO. The only GPIO in
  this file, `PIN_VBUS_DETECT`, is read-only input, gated by `s_ready` (which
  is write-once at boot, steady-state-read only afterward — matches the
  prior review's item 3, still a non-issue).
- **HA discovery entities**: `mqtt_discovery.c:59` wraps the same call but is
  invoked either at boot or synchronously inside `config_post()` itself (same
  task as the writer) — no cross-task race on that call site specifically.
- **Radiation-counting path** (the actual safety-relevant function of this
  device): `tube.c`'s GMC pulse/HV logic is entirely independent of
  `fuel_gauge.c`; nothing here can affect pulse counting, dead-time guarding,
  or HV control.

So the consequence is, and remains, **purely cosmetic display/telemetry
noise for a single cycle** — exactly the same *category* of impact the
2026-07-03 review found for the old VCELL-gated flag. The "sole gate vs.
secondary confirmation" distinction agent 5 raised is real as a description
of what changed in the code, but it does not change the *outcome* of the
race: in both the old and new code, the only thing ever downstream of this
flag is "show/hide a battery info block" — there was never a case, old or
new, where the race could feed a value into NVS, a GPIO, or the counting
path. The old code's secondary VCELL check didn't bound the race's blast
radius; it was just another input to the same cosmetic display decision.

## 5. Precedent check — does this project's `boot_epoch` fix set a stricter bar?

`main/ntp.c:29`:
```c
static volatile uint32_t s_boot_epoch_off = 0;
```
read at `ntp.c:130` (`uint32_t off = s_boot_epoch_off;`), written once from the
SNTP callback task at `ntp.c:46`, read from `http_server.c` and elsewhere.

This is a **different risk class**, and confirms rather than overrides the
2026-07-03 framing:
- `s_boot_epoch_off` is a **32-bit** word. On Xtensa/RISC-V a 32-bit aligned
  load/store is also atomic (no tearing) — so `volatile` here isn't fixing a
  torn-read hazard either. It's fixing the *compiler-reordering* hazard
  described in §3 above: `ntp_boot_epoch()` is called repeatedly and the
  value is genuinely expected to transition asynchronously (NTP sync landing
  at an unpredictable time relative to callers), so the precedent this
  project actually set is **"cross-task scalar flags get `volatile`, not a
  mutex/critical-section"** — not "cross-task scalar flags need
  `portENTER_CRITICAL`." No sensor-driver static in this codebase (VEML7700,
  BME280, BMP581/390, SHT45, SPS30, DNMS, GNSS's fast-path fields) uses a
  mutex for a plain presence/ready bool; `gnss.c:71`'s `s_fix_mux` and
  `tube.c`'s several `portMUX_TYPE`s guard genuinely multi-field / ISR-shared
  state (fix lat+lon+alt as a tuple, or pulse-count + capture state touched
  from an ISR), which is a materially different hazard (multi-word
  consistency, or ISR-vs-task races) than a single independent bool.
- Applying that precedent to `fuel_gauge.c`: `s_user_present` (and `s_ready`)
  should be `volatile bool` for the same reason `s_boot_epoch_off` is
  `volatile uint32_t` — not because a torn read or hardware-cache staleness
  is possible, but to give the compiler an explicit signal that the value
  can change between calls from another task, foreclosing even the
  theoretical reordering risk from §3, and to keep this file consistent with
  the project's own established idiom for this exact situation.

## 6. I2C consolidation (`i2c_bus.h`, commit `959a39c`) — new shared state?

Read `main/i2c_bus.h` in full. The per-device helpers added in the
consolidation (`i2c_dev_write_reg`, `i2c_dev_read_regs`, `i2c_dev_read_u16_be`,
`i2c_dev_read_u16_le`, `i2c_dev_write_u16_le`, `i2c_add_device`,
`i2c_probe_and_add`, `i2c_dev_teardown`, `i2c_bus.h:116-190`) are all
**`static inline`** functions operating purely on parameters and stack-local
buffers (`uint8_t buf[2]`/`buf[3]`, `uint8_t in[2]`) — the file header
explicitly notes this is deliberate, matching the project's existing
`static inline` shared-header convention (e.g. `util.h`) specifically so each
translation unit gets its own copy with **no shared static/global state and
no ODR concerns**. No scratch buffer, no "last error" variable, no counter
is introduced by this refactor. The bus-lifecycle owner functions
(`i2c_bus_get_primary/secondary`, `i2c_bus_finalize`, etc.) are unchanged by
this specific refactor and out of scope (pre-existing, not touched by
commit `959a39c`).

**Finding: no new cross-task shared mutable state from the I2C
consolidation.** Non-issue.

## Findings summary

| # | Finding | File:line | Severity |
|---|---|---|---|
| 1 | `s_user_present` read/write not `volatile`/atomic across `httpd` task (core-unpinned) and `tx_task` (pinned core 1) — genuine SMP race, theoretically reachable via compiler reordering, but bounded to a single stale/missing display cycle with no persistent or control-path consequence | `fuel_gauge.c:31,97-103` | Minor |
| 2 | `s_ready` same non-`volatile` pattern, but write-once at boot before either racing task exists (per prior review's task-ordering proof, re-confirmed here) — steady-state reads only | `fuel_gauge.c:30,87` | Minor (non-issue in practice) |
| 3 | I2C consolidation (`i2c_bus.h`) introduces no new shared mutable state — all new helpers are stateless `static inline` | `i2c_bus.h:116-190` | Non-issue |

No Critical or Important findings.

## Overall verdict

**Still benign — no functional fix required, but worth a one-line hygiene
change to match this project's own precedent.** Re-deriving from scratch
(not inheriting either prior verdict): the stakes-change agent 5 flagged is
real as a *code* change (secondary-confirmation → sole gate) but does not
change the *consequence* of the race, because in both the old and new
implementations `s_user_present`/`s_batt_present` only ever drives a
read-only display/telemetry decision (show battery block or not; include
`batt_v`/`batt_soc`/`batt_rate` in the MQTT JSON or not) — it never reaches
NVS, a GPIO, or the counting/HV path. A lost race costs at most one TX
cycle's worth of battery-data visibility, self-correcting on the next cycle;
that was true before V2.6.6 and remains true after. `bool` read/write is
provably atomic on both Xtensa and RISC-S3 targets (no torn-read risk), and
the ESP32-S3's internal-SRAM access path has no per-core data cache to create
hardware-level visibility lag — so the only theoretical gap is compiler-level
register caching of a value the compiler doesn't know is shared, which this
project already has a precedent for closing cheaply: `ntp.c`'s
`s_boot_epoch_off` uses `volatile uint32_t` for exactly this class of
"infrequently-written, cross-task-read scalar," not a mutex or atomic type.
Recommend applying the same idiom here — mark `s_user_present` (and, for
consistency, `s_ready`) `volatile bool` in `fuel_gauge.c:30-31` — as cheap
defensive hygiene and codebase-consistency, not as a correctness-blocking
fix; a full `portENTER_CRITICAL`/`_Atomic` treatment would be over-engineering
for a value with this consequence profile. The I2C consolidation refactor
introduces no new shared-state risk at all.
