# Independent review: radiation over-counting on the Revision B / Revision C boards

**Date:** 2026-07-09
**Method:** Neutral, evidence-only investigation. Every claim below is traced to a
primary source I personally inspected in this session: the original ecocurious2
MultiGeiger repository (fetched fresh from GitHub `master`), the local Revision B /
Revision C KiCad sources and BOMs, the current ESP-IDF firmware in `main/`, the
per-board `sdkconfig` files, the official Espressif datasheets/TRMs (ESP32 Series
Datasheet v5.2, ESP32-S3 Series Datasheet v2.2, ESP32 TRM v5.7, ESP32-S3 TRM v1.8,
all downloaded from espressif.com), and the raw per-cycle measurement CSV
(`Geiger_Log/pcnt_experiment_cycles.csv`, 2026-06-08 → 06-16) which I re-analysed
from scratch with my own scripts rather than trusting any existing analysis.
Files excluded by the isolation rules (`.claude/`, `.remember/`, `CLAUDE.md`,
`README*`, `CHANGELOG*`, memory/analysis documents such as
`reference_radiation_data_analysis`) were not read. Code comments that assert
causal explanations are treated as **unverified prior claims** and are explicitly
labelled as such wherever my reasoning touches them (see §6).

> **Update 2026-07-09 (same day, evening):** the tube-removed bench test
> proposed under H1 was run and **confirmed H1, variant (a) — copper/supply
> coupling injection**. See §10 (addendum) for the data and the revised
> ranking; §7's ranking is retained as-written for the record.

---

## 1. What is actually identical (verified, not assumed)

### 1.1 Counting mechanism — identical across all three targets

* **Original firmware** (`multigeiger/tube.cpp`, fetched from GitHub): plain GPIO
  edge interrupt, `attachInterrupt(..., isr_GMC_count, FALLING)` on GPIO 2
  (tube.cpp:220, tube.cpp:18), with a software dead-time gate
  `#define GMC_DEAD_TIME 190` µs (tube.cpp:22) applied inside the ISR
  (tube.cpp:177). No hardware pulse-counter peripheral.
* **Our firmware** (`main/tube.c`): plain GPIO edge interrupt,
  `GPIO_INTR_NEGEDGE` (tube.c:291), ISR installed with `ESP_INTR_FLAG_IRAM`
  (tube.c:295), software dead-time gate `GMC_DEAD_TIME_US 190` (tube.h:22,
  applied at tube.c:206). **One shared `tube.c` for every board**; the only
  per-board difference is the pin number from `hal.h` (Heltec GPIO 2, FeatherS3-D
  GPIO 18, XIAO GPIO 2 — hal.h:101, hal.h:178, hal.h:395).
* The hardware PCNT path (`main/tube_pcnt.c`) is **opt-in only**: it is
  initialised only when the user enables `pcnt_filter` in /config
  (main.c:1140-1142), which defaults to **false** on every board
  (config_fields.def:353). With defaults, all boards count via the identical ISR.
* Grep of the whole `main/` tree confirms `GMC_DEAD_TIME_US` is defined exactly
  once, with no `#ifdef`/config override anywhere (only tube.h:22 defines it).
* The optional dead-time guard (`deadtime_guard`) also defaults off
  (config_fields.def:402) and, per my analysis of the raw CSV (§4), was off for
  essentially the whole comparison dataset (only qtpy 06-15/16 had it on, 3.5%
  effect).

**Conclusion:** a counting-mechanism difference between board targets is ruled
out as a root cause. The mechanism is also behaviourally equivalent to the
original firmware (same edge polarity, same 190 µs constant, same
gate-on-last-counted-pulse semantics).

### 1.2 CPM → dose conversion — identical, no board conditionals

`usvph = cps * tube_cps_to_usvph(tube_type)` (main.c:562-563);
`cpm = counts * 60000 / dt_ms` (main.c:564). The Si22G factor is
`1/12.2792` (tube_types.c:19), byte-identical to the upstream original
(tube.cpp:35 in the ecocurious2 repo: `{"Radiation Si22G", 22, 1 / 12.2792}`).
No board-conditional scaling exists anywhere in the counts→CPM→dose path.
A units/math bug is ruled out as the cause of a *board-dependent* difference.

### 1.3 sdkconfig — nothing counting-relevant differs

Comparing `sdkconfig.heltec_v2`, `sdkconfig.feathers3_d`,
`sdkconfig.seeed_xiao_esp32s3` (generated caches) and their
`sdkconfig.defaults.*` overlays:

* CPU frequency 160 MHz on all three; `CONFIG_FREERTOS_HZ=100` on all three;
  WiFi task pinned to core 0 on all three; `CONFIG_ESP_INTR_IN_IRAM=y` on all
  three.
* The overlays differ only in flash size, PSRAM mode (none / quad / octal),
  console routing (UART vs USB-Serial-JTAG) and TLS/heap tuning — nothing that
  touches GPIO interrupt behaviour, glitch filtering, or interrupt affinity.
* `CONFIG_SOC_GPIO_SUPPORT_PIN_GLITCH_FILTER=y` appears in the S3 caches, but
  that is a *soc capability constant*, not an enable. A grep of `main/` shows no
  call to any per-pin glitch-filter API; the only glitch filters in the codebase
  are the opt-in PCNT unit filters (tube_pcnt.c:92-95) and I2C (irrelevant).
  **The count input is electrically unfiltered on every board.**

### 1.4 HV generation firmware — identical timing on all boards

The 100 µs gptimer state machine (tube.c:101-165) is a line-for-line port of the
original `isr_recharge` (tube.cpp:59-152): 1500 µs FET-on, 1000 µs settle, check
cap-full, repeat until full; adaptive idle targeting 2 charge pulses per train.
Note for §5/H1: **consecutive charge pulses within one train are spaced 2.5 ms
apart** (1500 + 1000 µs), and the controller deliberately aims for 2-pulse
trains (tube.c:147-148, original tube.cpp:124-131).

---

## 2. Three-way schematic diff (comparison task 1)

Sources: original Eagle schematic `geiger.sch` + Eagle board `geiger.brd` +
original BOM `MultiGeiger_Partlist.ods` (all fetched from GitHub); Rev B/Rev C
`geiger.kicad_sch`, `geiger.kicad_pcb`, `Deliverables/BOM/geiger.csv`.

Netlist topology was extracted programmatically from `geiger.brd` (Eagle
`<signal>` contactrefs) and both `geiger.kicad_pcb` files (pad→net map). The
tube-facing analog chain is **topologically identical in all three designs** —
same nets, same component roles, and the new designs even inherit the original
net name `GMZ_COUNT`:

| Role | Original (Eagle .sch / .brd / Partlist.ods) | Rev B (BOM + sch) | Rev C (BOM + sch) | Verdict |
|---|---|---|---|---|
| Boost inductor | L1 = L-11P 150 mH (Partlist: "L-11P 150M") | L1 150 mH, PN 11P-154J-50 | same | identical |
| HV switch FET | BSP125 (U4) | BSP125H6327 SOT223 (U1) | BSP125 (U3) | identical |
| HV rectifiers | D4 BYV26E + D6 GP10Y in series | D4 BYV26E + D6 GP10Y | D4 BYV26E + D5 GP10Y | identical |
| HV regulation / cap-full sense | D1+D2 = ZY200 (2 × 200 V zener) from flyback node down to HV_CAP_FUL | same (D1, D2 ZY200) | same | identical |
| Cap-full clamp/bleed | D3 BZX55C3V3 + R1 10k + C2 10 nF on HV_CAP_FUL | same | same | identical |
| HV storage | C3 100 nF 630 V | C3 100 nF (R75PI3100AA30J) | same | identical |
| Anode series R | **Eagle sch: R3=R4=2M7. Partlist.ods: R3,R4 = 4.7 MΩ** | R3=R4=4M7 (MFR-25FBF52-4M7) | same | see note ① |
| HV-side divider R | R2 = 1M | R2 = 1M | R2 = 1M | identical |
| Pickup coupling | C4 = 100 pF 1000 V (FKP2) from R2/R3 junction to GMZ_COUNT | C4 = 100 pF (FKP2O101001D00JSSD) | same | identical |
| Count-node pull-up | R5 = 1M to 3V3 | R5 = 1M | R5 = 1M | identical |
| Count pin connection | direct to module pin (GPIO 2), no series R, no Schmitt buffer | direct to Feather A1 (GPIO 18) | direct to XIAO D1 (GPIO 2) | identical topology |
| Bulk cap | C1 100 µF | C1 100 µF (16SEPC100MW) | same | identical |

Pickup topology (verified identical in all three netlists): HV rail → R2 (1M) →
node X → R3+R4 (→ tube anode); C4 (100 pF) couples node X to GMZ_COUNT, which
idles at 3V3 through R5 (1M) and goes **directly** into the MCU pin. A tube
discharge pulls node X down; C4 couples a negative edge onto the count pin.
Recovery time constant R5·C4 ≈ 100 µs, which is why the 190 µs dead time exists.

**Note ① — the only real component-value discrepancy found:** the original
*Eagle schematic* says R3=R4=2.7 MΩ, but the original *published BOM*
(Partlist.ods) says 4.7 MΩ, matching the new boards. So "same solder parts" is
true **if** the baseline unit was built from the Partlist, and false (5.4 MΩ vs
9.4 MΩ total anode resistance) if it was built from the schematic. This is
checkable in minutes with a multimeter on the physical baseline unit. Direction
analysis: if the baseline really has 2×2M7, its pickup divider
R2/(R2+R3+R4) delivers a *larger* pulse (≈15.6% of the anode swing vs ≈9.6%)
and its tube recovers *faster* — that would make the **original count more, not
less**, so this discrepancy cannot by itself explain the new boards reading
higher; but it does change pulse amplitude margins and should be verified.

I2C pull-ups (R6/R7 10k) and connectors differ in packaging but are outside the
analog chain. The new BOMs specify Vishay UXB precision 10k for R1/R6/R7 and
MFR-25 1% metal film for the megohm parts; the original Partlist specified 5%
carbon film ("Kohleschicht") — tolerance upgrade only, wrong direction to cause
over-counting and negligible in this circuit.

---

## 3. PCB layout / copper geometry diff (comparison task 2)

Extracted programmatically from the three board files (footprint positions,
net-segment geometry, zone definitions).

* **Board size / module placement.** Original: 100×32 mm; Heltec module at
  x≈0-51, analog+tube section x≈62-100. Rev B: 123×32 mm; Feather module
  x≈114-152, analog section x≈177-224. Rev C: 85×32 mm; XIAO x≈153-169, analog
  x≈177-224. Module-to-analog separation is of the same order (~10-25 mm) in
  all three.
* **Ground pours.** Original has GND polygons on the **bottom layer only**
  (Eagle layer 16, two polygons spanning most of the board; no top pour).
  Rev B and Rev C both have full-board GND zones on **both** F.Cu and B.Cu.
  On paper the new boards are *better* shielded, not worse.
* **Count trace length** (net GMZ_COUNT): original 54.5 mm, Rev B 63.5 mm
  (crosses to B.Cu), Rev C 24.1 mm (single layer). Comparable; Rev C shortest.
* **Aggressor-to-victim clearances** (minimum segment-to-segment distance,
  computed from copper geometry):

  | Aggressor → victim | Original | Rev B | Rev C |
  |---|---|---|---|
  | Flyback node (FET drain / L1) → pickup node (C4/R2/R3) | 5.6 mm | 3.8 mm | 3.8 mm |
  | Flyback node → GMZ_COUNT trace | 6.3 mm | 8.7 mm | 8.7 mm |
  | HV_FET gate drive → pickup node | **1.3 mm** | 1.5 mm | 3.6 mm |
  | HV_FET gate drive → GMZ_COUNT | 2.5 mm | 1.6 mm (opposite layers) | 7.0 mm |

  The original actually has the *tightest* gate-drive-to-pickup clearance.
  Nothing in the raw copper geometry marks the new boards as obviously more
  coupling-prone; if anything the double-sided pours should help them.
* **What cannot be determined from the files:** actual antenna position relative
  to the tube. The FeatherS3-D has an onboard PCB antenna + u.FL with an RF
  switch (verified on the manufacturer page, esp32s3.com/feathers3d.html;
  firmware defaults to the onboard antenna, `use_external_antenna=false`,
  config_fields.def:61). The **plain XIAO ESP32-S3 has no onboard antenna at
  all — it has a u.FL connector and ships with a loose external antenna that
  the builder sticks somewhere** (verified on the Seeed wiki,
  wiki.seeedstudio.com/xiao_esp32s3_getting_started/). Note: this **contradicts
  the comment in hal.h:367** ("PCB antenna only (no u.FL on standard XIAO
  ESP32-S3)") — that comment is wrong about the hardware, though the flag it
  documents (`HAL_HAS_ANTENNA_SWITCH 0`, i.e. no firmware-controllable RF
  switch) is still correct. Where the XIAO's antenna pigtail is physically
  placed relative to the tube/pickup is uncontrolled by the PCB design and
  invisible in these files.

---

## 4. What the raw measurement data actually shows (my own re-analysis)

`Geiger_Log/pcnt_experiment_cycles.csv` contains per-cycle device logs
(2026-06-08 → 06-16) for four simultaneously running units: `heltec` (original
PCB + Heltec V2), `feather` (Rev B + FeatherS3-D), `qtpy` (shared small PCB +
**classic ESP32** QT Py PICO — same PCB design family as Rev C), `xiao` (shared
small PCB + XIAO ESP32-S3, i.e. the Rev C configuration). I computed all numbers
below directly from the CSV (pre-filter ISR counts, so the PCNT width filter
does not distort them; the dead-time guard was verified off except a 3.5%
qtpy tail on 06-15/16).

**4.1 The test protocol was genuinely simultaneous.** Cycles from all units
overlap in wall-clock time and were captured together (`aligned_*` logs match
cycles within 90 s). This was a controlled side-by-side, not sequential anecdote.
However, each unit necessarily contains **its own physical SI-22G tube** — the
"same tube" claim cannot be literally true for a simultaneous comparison.

**4.2 The units were physically manipulated mid-experiment.** Daily pooled CPM:

| board | 06-08 | 06-09 | 06-10 | 06-11 | 06-12 | 06-13 | 06-14 | 06-15 |
|---|---|---|---|---|---|---|---|---|
| heltec | 85 | 89 | 92 | 90 | 91 | 90 | 92 | 91 |
| feather | 99 | 91 | 72 | 72 | 71 | 72 | 72 | 71 |
| qtpy | 162 | 105 | — | — | — | 100 | 97 | 92 |
| xiao | — | 106 | 101 | **264** | **320** | 95 | — | — |

The heltec baseline is rock-stable (~90 CPM). The feather went from **+16%
above** heltec to **−20% below** heltec between 06-08 and 06-10 with no firmware
knob involved (guard off, raw counts) — something physical changed (position,
tube, shielding, source). The xiao spent two days at ~3× background. **Pooled
multi-day cross-board ratios from this dataset therefore do not cleanly measure
a board-intrinsic effect**; per-window ratios do. The often-quoted "+14%"-class
numbers correspond to specific windows (e.g. 06-07/06-08 feather vs heltec:
+13-16% at ~9σ in the `aligned_pool_2026-06-07.log` windows, consistent with my
own 06-08 daily numbers).

**4.3 The board-linked artifact: HV-recharge-correlated burst counts.**
Per-cycle Pearson correlations I computed between HV charge pulses
(`hv_pulses`) and edges arriving with 1-5 ms spacing (`edt_lt5k` histogram bin —
these edges pass the 190 µs gate and ARE counted):

| board | mean hv_pulses/cycle | mean 1-5 ms edges/cycle | corr(hv, 1-5 ms edges) | 1-5 ms edges ÷ hv_pulses |
|---|---|---|---|---|
| heltec (original PCB) | 28.5 | 1.6 | **−0.02** | 0.06 (Poisson-consistent) |
| feather (Rev B) | 19.8 | 2.9 | **+0.86** | 0.15 |
| qtpy (new small PCB, classic ESP32) | 40.0 | 19.7 | **+0.99** | **0.49** |
| xiao (new small PCB, ESP32-S3) | 325.8 | 162.7 | **+0.98** | **0.50** |

Poisson cross-check: at ~90-100 CPM the expected fraction of genuine inter-pulse
gaps under 5 ms is ~0.8%; heltec measures exactly that (~0.7%), while qtpy
measures 6.4% and xiao 25.8% — the excess is bursts, not statistics.

Three facts make this the single most probative finding:

1. On the new-design boards the 1-5 ms edge count per cycle tracks HV recharge
   activity almost perfectly (r ≈ 0.98-0.99); on the original board there is
   **zero** correlation despite comparable HV activity (heltec 28.5 vs qtpy 40
   pulses/cycle).
2. The ratio is almost exactly **one extra counted edge per two HV charge
   pulses** on qtpy and xiao (0.49, 0.50) — and the firmware deliberately
   targets **2-pulse charge trains** whose two pulses are **2.5 ms apart**
   (§1.4), i.e. one train ≈ one spurious count landing precisely in the 1-5 ms
   histogram bin.
3. It is present on the classic-ESP32 QT Py exactly as on the S3 XIAO — so it
   is a property of the **new board design (or its build/tube/HV unit)**, not
   of ESP32-S3 silicon.

Magnitude check: qtpy's ~19.7 burst edges/cycle ≈ +6.6 CPM ≈ +7% — matching its
measured +7.6% over heltec almost exactly. The xiao's high days (06-11/12,
264-320 CPM) coincide with its HV activity exploding to ~326 pulses/cycle;
bursts = hv/2 throughout. The feather's coupling is weaker (0.15 counts per hv
pulse, ≈+1.3% at its HV rate).

**4.4 Narrow-pulse (width) populations exist but differ per unit.** From the
PCNT width comb (per-board, same pin, parallel to the ISR): fraction of edges
narrower than the given width — feather: 1.2% <250 ns, 1.2% <1 µs, 5-8% <4 µs
(so its narrow population is 1-4 µs wide); xiao: 5.3% <250 ns, 6.0% <4 µs
(mostly ultra-narrow <250 ns); qtpy: ~1% total; heltec (short sample, 17k
edges): 3.9% <250 ns. Every unit carries some narrow-edge population; the S3
units are not uniquely afflicted, and the classic-ESP32 qtpy on the same PCB as
the xiao shows the *least*. GM tube output pulses at this network are ~100 µs
wide, so sub-µs edges are electrical artifacts (ringing/EMI) wherever they occur.

---

## 5. GPIO electrical characteristics (comparison task 5)

From the official documents (downloaded from espressif.com this session):

* **Input thresholds are specified identically.** ESP32 Series Datasheet v5.2,
  Table 5-3: VIH min = 0.75×VDD, VIL max = 0.25×VDD. ESP32-S3 Series Datasheet
  v2.2, Table 5-4: VIH min = 0.75×VDD, VIL max = 0.25×VDD. Same pin capacitance
  (2 pF), same weak pull-up (45 kΩ).
* **Neither datasheet documents input hysteresis** (no Schmitt-trigger spec for
  digital GPIO in either document). The spec band between 0.25×VDD and
  0.75×VDD (0.83 V - 2.48 V at 3.3 V) is undefined territory in both — actual
  switching points of individual chips can sit anywhere in it. So a *real*
  per-chip threshold difference is possible but **undocumented and cannot be
  established from datasheets**; it would also not explain the qtpy (classic
  ESP32) over-reading on the new PCB.
* **Glitch filtering.** ESP32-S3 TRM v1.8 §6.4.2: the S3 has an optional per-pin
  input filter (`IO_MUX_FILTER_EN`, rejects pulses shorter than two sample
  clocks) — default **disabled**, and our firmware never enables it (grep of
  `main/`: no per-pin filter API call). The classic ESP32 TRM v5.7 chapter 6
  documents **no per-pin GPIO filter at all** (its only input filters are inside
  peripherals: PCNT §19, UART, I2C). Net effect: **both chips run the count pin
  unfiltered**; the S3's extra filter hardware is unused. Neither TRM specifies
  a minimum pulse width for GPIO edge-interrupt latching, so "S3 latches
  narrower edges than ESP32" is not supported (or refuted) by the documentation.
* The S3 TRM (§6.4.1) documents two-stage APB-clock synchronisation of GPIO
  matrix inputs; the ESP32 TRM does not describe an equivalent stage. At 80 MHz
  APB, any documented sampling difference is at the ~12-25 ns scale —
  irrelevant to µs-scale pulses.

---

## 6. Prior causal claims encountered in listed files (isolation-rule audit)

| Location | Claim | My independent evidence |
|---|---|---|
| ecocurious2 tube.cpp:178-180 (and inherited at tube.h:10, tube.h:21-22) | pulses shortly after a valid pulse are false re-triggers "because we don't have a Schmitt trigger on this controller pin" | Datasheets are **silent** on hysteresis (§5) — the no-Schmitt premise is plausible but unverifiable from docs. The 50-190 µs edge population it gates does exist (heltec 2.1% of edges, xiao 5.7%) — the gate is doing real work; the stated *reason* is untested. |
| config_fields.def:333-341 | a "1-4 µs marginal-pulse population ... makes the Feather (ESP32-S3) read ~+14 % over the Heltec (ESP32)" | **Partially contradicted.** A 1-4 µs-wide population does exist on the feather (§4.4, 5-8% of edges). But (a) the feather's excess over heltec was **not stable** (+16% on 06-08 → −20% by 06-10, §4.2), so "+14%" was a snapshot, not a board constant; (b) the classic-ESP32 qtpy over-reads on the new PCB with only ~1% narrow pulses, via the HV-burst channel instead (§4.3) — so narrow-pulse capture by S3 silicon cannot be the general explanation, and no documentary basis exists for S3-vs-ESP32 narrow-pulse sensitivity (§5). |
| config_fields.def:363-401 | "~60% of the QT Py over-count is exactly this 1-5ms burst, the other ~40% is genuine and unfilterable" | The 1-5 ms burst population on qtpy is real and I measure it at ≈100% of qtpy's excess vs heltec in the June window (+6.6 CPM of +7.6%) — my data **supports the burst finding and its timing band**, though I get a larger share than the quoted 60/40 split (different window, different comparison baseline; not a contradiction I can resolve from this CSV alone). |
| main.c:534-536 | "the sub-190 µs ringing the ISR rejects is <250 ns wide (measured, 2026-06-08)" | Consistent with the CSV: units whose sub-190 µs edge bins are populated also show matching <250 ns PCNT removal (xiao 5.7% lt190 ↔ 5.3% <250 ns). Supported. |
| hal.h:367 | XIAO ESP32-S3 is "PCB antenna only (no u.FL)" | **Contradicted** by the Seeed wiki (§3): the plain XIAO ESP32-S3 has a u.FL connector and an external antenna. |
| hal.h:385-387, hal.h:291-293 | placing HV_FET on the opposite board edge "keeps GMC pulse pickup quieter" | Not borne out as sufficient: the new small PCB (qtpy/xiao) shows the strongest HV-correlated count injection despite that placement (§4.3). |

---

## 7. Hypotheses, ranked

### H1 — HV recharge activity generates counted edges on the new boards (most likely)

**Mechanism.** Each HV charge train (FET switching ~400 V flyback transients,
pulses 2.5 ms apart, deliberately 2 per train) produces one counted falling edge
on GMZ_COUNT on the new-design boards. Two physical variants, indistinguishable
from digital data alone: (a) **electrical coupling** of the flyback/gate-drive
transient into the high-impedance pickup node (1 MΩ/9.4 MΩ node, 100 pF
coupling) or into the shared supply/ground return; (b) **tube re-ignition** —
the recharge step bumps the anode voltage and re-fires a tube sitting at a
marginal operating point (which also explains elevated HV demand feeding back).
Either way, reported CPM inflates in proportion to HV recharge activity — which
itself varies with leakage (humidity, flux residue, tube condition), explaining
why the effect ranges from +1% (feather) to +7% (qtpy) to +200% (xiao bad days).

**Pros.**
* corr(hv_pulses, 1-5 ms edges) = +0.99/+0.98/+0.86 on qtpy/xiao/feather vs
  **−0.02** on heltec (§4.3, my computation from the raw CSV).
* Burst-per-hv-pulse ratio ≈ 0.50 on qtpy and xiao — exactly one count per
  2-pulse charge train, whose intra-train spacing (2.5 ms — tube.c:125,131)
  lands exactly in the observed 1-5 ms histogram bin.
* Quantitatively accounts for qtpy's entire measured +7.6% vs heltec.
* Present on classic-ESP32 and S3 hosts alike → board-level, chip-independent.

**Cons.**
* Copper clearances between HV aggressor nets and the pickup are *not* worse on
  the new boards than the original (§3, table) and the new boards have more
  ground pour — the coupling path is therefore not obvious in the layout files;
  common-impedance (ground/supply return through the module pins) or
  tube/HV-unit behaviour (variant b) remain, but I could not pin the path from
  the files.
* Each unit has its own tube and its own ZY200 zener pair (HV setpoint tolerance
  unspecified in any of the three BOMs), so variant (b) is confounded with unit
  variance (H2).

**Confidence & discriminating tests.** High confidence that HV-correlated spurious
counts are the dominant *board-linked* artifact (the correlation structure is
about as clean as field data gets). Medium confidence on the exact coupling path.
Tests: (1) scope the count pin and the HV FET gate simultaneously — do count-pin
edges align with charge pulses? (2) with the tube removed (fuse clips empty) and
HV forced to cycle, watch for count edges: edges present → copper/supply
coupling; absent → tube-mediated re-ignition. (3) firmware A/B: temporarily lock
`next_charge` to a fixed long interval on one unit; CPM should drop by exactly
the burst population if H1 holds. (4) measure actual HV at the tube on all units
(zener stack tolerance).

### H2 — Tube unit-to-unit variance + uncontrolled test-setup changes (co-equal contributor, definitely present)

**Mechanism.** Every simultaneous comparison in the dataset used a different
physical SI-22G per unit. GM tube background sensitivity varies unit-to-unit;
the original Partlist itself warns the eBay-sourced tubes are often defective
("oftmals sind Röhren defekt", Partlist.ods, tube row). A hotter/leakier tube
reads high on whatever board hosts it.

**Pros.**
* The feather unit's raw CPM changed −27% between 06-08 and 06-10 with firmware
  knobs verified off (§4.2) — physical setup changes of that magnitude were
  happening *during* the reference experiment, so at least part of any
  "new board reads X% high" figure is setup, not board.
* The xiao's two days at 264-320 CPM (with HV activity ×11 the heltec's) show
  unit-specific pathology (tube/HV/leakage), not a stable board property —
  on 06-13 the same xiao read 95 CPM, within 6% of the heltec.
* Nothing in the repo documents a tube swap between hosts; "same tube" is
  unverifiable from the files and physically impossible for the simultaneous runs.

**Cons.**
* Both S3-hosted units *and* the qtpy read high in the same direction in their
  respective windows — random tube variance has no preferred direction, so
  directionality across ≥3 units is weak evidence against pure tube luck
  (~12.5% by chance).
* The HV-burst correlation (H1) is structural, not tube-random: heltec's zero
  correlation vs 0.98-0.99 on the new boards cannot be produced by a merely
  hotter tube.

**Confidence & test.** Certain that it contaminates the *magnitude* of every
reported ratio; unlikely to be the whole story (H1's correlations survive it).
Test: swap one physical tube between the heltec and a new board and see how much
of the difference follows the tube; repeat with positions exchanged (same shelf
spot) to control geometry/background gradient.

### H3 — Narrow (<4 µs) edge population counted on some units ("S3 more sensitive" in its original form) — real signal, wrong attribution

**Mechanism (as claimed).** The count line carries sub-4 µs edges; if ESP32-S3
inputs latched narrow/marginal edges that classic-ESP32 inputs miss, S3 boards
would over-read.

**Pros.**
* The narrow population is real and measured per-board by the PCNT comb (§4.4):
  feather 5-8% (1-4 µs wide), xiao ~6% (<250 ns), and applying the 4 µs width
  filter is an existing, working mitigation on those units.
* For the feather specifically — whose HV-burst contribution is only ~1.3% — the
  1-4 µs population is the largest identified artifact candidate for its
  (transient) +16% window.

**Cons.**
* No documentary support for a chip-sensitivity difference: identical specified
  thresholds, no hysteresis spec on either side, no minimum-pulse-width spec on
  either side (§5).
* The heltec itself showed ~3.9% <250 ns edges in its comb sample — classic
  ESP32 on the original board *also* latches ultra-narrow edges; and the
  classic-ESP32 qtpy on the new PCB shows the **smallest** narrow population
  (~1%). The narrow-edge population varies by *board/unit*, not by chip family.
* An ISR-counted edge is an ISR-counted edge on both families; the question is
  whether the *analog signal* exists on the pin, which is board/tube-dependent.

**Confidence & test.** The population is real (high confidence); the "S3 silicon"
attribution is unsupported (low confidence it's chip-intrinsic). Test: drive an
identical calibrated narrow pulse (0.1-4 µs, amplitude sweep) into the count pin
of a Heltec V2 and an S3 board and compare latch thresholds; that isolates chip
from board in one afternoon.

### H4 — RF/antenna proximity (WiFi bursts counted as pulses)

**Mechanism.** 2.4 GHz TX bursts rectify in the pickup network or inject edges;
new boards put the radio in a different (or uncontrolled) position relative to
the tube.

**Pros.** The XIAO's antenna is a loose external flex on a pigtail
(manufacturer-verified, §3) — its placement is uncontrolled and could sit against
the tube or pickup wiring. hal.h:367's contrary claim means this was likely never
considered during bring-up.

**Cons.** No correlation channel for it in the data I have (WiFi TX happens each
cycle on all units equally; the excess instead tracks HV activity, §4.3). Copper
distances from module to pickup are comparable across designs (§3). The heltec's
count trace runs directly under/past its own module for ~40 mm.

**Confidence & test.** Low-to-medium as a primary cause; cheap to rule out: log
raw edge rate with WiFi disabled vs enabled, and move the XIAO antenna to the
far end of the enclosure vs taped to the tube.

### H5 — Supply/regulator noise differences between modules

**Mechanism.** The count node's 1 MΩ pull-up references the module 3V3; the three
modules use different regulators (Heltec/UM/Seeed designs). HF noise on 3V3
modulates the idle level toward VIL.

**Pros/cons.** Not assessable from the repo files (module-internal). The R5·C_node
low-pass makes the node fairly immune to HF rail noise; no data channel points at
it. Listed for completeness — flagged as **unverifiable from digital files**.
Test: scope 3V3 and the idle count-node level on each board.

### H6 — Mechanical / enclosure factors

Tube mounting, proximity of the tube to the switching inductor L1 (which is
5-15 mm from the fuse-clip tube in all three layouts), enclosure material and
grounding, and bench position (shelf height, wall vs window) all modulate real
count rate and coupling. **Cannot be verified from the repository** — explicitly
flagged rather than omitted. The 06-08→06-10 feather step (§4.2) proves factors
of this class were active during the reference measurements.

---

## 8. Hypotheses considered and ruled out

| Hypothesis | Ruled out by |
|---|---|
| Different counting mechanism per board (PCNT vs ISR) | Single shared `tube.c` ISR for all boards; PCNT is opt-in, default off (config_fields.def:353, main.c:1140). §1.1 |
| Dead-time constant differs per board | `GMC_DEAD_TIME_US 190` defined once, no per-board override anywhere (grep, §1.1); identical to original tube.cpp:22. |
| CPM→dose math / units bug per board | Board-agnostic math (main.c:562-564); identical Si22G factor to upstream (tube_types.c:19 vs tube.cpp:35). §1.2 |
| sdkconfig knob (glitch filter, core pinning, CPU freq, ISR placement) | All counting-relevant symbols identical across the three caches (§1.3). |
| S3 pin glitch filter enabled on some boards | Never enabled anywhere in `main/`; default off per S3 TRM §6.4.2. |
| Documented GPIO threshold difference ESP32 vs S3 | Identical spec values in both datasheets (§5). What remains possible is *undocumented* actual-silicon spread — moved into H3's test, not assumable. |
| HV generation firmware timing differs per board | Line-for-line identical state machine and constants (§1.4). |
| "New boards use different analog component values" | BOM-level diff shows identical values/parts (§2) — with the single R3/R4 Eagle-schematic-vs-Partlist ambiguity flagged (note ①), whose direction is wrong for explaining the symptom. |
| Sequential/anecdotal test methodology (no real effect) | The June dataset is a genuine simultaneous side-by-side with Poisson-significant window ratios (§4.1); *some* real per-unit effect exists — though its size is contaminated (H2). |

---

## 9. Bottom line — the 1-3 hypotheses worth follow-up

1. **H1: HV-recharge-correlated spurious counts on the new boards.** This is the
   only candidate with a near-perfect quantitative signature in the raw data
   (r ≈ 0.99, one count per 2-pulse charge train, absent on the original board,
   chip-family-independent). Bench-discriminate the coupling path (scope +
   tube-removed test), then fix at the source — likely candidates: snubbing the
   flyback edge, low-impedance return for the charge loop, or moving/filtering
   the pickup node; alternatively verify per-unit HV setpoint (ZY200 spread)
   against the SI-22G plateau.
2. **H2: tube and setup variance.** Before quantifying any board effect, do a
   tube-swap + position-swap crossover between the heltec baseline and one new
   board. Every ratio published so far is contaminated by unit variance and by
   mid-experiment physical changes (feather −27% step, xiao ×3 days).
3. **H3 (narrow-edge population), specifically for Rev B/feather:** its excess is
   *not* explained by H1 (bursts ≈ +1.3% only); its 1-4 µs edge population
   (5-8%) is the best in-data lead. Characterise with the injection test in §7-H3
   before attributing anything to ESP32-S3 silicon — the documentation gives no
   basis for a chip-sensitivity claim, and the classic-ESP32 qtpy data actively
   undercuts it.

Not modified: any repository file other than this report. All analysis scripts
were run from the session scratchpad.

---

## 10. Addendum (2026-07-09 evening) — tube-removed bench test: H1 confirmed as coupling injection

The discriminating test proposed in §7-H1 ("with the tube removed and HV forced
to cycle, watch for count edges: edges present → copper/supply coupling; absent
→ tube-mediated re-ignition") was run on the Rev C unit (XIAO ESP32-S3, firmware
V2.6.5, `pcnt_filter` on @4000 ns, dead-time guard off — both verified from the
boot config dump in the log). Sources, both parsed directly:

* `Geiger_Log/Xiao_With_Tube.log` — 41 cycles, 82 min, tube installed.
* `Geiger_Log/Xiao_Without_Tube.log` — 8 cycles, 16 min, tube physically out of
  the clips. One continuous boot (verified: `hv_pulses cum` runs 257→269→…341
  across all cycles; the mid-file boot banner is a delayed applog/syslog replay,
  not a reboot).

### 10.1 Result: counts without a tube, locked 1:1 to HV charge pulses

* **Steady state (cycles #2-8): `counts = hv_pulses = 12` in every cycle.**
  Ratio 1.000, per-cycle correlation +1.000, zero scatter. The tube-less board
  fabricates ~6 CPM (logged as 0.008 µSv/h) at the firmware's minimum HV
  activity (one charge pulse per 10 s).
* **Phase-locked timing fingerprint:** `min_us = 10,002,500 µs` every cycle =
  the 10.000 s maximum recharge idle **+ 2.5 ms**, the charge-pulse state
  machine's exact duration (1500 µs FET-on + 1000 µs settle, tube.c:125,131).
  Observed jitter ±~60 µs. The counted edge is the charge pulse.
* **Cycle #1 (initial charge of the empty cap):** 257 charge pulses in rapid
  trains → 205 raw edges, 165 of them in the 1-5 ms spacing bin,
  `min_us = 2492 µs` ≈ the 2.5 ms intra-train spacing — the same signature that
  dominated the June xiao data (§4.3), reproduced with **no tube present** and
  **before WiFi associated** (rssi=-127, bssid all-zeros during that cycle).
* **Injected edges are wide (>4 µs):** the PCNT width filter removed **0 of 12**
  per steady-state cycle (`w_ns[0|250|1000|4000]=12 12 12 12`) and only 39/205
  during the charge-up burst; `rejected=0` throughout (nothing near the 190 µs
  gate). **The existing `pcnt_filter` mitigation cannot remove this artifact.**
* **With-tube cross-check:** pooled 95.2 CPM at hv≈12-13/cycle. In 6 of the 8
  cycles where `hv_pulses=13` (exactly one 2-pulse train), `min_us` collapses to
  2492-2499 µs — two injected counts 2.5 ms apart — while `hv=12` cycles show
  random `min_us`. The injection operates identically with the tube installed.
* Suggestive arithmetic (not a controlled comparison — different days):
  95.2 CPM measured − ~6 CPM injected ≈ 89 CPM, i.e. right at the Heltec
  baseline's stable ~90 CPM (§4.2).

Artifact model: **injected_counts ≈ 0.8-1.0 × hv_pulses**, so the CPM error
scales with whatever drives HV recharge activity (leakage, humidity, tube
condition, real count rate). This is the multiplier that turned a ~6 CPM floor
into +200% on the June 11-12 xiao days (~326 hv_pulses/cycle then vs 12 now).

### 10.2 Revised hypothesis ranking

| Hypothesis (§7) | Status after this test |
|---|---|
| H1(a) copper/supply coupling injection | **CONFIRMED** — counts appear with no tube, 1:1 with charge pulses, phase-locked to 2.5 ms. |
| H1(b) tube re-ignition | Not needed to explain any bench data; retained only as a possible *additional* contributor on the June high-HV days. |
| H2 tube/setup variance | Unchanged as a magnitude confound for all June ratios; slightly reduced — this xiao's residual rate (~89 CPM) is unremarkable. |
| H3 narrow-pulse / "S3 more sensitive" | Further downgraded: the injected edges are ≥4 µs wide, so the width filter is ineffective against the dominant artifact; the with-tube narrow population (14.8%, mostly <250 ns) is ringing already handled by the 190 µs gate (pre- vs post-filter CPM differ by 1 count in ~7,800). Survives only as the unexplained feather-specific 1-4 µs population (§4.4). |
| H4 RF/antenna | Effectively ruled out as primary: 166 counts injected before WiFi association. |
| H5 supply noise | Absorbed into H1(a) as one candidate coupling path (5V/GND common impedance through the module pins). |
| H6 mechanical/enclosure | Irrelevant to the identified artifact. |

### 10.3 Open questions (flagged, not assumed)

1. **Coupling path unknown.** FET gate-drive trace, flyback node, or shared
   5V/GND return are all still candidates; §3's clearance table does not single
   one out.
2. **What drove hv_pulses to ~326/cycle in June** (27× today's floor) on this
   same unit is unresolved — humidity/leakage/tube seating. This determines how
   large the artifact gets in the field, and deserves its own experiment.
3. Today's runs and June's Heltec baseline are different days; the "95−6≈90"
   agreement is suggestive only.

### 10.4 Next tests (bench-level)

1. **Firmware charge-pulse veto/attribution (no hardware):** the recharge
   callback knows the µs timestamp of every FET edge; record it and have
   `gmc_count_isr` tag (later: suppress) edges arriving within a small window of
   a charge pulse, logged per cycle like `guard_removed`. One overnight run
   gives per-edge attribution; as a suppressor it is a candidate production fix
   with a quantifiable dead-time cost (~0.03% at 12 pulses/120 s). Run on the
   Heltec too — it should tag ~zero, closing the board-vs-board loop.
2. **Repeat the tube-removed run on Rev B (feather) and the original Heltec
   board** — expected partial (~0.15/pulse, §4.3) and zero injection
   respectively; completes the cross-design picture with no equipment.
3. **Leakage experiment on the xiao** (humid vs dry, flux-washed vs not) to
   explain the June HV-activity explosion — the artifact's field multiplier.
