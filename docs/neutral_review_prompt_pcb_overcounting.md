# Prompt: Independent, neutral root-cause review — new PCBs read high vs original Heltec V2

Paste everything below as your first message in a brand-new session, started in an
empty working directory that has no CLAUDE.md, no `.claude/`, no `.remember/`, and
no prior project memory of any kind. Do not let the session gain any of those
things mid-run either (see the isolation rules below).

---

## Role and neutrality rules

You are conducting a neutral, skeptical, evidence-only engineering investigation.
You have not been given any prior analysis, conclusion, or hypothesis about this
problem, and you must not adopt one from anywhere except what you personally
verify in the primary sources listed below (schematics, PCB layout files, BOMs,
firmware source, and official chip datasheets/technical reference manuals).

Hard isolation rules — follow these exactly:
- Do NOT read, search, or open any file under `.claude/`, `.remember/`,
  `.superpowers/`, `CLAUDE.md`, `AGENTS.md`, `CHANGELOG*`, `README*`, or any file
  whose name suggests a summary, history, or prior analysis, anywhere in the
  firmware repository, UNLESS it is explicitly listed as a resource below. These
  are excluded specifically because they may contain a previous developer's
  conclusions, and the entire point of this exercise is to reach conclusions
  independently.
- If, while reading a *listed* file, you encounter a code comment that states or
  implies a causal explanation for the symptom (e.g. a comment asserting "this is
  because chip X does Y"), treat it exactly like any other unverified claim —
  something to test against independent evidence (a datasheet, a second file, a
  logical trace through the code), not something to adopt. Explicitly say so in
  your report wherever your reasoning touches such a comment: name the file/line,
  state that it is an unverified prior claim, and say whether your own
  independent evidence supports, contradicts, or is silent on it.
- Do not assume the project owner's framing is correct. In particular, question
  the claim that the new boards use "the same tube and same solder parts" as the
  original — verify this yourself by diffing actual BOM/schematic component
  values, don't take it at face value.

## Background (facts as reported by the project owner — verify, don't assume)

This is the open-source "MultiGeiger" Geiger counter project. The original
design (by ecocurious2) is built around a Heltec WiFi Kit 32 V2 dev board
(classic ESP32) — there is no separate custom PCB for that variant beyond the
Heltec dev board itself plugged into the original geiger board. Two new custom
PCBs have since been designed that carry the same physical tube type (an SI-22G
Geiger-Müller tube) but pair the tube's HV-generation / pulse-detection analog
circuit with a different host MCU module:

1. **Revision B** — hosts an Unexpected Maker FeatherS3-D (ESP32-S3).
   Product page: https://esp32s3.com/feathers3d.html
2. **Revision C** — hosts a Seeed Studio XIAO ESP32-S3 (plain variant, not
   Sense/Plus). Product page: https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/

**The reported symptom:** both new boards (Revision B and Revision C) give
consistently higher radiation readings (CPM and/or computed dose) than the
original Heltec V2 build, allegedly with the same tube and same components.

Your job is to find every plausible root cause for this, backed by evidence you
personally trace through the files below — not general Geiger-counter knowledge,
not assumptions, not anything you weren't able to verify.

## Resources to fetch and review

### A. Original ecocurious2 reference design (baseline — fetch these fresh, they are the comparison anchor)

Repository: https://github.com/ecocurious2/MultiGeiger/ (branch `master`)

Hardware (original Eagle-format design, files are text/XML so directly readable):
- Schematic (PDF, easiest first read): https://raw.githubusercontent.com/ecocurious2/MultiGeiger/master/docs/hardware/Schematic.pdf
- Schematic source (Eagle XML — exact component values/nets): https://raw.githubusercontent.com/ecocurious2/MultiGeiger/master/docs/hardware/geiger.sch
- Board layout (Eagle XML — copper geometry, for the EMI/layout hypothesis): https://raw.githubusercontent.com/ecocurious2/MultiGeiger/master/docs/hardware/geiger.brd
- Original BOM: https://raw.githubusercontent.com/ecocurious2/MultiGeiger/master/docs/hardware/MultiGeiger_Partlist.ods
- Heltec pinout reference: https://raw.githubusercontent.com/ecocurious2/MultiGeiger/master/docs/hardware/Pinning_of_HeltecBoards.pdf

Firmware (original is a PlatformIO/Arduino codebase — different structure to ours):
- Pulse counting logic: https://raw.githubusercontent.com/ecocurious2/MultiGeiger/master/multigeiger/tube.cpp
- Header: https://raw.githubusercontent.com/ecocurious2/MultiGeiger/master/multigeiger/tube.h
- Heltec V2 board HAL (pin assignments, pull config): https://raw.githubusercontent.com/ecocurious2/MultiGeiger/master/multigeiger/hal/heltecv2.h
- Main sketch (setup/loop, to see how tube.cpp is driven): https://raw.githubusercontent.com/ecocurious2/MultiGeiger/master/multigeiger/multigeiger.ino
- Timers (check for anything ticker/ISR-adjacent relevant to counting): https://raw.githubusercontent.com/ecocurious2/MultiGeiger/master/multigeiger/timers.cpp and https://raw.githubusercontent.com/ecocurious2/MultiGeiger/master/multigeiger/timers.h
- Full source tree if you need to look further: https://github.com/ecocurious2/MultiGeiger/tree/master/multigeiger

### B. New PCB — Revision B (FeatherS3-D), local files

Base path: `C:\Users\matth\OneDrive\Claude_Code\Geiger\Git_Repository_Geiger_V2\Hardware\Revision_B\`
- Schematic (KiCad, text S-expression, directly readable): `geiger.kicad_sch`
- Schematic PDF: `Deliverables\Schematic\geiger.pdf`
- PCB layout (copper geometry): `geiger.kicad_pcb`
- BOM: `Deliverables\BOM\geiger.csv` and `Deliverables\BOM\SMD-geiger.csv`

### C. New PCB — Revision C (XIAO ESP32-S3), local files — use as your primary worked comparison example against the original

Base path: `C:\Users\matth\OneDrive\Claude_Code\Geiger\Git_Repository_Geiger_V2\Hardware\Revision_C\`
- Schematic (KiCad, text S-expression, directly readable): `geiger.kicad_sch`
- Schematic PDF: `Deliverables\Schematic\MultiGeiger 2.0 Rev C.pdf`
- PCB layout (copper geometry): `geiger.kicad_pcb`
- BOM: `Deliverables\BOM\geiger.csv` and `Deliverables\BOM\SMD-geiger.csv`

### D. Our current firmware (ESP-IDF v6.0, shared codebase, multiple board targets)

Base path: `C:\Users\matth\OneDrive\Claude_Code\Geiger\Git_Repository_Geiger_V2\main\`
- Pulse counting core: `tube.c`, `tube.h`
- Hardware PCNT peripheral path (if used): `tube_pcnt.c`, `tube_pcnt.h`
- `tube_logic.h`, `tube_types.c`, `tube_types.h`
- Per-board hardware abstraction: `hal.h`
- Runtime config schema/defaults: `config.c`, `config.h`, `config_fields.def`
- Where CPM/dose is computed/reported: `main.c`

Per-board build configs (compare GPIO assignment, any board-specific `CONFIG_*`
knobs, e.g. GPIO glitch filter, interrupt flags, PSRAM/cache settings that could
affect ISR latency):
`C:\Users\matth\OneDrive\Claude_Code\Geiger\Git_Repository_Geiger_V2\`
- `sdkconfig.heltec_v2` / `sdkconfig.defaults.heltec_v2`
- `sdkconfig.feathers3_d` / `sdkconfig.defaults.feathers3_d`
- `sdkconfig.seeed_xiao_esp32s3` / `sdkconfig.defaults.seeed_xiao_esp32s3`
- `sdkconfig.defaults` (shared baseline all boards inherit)

### E. Official chip electrical references (for the MCU-input-sensitivity hypothesis)

- ESP32 datasheet: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
- ESP32-S3 datasheet: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
- ESP32 technical reference manual (GPIO matrix / glitch filter section): https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
- ESP32-S3 technical reference manual (GPIO matrix / PCNT peripheral sections): https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf

(If any of these URLs have moved, search espressif.com's documentation site for
the current link — do not substitute a third-party mirror without saying so.)

## Specific comparison tasks

1. **Schematic diff, three-way.** Compare the tube-facing analog front end
   (HV generation, coupling/shaping network, comparator/transistor stage feeding
   the MCU) across: original (`geiger.sch`), Revision B, and Revision C. List
   every component (designator, value, tolerance if stated, part number) in the
   analog chain and state for each one: identical across all three / differs —
   and if it differs, by how much and where.
2. **PCB layout / copper geometry diff.** Using `geiger.brd` (original) and both
   `.kicad_pcb` files, assess: how close is the MCU module's RF antenna to the
   pulse-detection node's traces/components, is there a ground plane/pour under
   or around the sensitive analog nodes, is there any shielding or copper moat
   separating analog from RF sections. State what you can and cannot determine
   from the file geometry alone.
3. **Counting-mechanism diff.** Determine, from actual code: does the original
   firmware (`tube.cpp`/`tube.h`) count pulses via a plain GPIO interrupt with a
   software dead-time/debounce check, or via a hardware pulse-counter peripheral?
   Does our firmware (`tube.c`, `tube_pcnt.c`) use the same mechanism on all
   three board targets, or does it select a different mechanism (e.g. hardware
   PCNT vs plain ISR) per board via `hal.h` or config defaults? If the mechanism
   itself differs between our Heltec-V2 build and our S3 builds, treat that as a
   first-class hypothesis in its own right, separate from "S3 silicon is just
   more sensitive."
4. **Dead-time / debounce constant diff.** Compare the numeric dead-time or
   debounce constant used in the original firmware against ours, and confirm
   whether ours is genuinely identical across all three board build targets (not
   just visually — check for any per-board `#ifdef`/config override).
5. **GPIO electrical characteristics.** Using the official datasheets/TRMs, note
   any documented difference in GPIO input threshold voltage, hysteresis, or
   glitch-filtering behavior between the classic ESP32 (Heltec V2) and ESP32-S3
   (FeatherS3-D, XIAO) that would plausibly cause narrow spurious edges to be
   latched as pulses on one chip family but not the other. Also check whether
   our firmware explicitly enables/disables the GPIO matrix's built-in glitch
   filter register, and whether that's set consistently across boards.
6. **CPM→dose conversion path.** Confirm whether our firmware's conversion from
   raw counts to displayed/transmitted CPM and dose is identical across all
   three board targets (no board-conditional scaling), to rule out a units/math
   bug as a confound before attributing the discrepancy to hardware.

## Other angles to evaluate (do not skip these just because they are not code/schematic diffs)

- **Tube unit-to-unit variance.** Three boards cannot literally share one
  physical tube in a side-by-side comparison. Flag this as a hypothesis: is
  there any evidence the "same tube" claim was actually a single tube swapped
  sequentially, vs three separate tubes? State what you can and cannot verify
  from the repo alone, and what physical test would rule this in/out.
- **Measurement methodology.** SI-22G background count rate implies real
  Poisson statistical variance over short windows, and background radiation
  itself drifts with time and location. Was there (as far as you can tell from
  any test logs/scripts in the repo, if any exist) a controlled simultaneous
  side-by-side comparison, or sequential/anecdotal testing? State what test
  protocol would be needed to trust the magnitude of the reported effect.
- **Mechanical/enclosure factors.** Tube shielding, proximity to switching
  components, enclosure material/grounding — flag as a hypothesis even if it
  cannot be verified from digital files, and say so explicitly rather than
  omitting it.
- **Interrupt/task scheduling.** Check whether the pulse-counting ISR/task has
  any documented or configured core affinity or priority difference across
  board targets (via `sdkconfig.*`) that could plausibly affect ISR latency or
  cause missed/duplicate servicing under WiFi radio activity.
- **Anything else you notice.** If tracing through these files surfaces a
  plausible mechanism not listed above, include it — this list is a floor, not
  a ceiling.

## Output requirements

For every hypothesis you report:
1. **Mechanism** — concretely, how it would cause a systematic *increase* in
   reported radiation specifically on the new boards (not just "could affect
   readings").
2. **Pros** — specific evidence FOR it, citing exact file/line/component/value
   you personally inspected. No hypothesis may be listed as supported without a
   concrete citation.
3. **Cons** — specific evidence AGAINST it. If you can't find one, say so
   explicitly rather than inventing one — but look actively.
4. **Confidence and reasoning** — your honest assessment of how likely this is
   the actual root cause, why (grounded in what you found, not intuition), and
   what specific test or measurement would confirm or rule it out.

Rank hypotheses most-likely-first by your own assessment. Explicitly list any
hypotheses you considered and ruled out, with the evidence that ruled them out.
End with a short summary of the 1–3 hypotheses most worth follow-up and why.

Do not modify any files. This is a read-only research task.

## Final output file

Write your complete report as plain Markdown to:

`C:\Users\matth\OneDrive\Claude_Code\Geiger\Git_Repository_Geiger_V2\docs\radiation_overcounting_independent_review.md`

If a file already exists at that path, do not overwrite it — write to
`C:\Users\matth\OneDrive\Claude_Code\Geiger\Git_Repository_Geiger_V2\docs\radiation_overcounting_independent_review_<YYYYMMDD-HHMM>.md`
instead, using the current date/time.
