# Hardware

PCB designs that the MultiGeiger V2 firmware runs on. Three revisions are
tracked. Each is a complete design with fabrication outputs under its
`Deliverables/` folder: Gerbers (plus an order-ready zip), pick-and-place
CSV, BOM, schematic PDF and 3D renderings.

## Revision → board → firmware

| Directory | Board | Module it carries | Firmware target(s) | Minimum firmware | Tool |
|---|---|---|---|---|---|
| `Revision_A/` | MultiGeiger mainboard **V1.10** | Heltec WiFi LoRa 32 V4, base/R2 variant | `heltec_wifi_lora32_v4_r2` | **V2.7.1** | Eagle (`geiger-v2.sch` / `.brd`) |
| `Revision_B/` | FeatherS3-D-format carrier, **V2 layout** | Unexpected Maker FeatherS3-D, SparkFun Thing Plus ESP32-S3, Adafruit ESP32-S3 TFT Feather, Adafruit ESP32 Feather V2, Adafruit ESP32-S3 Feather #5477 | `feathers3_d`, `sparkfun_thing_plus_esp32s3`, `adafruit_esp32s3_tft_feather`, `adafruit_esp32_feather_v2`, `adafruit_esp32s3_feather_4mb_2mbpsram` | **V2.6.34** | KiCad 8 |
| `Revision_C/` | Shared-footprint carrier, **V2 layout** | Adafruit QT Py ESP32-PICO **or** Seeed XIAO ESP32-S3 in the same U1 socket | `adafruit_qtpy_esp32_pico`, `seeed_xiao_esp32s3` | **V2.6.34** | KiCad 8 |

Not in this directory:

- The Heltec WiFi Kit 32 V2 targets (`heltec_v2`, `heltec_v2_4mb`) run on the
  upstream [ecocurious2 MultiGeiger](https://github.com/ecocurious2/MultiGeiger)
  mainboard (V1.4, Eagle files in that repository's `docs/hardware/`).
- The SparkFun Thing Plus ESP32-C5 target has its own carrier; that design is
  not published here.

## Firmware compatibility — read before flashing

There is no runtime board-revision detection: **the firmware version is the
cutover line.** The pin maps live in `main/hal.h`, whose header comment is
the authoritative statement of this rule.

- **Rev B V2 and Rev C V2 layouts (the files tracked here)** need firmware
  **≥ V2.6.34**. That release moved the Geiger count and HV-FET signals to
  different socket pads to remove HV-to-counter crosstalk.
- **The original (pre-V2) Rev B / Rev C layouts** must stay on firmware
  **≤ V2.6.33**. Flashing a newer build onto an original carrier drives the
  HV FET PWM into the tube pickup and reads "counts" off the FET gate trace.
  Those layouts are superseded and not tracked.
- **V1.10 mainboard (Revision A)** needs firmware **≥ V2.7.1**. V2.6.34 was
  written against a pre-fabrication pad assignment that was never
  manufactured, so for this target V2.6.34 matches no physical board.
- **V1.9 mainboard** (an earlier third-party respin for the same module, not
  tracked) stays on **≤ V2.6.33**.

The browser flasher always installs the latest release, so it is only
suitable for boards in the first and third groups.

## Provenance

- **Revision A** is the V1.10 iteration of the upstream MultiGeiger mainboard
  lineage (ecocurious2 V1.4 → a third-party V1.9 respin for the Heltec WiFi
  LoRa 32 V4 module → this V1.10 rework: HV_CAP_FULL moved from GPIO2 to
  GPIO3 and GMC_COUNT to GPIO6 — the V2.6.34 design had those two the other
  way round and was never built — piezo on GPIO4/5). Kept in Eagle like its
  ancestors.
- **Revision B** and **Revision C** are this project's own KiCad 8 designs.
  They keep the upstream HV topology (flyback boost, zener-regulated rail,
  Si22G tube) and add module sockets, a Qwiic / STEMMA QT connector and
  (Rev C) a sealed, speaker-less form factor. The "V2 layout" of each is the
  current file; the first layout was replaced after the crosstalk
  investigation described in `main/hal.h`.

BOM part numbers are maintained at the source — the Revision A BOM is
generated, the Revision B / C part numbers live in a schematic field pushed
to the footprints — so edit there, not in the exported CSVs.

## Licence

All three designs: **GPL-3.0-or-later**, the same licence as the firmware
and as the upstream MultiGeiger project they descend from. See the
repository [LICENSE](../LICENSE). Upstream attribution: ecocurious2 /
MultiGeiger.
