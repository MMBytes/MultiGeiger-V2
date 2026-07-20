#pragma once

/** @file
 *  @brief Board-level hardware abstraction — pin map and feature flags.
 *
 *  One of `BOARD_HELTEC_V2`, `BOARD_FEATHERS3_D`,
 *  `BOARD_ADAFRUIT_QTPY_ESP32_PICO`, `BOARD_SEEED_XIAO_ESP32S3`,
 *  `BOARD_HELTEC_WIFI_LORA32_V4_R2`, `BOARD_SPARKFUN_THING_PLUS_ESP32S3`,
 *  `BOARD_SPARKFUN_THING_PLUS_ESP32C5`,
 *  `BOARD_ADAFRUIT_ESP32S3_TFT_FEATHER`,
 *  `BOARD_ADAFRUIT_ESP32_FEATHER_V2`, or
 *  `BOARD_ADAFRUIT_ESP32S3_FEATHER_4MB_2MBPSRAM` is defined by the top-level
 *  CMakeLists.txt based on the `BOARD` variable (default `heltec_v2`). All
 *  module .c/.h files include this header and reference pins / features by
 *  the macros below — never by raw GPIO numbers.
 *
 *  Adding a new board: pick a `BOARD_*` macro name, add a branch below that
 *  sets the same `PIN_*` and `HAL_HAS_*` symbols, and extend the CMake board
 *  selector in `CMakeLists.txt`.
 *
 *  Feature flags every branch must define (no implicit defaults — explicit is
 *  better than surprising):
 *    HAL_HAS_OLED            display.c stubs out when 0
 *    HAL_HAS_TFT             V2.6.11: 1 = onboard SPI ST7789 color TFT
 *                            (Adafruit ESP32-S3 TFT Feather #5483, 240x135).
 *                            Mutually exclusive with HAL_HAS_OLED — display.c
 *                            gains a third top-level `#elif HAL_HAS_TFT`
 *                            branch that implements the same display_*()
 *                            API via display_tft.c, always resolving to
 *                            multi-page rotation (no I²C probe needed, the
 *                            panel is always present). 0 elsewhere.
 *    HAL_HAS_PSRAM           applog.c picks PSRAM vs internal DRAM ring
 *    HAL_HAS_NATIVE_USB      console routing in sdkconfig overlay
 *    HAL_HAS_VEXT_GATE       Heltec-style active-LOW power gate
 *    HAL_HAS_ANTENNA_SWITCH  external u.FL antenna toggle in /config UI
 *    HAL_HAS_I2C_PINOUT_SWITCH  V2.5.19: 1 = the board exposes a SECOND set of
 *                            I²C pads distinct from the onboard/STEMMA route, so
 *                            /config offers an `i2c_pinout` toggle to move the
 *                            primary bus there (QT Py ESP32-PICO: STEMMA IO22/IO19
 *                            ↔ SDA/SCL pads IO4/IO33). 0 elsewhere = no toggle.
 *    HAL_HAS_SPEAKER         speaker.c stubs out when 0 (small-board path)
 *    HAL_HAS_NEOPIXEL        neopixel.c init + tube-pulse hook gated on this
 *    HAL_HAS_ALS             V2.3.29: 1 = onboard ALS-PT19 ambient-light sensor
 *                            on PIN_ALS (FeatherS3-D's IO4); als.c is active
 *                            and /status renders an ambient-light row. 0 elsewhere
 *                            stubs the driver out.
 *    HAL_HAS_FUEL_GAUGE      V2.6.6: 1 = onboard MAX17048 battery fuel gauge
 *                            at I²C 0x36 on the primary bus (FeatherS3-D's
 *                            STEMMA1/IO8-9); fuel_gauge.c is active and
 *                            /status + MQTT + HA discovery gain battery
 *                            rows once the user ticks the /config "Battery
 *                            attached" checkbox (no auto-detect — see
 *                            fuel_gauge.h for why). 0 elsewhere stubs the
 *                            driver out entirely.
 *    HAL_HAS_LORAWAN         V2.6.23: 1 = onboard SX1262 LoRa radio with
 *                            LoRaWAN uplink support (lorawan.cpp — compiled
 *                            only where CONFIG_GEIGER_LORAWAN is set; the
 *                            flag gates call sites via lorawan.h's no-op
 *                            stubs). 0 elsewhere.
 *    HAL_LOG_RING_BYTES      applog ring size — varies by available memory
 *    HAL_LOG_SNAP_SCRATCH_BYTES  snapshot scratch for the wrap-corruption fix —
 *                            small (6 KB) on internal-DRAM-only boards, larger
 *                            (16 KB) when scratch lives in PSRAM (V2.3.24)
 */

#if defined(BOARD_HELTEC_V2)

    // V2.3.16-pre2: BOARD_NAME reports the build variant for the boot log.
    // BOARD_HELTEC_V2_4MB is set additionally on the 4 MB knock-off build via
    // CMakeLists.txt so the same hal.h branch (same hardware) emits two
    // different name strings depending on flash variant.
    #if BOARD_HELTEC_V2_4MB
        #define BOARD_NAME          "heltec_v2_4mb"
    #else
        #define BOARD_NAME          "heltec_v2"
    #endif
    #define HAL_HAS_OLED            1   // SSD1306 on shared I2C bus
    #define HAL_HAS_TFT             0
    #define HAL_HAS_PSRAM           0
    #define HAL_HAS_NATIVE_USB      0   // Console via CP2102 UART0
    #define HAL_HAS_VEXT_GATE       1   // GPIO 21 = active-LOW MOSFET on V2+ Heltec carriers
    #define HAL_HAS_ANTENNA_SWITCH  0   // PCB antenna only (no u.FL / no RF switch)
    #define HAL_HAS_I2C_PINOUT_SWITCH 0 // Single fixed I²C route
    #define HAL_HAS_SPEAKER         1   // Onboard piezo wired to PIN_SPEAKER_P/N
    #define HAL_HAS_NEOPIXEL        0   // No onboard NeoPixel
    #define HAL_HAS_SD_CARD         0   // No microSD slot on this board
    #define HAL_HAS_LORAWAN         0   // No LoRa radio
    #define HAL_HAS_ALS             0   // No onboard ambient-light sensor
    #define HAL_HAS_FUEL_GAUGE      0   // No onboard fuel gauge
    // V2.4.5: trimmed 60 KB → 45 KB. The 60 KB ring was the largest single
    // permanent heap allocation on the Heltec, sitting in internal SRAM
    // forever. Dropping 15 KB gives that back to free heap (~140 KB → ~155 KB
    // at boot) on top of the V2.4.5 mbedTLS dynamic-buffer transient win.
    // /log scrollback shrinks proportionally (~500 lines → ~380 lines) —
    // still plenty for diagnosing a TX cycle. FeatherS3-D / QT Py rings
    // are in PSRAM and not affected.
    #define HAL_LOG_RING_BYTES      (45 * 1024)  // Internal SRAM only — keep small
    // V2.4.6: /config form render buffer. 24 KB on Heltec (internal-DRAM-only,
    // tight; was 16 KB pre-fix). FeatherS3-D / QT Py / XIAO override to
    // 32 KB (PSRAM-backed, can afford the transient). V2.5.29: bumped 16→24 KB —
    // the openSenseMap-staging rows (V2.5.26) + a custom MQTT-TLS CA cert pushed
    // the worst-case page to ~16.9 KB, past the old 16 KB ceiling, truncating the
    // heltec /config tail (page is one malloc'd buffer; the PSRAM boards' 32 KB
    // hid it). 24 KB gives ~7 KB slack incl. a 2.4 KB CA cert; the transient
    // malloc is fine against the heltec's ~64 KB largest free block.
    // V2.6.24 caveat: with escape buffers now sized to html_esc's 6x worst
    // case, a maximally metacharacter-dense config could contribute ~23 KB
    // of escaped content — beyond ANY sane buffer here. That pathological
    // case is intentionally handled by config_get's loud truncate-and-clamp
    // (ESP_LOGE) rather than by growing this buffer; realistic configs
    // (real PEM is metachar-free) stay ~17 KB and fit with slack.
    #define HAL_CFG_FORM_BUF_SIZE   (24 * 1024)
    // V2.3.24: 4 KB snapshot scratch in internal DRAM. Min_free during FTPS
    // handshake on Heltec is already ~11 KB (free ~110 KB, peak transient
    // demand ~99 KB) so internal-DRAM headroom matters. Realistic writes
    // during a single FTPS upload window dropped to <500 B in V2.3.24 once
    // the FTPS-internal TLS handshake/shutdown chatter was downgraded to
    // DEBUG (cipher + NewSessionTicket + drain summary lines). 4 KB is now
    // ~8× the typical case, with concurrent TX-cycle overlap (~2-20 % of
    // uploads, ~500 B extra) still covered.
    #define HAL_LOG_SNAP_SCRATCH_BYTES  (4 * 1024)
    #define PIN_VEXT                21

    // Geiger / HV pins
    #define PIN_HV_FET_OUTPUT       23
    #define PIN_HV_CAP_FULL_INPUT   22
    #define PIN_GMC_COUNT_INPUT      2

    // Piezo pins
    #define PIN_SPEAKER_P           12
    #define PIN_SPEAKER_N            0

    // Onboard LED (lit during LED-tick if config flag set)
    #define PIN_LED_BUILTIN         25

    // I2C bus (shared between OLED and env sensor)
    #define PIN_I2C_SDA              4
    #define PIN_I2C_SCL             15

    // OLED reset line (only meaningful when HAL_HAS_OLED)
    #define PIN_OLED_RESET          16

#elif defined(BOARD_FEATHERS3_D)

    #define BOARD_NAME              "feathers3_d"
    // V2.3.28: SSD1309 2.42" external OLED on STEMMA1 or STEMMA2 at 0x3C
    // (Core Electronics CE09964 — 4-pin I²C, no dedicated reset line;
    // relies on chip POR). V2.3.29 added auto-detect across both buses
    // (display.c probes STEMMA1 first, falls through to STEMMA2). Probe
    // is silent — if no panel present, display.c stays dormant. Driver
    // is register-compatible with SSD1306.
    #define HAL_HAS_OLED            1   // External SSD1309 on STEMMA1 (optional — probe-detected)
    #define HAL_HAS_TFT             0
    #define HAL_HAS_ALS             1   // V2.3.29: onboard ALS-PT19 ambient-light sensor on PIN_ALS
    #define HAL_HAS_FUEL_GAUGE      1   // V2.6.6: onboard MAX17048 fuel gauge at I2C 0x36
    #define HAL_HAS_PSRAM           1   // 8 MB QSPI PSRAM
    #define HAL_HAS_NATIVE_USB      1   // Console via USB-Serial-JTAG (USB-C)
    #define HAL_HAS_VEXT_GATE       0   // No Vext gate — sensors powered via Qwiic 3V3
    #define HAL_HAS_ANTENNA_SWITCH  1   // u.FL external antenna + onboard SPDT RF switch
    #define HAL_HAS_I2C_PINOUT_SWITCH 0 // Single fixed I²C route (STEMMA1 + STEMMA2 are separate buses, not a pinout toggle)
    #define HAL_HAS_SPEAKER         1   // Piezo wired to A3/A4 of the Feather harness
    #define HAL_HAS_NEOPIXEL        0   // FeatherS3-D has an RGB LED on IO40 but we don't drive it
    #define HAL_HAS_SD_CARD         0   // No microSD slot on this board
    #define HAL_HAS_LORAWAN         0   // No LoRa radio
    #define HAL_LOG_RING_BYTES      (4 * 1024 * 1024)   // 4 MB of 8 MB PSRAM (V2.3.18)
    // V2.3.24: 16 KB snapshot scratch in PSRAM — negligible vs the 4 MB
    // PSRAM pool, and 2× the Heltec margin since the PSRAM cost is free.
    #define HAL_LOG_SNAP_SCRATCH_BYTES  (16 * 1024)
    // V2.4.6: 32 KB /config form render buffer. Doubles the Heltec budget
    // because PSRAM (and ample internal-DRAM heap) makes the transient cost
    // a non-issue here. Generous slack for future MQTT TLS PEM textareas
    // and any further config sections.
    #define HAL_CFG_FORM_BUF_SIZE   (32 * 1024)
    // PIN_ANTENNA_SELECT controls the onboard SPDT RF switch (NOT exposed as a
    // user header — this is an MCU↔switch trace internal to the FeatherS3-D).
    // Per the FeatherS3-D pinout silkscreen:
    //   IO41 = HIGH → u.FL external antenna connector
    //   IO41 = LOW  → onboard PCB chip antenna
    #define HAL_ANTENNA_SELECT_VERIFIED     1
    #define PIN_ANTENNA_SELECT             41
    #define ANTENNA_SELECT_HIGH_IS_EXTERNAL 1

    // Geiger / HV / speaker pins — Feather form-factor hole positions. A0–A5
    // are analog-capable on every Feather; on ESP32-family Feathers all six
    // also support digital I/O, PWM (LEDC) and interrupts. D5–D13 are the
    // standard Feather digital-pin positions on the OPPOSITE long edge.
    //
    // V2.3.27 PCB harness moved HV_FET / SPEAKER off A2..A4 onto A5 + D9/D10
    // to free the contiguous A2..A4 trio for future analog use. Side effects
    // worth knowing about:
    //   * D10 = IO3 is an ESP32-S3 boot strap (JTAG vs USB-Serial-JTAG select).
    //     Internal pull-up holds it HIGH at boot → default USB-Serial-JTAG
    //     mode. We only drive it post-boot (speaker is initialised in
    //     speaker_setup() AFTER config + WiFi bring-up), so the strap reads
    //     correctly. A piezo at hi-Z does not pull it down.
    //   * A5 = IO5 was previously listed as a reserved future-HWTESTPIN slot
    //     in the wiring harness; that reservation is dropped in V2.3.27.
    //
    // Position    FeatherS3-D    Adafruit ESP32-S3 Feather (#5323) — for ref
    // --------    -----------    ------------------------------------------
    // A0          GPIO 17        GPIO 18
    // A1          GPIO 18        GPIO 17
    // A5          GPIO  5        GPIO  8
    // D9          GPIO  1        GPIO  6
    // D10         GPIO  3        GPIO  5
    #define PIN_HV_CAP_FULL_INPUT   17   // A0   — comparator interrupt (digital)
    #define PIN_GMC_COUNT_INPUT     18   // A1   — Geiger pulse interrupt
    #define PIN_HV_FET_OUTPUT        5   // A5   — HV MOSFET gate (LEDC PWM via gptimer)

    // Piezo pins
    #define PIN_SPEAKER_P            3   // D10  — LEDC PWM. IO3 is a boot strap (see note above)
    #define PIN_SPEAKER_N            1   // D9   — digital low

    // Onboard Blue LED (FeatherS3-D internal — IO13). Drives during LED-tick
    // if the config flag is set; harmless if not.
    #define PIN_LED_BUILTIN         13

    // V2.6.6: VBUS-present detect (digital only — ESP32-S3 ADCs only cover
    // GPIO 1-20). High = USB 5V present. Read alongside the fuel gauge so
    // the power-supply log line can report *why* VCELL might be sitting at
    // a plausible-looking voltage with no LiPo attached (charger IC output
    // floats near its ~4.2V regulation setpoint when unloaded, which is
    // indistinguishable from a real battery by voltage alone).
    #define PIN_VBUS_DETECT         34

    // I2C bus = STEMMA QT / Qwiic connector (the env sensor breakout plugs in
    // here directly via a Qwiic cable; no I2C wiring lands on the PCB).
    #define PIN_I2C_SDA              8
    #define PIN_I2C_SCL              9

    // V2.3.29: onboard ALS-PT19 ambient-light sensor (analog phototransistor).
    // Wired to GPIO 4 = ADC1_CH3. ADC1 is independent of WiFi (only ADC2
    // has the front-end coexistence conflict). See als.c for the read path.
    #define PIN_ALS                  4

    // PIN_OLED_RESET intentionally undefined — the external SSD1309 breakout
    // (4-pin I²C) has no reset line; display.c skips the reset pulse when
    // PIN_OLED_RESET is not defined.

    // RESERVED pins on FeatherS3-D — never repurpose these in firmware:
    //   IO0  strap (BOOT button)         IO19/20  native USB D-/D+
    //   IO34 VBUS-present detect         IO45/46  strap pins (not exposed)
    //   IO40 onboard RGB LED (NeoPixel)  IO4      ambient light sensor
    //   IO2  fuel gauge interrupt        IO41     antenna SPDT select
    //                                             (used by PIN_ANTENNA_SELECT above)
    //   IO8/9 STEMMA1 connector + MAX17048 fuel gauge bus (env / PM / noise sensors)
    //
    // PINS WE NOW DRIVE (despite reservation / strap status):
    //   IO3  strap (JTAG vs USB-Serial-JTAG select). Used as PIN_SPEAKER_P;
    //        safe because the speaker driver is hi-Z at boot — strap reads
    //        correctly during ROM bootloader.
    //   IO39 LDO2 enable. V2.3.28 introduced eager LDO2 enable for STEMMA2;
    //        V2.3.29 made it lazy + sheddable — i2c_bus.c only enables LDO2
    //        when a consumer (display or sensor) actually requests the
    //        secondary bus, and tears it down at end-of-init if nothing is
    //        bound there (saves ~5–10 mA quiescent + NeoPixel idle). Default
    //        is hi-Z + internal pull-down → LDO2 OFF. Side effect when on:
    //        powers the onboard NeoPixel on IO40 (we leave that data line
    //        floating; WS2812 POR keeps it dark).
    //   IO15/IO16 STEMMA2 I²C bus — second I²C controller (I2C_NUM_1) on
    //        FeatherS3-D, powered by LDO2 above. Currently used only by the
    //        external OLED (display.c brings up the bus); see deferred memory
    //        `project_stemma2_software_enable_deferred.md` for the broader
    //        per-driver refactor needed to host SENSORS on this bus.

#elif defined(BOARD_ADAFRUIT_QTPY_ESP32_PICO)

    // Adafruit QT Py ESP32-PICO (PID 5395). ESP32-PICO-V3-02 SiP — original
    // ESP32 LX6 dual-core (NOT S3) with in-package 8 MB flash + 2 MB PSRAM.
    // 11 castellated GPIO pads + STEMMA QT connector + onboard NeoPixel.
    //
    // Pin budget: too tight to keep the speaker (only 4 of A0..A5 exist on
    // the QT Py form factor, A3 is a strapping pin we'd rather avoid for
    // outputs). HAL_HAS_SPEAKER=0 stubs speaker.c entirely. Geiger uses A0
    // / A1 / SCK — V2.4.25 moved HV_FET off A2 onto SCK to enable a single
    // shared PCB design that hosts EITHER this board OR the Seeed XIAO
    // ESP32-S3. A2 strap-clashes on the XIAO (GPIO 3 = USB-Serial-JTAG
    // selector); SCK is strap-free on both boards. See the BOARD_SEEED_
    // XIAO_ESP32S3 block below for the matching XIAO pin map. Env sensor
    // plugs into the STEMMA QT connector (no pad cost — uses the secondary
    // I²C bus IO22/IO19 routed to the connector). Visible feedback comes
    // from the onboard NeoPixel via neopixel.c (HAL_HAS_NEOPIXEL=1)
    // flashing red on each Geiger pulse.
    //
    // GPIO numbers verified against
    // github.com/espressif/arduino-esp32 variants/adafruit_qtpy_esp32/pins_arduino.h
    #define BOARD_NAME              "adafruit_qtpy_esp32_pico"
    // V2.3.29: like FeatherS3-D, no onboard display BUT supports an external
    // SSD1306/SSD1309 OLED or SparkFun SerLCD via the STEMMA QT connector
    // (auto-detected at boot). Display lives on the SAME I²C bus as the env
    // sensor (IO22 SDA / IO19 SCL) — no LDO2 / second-bus complexity unlike
    // FeatherS3-D's STEMMA2.
    #define HAL_HAS_OLED            1   // External display via STEMMA QT (optional — probe-detected)
    #define HAL_HAS_TFT             0
    #define HAL_HAS_PSRAM           1   // 2 MB in-package SiP PSRAM
    #define HAL_HAS_NATIVE_USB      0   // USB-C via CH9102F or CP2102N UART bridge (NOT native — original ESP32 has no USB-OTG)
    #define HAL_HAS_VEXT_GATE       0   // No power gate — STEMMA QT bus always powered
    #define HAL_HAS_ANTENNA_SWITCH  0   // PCB antenna only (no u.FL on this board)
    #define HAL_HAS_I2C_PINOUT_SWITCH 1 // V2.5.19: STEMMA QT (IO22/19) ↔ SDA/SCL pads (IO4/33) via i2c_pinout
    #define HAL_HAS_SPEAKER         0   // Dropped — pin budget + small-board context
    #define HAL_HAS_NEOPIXEL        1   // Onboard WS2812 — flashes blue on Geiger pulse (V2.6.22, was red)
    #define HAL_HAS_SD_CARD         0   // No microSD slot on this board
    #define HAL_HAS_LORAWAN         0   // No LoRa radio
    // V2.4.8 hardcoded HAL_MULTIPAGE_ROTATION=0 here (radiation-only) to match
    // a QT Py + Adafruit 326 deployment. V2.4.9 removed HAL_MULTIPAGE_ROTATION
    // entirely — page-layout selection is now runtime via `display_mode` in
    // /config (auto / radiation / rotation). Auto on QT Py + SSD1306 still
    // picks radiation (small panel → single page).
    #define HAL_HAS_ALS             0   // No onboard ambient-light sensor
    #define HAL_HAS_FUEL_GAUGE      0   // No onboard fuel gauge
    #define HAL_LOG_RING_BYTES      (1 * 1024 * 1024)   // 1 MB of 2 MB PSRAM (50% headroom)
    // V2.3.24: 16 KB snapshot scratch in PSRAM — same generous margin as
    // FeatherS3-D since the PSRAM cost is negligible.
    #define HAL_LOG_SNAP_SCRATCH_BYTES  (16 * 1024)
    // V2.4.6: 32 KB /config form render buffer. PSRAM-backed board — same
    // generous bump as FeatherS3-D.
    #define HAL_CFG_FORM_BUF_SIZE   (32 * 1024)

    // Geiger / HV pins — wired to QT Py A0 / A1 / SCK castellated pads.
    // V2.4.25 moved HV_FET from A2 to SCK so this PCB design ALSO drops
    // onto the Seeed XIAO ESP32-S3 (same form factor; A0/A1/SCK are the
    // only three pads strap-free on BOTH boards). HV_FET on SCK lives on
    // the opposite long edge of the board from the sensitive GMC_COUNT
    // input — physical separation reduces switching-noise coupling into
    // the pulse pickup. A2 + A3 are now free for future analog use.
    //
    // Position    QT Py ESP32-PICO    Notes
    // --------    ----------------    ------------------------------------
    // A0          GPIO 26             RTC, DAC2, ADC2 — interrupt-capable
    // A1          GPIO 25             RTC, DAC1, ADC2 — interrupt-capable
    // SCK         GPIO 14             ADC2, touch — LEDC PWM capable, non-strap
    #define PIN_HV_CAP_FULL_INPUT   26   // A0  — comparator interrupt (digital)
    #define PIN_GMC_COUNT_INPUT     25   // A1  — Geiger pulse interrupt
    #define PIN_HV_FET_OUTPUT       14   // SCK — HV MOSFET gate (LEDC PWM via gptimer)

    // No piezo — HAL_HAS_SPEAKER=0 above stubs the entire speaker path.
    // PIN_SPEAKER_P / PIN_SPEAKER_N intentionally undefined.

    // Onboard NeoPixel — single WS2812 with software-gated power. neopixel.c
    // drives a brief red flash from a FreeRTOS task notified by the tube
    // pulse ISR. NEOPIXEL_POWER must be driven HIGH to power the NeoPixel.
    #define PIN_NEOPIXEL_DATA        5   // WS2812 data line (also a strap pin — default pull-up; only driven post-boot)
    #define PIN_NEOPIXEL_POWER       8   // HIGH = NeoPixel powered (VDD_SDIO power domain — fine post-boot)

    // No "user" LED separate from the NeoPixel. PIN_LED_BUILTIN intentionally
    // undefined; the led_tick config flag has no effect on this board.

    // I2C bus = STEMMA QT connector. Adafruit routes the secondary I²C bus
    // (IO22 SDA / IO19 SCL) to the Qwiic connector — env sensor breakout
    // plugs in via a Qwiic cable, no GPIO pad cost. (The primary bus on the
    // SDA/SCL pads = IO4/IO33 is left free for user expansion.)
    #define PIN_I2C_SDA             22
    #define PIN_I2C_SCL             19

    // V2.5.19: ALTERNATE I²C route on the broken-out SDA/SCL castellated pads
    // (the "primary" bus IO4/IO33 previously left free for user expansion).
    // When the `i2c_pinout` config bool is set, i2c_bus.c brings the primary
    // master bus up here INSTEAD of the STEMMA QT pins above — for wiring a
    // sensor/display straight to the header pads rather than via a Qwiic cable.
    // Reboot-required (the bus is created once at first sensor probe). The
    // STEMMA QT route (IO22/IO19) stays the default.
    #define PIN_I2C_SDA_ALT          4   // SDA castellated pad
    #define PIN_I2C_SCL_ALT         33   // SCL castellated pad

    // RESERVED pins — do not repurpose:
    //   IO0   BOOT button (strap)
    //   IO5   NeoPixel data (strap; default pull-up — fine if only driven post-boot)
    //   IO6   internal flash (CMD)         IO9   internal PSRAM (SD2)
    //   IO10  internal PSRAM (SD3)         IO11  internal flash (CMD2)
    //   IO12  MTDI strap (DANGEROUS — pulling HIGH at boot bricks flash boot until power-cycle)
    //   IO15  MTDO strap (silences boot log if LOW at boot — annoying but recoverable)

#elif defined(BOARD_SEEED_XIAO_ESP32S3)

    // Seeed Studio XIAO ESP32-S3 (SKU 113991054). ESP32-S3 LX7 dual-core with
    // 8 MB QSPI flash + 8 MB OPI (octal-mode) PSRAM. Tiny 21×17.5 mm form
    // factor (same as Adafruit QT Py), USB-C native, 11 castellated GPIO
    // pads + Type-C, no onboard sensors / display / NeoPixel.
    //
    // V2.4.25: Geiger pins now match QT Py ESP32-PICO so a single shared
    // PCB design can host EITHER board. The chosen trio (A0 / A1 / SCK)
    // is the only set of three pads that is strap-free on BOTH boards —
    // see the BOARD_ADAFRUIT_QTPY_ESP32_PICO block above for the matching
    // pin map and the strap analysis behind the choice.
    //
    // The XIAO is also fine as an I²C-only sensor host (no Geiger tube
    // wired) — the firmware boots, joins WiFi, probes the I²C bus on
    // D4/D5 for any env / PM / noise / display sensor, and reports CPM=0
    // throughout. Disable the Geiger upload targets in /config to avoid
    // posting CPM=0 readings to public servers in that configuration.
    //
    // GPIO numbers verified against the official Seeed wiki:
    //   https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/
    #define BOARD_NAME              "seeed_xiao_esp32s3"
    #define HAL_HAS_OLED            1   // External OLED via D4/D5 I²C (probe-detected — dormant if absent)
    #define HAL_HAS_TFT             0
    #define HAL_HAS_PSRAM           1   // 8 MB OPI PSRAM (octal mode — see sdkconfig overlay)
    #define HAL_HAS_NATIVE_USB      1   // Console via USB-Serial-JTAG (USB-C)
    #define HAL_HAS_VEXT_GATE       0   // No power gate — 3V3 / 5V always live
    #define HAL_HAS_ANTENNA_SWITCH  0   // PCB antenna only (no u.FL on standard XIAO ESP32-S3)
    #define HAL_HAS_I2C_PINOUT_SWITCH 0 // Single fixed I²C route (D4/D5)
    #define HAL_HAS_SPEAKER         0   // Not wired — no spare pad on the shared-PCB footprint
    #define HAL_HAS_NEOPIXEL        0   // No onboard NeoPixel
    #define HAL_HAS_SD_CARD         0   // No microSD slot on this board
    #define HAL_HAS_LORAWAN         0   // No LoRa radio
    #define HAL_HAS_ALS             0   // No onboard ambient-light sensor
    #define HAL_HAS_FUEL_GAUGE      0   // No onboard fuel gauge
    #define HAL_LOG_RING_BYTES      (4 * 1024 * 1024)   // 4 MB of 8 MB PSRAM (matches FeatherS3-D budget)
    // V2.3.24: 16 KB snapshot scratch in PSRAM — same generous margin as
    // FeatherS3-D / QT Py since the PSRAM cost is negligible.
    #define HAL_LOG_SNAP_SCRATCH_BYTES  (16 * 1024)
    // V2.4.6: 32 KB /config form render buffer — PSRAM-backed board.
    #define HAL_CFG_FORM_BUF_SIZE   (32 * 1024)

    // Geiger / HV pins — wired to XIAO D0 / D1 / D8 castellated pads, which
    // physically align with A0 / A1 / SCK on the QT Py PICO footprint.
    // V2.4.25 chose this trio specifically because:
    //   * D0 (GPIO 1) and D1 (GPIO 2) are strap-free on the S3 (avoids the
    //     GPIO 3 USB-Serial-JTAG selector that lurks under D2)
    //   * D8 (GPIO 7) is also strap-free AND lives on the opposite long
    //     edge from D0/D1 — physical separation from the switching HV_FET
    //     output keeps GMC pulse pickup quieter
    //
    // Position    XIAO pad    GPIO    Notes
    // --------    --------    ----    -----------------------------------
    // A0          D0          1       ADC1_CH0, TOUCH1, interrupt-capable
    // A1          D1          2       ADC1_CH1, TOUCH2, interrupt-capable
    // SCK         D8          7       ADC1_CH6, TOUCH7, LEDC PWM-capable
    #define PIN_HV_CAP_FULL_INPUT    1   // D0  ≡ A0 — comparator interrupt
    #define PIN_GMC_COUNT_INPUT      2   // D1  ≡ A1 — Geiger pulse interrupt
    #define PIN_HV_FET_OUTPUT        7   // D8  ≡ SCK — HV MOSFET gate (LEDC PWM)

    // No piezo, no NeoPixel — those stubs above. But the XIAO ESP32-S3 DOES
    // have a single onboard user LED on GPIO21, and it is ACTIVE-LOW (drive the
    // pin LOW to light it — confirmed by Seeed's own ESPHome config which sets
    // `pin: GPIO21 inverted: true`). led.c flashes it per GM pulse when led_tick
    // is enabled (V2.5.19). PIN_SPEAKER_P / PIN_SPEAKER_N stay undefined
    // (HAL_HAS_SPEAKER=0 stubs speaker.c, so led.c — not speaker.c — owns the
    // pulse-tick LED here).
    #define PIN_LED_BUILTIN         21   // onboard user LED (active-LOW)
    #define HAL_LED_ACTIVE_LOW       1   // GPIO21: LOW = on, HIGH = off

    // I²C bus — default XIAO pinout: D4 = SDA = GPIO5, D5 = SCL = GPIO6.
    // External sensor breakouts (env / PM / noise / display) attach here via
    // the 4-pin header (3V3 / GND / SDA / SCL) or jumper wires. The XIAO has
    // no STEMMA QT / Qwiic connector — wire your own.
    #define PIN_I2C_SDA              5
    #define PIN_I2C_SCL              6

    // RESERVED / strap pins — never repurpose:
    //   GPIO0   BOOT button (strap)
    //   GPIO45  flash voltage select (strap)
    //   GPIO46  boot mode (strap)
    //   GPIO19/20  native USB D-/D+ (USB-Serial-JTAG)
    //   GPIO26-32, 33-37  internal flash / PSRAM (not broken out)
    //   GPIO43/44  default UART0 TX/RX (D6/D7 on header — re-purposable
    //              post-boot but UART0 console is disabled in our sdkconfig
    //              in favour of USB-Serial-JTAG, so they're free)

#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)

    // Heltec WiFi LoRa 32 V4 — base/R2 variant (ESP32-S3R2 chip, 2 MB
    // in-package quad PSRAM). NOT the V4-R8 variant (ESP32-S3R8, 8 MB octal
    // PSRAM, different GPIOs for Vext/VGNSS/LED/PA_CTX) — confirmed via
    // heltec.org's own datasheet and the HelTecAutomation/Heltec_ESP32
    // Arduino library, which defines WIFI_LORA_32_V4 and WIFI_LORA_32_V4_R8
    // as distinct targets. "_r2" in the board name permanently disambiguates
    // from a possible future R8 port.
    //
    // Third-party PCB, not our own design: a separate team populated the
    // standard Multigeiger V2 mainboard
    // (Multigeiger_V1.9/Hardware/Eagle/projects/Multigeiger_V2/) with this
    // Heltec module instead of the Heltec V2. Pin map sourced from
    // Multigeiger_V1.9/Pin-Matrix_Heltec_MG_neu-V1.9.ods/.pdf, cross-validated
    // pin-for-pin against Heltec's V4 datasheet §2.2.1/2.2.2/2.2.3 — zero
    // contradictions. Full rationale (incl. the Vext/GPIO2/GPIO46
    // non-conflict analysis and the future-LoRaWAN pin reservations) is
    // captured in the per-pin comments throughout this block below.
    #define BOARD_NAME              "heltec_wifi_lora32_v4_r2"
    #define HAL_HAS_OLED              1   // SSD1315 on a dedicated I2C_NUM_1 bus (GPIO17/18) — see PIN_OLED_SDA/SCL below
    #define HAL_HAS_TFT               0
    #define HAL_HAS_ALS               0   // No onboard ambient-light sensor
    #define HAL_HAS_FUEL_GAUGE        0   // No onboard fuel gauge
    #define HAL_HAS_PSRAM             1   // 2 MB in-package, quad
    #define HAL_HAS_NATIVE_USB        0   // Console via USB-UART bridge on GPIO43/44 (TX/RX), not native USB
    // V2.6.20: the datasheet's §3.3 "Power Output" was read at port time
    // (V2.6.7) as GPIO36 (Vext_Ctrl) only gating the external "Ve" header,
    // not the OLED — never bench-verified, since no physical unit existed
    // at port time. First real hardware (2026-07-15) instead showed the
    // OLED's I²C bus (I2C_NUM_1, GPIO17/18) timing out on every
    // transaction — the classic unpowered-target signature — and driving
    // Vext_Ctrl LOW before the OLED bus comes up (see PIN_VEXT below and
    // i2c_bus_get_secondary()'s BOARD_HELTEC_WIFI_LORA32_V4_R2 branch)
    // confirmed it: the display now enumerates (SSD1315 @ 0x3C) and shows
    // correctly. The datasheet reading was wrong; this board's Vext_Ctrl
    // behaves like Heltec V3's, gating the OLED rail.
    //
    // Staying at HAL_HAS_VEXT_GATE=0 rather than folding this into the
    // flag: that flag's one existing code path lives in
    // i2c_bus_get_primary() (see i2c_bus.c), which on THIS board is the
    // external sensor header — a different bus from the OLED's. Making the
    // flag drive Vext ahead of the wrong bus wouldn't be right either;
    // the fix belongs where it now lives, scoped to the OLED bus in
    // i2c_bus_get_secondary(). PIN_VEXT is driven directly there.
    #define HAL_HAS_VEXT_GATE         0
    #define HAL_HAS_ANTENNA_SWITCH    0   // PCB antenna only (no u.FL / no RF switch)
    #define HAL_HAS_I2C_PINOUT_SWITCH 0   // Single fixed route per bus
    // V2.6.7: disabled. PIN_SPEAKER_P (GPIO26) is the ESP32-S3R2/RH2's
    // internal PSRAM chip-select (SPICS1) per Espressif's ESP32-S3 datasheet
    // v2.2 §2.3.5 Table 2-9 (Priority 4, "SPI0/1 interface connected to the
    // in-package flash and PSRAM") — confirmed hardwired to the piezo via
    // KiCad/Eagle trace review of the Multigeiger V2 mainboard, and
    // independently corroborated by Heltec's own pins_arduino.h for V4/V4_R8
    // (and V3/ESP32-S3FN8, which hits the same restriction for its in-package
    // flash) never exposing GPIO26-37. Driving it as a PWM tone output risks
    // bus contention with live SPI0 flash traffic → crash/hang correlated
    // with speaker use. Not fixable in firmware (the PSRAM chip-select bond
    // is fixed at chip fabrication); see
    // docs/superpowers/reviews/2026-07-08-v2.6.7-max-review/ for the full
    // writeup.
    //
    // CORRECTION (post-release): the review's suggested firmware-side
    // replacement pins, GPIO34 and GPIO37, are themselves NOT free — Heltec's
    // own official V4 pin-mapping document names GPIO34 = VGNSS_Ctrl and
    // GPIO37 = ADC_Ctrl (confirmed against the Multigeiger Pin-Matrix too,
    // which independently corroborates GPIO34=VGNSS_Ctrl — that one was even
    // in this board's own §1 V4-vs-V4_R8 comparison table in the design spec
    // the whole time, just never carried into this reserved-pin list, which
    // is how it slipped past review). Of the whole GPIO33-37 octal-PSRAM
    // extension range, only GPIO33 has no named Heltec control function
    // (hence safe to repurpose as PIN_HV_FET_OUTPUT below); 34/36/37 are
    // Heltec-reserved control lines and 35 is already correctly used for its
    // own intended purpose (the onboard LED). No verified-free replacement
    // GPIO is known for the speaker — see
    // [[project_heltec_wifi_lora32_v4_r2_board_port]] memory for the
    // hardware-side next step (Pin-Matrix header audit + continuity check).
    #define HAL_HAS_SPEAKER           0   // Piezo present but GPIO26 (P) collides with PSRAM SPICS1 — disabled
    #define HAL_HAS_NEOPIXEL          0   // No onboard NeoPixel
    #define HAL_HAS_SD_CARD           0   // No microSD slot on this board
    #define HAL_HAS_LORAWAN           1   // Onboard SX1262 LoRa radio — see PIN_LORA_* below

    // Ring/scratch/form-buffer sizes modeled on BOARD_ADAFRUIT_QTPY_ESP32_PICO
    // (closest analog: 2 MB in-package PSRAM, not FeatherS3-D's 8 MB external).
    #define HAL_LOG_RING_BYTES      (1 * 1024 * 1024)   // 1 MB of 2 MB PSRAM (50% headroom)
    #define HAL_LOG_SNAP_SCRATCH_BYTES  (16 * 1024)
    #define HAL_CFG_FORM_BUF_SIZE   (32 * 1024)

    // Geiger / HV / speaker pins — from the Multigeiger mainboard's J2/J3
    // wiring intent (Pin-Matrix_Heltec_MG_neu-V1.9.ods/.pdf), cross-validated
    // against the V4 datasheet.
    #define PIN_HV_FET_OUTPUT       33   // J2 pin12 — HV MOSFET gate
    #define PIN_HV_CAP_FULL_INPUT    2   // J3 pin13 — ADC1_CH1/TOUCH2, plain
    // GPIO on base V4 (spec §5 confirmed no conflict with LoRa's FEM_PA on
    // this SKU — but that check predates a later finding: GPIO2 is ALSO
    // Heltec's FEM_EN, per Table 2.2.2 row 13, a second/distinct LoRa
    // front-end-enable signal from GPIO7's VFEM_Control). Same double-booked
    // shape as GPIO46/DIP1 vs FEM_PA below — see PIN_LORA_FEM_EN's comment:
    // V2.6.23's HAL_HAS_LORAWAN=1 makes this pin live on reworked-hardware
    // boards only (g_cfg.lorawan_fem_en); on unmodified boards this
    // comparator input stays the sole driver and PIN_LORA_FEM_EN is never
    // asserted.
    //
    // FUTURE HARDWARE REWORK (not yet built, no PCB revision exists yet):
    // hardware team has been asked to move this input to GPIO6 (J3 pin17,
    // currently wired only to the dead "DIP3" switch, see reserved-pin list
    // below) once that switch is disconnected — GPIO6 has no Heltec-side
    // claim and, unlike GPIO45, is NOT one of the ESP32-S3's four strapping
    // pins (GPIO0/3/45/46 only), so it carries no boot-strap risk either.
    // Same rule as the speaker rework: this is a copper change, existing
    // boards keep GPIO2 wired as-is, and firmware needs a way to tell old-
    // wiring boards from new-wiring boards before this #define can change.
    // IO3 is an ESP32-S3 strapping pin, same strap BOARD_FEATHERS3_D reuses
    // for PIN_SPEAKER_P below — but that reuse is safe ONLY because the
    // speaker driver stays hi-Z until code drives it post-boot. This pin is
    // different: it's an always-connected external input from the tube
    // pulse-conditioning circuit, whose level during the ROM bootloader's
    // strap-sampling window is NOT under firmware control.
    //
    // V2.6.7: narrowed after re-reading the Espressif ESP32-S3 datasheet
    // v2.2 ch.3 "Boot Configurations" directly. GPIO3 controls ONLY "JTAG
    // signal source" (which physical path a hardware JTAG debugger would
    // use) — it has no role in the SPI-boot-vs-download-mode decision
    // (that's GPIO0 + GPIO46, per Table 3-3) and no effect on this board's
    // console/flashing path, which runs over a separate USB-UART bridge
    // chip on GPIO43/44, not native USB-Serial-JTAG (see
    // HAL_HAS_NATIVE_USB=0 above). The datasheet also states the eFuse bits
    // that make the chip act on this strap at all (EFUSE_DIS_PAD_JTAG,
    // EFUSE_DIS_USB_JTAG, EFUSE_STRAP_JTAG_SEL) default to 0/not-burnt on
    // every stock chip, so in practice GPIO3's boot-time level here is
    // inert unless someone has deliberately burnt those fuses. Left as a
    // documented strap (not a real risk) rather than a bench-verify item.
    #define PIN_GMC_COUNT_INPUT      3   // J3 pin14 — Geiger tube pulse

    // Piezo pins. The pin-matrix marks BOTH GPIO26 (J2 pin15) and GPIO5 (J3
    // pin16) as "SPK" without distinguishing P/N. PIN_SPEAKER_P/N are
    // intentionally left undefined — HAL_HAS_SPEAKER=0 above stubs the
    // entire speaker path, since GPIO26 (J2 pin15) collides with this
    // chip's internal PSRAM chip-select (see the HAL_HAS_SPEAKER comment).
    //
    // FUTURE HARDWARE REWORK (not yet built, no PCB revision exists yet):
    // GPIO4 (J3 pin15) is the first fully-verified-free GPIO found on this
    // board — no Heltec-named "Connected" function (datasheet Table 2.2.2
    // row 15 lists only ADC1_CH3/TOUCH4) AND the Multigeiger Pin-Matrix
    // itself marks this same header pin "RESERVE" (unwired on the
    // mainboard). Hardware team has been asked to reroute the speaker's P
    // leg from GPIO26 to GPIO4 on a future PCB revision, keeping the N leg
    // on GPIO5 (J3 pin16) unchanged — GPIO5 has no conflict and is already
    // connected. This is a copper change, not a firmware one: existing
    // fabricated boards still have the P leg hard-traced to GPIO26 and
    // cannot be fixed by flipping HAL_HAS_SPEAKER. When the reworked PCB
    // exists, firmware will need a way to distinguish old-wiring boards
    // from new-wiring boards (new BOARD variant, or a build-time/runtime
    // hardware-revision flag) before PIN_SPEAKER_P can safely become 4 —
    // do not just flip HAL_HAS_SPEAKER=1 with GPIO4 for this board target
    // as it stands, or every board already in the field breaks.

    // Onboard LED. GPIO35, ACTIVE-HIGH — confirmed via the independent
    // DN9KGB/rMesh project's hal_HELTEC_WiFi_LoRa_32_V4.c (whose other pin
    // values match this exact base-V4 pinout): LOW at boot = off, HIGH =
    // on. Not Heltec's own datasheet text, so bench-verify polarity on
    // first flash (same standard applied to the XIAO ESP32-S3's LED).
    // HAL_LED_ACTIVE_LOW omitted (led.c defaults to active-high, matching
    // this pin's confirmed polarity). V2.6.7: HAL_HAS_SPEAKER=0 means
    // speaker.c no longer owns this pin — led.c's own driver compiles IN
    // (per led.c's `!HAL_HAS_SPEAKER && !HAL_HAS_NEOPIXEL` gate) and drives
    // the pulse-tick LED directly.
    #define PIN_LED_BUILTIN         35   // J2 pin10

    // I2C bus (env sensor — the bus every sensor driver targets by default).
    // From the Multigeiger mainboard's J2 pins 13/14, NOT the OLED's bus
    // (see PIN_OLED_SDA/SCL below and spec §3 for why this board needs two
    // separate I²C controllers).
    #define PIN_I2C_SDA             48   // J2 pin14
    #define PIN_I2C_SCL             47   // J2 pin13

    // OLED bus — fixed module-internal bus, NOT on the J2/J3 header at all.
    // Dedicated permanently to the onboard SSD1315; i2c_bus.c brings this up
    // as I2C_NUM_1 directly (no probe/fallback — this board always has the
    // OLED, unlike FeatherS3-D's optional STEMMA2 plug-in). See spec §3.
    #define PIN_OLED_SDA            17
    #define PIN_OLED_SCL            18
    #define PIN_OLED_RESET          21   // J2 pin16
    // V2.6.20: bench-confirmed active-LOW MOSFET gating the OLED rail (see
    // HAL_HAS_VEXT_GATE comment above). Driven LOW by
    // i2c_bus_get_secondary() before the OLED bus init, mirroring Heltec
    // V2's PIN_VEXT pattern. Side effect: the same MOSFET also feeds the
    // external "Ve" header (datasheet §3.3 "Power Output", 500 mA
    // peripheral rail), so anything wired to Ve is energized from boot on
    // every unit — budget for that on battery deployments.
    #define PIN_VEXT                36   // Vext_Ctrl — active-LOW MOSFET, gates OLED rail + Ve header (bench-confirmed)

    // V2.6.23: SX1262 LoRa radio — module-internal dedicated SPI bus, pins
    // promoted from the reserved-comment block below to real defines now
    // that lorawan.cpp drives them. Sources: Heltec V4 pin-mapping diagram
    // (verified against WiFi_LoRa_32_V4.3.1_Datasheet.pdf) + independent
    // bring-up confirmation in jgromes/RadioLib discussion #1665.
    #define PIN_LORA_NSS             8
    #define PIN_LORA_SCK             9
    #define PIN_LORA_MOSI           10
    #define PIN_LORA_MISO           11
    #define PIN_LORA_RST            12
    #define PIN_LORA_BUSY           13
    #define PIN_LORA_DIO1           14
    // VFEM_Control (J3 pin18): switches power to the LoRa RF front end.
    // Driven HIGH by lorawan_setup() before radio init (V1.9 precedent).
    #define PIN_LORA_VFEM            7
    // FEM_EN = GC1109 FEM enable/CSD (J3 pin13). DOUBLE-BOOKED with this
    // board's HV cap-full comparator input (PIN_HV_CAP_FULL_INPUT above).
    // NEVER driven unless g_cfg.lorawan_fem_en is set (reworked-hardware
    // boards only — driving it on current wiring contends with the live
    // comparator output). Per RadioLib discussion #1665 the GC1109 must be
    // enabled for RX; on current wiring the comparator wiggles this line,
    // so LoRaWAN RX reliability on unmodified boards is a bench question.
    #define PIN_LORA_FEM_EN          2
    // FEM_PA (Heltec pin-map name; GC1109 gain-stage select for the 28 dBm
    // path). DOUBLE-BOOKED with the mainboard's DIP1 switch. NEVER driven
    // unless g_cfg.lorawan_high_power is set (reworked boards only).
    #define PIN_LORA_FEM_PA         46

    // FORMERLY "reserved for future LoRaWAN/Meshtastic work" (hardware-
    // reservation only, spec §6) — as of V2.6.23 the SX1262 SPI bus and
    // VFEM_Control are LIVE via the PIN_LORA_* defines above
    // (HAL_HAS_LORAWAN=1, lorawan.cpp). Left below for the per-pin
    // rationale; GPIO2/GPIO46 still need the double-booked-conflict read
    // since lorawan.cpp only drives them conditionally:
    //   GPIO8/9/10/11/12/13/14  SX1262 LoRa radio's dedicated internal SPI
    //                 bus: GPIO8=LoRa_NSS, GPIO9=LoRa_SCK, GPIO10=LoRa_MOSI,
    //                 GPIO11=LoRa_MISO, GPIO12=LoRa_RST, GPIO13=LoRa_BUSY,
    //                 GPIO14=DIO1 (Heltec's own V4 pin-mapping diagram,
    //                 verified against WiFi_LoRa_32_V4.3.1_Datasheet.pdf).
    //                 Never appears in the J2/J3 header tables — nothing
    //                 else could claim these pins anyway.
    //   GPIO7         VFEM_Control (LoRa front-end enable, J3 pin18 per the
    //                 datasheet's own Table 2.2.2). Now driven HIGH by
    //                 lorawan_setup() — see PIN_LORA_VFEM above.
    //   GPIO2         FEM_EN (LoRa front-end enable, J3 pin13 per Table
    //                 2.2.2 — a second, distinct front-end control signal
    //                 from GPIO7/VFEM_Control). See PIN_LORA_FEM_EN above:
    //                 double-booked with PIN_HV_CAP_FULL_INPUT, only driven
    //                 on reworked-hardware boards (g_cfg.lorawan_fem_en).
    //   GPIO46        FEM_PA (LoRa PA control) per Heltec's own pin-mapping
    //                 diagram — dropped from the datasheet PDF's own summary
    //                 table (same class of gap as GPIO34/VGNSS_Ctrl below),
    //                 but confirmed via direct visual cross-check of the
    //                 diagram against the PDF. Multigeiger's own board also
    //                 calls this pin "DIP1" (unused input). See
    //                 PIN_LORA_FEM_PA above: double-booked with the DIP1
    //                 switch, only driven on reworked-hardware boards
    //                 (g_cfg.lorawan_high_power).
    //                 Hardware team has been asked to remove/disconnect the
    //                 DIP1 switch on the next PCB revision rather than leave
    //                 this landmine for whoever eventually does LoRa work.
    //
    // RESERVED / never repurpose (out of scope or module-internal — see
    // spec §2 and §9, and WiFi_LoRa_32_V4.3.1_Datasheet.pdf Table 2.2.1/2.2.2
    // for the primary source):
    //   GPIO36        Vext_Ctrl — see PIN_VEXT / HAL_HAS_VEXT_GATE above.
    //                 V2.6.20: bench-confirmed to gate the OLED rail and
    //                 now firmware-driven, but it stays on this list: it
    //                 is the module's power-gate control, NEVER free GPIO.
    //   GPIO19/20     native USB D-/D+ (module-internal strap pair; this
    //                 board doesn't use native USB but the pins are still
    //                 module wiring, not free GPIO)
    //   GPIO1         VBAT_Read — out of scope (no battery ADC support).
    //                 Table 2.2.2 footnote: reading it requires ADC_CTRL
    //                 (GPIO37) pulled high first — see GPIO37 entry below.
    //   GPIO38-42     GNSS connector — GPIO38=GNSS_RX, GPIO39=GNSS_TX,
    //                 GPIO40=GNSS_Wakeup, GPIO41=GNSS_PPS, GPIO42=GNSS_RST
    //                 (Table 2.2.2 rows 7-11) — out of scope
    //   GPIO45/46     DIP-switch inputs on the Multigeiger mainboard itself
    //                 (J3 pin6=GPIO45=DIP0, J3 pin5=GPIO46=DIP1), unused by
    //                 V2 firmware on every board. GPIO46/DIP1 also = FEM_PA,
    //                 see LoRa block above — two stacked claims, not free.
    //                 GPIO45/DIP0 has no competing Heltec-side function
    //                 (Table 2.2.2 row 6) — free of Heltec's reservations,
    //                 but IS one of the ESP32-S3's four strapping pins
    //                 (GPIO0/3/45/46): VDD_SPI voltage-select, sampled at
    //                 reset to choose the flash/PSRAM regulator voltage —
    //                 and unlike GPIO3's JTAG-source strap, this one is NOT
    //                 eFuse-gated off by default. The existing DIP0 switch
    //                 evidently coexists safely with it (board ships and
    //                 boots today), but do NOT repurpose this pin for a new
    //                 actively-driven external signal without hardware
    //                 verifying it against the strap requirement first —
    //                 left as DIP0/unused, no repurposing planned.
    //   GPIO6         "DIP3" in the Multigeiger matrix — no Heltec-side
    //                 claim (Table 2.2.2 row 17: generic ADC1_CH5/TOUCH6
    //                 only) and NOT a strapping pin (unlike GPIO45 above).
    //                 FUTURE HARDWARE REWORK: earmarked as the new home for
    //                 PIN_HV_CAP_FULL_INPUT (currently GPIO2, see that
    //                 #define's comment) once the hardware team disconnects
    //                 the DIP3 switch — no known conflicts either way.
    //   GPIO15/16     XTAL_32K_P / XTAL_32K_N — external 32kHz RTC crystal
    //                 pins (Table 2.2.3). Not currently used by this board
    //                 (no crystal populated), but not free GPIO either.
    //   GPIO26-32     internal flash/PSRAM SPI0/1 (never usable, any PSRAM
    //                 mode) — see PIN_SPEAKER_P above, currently GPIO26
    //   GPIO34        VGNSS_Ctrl — Heltec module-level named control signal
    //                 (confirmed via Heltec's own V4 pin-mapping diagram AND
    //                 independently via the Multigeiger Pin-Matrix; was in
    //                 this board's design spec §1 comparison table but never
    //                 carried into this list until the post-release GPIO26
    //                 correction). Curiously absent from the datasheet PDF's
    //                 own Table 2.2.1 row 11 text despite being on the same
    //                 PDF's pin-layout diagram — a gap in Heltec's own docs,
    //                 not evidence it's free. NOT a free/spare GPIO.
    //   GPIO37        ADC_Ctrl — Heltec module-level named control signal
    //                 per Table 2.2.2 row 4 (J3 pin4). Function: gates the
    //                 VBAT_Read (GPIO1) voltage divider — must be pulled
    //                 high before reading GPIO1's ADC. NOT a free/spare GPIO.
    //   GPIO43/44     UART0 console (USB-UART bridge, HAL_HAS_NATIVE_USB=0)
    //                 — not free GPIO
    //   GPIO0         BOOT strap

#elif defined(BOARD_SPARKFUN_THING_PLUS_ESP32S3)

    // SparkFun Thing Plus ESP32-S3 (WRL-24408). ESP32-S3-MINI-1 SiP — flash +
    // PSRAM integrated in the module package, same silicon class as
    // BOARD_HELTEC_WIFI_LORA32_V4_R2 (2 MB in-package quad PSRAM), NOT
    // external QSPI/OPI like FeatherS3-D or XIAO S3.
    //
    // Second-source board for the existing FeatherS3-D-populated carrier PCB
    // (project_feathers3d_new_pcb.md, in fab since 2026-05-14): same physical
    // Feather-format socket, same Geiger/HV/speaker/I2C wiring at the header
    // hole positions — only the GPIO number each position maps to changes.
    // Pin map extracted from SparkFun's official labeled pinout photo +
    // firmware examples + the ESP32-S3-MINI-1/MINI-1U datasheet v1.1, then
    // confirmed by the user (who designed the shared carrier PCB) against the
    // physical board in hand. Full verification writeup, including the
    // GPIO26-37 in-package-PSRAM hazard check (same class of incident as
    // BOARD_HELTEC_WIFI_LORA32_V4_R2's GPIO26/speaker collision, ruled out
    // here) and the strapping-pin review, is in
    // docs/superpowers/specs/2026-07-09-sparkfun-thing-plus-esp32s3-board-port-design.md §2.
    #define BOARD_NAME              "sparkfun_thing_plus_esp32s3"
    #define HAL_HAS_OLED              1   // External SSD1309/SSD1306 on Qwiic, probe-detected — same as FeatherS3-D
    #define HAL_HAS_TFT               0
    #define HAL_HAS_ALS               0   // ALS-PT19 is onboard the FeatherS3-D module itself (GPIO4, not header-routed) — the shared carrier PCB carries no ALS, and this board has no onboard equivalent. Confirmed by user (PCB designer).
    #define HAL_HAS_FUEL_GAUGE        1   // Onboard MAX17048 @ 0x36 — same chip/address/driver as FeatherS3-D
    #define HAL_HAS_PSRAM             1   // 2 MB in-package, quad
    #define HAL_HAS_NATIVE_USB        1   // USB-C, USB-Serial-JTAG console
    // Shared-PCB sensors are powered from L2 (the carrier PCB's primary,
    // always-on Feather 3V3 pin), not from this board's own onboard Qwiic
    // connector (J4). J4's rail IS gated on-board (RT9080 LDO + GPIO45/Q_EN,
    // see the reserved-pin note below) — but we don't use J4, so no firmware
    // gating applies to our sensor path.
    #define HAL_HAS_VEXT_GATE         0
    #define HAL_HAS_ANTENNA_SWITCH    0   // PCB antenna only
    #define HAL_HAS_I2C_PINOUT_SWITCH 0   // Single fixed I2C route
    #define HAL_HAS_SPEAKER           1   // Piezo via PCB harness, same as FeatherS3-D — see pin table below
    // Onboard WS2812 (D6, DIN=GPIO46). Schematic confirms VDD ties to
    // "3.3V_P", the same peripheral rail as the Qwiic connector (J4) and
    // microSD — NOT an always-driven-by-firmware rail, but the schematic's
    // own note plus the JP2/R17/R19 divider mean it defaults to always-on
    // (see the GPIO45/Q_EN reserved-pin note below): no PIN_NEOPIXEL_POWER
    // needed. neopixel.c's power-gate step is conditional on
    // PIN_NEOPIXEL_POWER being #defined (same "intentionally undefined"
    // idiom used for PIN_OLED_RESET) — not defined here.
    #define HAL_HAS_NEOPIXEL          1

    // V2.6.19: onboard microSD, full 4-bit SDIO wiring (SparkFun schematic
    // global labels + their SD_SDIO_Benchmark.ino, verified 2026-07-15).
    // SDMMC host (native SD protocol, per-transfer data CRC) — the S3 SDMMC
    // host routes through the GPIO matrix, so these non-IOMUX pins are fine.
    // Slot is powered from the always-on "3.3V_P" rail (RT9080/JP2 — see the
    // GPIO45/Q_EN note above); no power gating needed. SD_DET (GPIO48) is
    // deliberately unused: card presence = mount attempt, one code path with
    // the C5 board (whose SD_DET isn't even connected by default).
    #define HAL_HAS_SD_CARD           1
    #define HAL_HAS_LORAWAN           0   // No LoRa radio
    #define HAL_SD_USE_SDMMC          1
    #define PIN_SD_CLK               38
    #define PIN_SD_CMD               34
    #define PIN_SD_D0                39
    #define PIN_SD_D1                40
    #define PIN_SD_D2                47
    #define PIN_SD_D3                33

    // Ring/scratch/form-buffer sizes modeled on BOARD_HELTEC_WIFI_LORA32_V4_R2
    // (identical 2 MB in-package PSRAM budget).
    #define HAL_LOG_RING_BYTES      (1 * 1024 * 1024)   // 1 MB of 2 MB PSRAM (50% headroom)
    #define HAL_LOG_SNAP_SCRATCH_BYTES  (16 * 1024)
    #define HAL_CFG_FORM_BUF_SIZE   (32 * 1024)

    // Geiger / HV / speaker / I2C pins — Feather-format header hole positions
    // shared with FeatherS3-D (see design spec §2 for the full position
    // table and vendor-GPIO cross-reference).
    //
    // Position    FeatherS3-D    WRL-24408 (this board)
    // --------    -----------    -----------------------
    // L5          GPIO 17        GPIO 10
    // L6          GPIO 18        GPIO 14
    // L10         GPIO  5        GPIO 18
    // R7          GPIO  3        GPIO  5  (not a strap pin on this module)
    // R8          GPIO  1        GPIO  4
    // R11         GPIO  9        GPIO  9  (exact match)
    // R12         GPIO  8        GPIO  8  (exact match)
    #define PIN_HV_CAP_FULL_INPUT   10   // L5 — comparator interrupt (digital)
    #define PIN_GMC_COUNT_INPUT     14   // L6 — Geiger pulse interrupt
    #define PIN_HV_FET_OUTPUT       18   // L10 — HV MOSFET gate (LEDC PWM via gptimer)

    // Piezo pins
    #define PIN_SPEAKER_P            5   // R7
    #define PIN_SPEAKER_N            4   // R8

    // No "user" LED separate from the NeoPixel. PIN_LED_BUILTIN intentionally
    // undefined; the led_tick config flag instead flashes the onboard NeoPixel
    // (speaker.c drives it directly via neopixel_notify_pulse() — see that
    // file and neopixel_register_pulse_tick()'s doc comment for why).

    // Onboard NeoPixel — single WS2812, data-only (see HAL_HAS_NEOPIXEL note
    // above for why there's no PIN_NEOPIXEL_POWER on this board).
    #define PIN_NEOPIXEL_DATA       46   // WS2812 DIN. Strapping pin (ROM boot-mode/log-print select, pull-down default) — safe: only matters when GPIO0 is also pulled low (Download Boot), and neopixel.c doesn't drive it until well after app startup. See design spec §2.1.

    // I2C bus = Qwiic connector (env sensor breakout plugs in directly; no
    // I2C wiring lands on the shared carrier PCB beyond the header pass-
    // through — same R11/R12 positions as FeatherS3-D).
    #define PIN_I2C_SDA              8   // R12 — exact GPIO match with FeatherS3-D
    #define PIN_I2C_SCL              9   // R11 — exact GPIO match with FeatherS3-D

    // PIN_OLED_RESET intentionally undefined — the external Qwiic OLED has no
    // reset line, same as FeatherS3-D; display.c skips the reset pulse when
    // PIN_OLED_RESET is not defined.

    // No PIN_VBUS_DETECT — this board's MCP73831 STAT pin drives only an
    // onboard status LED (D4), no GPIO net. See design spec §2 "VBUS/charge-
    // status pin" note.

    // RESERVED pins on this board — never repurpose:
    //   GPIO0         BOOT strap
    //   GPIO3         JTAG signal-source strap (floating default) — not used
    //                 anywhere in this board's pin map, unlike FeatherS3-D
    //                 where it drives the speaker.
    //   GPIO26-37     internal flash/PSRAM SPI0/1 bus (never usable on any
    //                 in-package-PSRAM module) — same hazard class as
    //                 BOARD_HELTEC_WIFI_LORA32_V4_R2's GPIO26/speaker
    //                 incident. None of this board's pins fall in this range.
    //   GPIO45        Q_EN — gates the onboard "3.3V_P" peripheral rail
    //                 (Qwiic connector J4, microSD, and the onboard NeoPixel)
    //                 via an RT9080 LDO. Schematic's own note: "Default:
    //                 Peripheral Power is on" — R17 (10k, to 3.3V) / R19
    //                 (100k, to GND) bias EN HIGH whenever jumper JP2 is
    //                 populated (its shipped state), so the rail is powered
    //                 without firmware ever touching this pin. JP2 exists so
    //                 firmware can drive GPIO45 LOW during deep sleep to shed
    //                 that rail's current — not implemented here (no sleep
    //                 mode in this port). Also one of the ESP32-S3's four
    //                 strapping pins (VDD_SPI voltage-select) — left
    //                 undriven, no repurposing planned.
    //   GPIO33/34/38/39/40/47   microSD SDIO bus — claimed V2.6.19 (PIN_SD_*
    //                 above, standalone SD-logging design 2026-07-15).
    //   GPIO48        microSD SD_DET — deliberately unused (mount-probe
    //                 instead; keeps one code path with the C5 board).
    //   GPIO19/20     native USB D-/D+ (module-internal)
    //   GPIO43/44     UART0 — not used (native USB-Serial-JTAG console)

#elif defined(BOARD_SPARKFUN_THING_PLUS_ESP32C5)

    // SparkFun Thing Plus ESP32-C5 (WRL-30678). ESP32-C5-WROOM-1 SiP — flash
    // + PSRAM integrated in the module package, but single-core RISC-V (up
    // to 240 MHz) — the first non-Xtensa, first single-core target this
    // codebase has ever built for. 2.4/5 GHz WiFi, BLE 5, Zigbee, Thread in
    // silicon; this firmware never calls esp_wifi_set_band_mode(), so it
    // runs on the IDF default of WIFI_BAND_MODE_AUTO (both bands) rather
    // than being pinned to 2.4 GHz — confirmed live via 5 GHz association
    // (channel 44). apply_radio_limits_sta() (main.c) branches on
    // esp_wifi_get_band_mode() to use the plural per-band protocol/
    // bandwidth APIs this mode requires. See design spec §8.
    //
    // Own standalone Thing Plus carrier PCB — NOT a drop-in for the
    // FeatherS3-D-format carrier shared by BOARD_FEATHERS3_D /
    // BOARD_SPARKFUN_THING_PLUS_ESP32S3. Pin map derived from SparkFun's
    // KiCad-exported schematic (SparkFun_Thing_Plus_ESP32_C5.kicad_sch, Rev
    // v10, 2026-01-22), cross-validated against 7 already-known-good S3
    // GPIO values via the shared ThingPlus footprint-pin-number scheme, then
    // confirmed by the user against the physical board in hand. Full
    // rationale (incl. the R7/GPIO26 speaker-position conflict resolution
    // and the strapping-pin review) in
    // docs/superpowers/specs/2026-07-09-sparkfun-thing-plus-esp32-c5-board-port-design.md §2.
    #define BOARD_NAME              "sparkfun_thing_plus_esp32c5"
    #define HAL_HAS_OLED              1   // External SSD1306/SSD1309 on Qwiic, probe-detected — same as S3/FeatherS3-D
    #define HAL_HAS_TFT               0
    #define HAL_HAS_ALS               0   // No onboard ALS on this board
    #define HAL_HAS_FUEL_GAUGE        1   // Onboard MAX17048 @ 0x36 — schematic-confirmed, same chip/address/driver as S3
    #define HAL_HAS_PSRAM             1   // 8 MB in-package, quad (this chip's Kconfig.spiram has no Octal option at all)
    #define HAL_HAS_NATIVE_USB        1   // USB-C, IO13/IO14 wired to USB_D-/D+
    // Regulator U4 (produces the "3.3V_P" peripheral rail feeding Qwiic,
    // microSD, and the NeoPixel) has its EN pin tied to the IO26/LP net
    // through a 100k pull-up (R6) to 3.3V. Since GPIO26 is left undriven by
    // firmware (see HAL_HAS_SPEAKER below), that pull-up holds EN high by
    // default — the peripheral rail is simply always on, without firmware
    // ever touching this pin. Same "no gating needed" outcome as the S3
    // board's GPIO45/Q_EN pull-up pattern, via a single resistor instead of
    // a divider. Bench-confirm at first boot — design spec §9 item 3.
    #define HAL_HAS_VEXT_GATE         0
    #define HAL_HAS_ANTENNA_SWITCH    0   // PCB antenna only
    #define HAL_HAS_I2C_PINOUT_SWITCH 0   // Single fixed I2C route
    // R7/GPIO26 and R8/GPIO27 are the only two candidate speaker-position
    // header pads on this board, and both are hard-committed to
    // board-specific functions: R7/GPIO26 is the peripheral-rail enable
    // above (no disconnect jumper exists for that net), and R8/GPIO27 is
    // the onboard NeoPixel's DIN (its "Open RGB" jumper would free the pad
    // but at the cost of losing the onboard LED, and wouldn't resolve the
    // R7 conflict anyway). User decision: disable the speaker on this
    // board rather than rework hardware — see design spec §2.1.
    #define HAL_HAS_SPEAKER           0
    // Onboard WS2812, DIN=GPIO27 — reuses neopixel.c unchanged. GPIO27 is
    // one of this chip's five strapping pins (pull-up=1, boot-mode +
    // UART0 ROM-print-control select) — same argument as the S3's GPIO46
    // and FeatherS3-D's GPIO3: it only matters during the boot-mode
    // sampling window, and neopixel.c doesn't drive DIN until well after
    // app startup, so enabling it here is safe.
    #define HAL_HAS_NEOPIXEL          1

    // V2.6.19: onboard microSD via SPI (user-supplied SparkFun hardware-
    // overview pin list, 2026-07-15; the page's "ESP32-C6" mention is
    // SparkFun's copy-paste typo). The C5 has NO SDMMC host peripheral —
    // SPI mode is the only option on this chip. GPIO25 (CS) is a strapping
    // pin (floating, clock-edge select): safe, the SPI master only drives
    // CS well after the boot-strap sampling window and idles it high.
    // SD_DET (GPIO7) is not connected by default (solder jumper) — unused.
    // Slot is powered from the always-on "3.3V_P" rail (U4/R6 pull-up).
    #define HAL_HAS_SD_CARD           1
    #define HAL_HAS_LORAWAN           0   // No LoRa radio
    #define HAL_SD_USE_SDMMC          0
    #define PIN_SD_CS                25
    #define PIN_SD_SCK               10
    #define PIN_SD_PICO               8   // MOSI (controller out)
    #define PIN_SD_POCI               9   // MISO (controller in)

    // Ring/scratch/form-buffer sizes: this board's 8 MB in-package PSRAM
    // gets the 4 MB ring budget (50% headroom), following the feathers3_d /
    // seeed_xiao_esp32s3 precedent for 8 MB-PSRAM boards — not the 1 MB
    // ring (also 50% headroom) used by the 2 MB-PSRAM boards
    // (sparkfun_thing_plus_esp32s3, adafruit_qtpy_esp32_pico,
    // heltec_wifi_lora32_v4_r2).
    #define HAL_LOG_RING_BYTES      (4 * 1024 * 1024)   // 4 MB of 8 MB PSRAM
    #define HAL_LOG_SNAP_SCRATCH_BYTES  (16 * 1024)
    #define HAL_CFG_FORM_BUF_SIZE   (32 * 1024)

    // Geiger / HV pins — ThingPlus header positions L5/L6/L10 (silkscreen
    // A0/A1/A5). None of these three GPIOs are strapping pins on this chip.
    #define PIN_HV_CAP_FULL_INPUT    1   // L5 (silkscreen A0) — comparator interrupt (digital)
    #define PIN_GMC_COUNT_INPUT      2   // L6 (silkscreen A1) — Geiger pulse interrupt
    #define PIN_HV_FET_OUTPUT        6   // L10 (silkscreen A5) — HV MOSFET gate (LEDC PWM via gptimer). Also carries an optional, default-open "battery/USB power-source status" jumper tap — harmless unless someone later solders that jumper.

    // No "user" LED separate from the NeoPixel — no PIN_LED_BUILTIN.
    // led_tick flashes the NeoPixel via neopixel_notify_pulse(), same as
    // the S3 board.

    // Onboard NeoPixel — single WS2812, data-only (see HAL_HAS_NEOPIXEL
    // note above for the strap-pin safety argument).
    #define PIN_NEOPIXEL_DATA       27   // R8 (silkscreen D9/DIN) — WS2812 DIN

    // I2C bus = Qwiic connector (external env sensor breakout plugs in
    // directly; also the onboard MAX17048's bus). Matches the board's own
    // Qwiic + MAX17048 wiring.
    #define PIN_I2C_SDA             23   // R12 (silkscreen SDA)
    #define PIN_I2C_SCL             24   // R11 (silkscreen SCL)

    // PIN_OLED_RESET intentionally undefined — the external Qwiic OLED has
    // no reset line, same as FeatherS3-D/S3 Thing Plus; display.c skips the
    // reset pulse when PIN_OLED_RESET is not defined.

    // No PIN_VBUS_DETECT — this board's MCP73831 STAT pin drives only the
    // onboard CHG status LED, no GPIO net (same charger IC, same finding,
    // as the S3 board).

    // RESERVED pins on this board — never repurpose:
    //   GPIO26        R7 — peripheral-rail ("3.3V_P") enable, pulled high by
    //                 R6 (100k to 3.3V); deliberately left undriven (see
    //                 HAL_HAS_VEXT_GATE above). Also a strapping pin
    //                 (floating, boot mode) — another reason to leave it
    //                 undriven.
    //   GPIO15        In-package PSRAM SPICS1 (WROOM-1 datasheet: "used as
    //                 SPICS1 for SPI PSRAM and cannot be used for other
    //                 functions") — not exposed on the ThingPlus header at
    //                 all, no collision with any signal above.
    //   GPIO25        Strapping pin (floating, clock-edge select) — SD
    //                 chip-select (PIN_SD_CS) as of V2.6.19 — driven only
    //                 post-boot, see microSD note.
    //   GPIO28        Strapping pin (pull-up=1, boot mode) — not assigned.
    //   GPIO7         Strapping pin (floating, JTAG signal source) — not
    //                 assigned.
    //   microSD (J4)  GPIO8/9/10/25 claimed V2.6.19 (PIN_SD_* above,
    //                 standalone SD-logging design 2026-07-15). GPIO7
    //                 (SD_DET) stays unclaimed — NC by default.

#elif defined(BOARD_ADAFRUIT_ESP32S3_TFT_FEATHER)

    // Adafruit ESP32-S3 TFT Feather (product #5483). ESP32-S3 with external
    // QSPI flash + PSRAM (feathers3_d-class, NOT an in-package SiP). Onboard
    // 240x135 ST7789V color SPI TFT — the first board in this codebase with
    // a non-OLED display backend (see HAL_HAS_TFT above / display_tft.c).
    // NOT the "ESP32-S3 Reverse TFT Feather" (#5691) — a different, later
    // board with the opposite screen orientation.
    //
    // Shares the Feathers3d_new_pcb carrier PCB with BOARD_FEATHERS3_D /
    // BOARD_SPARKFUN_THING_PLUS_ESP32S3 (user decision). Pin map derived
    // from arduino-esp32's official
    // variants/adafruit_feather_esp32s3_tft/pins_arduino.h and confirmed by
    // the user against the physical board (2 corrections during that
    // check: L3=3V3 not AREF, L16=TXD0). Full pin table, ST7789 bring-up
    // parameter derivation, and the I2C/TFT power-gate rationale in
    // docs/superpowers/specs/2026-07-10-adafruit-esp32s3-tft-feather-board-port-design.md §2/§4.
    #define BOARD_NAME              "adafruit_esp32s3_tft_feather"
    #define HAL_HAS_OLED              0   // No I2C OLED path — the onboard TFT is the display (HAL_HAS_TFT)
    #define HAL_HAS_TFT               1   // Onboard 240x135 ST7789 SPI color TFT — see display_tft.c
    #define HAL_HAS_ALS               0   // No onboard ALS on this board
    #define HAL_HAS_FUEL_GAUGE        1   // Onboard fuel gauge @ 0x36 — MAX17048 assumed, confirm chip at bring-up (design spec §12 item 5)
    #define HAL_HAS_PSRAM             1   // 2 MB, external QSPI (feathers3_d-class)
    #define HAL_HAS_NATIVE_USB        1   // USB-C, USB-Serial-JTAG console
    // No always-on peripheral rail on this board — unlike every other
    // PSRAM board's VEXT/Q_EN-style pull-up-defaults-on pattern, this
    // board's ONLY I2C bus (env sensors + fuel gauge) and its TFT are both
    // dead until PIN_I2C_POWER_GATE (GPIO21, Adafruit's own
    // "TFT_I2C_POWER" net) is driven high. V2.6.11 review fix: this used
    // to be driven inside display_tft_init(), which runs too late (after
    // fuel_gauge_init() and every I2C sensor probe in main.c); it now
    // lives in i2c_bus_get_primary() (main/i2c_bus.c), the first I2C
    // consumer main.c calls, guarded by `#if defined(BOARD_ADAFRUIT_...)`
    // rather than HAL_HAS_VEXT_GATE (that flag's Heltec-style
    // active-LOW-gate semantics don't fit this board's active-HIGH gate).
    // See design spec §2.1.
    #define HAL_HAS_VEXT_GATE         0
    #define HAL_HAS_ANTENNA_SWITCH    0   // PCB antenna only
    #define HAL_HAS_I2C_PINOUT_SWITCH 0   // Single fixed I2C route
    #define HAL_HAS_SPEAKER           1   // Piezo via shared-carrier PCB harness — pins verified §2
    #define HAL_HAS_NEOPIXEL          1   // Onboard WS2812 (GPIO33, power-gated via GPIO34) — reuses neopixel.c unchanged
    #define HAL_HAS_SD_CARD           0   // No microSD slot on this board
    #define HAL_HAS_LORAWAN           0   // No LoRa radio

    // Ring/scratch/form-buffer sizes: 2 MB external-QSPI PSRAM — same size
    // class as sparkfun_thing_plus_esp32s3 / heltec_wifi_lora32_v4_r2's
    // shared-PCB variant (both 1 MB of 2 MB PSRAM, 50% headroom), NOT
    // feathers3_d (that board has 8 MB PSRAM / a 4 MB ring — "external
    // QSPI" above only means the same PSRAM *class* as feathers3_d, not
    // the same *size*). Opus 4.8 review fix: prior wording here claimed
    // "identical PSRAM class/size" as feathers3_d, which would mislead a
    // future maintainer into 4x-ing this ring into a 2 MB PSRAM pool.
    #define HAL_LOG_RING_BYTES      (1 * 1024 * 1024)
    #define HAL_LOG_SNAP_SCRATCH_BYTES  (16 * 1024)
    #define HAL_CFG_FORM_BUF_SIZE   (32 * 1024)

    // Geiger / HV pins — header positions L5/L6/L9 (silkscreen A0/A1/A5).
    #define PIN_HV_CAP_FULL_INPUT    18  // L5 (silkscreen A0)
    #define PIN_GMC_COUNT_INPUT      17  // L6 (silkscreen A1)
    #define PIN_HV_FET_OUTPUT         8  // L9 (silkscreen A5)

    #define PIN_LED_BUILTIN          13  // onboard LED — since V2.6.24 unused for led_tick (NeoPixel preferred); held OFF

    // Speaker — D9/D10 header pads.
    #define PIN_SPEAKER_P            10  // D10
    #define PIN_SPEAKER_N             9  // D9

    // Onboard NeoPixel — WS2812 data + power-gate.
    #define PIN_NEOPIXEL_DATA        33  // PIN_NEOPIXEL
    #define PIN_NEOPIXEL_POWER       34  // NEOPIXEL_POWER — driven high by neopixel.c before use

    // I2C bus (STEMMA QT connector; also the onboard fuel-gauge's bus).
    #define PIN_I2C_SDA              42  // SDA
    #define PIN_I2C_SCL              41  // SCL
    // Shared TFT/I2C peripheral-rail power gate — see HAL_HAS_VEXT_GATE
    // note above and design spec §2.1. Driven from i2c_bus_get_primary()
    // (main/i2c_bus.c), before any I2C bus creation or probe.
    #define PIN_I2C_POWER_GATE       21  // TFT_I2C_POWER

    // Onboard TFT — dedicated SPI bus (MOSI/SCK only, no MISO; ST7789 is
    // write-only in this driver). Not header-routed.
    #define PIN_TFT_CS                7
    #define PIN_TFT_DC               39
    #define PIN_TFT_RST              40
    #define PIN_TFT_BACKLITE         45  // also a VDD_SPI strapping pin; Adafruit's own reference design drives it post-boot as a backlight enable line — same precedented-repurposing argument as other boards' strap pins
    #define PIN_TFT_MOSI             35
    #define PIN_TFT_SCK              36

    // PIN_OLED_RESET intentionally undefined — no OLED on this board.

    // RESERVED pins on this board — never repurpose:
    //   GPIO0         BOOT strap (chip-standard)
    //   GPIO19/20     native USB D-/D+ (module-internal)
    //   GPIO37        TFT MISO (pins_arduino.h `MISO`) — not wired up, the
    //                 ST7789 panel is write-only in this driver
    //   GPIO43/44     UART0 — not used (native USB-Serial-JTAG console)

#elif defined(BOARD_ADAFRUIT_ESP32_FEATHER_V2)

    // Adafruit ESP32 Feather V2 (product #5400 / #5900 — identical, #5900
    // just ships with headers pre-soldered). ESP32-PICO-MINI-02 SiP —
    // original ESP32 LX6 dual-core (NOT S3), in-package 8 MB flash + 2 MB
    // PSRAM reached over the module's internal SDIO bus — same module
    // *class* as BOARD_ADAFRUIT_QTPY_ESP32_PICO's ESP32-PICO-V3-02 (same
    // reserved-pin set: IO6/9/10/11). No I2C fuel-gauge IC on this board —
    // only a raw ADC BAT_VOLT_PIN/GPIO35 divider — the first board on this
    // shared carrier without one; that's an out-of-scope new-driver feature
    // (design spec §8), not a port detail.
    //
    // Second-source MCU for the Feathers3d_new_pcb shared carrier
    // (BOARD_FEATHERS3_D / BOARD_SPARKFUN_THING_PLUS_ESP32S3 /
    // BOARD_ADAFRUIT_ESP32S3_TFT_FEATHER): same physical Feather-format
    // socket, same Geiger/HV/speaker/I2C wiring at the header hole
    // positions — only the GPIO number at each position changes. Pin map
    // extracted from arduino-esp32's official
    // variants/adafruit_feather_esp32_v2/pins_arduino.h, then fully
    // user-confirmed against the physical board (2026-07-10) — including
    // the two D9/D10 speaker-slot GPIOs that pins_arduino.h names no direct
    // symbol for (derived via position cross-walk + the file's own A8/A9
    // analog-alias corroboration, then confirmed a third way against the
    // board). NEOPIXEL_I2C_POWER (GPIO2) gate scope also confirmed directly
    // via Adafruit's own Learn guide: it switches only the STEMMA QT
    // connector's own 3.3V regulator, not the header SDA/SCL pins (which
    // ride the board's main, always-on 3.3V rail) — matches the SparkFun S3
    // precedent below, not the TFT Feather one. Full pin table and both
    // resolutions in
    // docs/superpowers/specs/2026-07-10-adafruit-esp32-feather-v2-board-port-design.md §2/§9.
    #define BOARD_NAME              "adafruit_esp32_feather_v2"
    #define HAL_HAS_OLED              1   // External SSD1309/SSD1306 on Qwiic/header I2C, probe-detected — same as feathers3_d/sparkfun_thing_plus_esp32s3
    #define HAL_HAS_TFT               0
    #define HAL_HAS_ALS               0   // No onboard ALS-PT19, carrier PCB carries none
    #define HAL_HAS_FUEL_GAUGE        0   // No I2C fuel-gauge IC on this board (only a raw ADC BAT_VOLT_PIN/GPIO35 divider) — FIRST board on this shared carrier without one. New ADC-battery-voltage feature is out of scope (design spec §8), not "same as the others."
    #define HAL_HAS_PSRAM             1   // 2 MB in-package PSRAM (internal SDIO, ESP32-PICO-MINI-02 SiP)
    #define HAL_HAS_NATIVE_USB        0   // USB-C via CH9102F/CP2102N bridge — classic ESP32 has no USB-OTG
    // NEOPIXEL_I2C_POWER (GPIO2) switches only the STEMMA QT connector's own
    // 3.3V regulator (per Adafruit's Learn guide). Our shared-carrier sensor
    // I2C bus rides the header SDA/SCL pins off the board's main, always-on
    // 3.3V regulator instead — same as sparkfun_thing_plus_esp32s3's Qwiic-
    // gate-is-irrelevant case, NOT adafruit_esp32s3_tft_feather's
    // only-I2C-bus-is-gated case. No i2c_bus.c pre-gate needed.
    #define HAL_HAS_VEXT_GATE         0
    #define HAL_HAS_ANTENNA_SWITCH    0   // PCB antenna only
    #define HAL_HAS_I2C_PINOUT_SWITCH 0   // Single fixed I2C route (header SDA/SCL)
    #define HAL_HAS_SPEAKER           1   // Piezo via PCB harness — pin assignment user-confirmed, see block comment above
    #define HAL_HAS_NEOPIXEL          1   // Onboard WS2812, PIN_NEOPIXEL_POWER=GPIO2 (NEOPIXEL_I2C_POWER) required before use
    #define HAL_HAS_SD_CARD           0   // No microSD slot on this board
    #define HAL_HAS_LORAWAN           0   // No LoRa radio

    // Ring/scratch/form-buffer sizes modeled on BOARD_ADAFRUIT_QTPY_ESP32_PICO
    // (identical 2 MB in-package PSRAM budget).
    #define HAL_LOG_RING_BYTES      (1 * 1024 * 1024)   // 1 MB of 2 MB PSRAM (50% headroom)
    #define HAL_LOG_SNAP_SCRATCH_BYTES  (16 * 1024)
    #define HAL_CFG_FORM_BUF_SIZE   (32 * 1024)

    // Geiger / HV pins — header positions L5/L6/L10 (silkscreen A0/A1/A5).
    #define PIN_HV_CAP_FULL_INPUT   26   // A0 (L5)
    #define PIN_GMC_COUNT_INPUT     25   // A1 (L6)
    #define PIN_HV_FET_OUTPUT        4   // A5 (L10)

    // Speaker — D9/D10 header pads (top-row silkscreen prints raw GPIO
    // numbers on this board, not "D9"/"D10" — see block comment above for
    // the three-source confirmation of these two GPIOs).
    #define PIN_SPEAKER_P           33   // D10 slot (R7)
    #define PIN_SPEAKER_N           15   // D9 slot (R8) — MTDO strap; driven only post-boot by speaker.c. adafruit_qtpy_esp32_pico documents this same IO15/MTDO strap as merely "annoying but recoverable" if LOW at boot (it never drives the pin); here it's actively driven post-boot, same reasoning, one step further

    // No "user" LED separate from the NeoPixel is wired on this shared
    // carrier. LED_BUILTIN=GPIO13 exists on this board (pins_arduino.h) but
    // isn't on a header position the carrier routes to anything — PIN_LED_BUILTIN
    // intentionally undefined; led_tick has no effect here, same as every
    // other NeoPixel-only board already shipped (design spec §8).

    // Onboard NeoPixel — single WS2812 with software-gated power.
    #define PIN_NEOPIXEL_DATA        0   // PIN_NEOPIXEL — also GPIO0/BOOT strap; only driven post-boot, same idiom as every prior NeoPixel-on-strap-pin board
    #define PIN_NEOPIXEL_POWER       2   // NEOPIXEL_I2C_POWER — HIGH powers the onboard NeoPixel (and, separately, the STEMMA QT connector we don't use)

    // I2C bus = header SDA/SCL pads (R11/R12), off the carrier's own
    // always-on 3.3V trace — see HAL_HAS_VEXT_GATE note above.
    #define PIN_I2C_SDA             22   // R12
    #define PIN_I2C_SCL             20   // R11

    // PIN_OLED_RESET intentionally undefined — external Qwiic/header OLED
    // has no reset line, same as feathers3_d / sparkfun_thing_plus_esp32s3.

    // RESERVED pins on this board — never repurpose:
    //   IO0   BOOT strap / PIN_NEOPIXEL_DATA (driven post-boot only)
    //   IO2   NEOPIXEL_I2C_POWER / boot strap (see block comment above)
    //   IO6   internal flash (CLK)         IO9   internal PSRAM (SD2)
    //   IO10  internal PSRAM (SD3)         IO11  internal flash (CMD)
    //   IO12  MTDI strap (DANGEROUS — pulling HIGH at boot with wrong
    //         flash-voltage setting bricks flash boot until power-cycle) —
    //         A11 on this board, NOT used in our pin map
    //   IO15  MTDO strap — PIN_SPEAKER_N, driven only post-boot
    //   IO35  BAT_VOLT_PIN/BATT_MONITOR — input-only ADC1, onboard
    //         battery-voltage sense, not used (HAL_HAS_FUEL_GAUGE=0, no
    //         ADC-battery driver implemented — design spec §8)

#elif defined(BOARD_ADAFRUIT_ESP32S3_FEATHER_4MB_2MBPSRAM)

    // Adafruit ESP32-S3 Feather, 4 MB Flash / 2 MB PSRAM, STEMMA QT
    // (product #5477). ESP32-S3 with external QSPI flash + PSRAM
    // (feathers3_d-class, NOT an in-package SiP) — same silicon class as
    // BOARD_ADAFRUIT_ESP32S3_TFT_FEATHER, but no onboard display: the
    // sensor OLED is external, probe-detected on the STEMMA QT bus
    // (HAL_HAS_OLED below), same as feathers3_d / sparkfun_thing_plus_esp32s3.
    //
    // Shares the Feathers3d_new_pcb carrier PCB with BOARD_FEATHERS3_D /
    // BOARD_SPARKFUN_THING_PLUS_ESP32S3 / BOARD_ADAFRUIT_ESP32S3_TFT_FEATHER
    // (user decision, 2026-07-11). Pin map cross-verified against
    // arduino-esp32's pins_arduino.h, CircuitPython's SKU-specific pins.c,
    // and the user's own visual read of Adafruit's pin-identical #5323
    // sibling-SKU picture (#5477 itself has no published picture; #5323
    // differs only in flash size and lacks PSRAM) — all three sources
    // agree. Full pin table and design rationale in
    // docs/superpowers/specs/2026-07-11-adafruit-esp32s3-feather-board-port-design.md §2/§4.
    #define BOARD_NAME              "adafruit_esp32s3_feather_4mb_2mbpsram"
    #define HAL_HAS_OLED              1   // External SSD1306/SSD1309 via STEMMA QT, probe-detected
    #define HAL_HAS_TFT               0   // No onboard display on this variant
    #define HAL_HAS_ALS               0   // No onboard ALS
    #define HAL_HAS_FUEL_GAUGE        1   // Onboard MAX17048 @ 0x36, user-confirmed 2026-07-11
    #define HAL_HAS_PSRAM             1   // 2 MB, external QSPI (feathers3_d-class, not in-package SiP)
    #define HAL_HAS_NATIVE_USB        1   // USB-C, USB-Serial-JTAG console
    // No always-on I2C bus on this board — same as BOARD_ADAFRUIT_ESP32S3_TFT_FEATHER:
    // the single STEMMA QT bus (also the onboard fuel gauge's bus) is dead
    // until PIN_I2C_POWER_GATE (GPIO7, Adafruit's "PIN_I2C_POWER" net) is
    // driven high. Confirmed required (not just precautionary) by Adafruit's
    // own product documentation: "There is an I2C power pin that needs to be
    // pulled high for the STEMMA QT connector ... to work properly." Reuses
    // the TFT Feather's i2c_bus_get_primary() gate plumbing (main/i2c_bus.c)
    // with its own GPIO7 guarded by `#if defined(BOARD_ADAFRUIT_...)`. See
    // design spec §2.1.
    #define HAL_HAS_VEXT_GATE         0
    #define HAL_HAS_ANTENNA_SWITCH    0   // PCB antenna only
    #define HAL_HAS_I2C_PINOUT_SWITCH 0   // Single fixed I2C route
    #define HAL_HAS_SPEAKER           1   // Piezo via shared-carrier PCB harness (P=GPIO10, N=GPIO9)
    #define HAL_HAS_NEOPIXEL          1   // Onboard WS2812 (GPIO33, power-gated via GPIO21) — reuses neopixel.c unchanged
    #define HAL_HAS_SD_CARD           0   // No microSD slot on this board
    #define HAL_HAS_LORAWAN           0   // No LoRa radio

    // Dual-LED note: this board defines both PIN_LED_BUILTIN and
    // HAL_HAS_NEOPIXEL. Since V2.6.24, speaker.c's tick_start() prefers the
    // NeoPixel when both are present (user decision at #5477 bench bring-up,
    // 2026-07-20 — reverses the original "PIN_LED_BUILTIN wins" port
    // decision, design spec §2), so the NeoPixel blue-flashes on each Geiger
    // pulse, same as BOARD_ADAFRUIT_ESP32S3_TFT_FEATHER. The red "#13" LED
    // is held at a deterministic OFF by speaker_setup() and is otherwise
    // unused (available for future status use).

    // Ring/scratch/form-buffer sizes: 2 MB external-QSPI PSRAM — same size
    // class as BOARD_ADAFRUIT_ESP32S3_TFT_FEATHER (both 1 MB of 2 MB
    // PSRAM, 50% headroom).
    #define HAL_LOG_RING_BYTES      (1 * 1024 * 1024)
    #define HAL_LOG_SNAP_SCRATCH_BYTES  (16 * 1024)
    #define HAL_CFG_FORM_BUF_SIZE   (32 * 1024)

    // Geiger / HV pins — header positions L5/L6/L10 (silkscreen A0/A1/A5).
    #define PIN_HV_CAP_FULL_INPUT    18  // L5 (silkscreen A0)
    #define PIN_GMC_COUNT_INPUT      17  // L6 (silkscreen A1)
    #define PIN_HV_FET_OUTPUT         8  // L10 (silkscreen A5)

    #define PIN_LED_BUILTIN          13  // R4 (silkscreen D13) — onboard red "#13" LED; since V2.6.24 unused for led_tick (NeoPixel preferred, see dual-LED note above); held OFF

    // Speaker — D9/D10 header pads.
    #define PIN_SPEAKER_P            10  // R7 (silkscreen D10)
    #define PIN_SPEAKER_N             9  // R8 (silkscreen D9)

    // Onboard NeoPixel — WS2812 data + power-gate.
    #define PIN_NEOPIXEL_DATA        33  // PIN_NEOPIXEL
    #define PIN_NEOPIXEL_POWER       21  // NEOPIXEL_POWER — driven high by neopixel.c before use

    // I2C bus (STEMMA QT connector; also the onboard fuel-gauge's bus).
    // GPIO3 (SDA) is a JTAG source-select strapping pin — safe post-boot
    // only, same category as FeatherS3-D's IO3; i2c_bus.c only drives it
    // after the power gate below, well past boot-strap sampling.
    #define PIN_I2C_SDA               3  // R12 (silkscreen SDA) — strapping pin, safe post-boot only
    #define PIN_I2C_SCL               4  // R11 (silkscreen SCL)
    // I2C/STEMMA-QT power gate — see HAL_HAS_VEXT_GATE note above and design
    // spec §2.1. Driven from i2c_bus_get_primary() (main/i2c_bus.c), before
    // any I2C bus creation or probe.
    #define PIN_I2C_POWER_GATE        7  // PIN_I2C_POWER

    // PIN_OLED_RESET intentionally undefined — external STEMMA QT OLED has
    // no reset line, same as feathers3_d / sparkfun_thing_plus_esp32s3.

    // RESERVED pins on this board — never repurpose:
    //   GPIO0         BOOT strap (chip-standard)
    //   GPIO19/20     native USB D-/D+ (module-internal)
    //   GPIO43/44     UART0 / "DB" debug-console pin (hardware UART debug
    //                 TX, per Adafruit's own docs — NOT where Serial.print()/
    //                 ESP_LOGx output goes; that's native USB-Serial-JTAG).
    //                 Not used — console is USB-C (HAL_HAS_NATIVE_USB=1).

#else
    #error "No board defined. Set -DBOARD_HELTEC_V2=1 / -DBOARD_FEATHERS3_D=1 / -DBOARD_ADAFRUIT_QTPY_ESP32_PICO=1 / -DBOARD_SEEED_XIAO_ESP32S3=1 / -DBOARD_HELTEC_WIFI_LORA32_V4_R2=1 / -DBOARD_SPARKFUN_THING_PLUS_ESP32S3=1 / -DBOARD_SPARKFUN_THING_PLUS_ESP32C5=1 / -DBOARD_ADAFRUIT_ESP32S3_TFT_FEATHER=1 / -DBOARD_ADAFRUIT_ESP32_FEATHER_V2=1 / -DBOARD_ADAFRUIT_ESP32S3_FEATHER_4MB_2MBPSRAM=1 via CMake."
#endif
