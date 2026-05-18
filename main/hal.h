#pragma once

/** @file
 *  @brief Board-level hardware abstraction — pin map and feature flags.
 *
 *  One of `BOARD_HELTEC_V2`, `BOARD_FEATHERS3_D`, or
 *  `BOARD_ADAFRUIT_QTPY_ESP32_PICO` is defined by the top-level CMakeLists.txt
 *  based on the `BOARD` variable (default `heltec_v2`). All module .c/.h files
 *  include this header and reference pins / features by the macros below —
 *  never by raw GPIO numbers.
 *
 *  Adding a new board: pick a `BOARD_*` macro name, add a branch below that
 *  sets the same `PIN_*` and `HAL_HAS_*` symbols, and extend the CMake board
 *  selector in `CMakeLists.txt`.
 *
 *  Feature flags every branch must define (no implicit defaults — explicit is
 *  better than surprising):
 *    HAL_HAS_OLED            display.c stubs out when 0
 *    HAL_HAS_PSRAM           applog.c picks PSRAM vs internal DRAM ring
 *    HAL_HAS_NATIVE_USB      console routing in sdkconfig overlay
 *    HAL_HAS_VEXT_GATE       Heltec-style active-LOW power gate
 *    HAL_HAS_ANTENNA_SWITCH  external u.FL antenna toggle in /config UI
 *    HAL_HAS_SPEAKER         speaker.c stubs out when 0 (small-board path)
 *    HAL_HAS_NEOPIXEL        neopixel.c init + tube-pulse hook gated on this
 *    HAL_HAS_ALS             V2.3.29: 1 = onboard ALS-PT19 ambient-light sensor
 *                            on PIN_ALS (FeatherS3-D's IO4); als.c is active
 *                            and /status renders an ambient-light row. 0 elsewhere
 *                            stubs the driver out.
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
    #define HAL_HAS_PSRAM           0
    #define HAL_HAS_NATIVE_USB      0   // Console via CP2102 UART0
    #define HAL_HAS_VEXT_GATE       1   // GPIO 21 = active-LOW MOSFET on V2+ Heltec carriers
    #define HAL_HAS_ANTENNA_SWITCH  0   // PCB antenna only (no u.FL / no RF switch)
    #define HAL_HAS_SPEAKER         1   // Onboard piezo wired to PIN_SPEAKER_P/N
    #define HAL_HAS_NEOPIXEL        0   // No onboard NeoPixel
    #define HAL_HAS_ALS             0   // No onboard ambient-light sensor
    // V2.4.5: trimmed 60 KB → 45 KB. The 60 KB ring was the largest single
    // permanent heap allocation on the Heltec, sitting in internal SRAM
    // forever. Dropping 15 KB gives that back to free heap (~140 KB → ~155 KB
    // at boot) on top of the V2.4.5 mbedTLS dynamic-buffer transient win.
    // /log scrollback shrinks proportionally (~500 lines → ~380 lines) —
    // still plenty for diagnosing a TX cycle. FeatherS3-D / QT Py rings
    // are in PSRAM and not affected.
    #define HAL_LOG_RING_BYTES      (45 * 1024)  // Internal SRAM only — keep small
    // V2.4.6: /config form render buffer. 16 KB on Heltec (internal-DRAM-only,
    // tight). FeatherS3-D / QT Py override to 32 KB to leave room for the
    // V2.4.6 MQTT TLS PEM textarea on a future config push (PSRAM-backed
    // boards can afford the transient). Bump cap on Heltec means a single
    // CA cert + every other field still fits with ~5 KB of slack.
    #define HAL_CFG_FORM_BUF_SIZE   (16 * 1024)
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
    #define HAL_HAS_ALS             1   // V2.3.29: onboard ALS-PT19 ambient-light sensor on PIN_ALS
    #define HAL_HAS_PSRAM           1   // 8 MB QSPI PSRAM
    #define HAL_HAS_NATIVE_USB      1   // Console via USB-Serial-JTAG (USB-C)
    #define HAL_HAS_VEXT_GATE       0   // No Vext gate — sensors powered via Qwiic 3V3
    #define HAL_HAS_ANTENNA_SWITCH  1   // u.FL external antenna + onboard SPDT RF switch
    #define HAL_HAS_SPEAKER         1   // Piezo wired to A3/A4 of the Feather harness
    #define HAL_HAS_NEOPIXEL        0   // FeatherS3-D has an RGB LED on IO40 but we don't drive it
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
    // outputs). HAL_HAS_SPEAKER=0 stubs speaker.c entirely. Geiger uses A0/
    // A1/A2 only; env sensor plugs into the STEMMA QT connector (no pad
    // cost — uses the secondary I²C bus IO22/IO19 routed to the connector).
    // Visible feedback comes from the onboard NeoPixel via neopixel.c
    // (HAL_HAS_NEOPIXEL=1) flashing red on each Geiger pulse.
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
    #define HAL_HAS_PSRAM           1   // 2 MB in-package SiP PSRAM
    #define HAL_HAS_NATIVE_USB      0   // USB-C via CH9102F or CP2102N UART bridge (NOT native — original ESP32 has no USB-OTG)
    #define HAL_HAS_VEXT_GATE       0   // No power gate — STEMMA QT bus always powered
    #define HAL_HAS_ANTENNA_SWITCH  0   // PCB antenna only (no u.FL on this board)
    #define HAL_HAS_SPEAKER         0   // Dropped — pin budget + small-board context
    #define HAL_HAS_NEOPIXEL        1   // Onboard WS2812 — flashes red on Geiger pulse
    // V2.4.8 hardcoded HAL_MULTIPAGE_ROTATION=0 here (radiation-only) to match
    // a QT Py + Adafruit 326 deployment. V2.4.9 removed HAL_MULTIPAGE_ROTATION
    // entirely — page-layout selection is now runtime via `display_mode` in
    // /config (auto / radiation / rotation). Auto on QT Py + SSD1306 still
    // picks radiation (small panel → single page).
    #define HAL_HAS_ALS             0   // No onboard ambient-light sensor
    #define HAL_LOG_RING_BYTES      (1 * 1024 * 1024)   // 1 MB of 2 MB PSRAM (50% headroom)
    // V2.3.24: 16 KB snapshot scratch in PSRAM — same generous margin as
    // FeatherS3-D since the PSRAM cost is negligible.
    #define HAL_LOG_SNAP_SCRATCH_BYTES  (16 * 1024)
    // V2.4.6: 32 KB /config form render buffer. PSRAM-backed board — same
    // generous bump as FeatherS3-D.
    #define HAL_CFG_FORM_BUF_SIZE   (32 * 1024)

    // Geiger / HV pins — wired to QT Py A0/A1/A2 castellated pads. Same
    // function-per-position as feathers3_d so a wiring harness designed for
    // the Feather form factor can drop onto the QT Py by re-mapping only the
    // GPIO numbers (positions A3/A4/A5 don't exist on this small board).
    //
    // Position    QT Py ESP32-PICO    Notes
    // --------    ----------------    ------------------------------------
    // A0          GPIO 26             RTC, DAC2, ADC2 — robust output
    // A1          GPIO 25             RTC, DAC1, ADC2 — interrupt-capable
    // A2          GPIO 27             RTC, ADC2, touch — interrupt-capable
    #define PIN_HV_CAP_FULL_INPUT   26   // A0  — comparator interrupt (digital)
    #define PIN_GMC_COUNT_INPUT     25   // A1  — Geiger pulse interrupt
    #define PIN_HV_FET_OUTPUT       27   // A2  — HV MOSFET gate (LEDC PWM via gptimer)

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

    // RESERVED pins — do not repurpose:
    //   IO0   BOOT button (strap)
    //   IO5   NeoPixel data (strap; default pull-up — fine if only driven post-boot)
    //   IO6   internal flash (CMD)         IO9   internal PSRAM (SD2)
    //   IO10  internal PSRAM (SD3)         IO11  internal flash (CMD2)
    //   IO12  MTDI strap (DANGEROUS — pulling HIGH at boot bricks flash boot until power-cycle)
    //   IO15  MTDO strap (silences boot log if LOW at boot — annoying but recoverable)

#else
    #error "No board defined. Set -DBOARD_HELTEC_V2=1 / -DBOARD_FEATHERS3_D=1 / -DBOARD_ADAFRUIT_QTPY_ESP32_PICO=1 via CMake."
#endif
