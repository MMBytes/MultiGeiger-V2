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
    #define HAL_LOG_RING_BYTES      (60 * 1024)  // Internal SRAM only — keep small
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
    #define HAL_HAS_OLED            0   // No onboard display
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
    // PIN_ANTENNA_SELECT controls the onboard SPDT RF switch (NOT exposed as a
    // user header — this is an MCU↔switch trace internal to the FeatherS3-D).
    // Per the FeatherS3-D pinout silkscreen:
    //   IO41 = HIGH → u.FL external antenna connector
    //   IO41 = LOW  → onboard PCB chip antenna
    #define HAL_ANTENNA_SELECT_VERIFIED     1
    #define PIN_ANTENNA_SELECT             41
    #define ANTENNA_SELECT_HIGH_IS_EXTERNAL 1

    // Geiger / HV pins — wired to Feather A0..A5 hole positions for cross-board
    // portability. A0..A5 are guaranteed analog-capable across the Feather
    // form factor; on ESP32-family Feathers all six also support digital I/O,
    // PWM (LEDC) and interrupts. Same PCB harness should drop onto a different
    // Feather (e.g. Adafruit ESP32-S3 #5323) by changing only the GPIO numbers
    // below — physical hole positions on the carrier stay identical.
    //
    // Position    FeatherS3-D    Adafruit ESP32-S3 Feather (#5323) — for ref
    // --------    -----------    ------------------------------------------
    // A0          GPIO 17        GPIO 18
    // A1          GPIO 18        GPIO 17
    // A2          GPIO 14        GPIO 16
    // A3          GPIO 12        GPIO 15
    // A4          GPIO  6        GPIO 14
    // A5          GPIO  5        GPIO  8
    #define PIN_HV_CAP_FULL_INPUT   17   // A0  — comparator interrupt (digital)
    #define PIN_GMC_COUNT_INPUT     18   // A1  — Geiger pulse interrupt
    #define PIN_HV_FET_OUTPUT       14   // A2  — HV MOSFET gate (LEDC PWM via gptimer)

    // Piezo pins
    #define PIN_SPEAKER_P           12   // A3  — LEDC PWM
    #define PIN_SPEAKER_N            6   // A4  — digital low

    // Onboard Blue LED (FeatherS3-D internal — IO13). Drives during LED-tick
    // if the config flag is set; harmless if not.
    #define PIN_LED_BUILTIN         13

    // I2C bus = STEMMA QT / Qwiic connector (the env sensor breakout plugs in
    // here directly via a Qwiic cable; no I2C wiring lands on the PCB).
    #define PIN_I2C_SDA              8
    #define PIN_I2C_SCL              9

    // No OLED on this board — PIN_OLED_RESET intentionally undefined.
    // display.c provides no-op stubs when HAL_HAS_OLED == 0.

    // RESERVED for future hardware test jumper (HWTESTPIN — A5 / IO5).
    // Position locked in the wire harness; firmware does not yet read it.
    // #define PIN_HWTEST              5

    // RESERVED pins on FeatherS3-D — never repurpose these in firmware:
    //   IO0  strap (BOOT button)         IO19/20  native USB D-/D+
    //   IO3  strap                       IO34     VBUS-present detect
    //   IO45/46 strap                    IO39     LDO2 enable (controls 3V3.2)
    //   IO40 onboard RGB LED             IO4      ambient light sensor
    //   IO2  fuel gauge interrupt        IO8/9    Qwiic + fuel gauge bus
    //   IO41 antenna SPDT select (used by PIN_ANTENNA_SELECT above)

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
    #define HAL_HAS_OLED            0   // No onboard display, no OLED in deployment
    #define HAL_HAS_PSRAM           1   // 2 MB in-package SiP PSRAM
    #define HAL_HAS_NATIVE_USB      0   // USB-C via CH9102F or CP2102N UART bridge (NOT native — original ESP32 has no USB-OTG)
    #define HAL_HAS_VEXT_GATE       0   // No power gate — STEMMA QT bus always powered
    #define HAL_HAS_ANTENNA_SWITCH  0   // PCB antenna only (no u.FL on this board)
    #define HAL_HAS_SPEAKER         0   // Dropped — pin budget + small-board context
    #define HAL_HAS_NEOPIXEL        1   // Onboard WS2812 — flashes red on Geiger pulse
    #define HAL_LOG_RING_BYTES      (1 * 1024 * 1024)   // 1 MB of 2 MB PSRAM (50% headroom)
    // V2.3.24: 16 KB snapshot scratch in PSRAM — same generous margin as
    // FeatherS3-D since the PSRAM cost is negligible.
    #define HAL_LOG_SNAP_SCRATCH_BYTES  (16 * 1024)

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
