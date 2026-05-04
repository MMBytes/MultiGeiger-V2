#pragma once

/** @file
 *  @brief Board-level hardware abstraction — pin map and feature flags.
 *
 *  One of `BOARD_HELTEC_V2` or `BOARD_FEATHERS3_D` is defined by the top-level
 *  CMakeLists.txt based on the `BOARD` variable (default `heltec_v2`). All
 *  module .c/.h files include this header and reference pins / features by
 *  the macros below — never by raw GPIO numbers.
 *
 *  Adding a new board: pick a `BOARD_*` macro name, add a branch below that
 *  sets the same `PIN_*` and `HAL_HAS_*` symbols, and extend the CMake board
 *  selector in `CMakeLists.txt`.
 */

#if defined(BOARD_HELTEC_V2)

    #define BOARD_NAME              "heltec_v2"
    #define HAL_HAS_OLED            1   // SSD1306 on shared I2C bus
    #define HAL_HAS_PSRAM           0
    #define HAL_HAS_NATIVE_USB      0   // Console via CP2102 UART0
    #define HAL_HAS_VEXT_GATE       1   // GPIO 21 = active-LOW MOSFET on V2+ Heltec carriers
    #define HAL_HAS_ANTENNA_SWITCH  0   // PCB antenna only (no u.FL / no RF switch)
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
    // A2          GPIO 14        GPIO 14
    // A3          GPIO 12        GPIO 12
    // A4          GPIO  6        GPIO  6
    // A5          GPIO  5        GPIO  5
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

#else
    #error "No board defined. Set -DBOARD_HELTEC_V2=1 or -DBOARD_FEATHERS3_D=1 via CMake."
#endif
