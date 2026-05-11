#pragma once
// Bump before build; commit after successful flash.
//
// V2.3.20 — feature release. Adds 4th board target + NeoPixel driver +
// HAL refactor for board-conditional log ring sizing.
//
//   1. **New board: Adafruit QT Py ESP32-PICO (PID 5395).** Original ESP32
//      LX6 SiP — *not* S3, despite the modern USB-C form factor. ESP32-
//      PICO-V3-02 with in-package 8 MB flash + 2 MB PSRAM. 11 castellated
//      GPIO pads + STEMMA QT connector + onboard NeoPixel.
//
//      Pin budget on the QT Py is much tighter than the Feather form
//      factor, so this board configuration drops the OLED (already gone
//      on PSRAM-class boards) AND the piezo speaker. Geiger uses 3 pads:
//      A0=IO26 (HV_CAP_FUL), A1=IO25 (GMC_COUNT), A2=IO27 (HV_FET).
//      Same function-per-A-position as feathers3_d so a Feather-style
//      wiring harness drops onto the QT Py with only the GPIO numbers
//      changing in hal.h. Env sensor plugs into the STEMMA QT connector
//      (IO22 SDA / IO19 SCL — the secondary I²C bus, no GPIO pad cost).
//
//      Strapping pin hazards on the QT Py — documented in hal.h:
//        - IO12 (MTDI / MISO pad): driving HIGH at boot bricks flash boot
//          until power-cycle. Avoid for outputs that go HIGH.
//        - IO15 (MTDO / A3 pad): driving LOW at boot silences the boot
//          log. Annoying but recoverable.
//        - IO5 (NeoPixel data) and IO0 (BOOT button): also strap pins,
//          handled by Adafruit and our boot sequence respectively.
//
//   2. **NeoPixel driver — `neopixel.[ch]` (~150 LOC).** Hand-rolled
//      WS2812 driver using ESP-IDF's RMT TX channel + bytes_encoder.
//      No external led_strip component needed.
//
//      Concurrency model: tube-pulse ISR notifies a small dedicated
//      FreeRTOS task via task notification; task wakes, drives pixel
//      red briefly (~40 ms), then black, then blocks again. ISR-side
//      cost is one xTaskNotifyGiveFromISR. Multiple coincident pulses
//      collapse to one visible flash (eye couldn't distinguish anyway).
//      Brightness intentionally low (R=20/255) to avoid being annoying
//      in dark rooms while still visible at arm's length.
//
//      Wired only when `HAL_HAS_NEOPIXEL=1` (currently QT Py only).
//      All other boards get no-op stubs — calls in main.c stay
//      unconditional, no `#if HAL_HAS_NEOPIXEL` guards needed at the
//      call sites.
//
//   3. **HAL refactor — `HAL_HAS_SPEAKER`, `HAL_HAS_NEOPIXEL`,
//      `HAL_LOG_RING_BYTES`.** Three new flags that every board branch
//      must define. Replaces the old `#if HAL_HAS_PSRAM` cascade in
//      `applog.c` (now consults `HAL_LOG_RING_BYTES` directly) and
//      gates the entire `speaker.c` body so boards without a piezo
//      compile to no-op stubs (mirror of `display.c`'s `HAL_HAS_OLED`
//      pattern). `HAL_LOG_RING_BYTES` per board: 60 KB on Heltec
//      (internal SRAM), 4 MB on FeatherS3-D (8 MB PSRAM), 1 MB on QT Py
//      (2 MB PSRAM — 50 % headroom).
//
//   4. **OTA upload form recognises the new board.** `http_server.c`
//      adds a 4th `BOARD_ADAFRUIT_QTPY_ESP32_PICO` branch in the
//      board-specific upload prompt — pairs with V2.3.13's chip-ID
//      validation to steer users to the right binary before upload.
//
// Heltec 8 MB / Heltec 4 MB / FeatherS3-D builds: byte-for-byte
// equivalent in observed behaviour to V2.3.19 — the new flags default
// to their previous values for those boards (HAS_SPEAKER=1,
// HAS_NEOPIXEL=0, LOG_RING_BYTES same as before).
//
// OTA-safe from V2.3.19 (no partition layout changes for existing
// boards, no sdkconfig changes for them). 20 release artefacts
// (5 × 4 boards).
#define VERSION_STR "V2.3.20"
