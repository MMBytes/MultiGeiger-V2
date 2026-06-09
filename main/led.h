#pragma once

/** @file
 *  @brief V2.5.19: plain onboard user-LED pulse-tick driver.
 *
 *  Some boards have a single GPIO-driven user LED but NO piezo and NO NeoPixel
 *  — the Seeed XIAO ESP32-S3 (GPIO21, active-LOW) is the case that motivated
 *  this. On those boards the per-GM-pulse "tick" feedback that speaker.c
 *  provides (for boards with HAL_HAS_SPEAKER) and neopixel.c provides (for
 *  HAL_HAS_NEOPIXEL) is absent — speaker.c is stubbed out and there is no
 *  NeoPixel — so the user LED was never driven. This module fills that gap.
 *
 *  It owns the tube pulse callback (tube_set_pulse_callback) the SAME way
 *  neopixel.c does, so it must only register on boards where neither speaker.c
 *  nor neopixel.c claims the single callback slot. That condition is encoded
 *  in led.c as `defined(PIN_LED_BUILTIN) && !HAL_HAS_SPEAKER && !HAL_HAS_NEOPIXEL`;
 *  on every other board both functions compile to no-op stubs so callers can
 *  invoke them unconditionally (same contract as neopixel.c).
 *
 *  Polarity is handled by the board's HAL_LED_ACTIVE_LOW flag (default 0 =
 *  active-high if a board defines PIN_LED_BUILTIN without it).
 */

#include "esp_err.h"

/** @brief Configure the user-LED GPIO as an output and drive it OFF.
 *  No-op (returns ESP_OK) on boards without a led.c-owned user LED.
 */
esp_err_t led_init(void);

/** @brief Create the flash worker and register the tube-pulse callback so the
 *  LED blinks briefly on every GM pulse. Call only when led_tick is enabled
 *  (the caller gates on g_cfg.led_tick) and the tube is enabled. Idempotent.
 *  No-op stub on boards without a led.c-owned user LED.
 */
void led_register_pulse_tick(void);
