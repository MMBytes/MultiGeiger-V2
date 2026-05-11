#pragma once

/** @file
 *  @brief WS2812 NeoPixel driver — single-pixel RGB output via RMT.
 *
 *  Hand-rolled to keep the dependency surface small. Uses ESP-IDF's RMT TX
 *  channel + the built-in `bytes_encoder` configured for WS2812B timing.
 *  No external led_strip component needed.
 *
 *  Build gating: every entry point is a no-op when `HAL_HAS_NEOPIXEL == 0`,
 *  so callers can invoke them unconditionally. neopixel_init() returns
 *  ESP_OK silently on no-NeoPixel boards.
 *
 *  Concurrency model: writes happen on a small dedicated FreeRTOS task that
 *  blocks on a task notification. The tube pulse ISR notifies the task —
 *  ISR-side cost is one xTaskNotifyGiveFromISR. The task drives the pixel
 *  red briefly (~50 ms), then black, then blocks again. Natural rate-limit:
 *  even at high CPM, visible flashes are bounded to ~10/s.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/** @brief Initialise the NeoPixel power gate, RMT channel, and worker task.
 *
 *  Safe to call once at boot. Returns ESP_OK on no-NeoPixel boards (no-op).
 *  After init the pixel is black; call neopixel_set_rgb() to drive a colour
 *  directly, or neopixel_register_pulse_tick() to flash on every tube pulse.
 */
esp_err_t neopixel_init(void);

/** @brief Drive the pixel to a specific RGB colour, immediately.
 *
 *  Each component 0..255. Synchronous — returns after the RMT TX completes
 *  (~30 µs for one pixel). Safe from any task context but NOT from ISR.
 */
void neopixel_set_rgb(uint8_t r, uint8_t g, uint8_t b);

/** @brief Register the tube-pulse ISR hook so each Geiger pulse triggers a
 *         brief red flash on the pixel.
 *
 *  Calls tube_set_pulse_callback() internally with an IRAM-safe handler.
 *  No-op on boards without HAL_HAS_NEOPIXEL.
 */
void neopixel_register_pulse_tick(void);
