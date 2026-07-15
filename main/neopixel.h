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
 *
 *  One-shot only: on a board with the pulse-tick worker running
 *  (neopixel_register_pulse_tick()), the very next tube pulse restores
 *  whatever colour neopixel_set_alert() last set (black by default) —
 *  it does NOT preserve a colour set via this call. Use neopixel_set_alert()
 *  for anything that must survive pulses.
 */
void neopixel_set_rgb(uint8_t r, uint8_t g, uint8_t b);

/** @brief Drive the pixel to a solid colour that SURVIVES the per-pulse
 *         flash worker — for standing alerts (e.g. sd_logger.c's 3-failure
 *         SD-error indication), where a Geiger pulse must not wipe it.
 *
 *  Sets the "restore to" colour the pulse worker uses after each flash, and
 *  draws it immediately (so it's visible even if no pulses arrive — e.g.
 *  tube_enabled=false, or the flash between pulses). Call with (0,0,0) to
 *  clear the alert; the intended caller for that is the alert owner itself
 *  on recovery (e.g. sd_logger_cycle() after a successful write), not an
 *  unrelated module.
 */
void neopixel_set_alert(uint8_t r, uint8_t g, uint8_t b);

/** @brief Register the tube-pulse ISR hook so each Geiger pulse triggers a
 *         brief red flash on the pixel.
 *
 *  Calls tube_set_pulse_callback() internally with an IRAM-safe handler —
 *  EXCEPT on boards that also have HAL_HAS_SPEAKER, where speaker.c already
 *  owns the single tube-pulse callback slot (for its own audio tick) and
 *  drives this pixel directly via neopixel_notify_pulse() instead. On such
 *  boards this call still creates the pulse-flash worker task (needed for
 *  that direct-drive path) but does not touch the callback registration.
 *  No-op entirely on boards without HAL_HAS_NEOPIXEL.
 */
void neopixel_register_pulse_tick(void);

/** @brief Directly trigger one pulse-flash, bypassing the tube-pulse
 *         callback registration.
 *
 *  For boards with both HAL_HAS_SPEAKER and HAL_HAS_NEOPIXEL (e.g.
 *  sparkfun_thing_plus_esp32s3, which has no separate PIN_LED_BUILTIN):
 *  speaker.c owns the tube callback slot and calls this from its own
 *  IRAM-resident handler when led_tick is enabled. IRAM-safe, ISR-callable.
 *  No-op on boards without HAL_HAS_NEOPIXEL, or before
 *  neopixel_register_pulse_tick() has created the worker task.
 */
void neopixel_notify_pulse(void);
