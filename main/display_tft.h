#pragma once

/** @file
 *  @brief Adafruit ESP32-S3 TFT Feather (#5483) onboard 240x135 ST7789 color
 *  SPI TFT.
 *
 *  Driven via ESP-IDF's built-in esp_lcd component (esp_lcd_panel_io_spi +
 *  esp_lcd_panel_st7789) — no GFX/bitmap library, no Component Registry
 *  dependency (both REQUIRES entries ship in-tree). A single PSRAM-resident
 *  RGB565 framebuffer is drawn with a handful of primitives built on the
 *  same FONT8 8x8 bitmap font the OLED backend uses (declared extern in
 *  display.h), then pushed to the panel in one esp_lcd_panel_draw_bitmap()
 *  call per page change.
 *
 *  Internal-to-display.c only — public callers go through the unified
 *  display_*() functions in display.h. On this board (HAL_HAS_TFT) those
 *  dispatch straight here; there is no I2C probe/fallback chain like the
 *  OLED/SerLCD boards since this board's TFT is always fitted (HAL_HAS_TFT
 *  and HAL_HAS_OLED are mutually exclusive per board — see hal.h).
 *
 *  Hardware: see hal.h under BOARD_ADAFRUIT_ESP32S3_TFT_FEATHER and
 *  docs/superpowers/specs/2026-07-10-adafruit-esp32s3-tft-feather-board-port-design.md
 *  §2/§4 for the full pin map and bring-up parameter derivation.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "display.h"      // display_snapshot_t

/** @brief Power the shared TFT/I2C rail (PIN_I2C_POWER_GATE) and bring up
 *  the ST7789 panel: SPI bus init, esp_lcd panel IO + driver creation,
 *  reset, orientation (swap_xy/mirror/gap), display-on, and PSRAM
 *  framebuffer allocation.
 *
 *  Returns ESP_OK on success. Returns ESP_ERR_NOT_SUPPORTED on boards
 *  without HAL_HAS_TFT (stub build). Any esp_lcd/SPI/allocation failure
 *  propagates its esp_err_t.
 */
esp_err_t display_tft_init(void);

/** @brief Draw the boot splash (project name + version string). */
void display_tft_boot_screen(const char *version_str);

/** @brief Backlight on/off via PIN_TFT_BACKLITE. pct==0 turns the
 *  backlight off; any pct>0 turns it fully on — plain GPIO in this first
 *  pass, no PWM dimming (design spec §11, out of scope).
 */
void display_tft_set_backlight(uint8_t pct);

/** @brief Clear the framebuffer to black and push it to the panel. */
void display_tft_clear(void);

// V2.6.11: per-page render functions invoked by the display_task in
// display.c's HAL_HAS_TFT branch on each 5 s tick. Each fills the
// framebuffer, draws the page, and pushes it via one whole-frame
// esp_lcd_panel_draw_bitmap() call. snap is the latest sensor-data
// snapshot from main.c. _uploads and _system read live state via
// tx_get_stats / esp_timer / heap accessors — no snapshot needed.
void display_tft_render_env(const display_snapshot_t *snap);
void display_tft_render_pm_mass(const display_snapshot_t *snap);
void display_tft_render_pm_number(const display_snapshot_t *snap);
void display_tft_render_uploads(void);
void display_tft_render_system(void);

/** @brief V2.6.32: Radiation page for the rotation — reads rad_nsvph /
 *  rad_cpm from the snapshot (written per TX cycle by main.c), uptime
 *  live. Layout mirrors the OLED single-page radiation screen: uptime +
 *  nSv/h header, big centred CPM.
 */
void display_tft_render_radiation(const display_snapshot_t *snap);

/** @brief V2.6.32: single-page radiation-mode entry point — same layout
 *  as display_tft_render_radiation() but fed directly from display.c's
 *  display_running() args (mode=RADIATION boards, one render per TX
 *  cycle, no rotation task).
 */
void display_tft_render_running(int time_sec, int rad_nsvph, int cpm);
