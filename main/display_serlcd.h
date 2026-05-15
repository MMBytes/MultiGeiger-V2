#pragma once

/** @file
 *  @brief SparkFun 20x4 SerLCD (RGB Backlight, Qwiic) backend.
 *
 *  Character LCD with onboard ATmega328p I²C-to-LCD bridge running the
 *  SparkFun "OpenLCD" firmware (github.com/sparkfun/OpenLCD). Default
 *  I²C address 0x72. Renders ASCII text only — no bitmap framebuffer,
 *  so the radiation `display_running()` layout (large CPM digits, etc)
 *  is NOT implemented. The Environment view IS — and lives in the same
 *  4 rows × 20 cols grid the LCD provides natively.
 *
 *  Internal-to-display.c only — public callers go through the unified
 *  `display_*()` functions in display.h, which dispatch to whichever
 *  backend ACK'd at boot (SerLCD@0x72 preferred, SSD1309/SSD1306@0x3C
 *  as fallback).
 *
 *  Hardware:
 *    Bus       : same Qwiic / STEMMA bus as other I²C devices on the
 *                board (FeatherS3-D STEMMA2 in the V2.3.28 deployment).
 *    Voltage   : 3.3 V via Qwiic, 4.5–41 mA depending on backlight.
 *    Boot      : ATmega bootloader takes ~500 ms before ACKing — caller
 *                MUST wait that long after powering the rail before
 *                calling display_serlcd_init().
 *
 *  Protocol summary (OpenLCD, command bytes per
 *  github.com/sparkfun/SparkFun_SerLCD_Arduino_Library/src/SerLCD.h):
 *    Plain text          : written into the cursor position
 *    0xFE prefix         : HD44780-style LCD command (cursor, clear, on/off)
 *    0x7C prefix         : OpenLCD setting command (RGB, contrast, address...)
 *    50 ms wait          : required after init sequence
 *    10 ms wait          : recommended between commands
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "display.h"      // display_snapshot_t

/** @brief Probe + initialise SerLCD on the given bus.
 *
 *  Returns ESP_OK if a SerLCD ACK'd at 0x72 and the init sequence
 *  completed. Returns ESP_ERR_NOT_FOUND if no ACK at 0x72, so caller
 *  can move on to other display types. Other esp_err_t values indicate
 *  bus-add or post-probe init failures.
 *
 *  Caller MUST delay >=500 ms after powering the bus rail (LDO2 enable
 *  on FeatherS3-D) before calling — the ATmega328p bridge needs that
 *  long to finish its bootloader and start ACKing on I²C.
 */
esp_err_t display_serlcd_init(i2c_master_bus_handle_t bus);

/** @brief Draw the boot splash. */
void display_serlcd_boot_screen(const char *version_str);

/** @brief Set the RGB backlight (0..255 each channel; 0,0,0 = off). */
void display_serlcd_set_backlight(uint8_t r, uint8_t g, uint8_t b);

/** @brief Clear the display (cursor returns home). */
void display_serlcd_clear(void);

// V2.3.29: per-page render functions invoked by the display_task in
// display.c on each 5 s tick. Each clears the display, draws the page,
// and returns. snap is the latest sensor-data snapshot from main.c.
// _uploads and _system read live state via tx_get_stats / esp_timer /
// heap accessors — no snapshot needed.
void display_serlcd_render_env(const display_snapshot_t *snap);
void display_serlcd_render_pm_mass(const display_snapshot_t *snap);
void display_serlcd_render_pm_number(const display_snapshot_t *snap);
void display_serlcd_render_uploads(void);
void display_serlcd_render_system(void);
