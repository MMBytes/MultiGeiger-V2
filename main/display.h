#pragma once

/** @file
 *  @brief OLED display driver — SSD1306 128x64 over I2C.
 *
 *  On the Heltec WiFi Kit 32 V2 the panel shares the I2C bus with the
 *  BME280 (SDA=GPIO4, SCL=GPIO15). The dedicated reset line is GPIO16.
 *  On boards without an onboard OLED (HAL_HAS_OLED == 0) the entire driver
 *  collapses to no-op stubs so callers in main.c don't change shape.
 *
 *  Layout: a boot splash, then a running screen with a relative-time stamp
 *  and nSv/h on top, a large CPM readout in the middle, and a five-character
 *  status line at the bottom reporting WiFi, server, and HV state.
 */

#include <stdbool.h>

// Status subsystem indices.
#define DSP_STATUS_WIFI   0
#define DSP_STATUS_SCOMM  1
#define DSP_STATUS_MADAVI 2
#define DSP_STATUS_RADMON 3
#define DSP_STATUS_HV     4
#define DSP_STATUS_MAX    5

// WiFi status values.
#define DSP_WIFI_OFF        0
#define DSP_WIFI_CONNECTED  1
#define DSP_WIFI_ERROR      2
#define DSP_WIFI_CONNECTING 3
#define DSP_WIFI_AP         4

// Per-server status values (shared encoding for sensor.community / Madavi / Radmon).
#define DSP_SRV_OFF     0
#define DSP_SRV_IDLE    1
#define DSP_SRV_ERROR   2
#define DSP_SRV_SENDING 3
#define DSP_SRV_INIT    4

// HV status values.
#define DSP_HV_NODISPLAY 0
#define DSP_HV_OK        1
#define DSP_HV_ERROR     2

/** @brief Initialise the OLED.
 *
 *  Pass show_display=false to keep the panel dark; the driver still
 *  initialises so set_status / display_running become safe no-ops.
 *  Returns true if the panel answered probe; false otherwise, after which
 *  subsequent calls are no-ops.
 */
bool display_setup(bool show_display);

/** @brief Draw the fixed boot splash (project name and version string). */
void display_boot_screen(void);

/** @brief Draw the running screen.
 *  @param time_sec     Seconds since boot (for the top-left timestamp).
 *  @param rad_nsvph    Dose rate as µSv/h × 1000.
 *  @param cpm          Counts per minute (large digits).
 *  @param use_display  If false, the panel is cleared once and the call
 *                      becomes a no-op until re-enabled.
 */
void display_running(int time_sec, int rad_nsvph, int cpm, bool use_display);

/** @brief Update a status indicator and redraw the status line.
 *         Safe to call from any task context.
 */
void display_set_status(int index, int value);
