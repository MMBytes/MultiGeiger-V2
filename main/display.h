#pragma once

/** @file
 *  @brief OLED display driver — SSD1306 / SSD1309 128x64 over I2C.
 *
 *  Per-board panel + bus:
 *    Heltec V2 (+ 4MB clone) : onboard SSD1306 on the env_sensor I2C bus
 *                              (SDA=GPIO4, SCL=GPIO15, dedicated reset=GPIO16).
 *    FeatherS3-D             : external SSD1309 breakout on STEMMA2
 *                              (SDA=IO16, SCL=IO15), powered from LDO2;
 *                              4-pin module — no reset line, chip POR only.
 *
 *  SSD1306 and SSD1309 are register-compatible — the same init sequence and
 *  command set drive both. Boot log distinguishes them via OLED_CHIP_NAME
 *  in display.c (per-board #define) so a glance at /log shows which silicon
 *  is fitted without needing to know the board variant.
 *
 *  On boards without an OLED (HAL_HAS_OLED == 0) the entire driver
 *  collapses to no-op stubs so callers in main.c don't change shape.
 *
 *  Two layouts:
 *    display_running()       — radiation-focused: time + nSv/h on top, big
 *                              CPM digits, 5-char W/sc/M/R/HV status line.
 *                              Driven per-TX-cycle on Heltec.
 *    display_update_snapshot — V2.3.29 multi-page rotation on FeatherS3-D.
 *                              Display task wakes every 5 s and rotates
 *                              through Env / PM Mass / PM Number / Uploads
 *                              / System pages, skipping any whose sensor
 *                              isn't present.
 */

#include <stdbool.h>
#include <stdint.h>
#include "pm_sensor.h"      // pm_sample_t
#include "noise_sensor.h"   // noise_sample_t

// V2.3.29: snapshot pushed from main task → display task on FeatherS3-D
// every TX cycle. Display task reads at 5 s tick to drive page rotation
// (Env / PM Mass / PM Number / Uploads / System). Single-writer torn-
// tolerant pattern — main task is the only writer (once per ~150 s),
// display task is the only reader. Dynamic data (uptime, free heap, TX
// cycles, upload stats) is NOT in this struct — read via accessors at
// render time so it updates every page refresh, not every TX cycle.
typedef struct {
    bool   env_valid;
    float  env_t_c;
    float  env_h_pct;
    float  env_p_pa;
    bool   pm_valid;
    pm_sample_t pm;
    bool   noise_valid;
    noise_sample_t noise;
} display_snapshot_t;

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

/** @brief Initialise the OLED / SerLCD.
 *
 *  Pass show_display=false to keep the panel dark; the driver still
 *  initialises so set_status / display_running become safe no-ops.
 *  brightness_pct is applied via display_set_contrast() at the end of
 *  init — caller passes g_cfg.oled_brightness_pct.
 *  Returns true if the panel answered probe; false otherwise, after which
 *  subsequent calls are no-ops.
 */
bool display_setup(bool show_display, uint8_t brightness_pct);

/** @brief V2.3.30: live-apply OLED contrast / SerLCD backlight brightness.
 *
 *  pct is 10..100 (clamped). For OLED, sends the SSD1306/9 contrast
 *  command (0x81 + register byte) — pixels brighten/dim within
 *  microseconds. For SerLCD, writes the RGB backlight at white at the
 *  given level — visible within ~10 ms of I²C settling. No-op when
 *  show_display is false (the new value is stored for next show=true).
 */
void display_set_contrast(uint8_t pct);

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

/** @brief V2.3.29: feed the FeatherS3-D multi-page rotation task with this
 *  cycle's sensor readings. Called once per do_tx_cycle from main.c.
 *
 *  On non-FeatherS3-D builds this is a no-op (Heltec keeps display_running
 *  as before). On FeatherS3-D the display task wakes every 5 s, advances
 *  the page index, and renders the page using the latest snapshot. Sensor
 *  fields are torn-tolerant 32-bit scalars / aggregates so reads from the
 *  display task don't need a lock.
 */
void display_update_snapshot(const display_snapshot_t *snap);
