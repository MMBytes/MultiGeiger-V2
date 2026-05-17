// SparkFun 20x4 SerLCD (RGB Backlight, Qwiic) — OpenLCD I²C protocol.
// Internal backend for display.c — see display_serlcd.h for the API surface
// and protocol overview. Compiled in whenever HAL_HAS_OLED is set (this is
// a misnomer post-V2.3.28 — the gate really means "any I²C display backend
// might be fitted on this board"; left as-is to avoid touching every board
// branch in hal.h).

#include "display_serlcd.h"
#include "hal.h"

#if HAL_HAS_OLED

#include <string.h>
#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main_status.h"    // V2.4.1 (A4): consolidated status snapshot
#include "transmission.h"   // V2.3.29: tx_get_stats() for the Uploads page

// main.c-defined accessor (separate from main_status — controls per-target
// row visibility on the Uploads page).
extern bool main_target_enabled(int target_id);

static const char *TAG = "serlcd";

// I²C address + protocol prefixes (OpenLCD firmware constants —
// github.com/sparkfun/SparkFun_SerLCD_Arduino_Library/blob/master/src/SerLCD.h)
#define SERLCD_ADDR             0x72
#define SERLCD_SETTING_PREFIX   0x7C   // OpenLCD setting command (RGB, contrast...)
#define SERLCD_SPECIAL_PREFIX   0xFE   // HD44780 special command (cursor, clear...)

// OpenLCD setting commands (0x7C-prefixed).
#define SERLCD_CMD_CLEAR             0x2D
#define SERLCD_CMD_SET_RGB           0x2B
#define SERLCD_CMD_DISABLE_SPLASH    0x31

// HD44780 LCD commands (0xFE-prefixed).
#define LCD_DISPLAYCONTROL_BASE      0x08
#define LCD_DISPLAY_ON               0x04   // OR into display-control base
#define LCD_ENTRYMODE_BASE           0x04
#define LCD_ENTRYMODE_LTR            0x02   // OR into entry-mode base (left-to-right)
#define LCD_SETDDRAMADDR             0x80

// Display geometry — 20 columns × 4 rows.
#define ROWS 4
#define COLS 20

// DDRAM offset per row — the 4-row HD44780 split (rows 2/3 sit at the
// "second display" addresses 0x14/0x54 because HD44780 was originally
// designed for 1- and 2-row panels).
static const uint8_t ROW_OFFSETS[4] = { 0x00, 0x40, 0x14, 0x54 };

static i2c_master_dev_handle_t s_dev = NULL;

// ------------------------------------------------------------------
// I²C primitives — every command/data write is wrapped to centralise
// the 200 ms transmit timeout and the recommended 10 ms inter-command
// quiet period (the ATmega328p bridge needs settle time between bytes
// or it drops chars on the LCD bus).
// ------------------------------------------------------------------

static esp_err_t serlcd_write(const uint8_t *buf, size_t n) {
    return i2c_master_transmit(s_dev, buf, n, 200);
}

static esp_err_t serlcd_send_special(uint8_t cmd) {
    uint8_t buf[2] = { SERLCD_SPECIAL_PREFIX, cmd };
    esp_err_t e = serlcd_write(buf, 2);
    vTaskDelay(pdMS_TO_TICKS(10));
    return e;
}

static esp_err_t serlcd_send_setting(uint8_t cmd) {
    uint8_t buf[2] = { SERLCD_SETTING_PREFIX, cmd };
    esp_err_t e = serlcd_write(buf, 2);
    vTaskDelay(pdMS_TO_TICKS(10));
    return e;
}

static void serlcd_set_cursor(uint8_t col, uint8_t row) {
    if (row >= ROWS) row = ROWS - 1;
    if (col >= COLS) col = COLS - 1;
    serlcd_send_special(LCD_SETDDRAMADDR | (col + ROW_OFFSETS[row]));
}

// Write a NUL-terminated string at the current cursor. Caller positions
// the cursor first via serlcd_set_cursor().
static void serlcd_write_text(const char *s) {
    size_t n = strlen(s);
    if (n == 0) return;
    if (n > COLS) n = COLS;   // never overflow the row — wraps onto an
                              // unrelated row in HD44780 4-row addressing
    serlcd_write((const uint8_t *)s, n);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// ------------------------------------------------------------------
// Public API (called from display.c)
// ------------------------------------------------------------------

void display_serlcd_clear(void) {
    if (!s_dev) return;
    serlcd_send_setting(SERLCD_CMD_CLEAR);
}

void display_serlcd_set_backlight(uint8_t r, uint8_t g, uint8_t b) {
    if (!s_dev) return;
    // setFastBacklight protocol: 0x7C, 0x2B, R, G, B (raw 0..255 each).
    uint8_t buf[5] = { SERLCD_SETTING_PREFIX, SERLCD_CMD_SET_RGB, r, g, b };
    serlcd_write(buf, 5);
    vTaskDelay(pdMS_TO_TICKS(10));
}

esp_err_t display_serlcd_init(i2c_master_bus_handle_t bus) {
    if (!bus) return ESP_ERR_INVALID_ARG;

    // Probe — fast path. If no ACK, we're done; caller can fall through
    // to a different display backend.
    if (i2c_master_probe(bus, SERLCD_ADDR, 100) != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SERLCD_ADDR,
        // SerLCD's ATmega328p I²C bridge is documented up to 400 kHz but
        // SparkFun's own examples run it at 100 kHz for reliability with
        // the 80-char incoming buffer. Stick with that conservative rate.
        .scl_speed_hz    = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &devcfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_bus_add_device failed: %s",
                 esp_err_to_name(err));
        s_dev = NULL;
        return err;
    }

    // Init sequence per SparkFun_SerLCD_Arduino_Library SerLCD::init():
    //  1. display ON, no cursor, no blink
    //  2. entry mode left-to-right, no auto-shift
    //  3. clear
    //  4. 50 ms quiet period before issuing any further commands
    serlcd_send_special(LCD_DISPLAYCONTROL_BASE | LCD_DISPLAY_ON);
    serlcd_send_special(LCD_ENTRYMODE_BASE | LCD_ENTRYMODE_LTR);
    serlcd_send_setting(SERLCD_CMD_CLEAR);

    // Suppress the SparkFun splash on subsequent power-cycles so we own
    // the boot-time pixels. Persistent NVS write inside the ATmega; only
    // needed once but cheap and idempotent.
    serlcd_send_setting(SERLCD_CMD_DISABLE_SPLASH);

    vTaskDelay(pdMS_TO_TICKS(50));

    // Default backlight: white (equal R=G=B=255) at full brightness.
    // display_set_contrast() in display.c overrides this with the user-
    // configured brightness immediately after init returns; the (255,
    // 255, 255) here is just a "panel comes up visible" safety so the
    // boot splash isn't drawn into a dark backlight if for any reason
    // display_set_contrast() doesn't run.
    display_serlcd_set_backlight(255, 255, 255);

    ESP_LOGI(TAG, "SerLCD up at 0x%02X (20x4)", SERLCD_ADDR);
    return ESP_OK;
}

void display_serlcd_boot_screen(const char *version_str) {
    if (!s_dev) return;
    display_serlcd_clear();
    serlcd_set_cursor(3, 0);
    serlcd_write_text("Multi-Geiger");
    if (version_str) {
        serlcd_set_cursor(7, 1);
        serlcd_write_text(version_str);
    }
    serlcd_set_cursor(3, 3);
    serlcd_write_text("Initializing...");
}

// ====================================================================
// V2.3.29: per-page render functions invoked by display_task in
// display.c. Each clears the panel (with backlight re-armed in case
// it was off), draws its page, and returns. The display task itself
// handles the show_display=false case (panel cleared + backlight off);
// these renderers always assume "show".
// ====================================================================

void display_serlcd_render_env(const display_snapshot_t *snap) {
    if (!s_dev || !snap) return;
    display_serlcd_clear();

    char line[32];
    if (snap->env_valid) {
        snprintf(line, sizeof(line), "Temperature  %5.1f C", snap->env_t_c);
        serlcd_set_cursor(0, 0); serlcd_write_text(line);
        snprintf(line, sizeof(line), "Humidity     %5.1f %%", snap->env_h_pct);
        serlcd_set_cursor(0, 1); serlcd_write_text(line);
        snprintf(line, sizeof(line), "Pressure    %4d hPa",
                 (int)(snap->env_p_pa / 100.0f + 0.5f));
        serlcd_set_cursor(0, 2); serlcd_write_text(line);
    } else {
        serlcd_set_cursor(0, 0); serlcd_write_text("Temperature   --- C");
        serlcd_set_cursor(0, 1); serlcd_write_text("Humidity      --- %");
        serlcd_set_cursor(0, 2); serlcd_write_text("Pressure    --- hPa");
    }
    if (snap->noise_valid) {
        snprintf(line, sizeof(line), "Noise    %5.1f dB(A)", snap->noise.laeq);
        serlcd_set_cursor(0, 3); serlcd_write_text(line);
    }
}

// 20-char rows aligned: "PMx.y" (5) + 6 spaces + 3-char value + " ug/m3" (6) = 20.
// Pressure-style padding for PM10 is "PM10  " (6) — equal label width.
void display_serlcd_render_pm_mass(const display_snapshot_t *snap) {
    if (!s_dev || !snap) return;
    display_serlcd_clear();

    char line[32];
    if (snap->pm_valid) {
        snprintf(line, sizeof(line), "PM1.0      %3d ug/m3",
                 (int)(snap->pm.pm1_0 + 0.5f));
        serlcd_set_cursor(0, 0); serlcd_write_text(line);
        snprintf(line, sizeof(line), "PM2.5      %3d ug/m3",
                 (int)(snap->pm.pm2_5 + 0.5f));
        serlcd_set_cursor(0, 1); serlcd_write_text(line);
        snprintf(line, sizeof(line), "PM4.0      %3d ug/m3",
                 (int)(snap->pm.pm4_0 + 0.5f));
        serlcd_set_cursor(0, 2); serlcd_write_text(line);
        snprintf(line, sizeof(line), "PM10       %3d ug/m3",
                 (int)(snap->pm.pm10 + 0.5f));
        serlcd_set_cursor(0, 3); serlcd_write_text(line);
    } else {
        serlcd_set_cursor(0, 0); serlcd_write_text("PM1.0      --- ug/m3");
        serlcd_set_cursor(0, 1); serlcd_write_text("PM2.5      --- ug/m3");
        serlcd_set_cursor(0, 2); serlcd_write_text("PM4.0      --- ug/m3");
        serlcd_set_cursor(0, 3); serlcd_write_text("PM10       --- ug/m3");
    }
}

// 20-char rows: "nX.X" (4) + 6 spaces + 5-char value + " /cm3" (5) = 20.
void display_serlcd_render_pm_number(const display_snapshot_t *snap) {
    if (!s_dev || !snap) return;
    display_serlcd_clear();

    char line[32];
    if (snap->pm_valid) {
        snprintf(line, sizeof(line), "n0.5      %5d /cm3",
                 (int)(snap->pm.nc0_5 + 0.5f));
        serlcd_set_cursor(0, 0); serlcd_write_text(line);
        snprintf(line, sizeof(line), "n1.0      %5d /cm3",
                 (int)(snap->pm.nc1_0 + 0.5f));
        serlcd_set_cursor(0, 1); serlcd_write_text(line);
        snprintf(line, sizeof(line), "n2.5      %5d /cm3",
                 (int)(snap->pm.nc2_5 + 0.5f));
        serlcd_set_cursor(0, 2); serlcd_write_text(line);
        snprintf(line, sizeof(line), "n4.0      %5d /cm3",
                 (int)(snap->pm.nc4_0 + 0.5f));
        serlcd_set_cursor(0, 3); serlcd_write_text(line);
    } else {
        serlcd_set_cursor(0, 0); serlcd_write_text("n0.5        --- /cm3");
        serlcd_set_cursor(0, 1); serlcd_write_text("n1.0        --- /cm3");
        serlcd_set_cursor(0, 2); serlcd_write_text("n2.5        --- /cm3");
        serlcd_set_cursor(0, 3); serlcd_write_text("n4.0        --- /cm3");
    }
}

// Only renders rows for targets enabled in /config — compacted from row 0
// down. Radmon is intentionally never shown (per page design). The
// display task skips this whole page when none of MA/SC/OS/AQ are
// enabled, so a fully-empty panel is never reached.
//
// Dynamic-width counts to match the user's mockup — value field shifts
// right as the counters grow. Labels left-padded to 9 chars so the
// value start column stays stable until counts hit 4+ digits.
void display_serlcd_render_uploads(void) {
    if (!s_dev) return;
    display_serlcd_clear();

    const struct { int id; const char *label; } entries[] = {
        { TX_TARGET_MADAVI,  "Madavi" },
        { TX_TARGET_SENSORC, "Sensor" },
        { TX_TARGET_OSM,     "OSM" },
        { TX_TARGET_AQI,     "AQI" },
    };

    tx_target_stats_t st;
    char line[32];
    int row = 0;
    for (size_t i = 0; i < sizeof(entries) / sizeof(entries[0]) && row < 4; i++) {
        if (!main_target_enabled(entries[i].id)) continue;
        tx_get_stats(entries[i].id, &st);
        snprintf(line, sizeof(line), "%-9s%lu/%lu",
                 entries[i].label,
                 (unsigned long)st.succeeded,
                 (unsigned long)st.attempted);
        serlcd_set_cursor(0, row);
        serlcd_write_text(line);
        row++;
    }
}

// SerLCD System page — option A from the design discussion: no Min row,
// keeps the 4-row budget on Cycles / Uptime / Free / Max. (OLED has the
// extra Min row via the 1x-font lower half of its mixed-font layout.)
void display_serlcd_render_system(void) {
    if (!s_dev) return;
    display_serlcd_clear();

    char line[32];

    main_status_t st;
    main_status_snapshot(&st);
    snprintf(line, sizeof(line), "TX Cycles %lu", (unsigned long)st.cycles);
    serlcd_set_cursor(0, 0); serlcd_write_text(line);

    int64_t us = esp_timer_get_time();
    uint32_t total_s = (uint32_t)(us / 1000000);
    int days  = total_s / 86400;
    int hours = (total_s % 86400) / 3600;
    int mins  = (total_s % 3600) / 60;
    snprintf(line, sizeof(line), "Uptime %dd %dh %dm", days, hours, mins);
    serlcd_set_cursor(0, 1); serlcd_write_text(line);

    snprintf(line, sizeof(line), "Free %lu bytes",
             (unsigned long)esp_get_free_heap_size());
    serlcd_set_cursor(0, 2); serlcd_write_text(line);

    snprintf(line, sizeof(line), "Max %lu bytes",
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    serlcd_set_cursor(0, 3); serlcd_write_text(line);
}

#else  // HAL_HAS_OLED == 0 — board has no display family compiled in.
// No-op stubs for boards (currently QT Py) that disable the display layer
// entirely. Keeps display.c's dispatch code able to call into this file
// unconditionally without #if guards everywhere.

esp_err_t display_serlcd_init(i2c_master_bus_handle_t bus) {
    (void)bus;
    return ESP_ERR_NOT_SUPPORTED;
}
void display_serlcd_boot_screen(const char *v) { (void)v; }
void display_serlcd_render_env(const display_snapshot_t *snap) { (void)snap; }
void display_serlcd_render_pm_mass(const display_snapshot_t *snap) { (void)snap; }
void display_serlcd_render_pm_number(const display_snapshot_t *snap) { (void)snap; }
void display_serlcd_render_uploads(void) {}
void display_serlcd_render_system(void) {}
void display_serlcd_set_backlight(uint8_t r, uint8_t g, uint8_t b) {
    (void)r; (void)g; (void)b;
}
void display_serlcd_clear(void) {}

#endif  // HAL_HAS_OLED
