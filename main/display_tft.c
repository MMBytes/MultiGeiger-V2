// Adafruit ESP32-S3 TFT Feather (#5483) onboard 240x135 ST7789 color SPI
// TFT. Internal backend for display.c — see display_tft.h for the API
// surface and architecture overview. Compiled in for every board (like
// display_serlcd.c) but only does real work when HAL_HAS_TFT is set;
// every other board gets the no-op stub at the bottom of this file.

#include "display_tft.h"
#include "hal.h"

#if HAL_HAS_TFT

#include <string.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"     // esp_lcd_new_panel_io_spi / esp_lcd_panel_io_spi_config_t
#include "esp_lcd_panel_vendor.h" // esp_lcd_new_panel_st7789 / esp_lcd_panel_dev_config_t
#include "esp_lcd_panel_ops.h"    // generic panel ops: reset/init/swap_xy/mirror/set_gap/disp_on_off/draw_bitmap
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"     // esp_get_free_heap_size / esp_get_minimum_free_heap_size
#include "esp_timer.h"
#include "main_status.h"    // V2.4.1 (A4): consolidated status snapshot
#include "transmission.h"   // tx_get_stats() for the Uploads page

// main.c-defined accessor (separate from main_status — controls per-target
// row visibility on the Uploads page). Same declaration as display.c/
// display_serlcd.c use for the identical Uploads-page gating logic.
extern bool main_target_enabled(int target_id);

static const char *TAG = "tft";

// Panel geometry (product #5483 datasheet). The ST7789 controller's native
// RAM is 240x320; this module's glass is a 240x135 cut with a 40/53 px
// row/col gap into that RAM — see esp_lcd_panel_set_gap() below.
#define TFT_W 240
#define TFT_H 135

// RGB565 colors used by the (currently monochrome, white-on-black) page
// layout — kept as named constants so a future pass can color-code labels
// without hunting for magic numbers.
#define TFT_COLOR_BLACK 0x0000
#define TFT_COLOR_WHITE 0xFFFF

// 2x scale on the 8x8 FONT8 glyphs — 16x16 px per character, giving a
// 15 col x 8 row grid (240/16, 135/16 rounded down) that maps directly
// onto the OLED backend's row-based page layouts (render_oled_* in
// display.c), just with fewer/larger characters per row.
#define GLYPH_W 16
#define GLYPH_H 16

static esp_lcd_panel_io_handle_t s_io = NULL;
static esp_lcd_panel_handle_t s_panel = NULL;
static uint16_t *s_fb = NULL;   // TFT_W * TFT_H px, PSRAM-resident RGB565
static bool s_backlight_on = false;

// ------------------------------------------------------------------
// Framebuffer primitives
// ------------------------------------------------------------------

static void fb_fill(uint16_t color) {
    if (!s_fb) return;
    for (size_t i = 0; i < (size_t)TFT_W * TFT_H; i++) s_fb[i] = color;
}

static void fb_set_px(int x, int y, uint16_t color) {
    if (x < 0 || x >= TFT_W || y < 0 || y >= TFT_H) return;
    s_fb[y * TFT_W + x] = color;
}

// Draws one FONT8 glyph at 2x scale (each source pixel becomes a 2x2
// block). FONT8 is declared extern in display.h and defined once, at
// file scope, in display.c — shared verbatim with the OLED backend so
// there is only one glyph table in the whole firmware image.
static void draw_glyph_2x(int x, int y, char ch, uint16_t color) {
    if (ch < 0x20 || ch > 0x7E) ch = ' ';
    const uint8_t *rows = FONT8[(unsigned char)ch - 0x20];
    for (int row = 0; row < 8; row++) {
        uint8_t bits = rows[row];
        for (int col = 0; col < 8; col++) {
            if (!(bits & (0x80 >> col))) continue;
            int px = x + col * 2;
            int py = y + row * 2;
            fb_set_px(px,     py,     color);
            fb_set_px(px + 1, py,     color);
            fb_set_px(px,     py + 1, color);
            fb_set_px(px + 1, py + 1, color);
        }
    }
}

static void draw_string_2x(int x, int y, const char *s, uint16_t color) {
    if (!s) return;
    int cx = x;
    for (; *s; s++) {
        draw_glyph_2x(cx, y, *s, color);
        cx += GLYPH_W;
    }
}

// Pushes the whole framebuffer to the panel in one transfer. esp_lcd's
// draw_bitmap end coordinates are exclusive — (0,0)..(TFT_W,TFT_H) covers
// every pixel.
static void fb_push(void) {
    if (!s_panel || !s_fb) return;
    esp_err_t err = esp_lcd_panel_draw_bitmap(s_panel, 0, 0, TFT_W, TFT_H, s_fb);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "draw_bitmap failed: %s", esp_err_to_name(err));
    }
}

// ------------------------------------------------------------------
// Public API (called from display.c)
// ------------------------------------------------------------------

void display_tft_set_backlight(uint8_t pct) {
    s_backlight_on = (pct > 0);
    gpio_set_level(PIN_TFT_BACKLITE, s_backlight_on ? 1 : 0);
}

void display_tft_clear(void) {
    fb_fill(TFT_COLOR_BLACK);
    fb_push();
}

esp_err_t display_tft_init(void) {
    // V2.6.11 review fix: the shared TFT/I2C peripheral rail (GPIO21) used
    // to be driven here, but display_tft_init() runs (via display_setup())
    // AFTER fuel_gauge_init() and every I2C sensor probe in main.c — every
    // one of those NACKs on a dead bus. The gate drive now lives in
    // i2c_bus_get_primary() (main/i2c_bus.c), which main.c calls first, so
    // the rail is already up by the time we get here. See hal.h
    // PIN_I2C_POWER_GATE note.
    gpio_config_t bl_cfg = {
        .pin_bit_mask = 1ULL << PIN_TFT_BACKLITE,
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&bl_cfg);
    gpio_set_level(PIN_TFT_BACKLITE, 0);   // stay dark until display_tft_set_backlight()

    spi_bus_config_t buscfg = {
        .mosi_io_num     = PIN_TFT_MOSI,
        .sclk_io_num     = PIN_TFT_SCK,
        .miso_io_num     = -1,   // ST7789 is write-only in this driver
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = TFT_W * TFT_H * 2,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num       = PIN_TFT_CS,
        .dc_gpio_num       = PIN_TFT_DC,
        .spi_mode          = 0,
        .pclk_hz           = 40 * 1000 * 1000,
        .lcd_cmd_bits      = 8,
        .lcd_param_bits    = 8,
        .trans_queue_depth = 10,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &s_io);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_lcd_new_panel_io_spi failed: %s", esp_err_to_name(err));
        spi_bus_free(SPI2_HOST);
        return err;
    }

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_TFT_RST,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    err = esp_lcd_new_panel_st7789(s_io, &panel_config, &s_panel);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_lcd_new_panel_st7789 failed: %s", esp_err_to_name(err));
        esp_lcd_panel_io_del(s_io);
        s_io = NULL;
        spi_bus_free(SPI2_HOST);
        return err;
    }

    // Bring-up sequence — bail out (with full cleanup) on the first failure
    // rather than pressing on with an unconfigured/half-configured panel.
    err = esp_lcd_panel_reset(s_panel);
    if (err == ESP_OK) err = esp_lcd_panel_init(s_panel);
    // Landscape orientation + the 40/53 px gap into the ST7789's native
    // 240x320 RAM — bring-up params derived for product #5483, see design
    // spec §4.
    if (err == ESP_OK) err = esp_lcd_panel_swap_xy(s_panel, true);
    if (err == ESP_OK) err = esp_lcd_panel_mirror(s_panel, true, false);
    if (err == ESP_OK) err = esp_lcd_panel_set_gap(s_panel, 40, 53);
    if (err == ESP_OK) err = esp_lcd_panel_disp_on_off(s_panel, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ST7789 bring-up failed: %s", esp_err_to_name(err));
        esp_lcd_panel_del(s_panel);
        s_panel = NULL;
        esp_lcd_panel_io_del(s_io);
        s_io = NULL;
        spi_bus_free(SPI2_HOST);
        return err;
    }

    s_fb = heap_caps_malloc((size_t)TFT_W * TFT_H * 2, MALLOC_CAP_SPIRAM);
    if (!s_fb) {
        ESP_LOGW(TAG, "framebuffer alloc failed (%d bytes)", TFT_W * TFT_H * 2);
        esp_lcd_panel_del(s_panel);
        s_panel = NULL;
        esp_lcd_panel_io_del(s_io);
        s_io = NULL;
        spi_bus_free(SPI2_HOST);
        return ESP_ERR_NO_MEM;
    }
    display_tft_clear();   // blank the panel — mirrors the OLED backend's
                            // unconditional post-init clear (apply_oled_init_sequence())

    ESP_LOGI(TAG, "ST7789 up, %dx%d, PSRAM framebuffer %d bytes",
             TFT_W, TFT_H, TFT_W * TFT_H * 2);
    return ESP_OK;
}

void display_tft_boot_screen(const char *version_str) {
    if (!s_fb) return;
    fb_fill(TFT_COLOR_BLACK);
    draw_string_2x(0, 0, "Multi-Geiger", TFT_COLOR_WHITE);
    if (version_str) {
        draw_string_2x(0, GLYPH_H, version_str, TFT_COLOR_WHITE);
    }
    fb_push();
}

// ====================================================================
// V2.6.11: per-page render functions invoked by display_task in
// display.c's HAL_HAS_TFT branch. Each fills the framebuffer, draws its
// page, and pushes the whole frame. Row content mirrors render_oled_*
// (display.c) and display_serlcd_render_* (display_serlcd.c) — same
// fields, formatted for this panel's 15 col x 8 row (2x font) grid.
// ====================================================================

void display_tft_render_env(const display_snapshot_t *snap) {
    if (!s_fb || !snap) return;
    fb_fill(TFT_COLOR_BLACK);

    char line[16];
    if (snap->env_valid) {
        snprintf(line, sizeof(line), "%5.1f C", snap->env_t_c);
        draw_string_2x(0, 0 * GLYPH_H, line, TFT_COLOR_WHITE);
        snprintf(line, sizeof(line), "%5.1f %%", snap->env_h_pct);
        draw_string_2x(0, 1 * GLYPH_H, line, TFT_COLOR_WHITE);
        snprintf(line, sizeof(line), "%4d hPa", (int)(snap->env_p_pa / 100.0f + 0.5f));
        draw_string_2x(0, 2 * GLYPH_H, line, TFT_COLOR_WHITE);
    } else {
        draw_string_2x(0, 0 * GLYPH_H, "--- C", TFT_COLOR_WHITE);
        draw_string_2x(0, 1 * GLYPH_H, "--- %", TFT_COLOR_WHITE);
        draw_string_2x(0, 2 * GLYPH_H, "--- hPa", TFT_COLOR_WHITE);
    }
    if (snap->noise_valid) {
        snprintf(line, sizeof(line), "%5.1f dB", snap->noise.laeq);
        draw_string_2x(0, 3 * GLYPH_H, line, TFT_COLOR_WHITE);
    }
    fb_push();
}

void display_tft_render_pm_mass(const display_snapshot_t *snap) {
    if (!s_fb || !snap) return;
    fb_fill(TFT_COLOR_BLACK);

    char line[16];
    if (snap->pm_valid) {
        snprintf(line, sizeof(line), "PM1.0%3d", (int)(snap->pm.pm1_0 + 0.5f));
        draw_string_2x(0, 0 * GLYPH_H, line, TFT_COLOR_WHITE);
        snprintf(line, sizeof(line), "PM2.5%3d", (int)(snap->pm.pm2_5 + 0.5f));
        draw_string_2x(0, 1 * GLYPH_H, line, TFT_COLOR_WHITE);
        snprintf(line, sizeof(line), "PM4.0%3d", (int)(snap->pm.pm4_0 + 0.5f));
        draw_string_2x(0, 2 * GLYPH_H, line, TFT_COLOR_WHITE);
        snprintf(line, sizeof(line), "PM10 %3d", (int)(snap->pm.pm10 + 0.5f));
        draw_string_2x(0, 3 * GLYPH_H, line, TFT_COLOR_WHITE);
    } else {
        draw_string_2x(0, 0 * GLYPH_H, "PM1.0---", TFT_COLOR_WHITE);
        draw_string_2x(0, 1 * GLYPH_H, "PM2.5---", TFT_COLOR_WHITE);
        draw_string_2x(0, 2 * GLYPH_H, "PM4.0---", TFT_COLOR_WHITE);
        draw_string_2x(0, 3 * GLYPH_H, "PM10 ---", TFT_COLOR_WHITE);
    }
    fb_push();
}

// Same value formatting as fmt_n_value() in display.c (kept as a local
// copy — both are tiny, static, and per-backend; not worth a shared
// helper header for one 7-line function).
static void fmt_n_value(char *out, size_t outsz, float v) {
    if (v < 1000.0f) {
        snprintf(out, outsz, "%4d", (int)(v + 0.5f));
    } else if (v < 10000.0f) {
        snprintf(out, outsz, "%3.1fk", v / 1000.0f);
    } else {
        int k = (int)(v / 1000.0f + 0.5f);
        if (k > 99) k = 99;
        snprintf(out, outsz, "%2dk ", k);
    }
}

void display_tft_render_pm_number(const display_snapshot_t *snap) {
    if (!s_fb || !snap) return;
    fb_fill(TFT_COLOR_BLACK);

    char line[16];
    char val[8];
    if (snap->pm_valid) {
        fmt_n_value(val, sizeof(val), snap->pm.nc0_5);
        snprintf(line, sizeof(line), "n0.5%s", val);
        draw_string_2x(0, 0 * GLYPH_H, line, TFT_COLOR_WHITE);
        fmt_n_value(val, sizeof(val), snap->pm.nc1_0);
        snprintf(line, sizeof(line), "n1.0%s", val);
        draw_string_2x(0, 1 * GLYPH_H, line, TFT_COLOR_WHITE);
        fmt_n_value(val, sizeof(val), snap->pm.nc2_5);
        snprintf(line, sizeof(line), "n2.5%s", val);
        draw_string_2x(0, 2 * GLYPH_H, line, TFT_COLOR_WHITE);
        fmt_n_value(val, sizeof(val), snap->pm.nc4_0);
        snprintf(line, sizeof(line), "n4.0%s", val);
        draw_string_2x(0, 3 * GLYPH_H, line, TFT_COLOR_WHITE);
    } else {
        draw_string_2x(0, 0 * GLYPH_H, "n0.5----", TFT_COLOR_WHITE);
        draw_string_2x(0, 1 * GLYPH_H, "n1.0----", TFT_COLOR_WHITE);
        draw_string_2x(0, 2 * GLYPH_H, "n2.5----", TFT_COLOR_WHITE);
        draw_string_2x(0, 3 * GLYPH_H, "n4.0----", TFT_COLOR_WHITE);
    }
    fb_push();
}

// Only renders rows for targets currently enabled in /config — compacted
// from the top, same rule as render_oled_uploads()/
// display_serlcd_render_uploads(). Radmon is intentionally never shown.
static int upload_pct(tx_target_id_t id) {
    tx_target_stats_t st;
    tx_get_stats(id, &st);
    if (st.attempted == 0) return 0;
    return (int)((100ULL * st.succeeded) / st.attempted);
}

void display_tft_render_uploads(void) {
    if (!s_fb) return;
    fb_fill(TFT_COLOR_BLACK);

    char line[16];
    const struct { int id; const char *abbrev; } entries[] = {
        { TX_TARGET_MADAVI,  "MA" },
        { TX_TARGET_SENSORC, "SC" },
        { TX_TARGET_OSM,     "OS" },
        { TX_TARGET_AQI,     "AQ" },
    };
    int row = 0;
    for (size_t i = 0; i < sizeof(entries) / sizeof(entries[0]) && row < 4; i++) {
        if (!main_target_enabled(entries[i].id)) continue;
        snprintf(line, sizeof(line), "%s %3d%%", entries[i].abbrev, upload_pct(entries[i].id));
        draw_string_2x(0, row * GLYPH_H, line, TFT_COLOR_WHITE);
        row++;
    }
    fb_push();
}

void display_tft_render_system(void) {
    if (!s_fb) return;
    fb_fill(TFT_COLOR_BLACK);

    char line[16];

    main_status_t st;
    main_status_snapshot(&st);
    uint32_t cycles = st.cycles;
    if (cycles < 100000) {
        snprintf(line, sizeof(line), "C %5lu", (unsigned long)cycles);
    } else {
        snprintf(line, sizeof(line), "C%5.1fk", cycles / 1000.0f);
    }
    draw_string_2x(0, 0 * GLYPH_H, line, TFT_COLOR_WHITE);

    int64_t us = esp_timer_get_time();
    uint32_t total_s = (uint32_t)(us / 1000000);
    int days = total_s / 86400;
    if (days >= 1) {
        int hours = (total_s % 86400) / 3600;
        snprintf(line, sizeof(line), "%3dd%2dh", days, hours);
    } else {
        int hours = total_s / 3600;
        int mins  = (total_s % 3600) / 60;
        snprintf(line, sizeof(line), "%2dh %2dm", hours, mins);
    }
    draw_string_2x(0, 1 * GLYPH_H, line, TFT_COLOR_WHITE);

    snprintf(line, sizeof(line), "Free%7lu", (unsigned long)esp_get_free_heap_size());
    draw_string_2x(0, 2 * GLYPH_H, line, TFT_COLOR_WHITE);

    snprintf(line, sizeof(line), "Min %7lu", (unsigned long)esp_get_minimum_free_heap_size());
    draw_string_2x(0, 3 * GLYPH_H, line, TFT_COLOR_WHITE);

    snprintf(line, sizeof(line), "Max %7lu",
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    draw_string_2x(0, 4 * GLYPH_H, line, TFT_COLOR_WHITE);

    fb_push();
}

#else  // HAL_HAS_TFT == 0 — board has no onboard TFT.
// No-op stubs so display.c's dispatch code (and any future caller) can
// call into this file unconditionally without #if guards everywhere —
// same pattern as display_serlcd.c's HAL_HAS_OLED==0 stub block.

esp_err_t display_tft_init(void) { return ESP_ERR_NOT_SUPPORTED; }
void display_tft_boot_screen(const char *version_str) { (void)version_str; }
void display_tft_set_backlight(uint8_t pct) { (void)pct; }
void display_tft_clear(void) {}
void display_tft_render_env(const display_snapshot_t *snap) { (void)snap; }
void display_tft_render_pm_mass(const display_snapshot_t *snap) { (void)snap; }
void display_tft_render_pm_number(const display_snapshot_t *snap) { (void)snap; }
void display_tft_render_uploads(void) {}
void display_tft_render_system(void) {}

#endif  // HAL_HAS_TFT
