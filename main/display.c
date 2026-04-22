// OLED display — SSD1306 128x64 over I2C on the shared BME280 bus.
// Hand-rolled SSD1306 driver (page-addressing mode) — no U8g2 dependency.
// Layout: boot splash, then a running screen with time + nSv/h on top, big
// CPM in the middle, status line at the bottom.

#include "display.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bme280.h"
#include "version.h"

static const char *TAG = "display";

#define SSD1306_ADDR    0x3C
#define PIN_OLED_RST    GPIO_NUM_16
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_PAGES      8

static i2c_master_dev_handle_t s_dev = NULL;
static bool s_show    = false;   // set by display_setup()
static bool s_cleared = true;    // panel is blank (no running screen drawn)

static int s_status[DSP_STATUS_MAX] = { 0, 0, 0, 0, 0 };

// One string per subsystem, indexed by status value.
static const char *STATUS_CHARS[DSP_STATUS_MAX] = {
    ".W0wA",   // WiFi:              off, connected, error, connecting, AP
    ".s1S?",   // sensor.community:  off, idle, error, sending, init
    ".m2M?",   // Madavi:            off, idle, error, sending, init
    ".r3R?",   // Radmon:            off, idle, error, sending, init
    ".H7",     // HV:                nodisplay, ok, error
};

// ------------------------------------------------------------------
// 8x8 bitmap font, printable ASCII 0x20..0x7E (95 glyphs).
// Row-major: each byte is one row, bit 7 = leftmost column.
// Public-domain font (dhepper/font8x8 "basic" subset).
// ------------------------------------------------------------------
static const uint8_t FONT8[95][8] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // ' '
    { 0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00 },  // '!'
    { 0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // '"'
    { 0x36, 0x36, 0x7F, 0x36, 0x7F, 0x36, 0x36, 0x00 },  // '#'
    { 0x0C, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x0C, 0x00 },  // '$'
    { 0x00, 0x63, 0x33, 0x18, 0x0C, 0x66, 0x63, 0x00 },  // '%'
    { 0x1C, 0x36, 0x1C, 0x6E, 0x3B, 0x33, 0x6E, 0x00 },  // '&'
    { 0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 },  // '\''
    { 0x18, 0x0C, 0x06, 0x06, 0x06, 0x0C, 0x18, 0x00 },  // '('
    { 0x06, 0x0C, 0x18, 0x18, 0x18, 0x0C, 0x06, 0x00 },  // ')'
    { 0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00 },  // '*'
    { 0x00, 0x0C, 0x0C, 0x3F, 0x0C, 0x0C, 0x00, 0x00 },  // '+'
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x06 },  // ','
    { 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00 },  // '-'
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00 },  // '.'
    { 0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00 },  // '/'
    { 0x3E, 0x63, 0x73, 0x7B, 0x6F, 0x67, 0x3E, 0x00 },  // '0'
    { 0x0C, 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x3F, 0x00 },  // '1'
    { 0x1E, 0x33, 0x30, 0x1C, 0x06, 0x33, 0x3F, 0x00 },  // '2'
    { 0x1E, 0x33, 0x30, 0x1C, 0x30, 0x33, 0x1E, 0x00 },  // '3'
    { 0x38, 0x3C, 0x36, 0x33, 0x7F, 0x30, 0x78, 0x00 },  // '4'
    { 0x3F, 0x03, 0x1F, 0x30, 0x30, 0x33, 0x1E, 0x00 },  // '5'
    { 0x1C, 0x06, 0x03, 0x1F, 0x33, 0x33, 0x1E, 0x00 },  // '6'
    { 0x3F, 0x33, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x00 },  // '7'
    { 0x1E, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x1E, 0x00 },  // '8'
    { 0x1E, 0x33, 0x33, 0x3E, 0x30, 0x18, 0x0E, 0x00 },  // '9'
    { 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x00 },  // ':'
    { 0x00, 0x0C, 0x0C, 0x00, 0x00, 0x0C, 0x0C, 0x06 },  // ';'
    { 0x18, 0x0C, 0x06, 0x03, 0x06, 0x0C, 0x18, 0x00 },  // '<'
    { 0x00, 0x00, 0x3F, 0x00, 0x00, 0x3F, 0x00, 0x00 },  // '='
    { 0x06, 0x0C, 0x18, 0x30, 0x18, 0x0C, 0x06, 0x00 },  // '>'
    { 0x1E, 0x33, 0x30, 0x18, 0x0C, 0x00, 0x0C, 0x00 },  // '?'
    { 0x3E, 0x63, 0x7B, 0x7B, 0x7B, 0x03, 0x1E, 0x00 },  // '@'
    { 0x0C, 0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x00 },  // 'A'
    { 0x3F, 0x66, 0x66, 0x3E, 0x66, 0x66, 0x3F, 0x00 },  // 'B'
    { 0x3C, 0x66, 0x03, 0x03, 0x03, 0x66, 0x3C, 0x00 },  // 'C'
    { 0x1F, 0x36, 0x66, 0x66, 0x66, 0x36, 0x1F, 0x00 },  // 'D'
    { 0x7F, 0x46, 0x16, 0x1E, 0x16, 0x46, 0x7F, 0x00 },  // 'E'
    { 0x7F, 0x46, 0x16, 0x1E, 0x16, 0x06, 0x0F, 0x00 },  // 'F'
    { 0x3C, 0x66, 0x03, 0x03, 0x73, 0x66, 0x7C, 0x00 },  // 'G'
    { 0x33, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33, 0x00 },  // 'H'
    { 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00 },  // 'I'
    { 0x78, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E, 0x00 },  // 'J'
    { 0x67, 0x66, 0x36, 0x1E, 0x36, 0x66, 0x67, 0x00 },  // 'K'
    { 0x0F, 0x06, 0x06, 0x06, 0x46, 0x66, 0x7F, 0x00 },  // 'L'
    { 0x63, 0x77, 0x7F, 0x7F, 0x6B, 0x63, 0x63, 0x00 },  // 'M'
    { 0x63, 0x67, 0x6F, 0x7B, 0x73, 0x63, 0x63, 0x00 },  // 'N'
    { 0x1C, 0x36, 0x63, 0x63, 0x63, 0x36, 0x1C, 0x00 },  // 'O'
    { 0x3F, 0x66, 0x66, 0x3E, 0x06, 0x06, 0x0F, 0x00 },  // 'P'
    { 0x1E, 0x33, 0x33, 0x33, 0x3B, 0x1E, 0x38, 0x00 },  // 'Q'
    { 0x3F, 0x66, 0x66, 0x3E, 0x36, 0x66, 0x67, 0x00 },  // 'R'
    { 0x1E, 0x33, 0x07, 0x0E, 0x38, 0x33, 0x1E, 0x00 },  // 'S'
    { 0x3F, 0x2D, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00 },  // 'T'
    { 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x3F, 0x00 },  // 'U'
    { 0x33, 0x33, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00 },  // 'V'
    { 0x63, 0x63, 0x63, 0x6B, 0x7F, 0x77, 0x63, 0x00 },  // 'W'
    { 0x63, 0x63, 0x36, 0x1C, 0x1C, 0x36, 0x63, 0x00 },  // 'X'
    { 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x0C, 0x1E, 0x00 },  // 'Y'
    { 0x7F, 0x63, 0x31, 0x18, 0x4C, 0x66, 0x7F, 0x00 },  // 'Z'
    { 0x1E, 0x06, 0x06, 0x06, 0x06, 0x06, 0x1E, 0x00 },  // '['
    { 0x03, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x40, 0x00 },  // '\\'
    { 0x1E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1E, 0x00 },  // ']'
    { 0x08, 0x1C, 0x36, 0x63, 0x00, 0x00, 0x00, 0x00 },  // '^'
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF },  // '_'
    { 0x0C, 0x0C, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00 },  // '`'
    { 0x00, 0x00, 0x1E, 0x30, 0x3E, 0x33, 0x6E, 0x00 },  // 'a'
    { 0x07, 0x06, 0x06, 0x3E, 0x66, 0x66, 0x3B, 0x00 },  // 'b'
    { 0x00, 0x00, 0x1E, 0x33, 0x03, 0x33, 0x1E, 0x00 },  // 'c'
    { 0x38, 0x30, 0x30, 0x3E, 0x33, 0x33, 0x6E, 0x00 },  // 'd'
    { 0x00, 0x00, 0x1E, 0x33, 0x3F, 0x03, 0x1E, 0x00 },  // 'e'
    { 0x1C, 0x36, 0x06, 0x0F, 0x06, 0x06, 0x0F, 0x00 },  // 'f'
    { 0x00, 0x00, 0x6E, 0x33, 0x33, 0x3E, 0x30, 0x1F },  // 'g'
    { 0x07, 0x06, 0x36, 0x6E, 0x66, 0x66, 0x67, 0x00 },  // 'h'
    { 0x0C, 0x00, 0x0E, 0x0C, 0x0C, 0x0C, 0x1E, 0x00 },  // 'i'
    { 0x30, 0x00, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1E },  // 'j'
    { 0x07, 0x06, 0x66, 0x36, 0x1E, 0x36, 0x67, 0x00 },  // 'k'
    { 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x1E, 0x00 },  // 'l'
    { 0x00, 0x00, 0x33, 0x7F, 0x7F, 0x6B, 0x63, 0x00 },  // 'm'
    { 0x00, 0x00, 0x1F, 0x33, 0x33, 0x33, 0x33, 0x00 },  // 'n'
    { 0x00, 0x00, 0x1E, 0x33, 0x33, 0x33, 0x1E, 0x00 },  // 'o'
    { 0x00, 0x00, 0x3B, 0x66, 0x66, 0x3E, 0x06, 0x0F },  // 'p'
    { 0x00, 0x00, 0x6E, 0x33, 0x33, 0x3E, 0x30, 0x78 },  // 'q'
    { 0x00, 0x00, 0x3B, 0x6E, 0x66, 0x06, 0x0F, 0x00 },  // 'r'
    { 0x00, 0x00, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x00 },  // 's'
    { 0x08, 0x0C, 0x3E, 0x0C, 0x0C, 0x2C, 0x18, 0x00 },  // 't'
    { 0x00, 0x00, 0x33, 0x33, 0x33, 0x33, 0x6E, 0x00 },  // 'u'
    { 0x00, 0x00, 0x33, 0x33, 0x33, 0x1E, 0x0C, 0x00 },  // 'v'
    { 0x00, 0x00, 0x63, 0x6B, 0x7F, 0x7F, 0x36, 0x00 },  // 'w'
    { 0x00, 0x00, 0x63, 0x36, 0x1C, 0x36, 0x63, 0x00 },  // 'x'
    { 0x00, 0x00, 0x33, 0x33, 0x33, 0x3E, 0x30, 0x1F },  // 'y'
    { 0x00, 0x00, 0x3F, 0x19, 0x0C, 0x26, 0x3F, 0x00 },  // 'z'
    { 0x38, 0x0C, 0x0C, 0x07, 0x0C, 0x0C, 0x38, 0x00 },  // '{'
    { 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x00 },  // '|'
    { 0x07, 0x0C, 0x0C, 0x38, 0x0C, 0x0C, 0x07, 0x00 },  // '}'
    { 0x6E, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  // '~'
};

// ------------------------------------------------------------------
// I2C helpers
// ------------------------------------------------------------------

static esp_err_t oled_cmd(uint8_t c) {
    uint8_t buf[2] = { 0x00, c };  // 0x00 = Co=0, D/C=0 (command stream)
    return i2c_master_transmit(s_dev, buf, 2, 200);
}

static esp_err_t oled_data(const uint8_t *d, int n) {
    // 0x40 = Co=0, D/C=1 (data stream). Prepend the control byte.
    static uint8_t buf[OLED_WIDTH + 1];
    if (n > OLED_WIDTH) n = OLED_WIDTH;
    buf[0] = 0x40;
    memcpy(buf + 1, d, n);
    return i2c_master_transmit(s_dev, buf, n + 1, 200);
}

static void oled_goto(int page, int col) {
    oled_cmd(0xB0 | (page & 0x07));           // set page
    oled_cmd(col & 0x0F);                     // lower nibble of column
    oled_cmd(0x10 | ((col >> 4) & 0x0F));     // upper nibble of column
}

static void oled_clear(void) {
    static const uint8_t zeros[OLED_WIDTH] = { 0 };
    for (int p = 0; p < OLED_PAGES; p++) {
        oled_goto(p, 0);
        oled_data(zeros, OLED_WIDTH);
    }
}

// ------------------------------------------------------------------
// Glyph rendering
// ------------------------------------------------------------------

// Transpose a row-major 8x8 glyph into 8 column-major bytes. Each output
// byte represents one column with bit 0 = top row — matching the SSD1306
// page-addressing layout. dhepper/font8x8 stores bit 0 as the leftmost
// pixel of each row, so col 0 picks bit 0 and col 7 picks bit 7.
static void transpose_char(uint8_t c, uint8_t out[8]) {
    if (c < 0x20 || c > 0x7E) c = '?';
    const uint8_t *rows = FONT8[c - 0x20];
    for (int col = 0; col < 8; col++) {
        uint8_t b = 0;
        uint8_t mask = (uint8_t)(1u << col);  // bit 0 = leftmost column in source
        for (int r = 0; r < 8; r++) {
            if (rows[r] & mask) b |= (1 << r);
        }
        out[col] = b;
    }
}

static void oled_draw_char(int page, int col, char c) {
    uint8_t bytes[8];
    transpose_char((uint8_t)c, bytes);
    oled_goto(page, col);
    oled_data(bytes, 8);
}

static void oled_draw_string(int page, int col, const char *s) {
    while (*s && col <= OLED_WIDTH - 8) {
        oled_draw_char(page, col, *s);
        s++;
        col += 8;
    }
}

static void oled_clear_page(int page) {
    static const uint8_t zeros[OLED_WIDTH] = { 0 };
    oled_goto(page, 0);
    oled_data(zeros, OLED_WIDTH);
}

// Draw a single glyph scaled 2x — 16 px wide × 16 px tall (2 pages).
// Space renders blank, digits render as digits; anything else maps to '?'.
static void oled_draw_glyph_2x(int page, int col, char c) {
    uint8_t bytes[8];
    transpose_char((uint8_t)c, bytes);
    for (int p = 0; p < 2; p++) {
        uint8_t out[16];
        for (int srccol = 0; srccol < 8; srccol++) {
            uint8_t src = bytes[srccol];
            uint8_t half = 0;
            // page 0 = top half (src bits 0..3), page 1 = bottom half (4..7)
            for (int b = 0; b < 4; b++) {
                int srcbit = (p == 0) ? b : (b + 4);
                if (src & (1 << srcbit)) half |= (uint8_t)(0x03 << (b * 2));
            }
            out[srccol * 2]     = half;
            out[srccol * 2 + 1] = half;
        }
        oled_goto(page + p, col);
        oled_data(out, 16);
    }
}

static void oled_draw_string_2x(int page, int col, const char *s) {
    while (*s && col <= OLED_WIDTH - 16) {
        oled_draw_glyph_2x(page, col, *s);
        s++;
        col += 16;
    }
}

// ------------------------------------------------------------------
// Status line
// ------------------------------------------------------------------

static char status_char(int idx) {
    if (idx < 0 || idx >= DSP_STATUS_MAX) return '?';
    int val = s_status[idx];
    const char *chars = STATUS_CHARS[idx];
    int len = (int)strlen(chars);
    return (val >= 0 && val < len) ? chars[val] : '?';
}

static void redraw_status_line(void) {
    char line[17];
    snprintf(line, sizeof(line), "%c %c %c %c %c",
             status_char(0), status_char(1), status_char(2),
             status_char(3), status_char(4));
    oled_clear_page(7);
    oled_draw_string(7, 0, line);
}

// ------------------------------------------------------------------
// Public API
// ------------------------------------------------------------------

bool display_setup(bool show_display) {
    i2c_master_bus_handle_t bus = bme280_get_i2c_bus();
    if (!bus) {
        ESP_LOGW(TAG, "no I2C bus — display disabled");
        return false;
    }

    // Pulse the dedicated reset line. The panel latches into its reset
    // state for ~3 µs minimum; 10 ms either side is plenty.
    gpio_reset_pin(PIN_OLED_RST);
    gpio_set_direction(PIN_OLED_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_OLED_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_OLED_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_OLED_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    if (i2c_master_probe(bus, SSD1306_ADDR, 100) != ESP_OK) {
        ESP_LOGW(TAG, "SSD1306 not responding at 0x%02X — display disabled",
                 SSD1306_ADDR);
        return false;
    }

    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SSD1306_ADDR,
        .scl_speed_hz    = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &devcfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_bus_add_device failed: %s",
                 esp_err_to_name(err));
        s_dev = NULL;
        return false;
    }

    // Standard SSD1306 128x64 init sequence. 0x20 0x02 selects page
    // addressing mode (the mode we drive via oled_goto).
    static const uint8_t init_cmds[] = {
        0xAE,              // display OFF
        0xD5, 0x80,        // clock divide ratio / osc freq
        0xA8, 0x3F,        // multiplex ratio (1/64)
        0xD3, 0x00,        // display offset 0
        0x40,              // start line 0
        0x8D, 0x14,        // charge pump on
        0x20, 0x02,        // memory mode: page addressing
        0xA1,              // segment remap (column 127 mapped to SEG0)
        0xC8,              // COM scan direction: reversed
        0xDA, 0x12,        // COM pins hardware config
        0x81, 0xCF,        // contrast
        0xD9, 0xF1,        // pre-charge period
        0xDB, 0x40,        // VCOMH deselect level
        0xA4,              // entire display follows RAM
        0xA6,              // normal (not inverted)
        0xAF,              // display ON
    };
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        oled_cmd(init_cmds[i]);
    }

    s_show    = show_display;
    s_cleared = true;
    oled_clear();

    ESP_LOGI(TAG, "SSD1306 up at 0x%02X (show=%d)", SSD1306_ADDR, show_display);
    return true;
}

void display_boot_screen(void) {
    if (!s_dev || !s_show) return;
    oled_clear();
    oled_draw_string(0, 0, "  Multi-Geiger");
    oled_draw_string(1, 0, "________________");
    oled_draw_string(5, 0, VERSION_STR);
    s_cleared = false;
}

static void format_time(int secs, char *out, size_t outsz) {
    int mins  = secs / 60;
    int hours = secs / 3600;
    int days  = secs / 86400;
    if (secs < 60)        snprintf(out, outsz, "%2ds", secs);
    else if (mins < 60)   snprintf(out, outsz, "%2dm", mins);
    else if (hours < 24)  snprintf(out, outsz, "%2dh", hours);
    else                  snprintf(out, outsz, "%2dd", days % 100);
}

void display_running(int time_sec, int rad_nsvph, int cpm, bool use_display) {
    if (!s_dev) return;
    if (!use_display) {
        if (!s_cleared) {
            oled_clear();
            s_cleared = true;
        }
        return;
    }
    oled_clear();
    s_cleared = false;

    char ts[4];
    format_time(time_sec, ts, sizeof(ts));
    char line[24];
    snprintf(line, sizeof(line), "%3s%7d nSv/h", ts, rad_nsvph);
    oled_draw_string(0, 0, line);

    // Big CPM: 5 digits × 16 px wide = 80 px, centred → col 24.
    // Pages 3–4 (pixel rows 24–39) keep it clear of both header and status.
    char digits[8];
    snprintf(digits, sizeof(digits), "%5d", cpm);
    oled_draw_string_2x(3, 24, digits);

    redraw_status_line();
}

void display_set_status(int index, int value) {
    if (index < 0 || index >= DSP_STATUS_MAX) return;
    s_status[index] = value;
    if (!s_dev || !s_show || s_cleared) return;
    redraw_status_line();
}
