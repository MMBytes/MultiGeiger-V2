// OLED display — SSD1306 / SSD1309 / SSD1315 128x64 over I2C.
// Hand-rolled driver (page-addressing mode) — no U8g2 dependency. SSD1306,
// SSD1309, and SSD1315 are register-compatible (init sequence + command
// set), so one driver handles all three. Per-board chip name comes from
// OLED_CHIP_NAME below for accurate boot-log identification.
//
//   Heltec V2 / Heltec V2 4MB : onboard SSD1306 on the shared env_sensor bus
//                               (SDA=GPIO4, SCL=GPIO15, RST=GPIO16).
//   FeatherS3-D               : external SSD1309 breakout (Core Electronics
//                               CE09964) on STEMMA2 (SDA=IO16, SCL=IO15),
//                               powered from LDO2 — see bring_up_stemma2_bus.
//   Heltec WiFi LoRa 32 V4-R2 : onboard SSD1315 on a bus dedicated
//                               permanently to the display (SDA=GPIO17,
//                               SCL=GPIO18, RST=GPIO21) — module-internal,
//                               not shared with the env-sensor bus at all
//                               (unlike every board above). Always present
//                               (on-module, not a plug-in breakout) — this
//                               board probes the secondary bus FIRST (see
//                               the BOARD_HELTEC_WIFI_LORA32_V4_R2 branch in
//                               display_setup() below), since the primary
//                               bus can never have a display on it.
//
// Layout: boot splash, then either the radiation-focused running screen
// (time + nSv/h on top, big CPM in the middle, status line at the bottom)
// or the Environment view (T/RH/P at 2x font, no status line).
//
// On boards without an OLED (HAL_HAS_OLED == 0), the entire driver compiles
// down to no-op stubs at the bottom of this file; callers in main.c don't
// need to know whether a display is fitted.

#include "display.h"
#include "display_serlcd.h"
#include "hal.h"

#include <stdint.h>

// ------------------------------------------------------------------
// 8x8 bitmap font, printable ASCII 0x20..0x7E (95 glyphs).
// Row-major: each byte is one row, bit 7 = leftmost column.
// Public-domain font (dhepper/font8x8 "basic" subset).
//
// File scope (not inside #if HAL_HAS_OLED below) and non-static since
// V2.6.11: the ST7789 TFT backend (display_tft.c, HAL_HAS_TFT boards)
// draws from this same table via the extern declaration in display.h —
// one font, two backends, no duplication.
// ------------------------------------------------------------------
const uint8_t FONT8[95][8] = {
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

#if HAL_HAS_OLED

#include <string.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "env_sensor.h"     // env_sensor_present() for page-skip logic
#include "i2c_bus.h"        // V2.3.29: bus lifecycle (replaces in-display bring_up_stemma2_bus)
#include "version.h"

static const char *TAG = "display";

// V2.3.28: which display backend is active. Decided at boot inside
// display_setup() by probing each candidate in priority order:
//   1. SerLCD at 0x72 (mutually exclusive with OLED — SparkFun 20x4)
//   2. SSD1306/SSD1309 at 0x3C
// Only one backend is ever active per boot; swapping the panel requires
// a power-cycle/reboot but no firmware change.
typedef enum {
    BACKEND_NONE = 0,
    BACKEND_OLED,
    BACKEND_SERLCD,
} display_backend_t;
static display_backend_t s_backend = BACKEND_NONE;

// V2.6.30: human-readable panel identity ("SerLCD 20x4 @0x72",
// "SSD1309 128x64 @0x3C", "none") for /status and the syslog boot banner.
// Written once at the probe site that ACK'd inside display_setup() —
// before any reader task exists — and read via display_backend_str().
// The initial "none" doubles as the answer for both "probe found nothing"
// and "display_setup() not yet run".
static char s_backend_desc[24] = "none";

// SSD1306 datasheet defines exactly two slave addresses, selected by the
// SA0 pin (sometimes exposed as a solder jumper). 0x3C is the default on
// Heltec WiFi Kit 32 V2 onboard, the Core Electronics CE09964 SSD1309
// breakout, and Adafruit 326 STEMMA QT OLED out-of-the-box. 0x3D is
// reachable on Adafruit 326 (and most other Adafruit/SparkFun OLEDs) by
// closing a solder jumper on the back of the breakout — some board
// revisions ship with that jumper closed. V2.4.19 probes both so a
// 0x3D-strapped unit doesn't silently look like a missing display.
#define SSD1306_ADDR        0x3C
#define SSD1306_ADDR_ALT    0x3D
// Per-board chip identifier for boot log accuracy. Both controllers respond
// to the SSD1306 init sequence (register-compatible), but the silicon
// actually present differs and the log should reflect that.
#if defined(BOARD_FEATHERS3_D)
    #define OLED_CHIP_NAME  "SSD1309"
#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    #define OLED_CHIP_NAME  "SSD1315"
#else
    #define OLED_CHIP_NAME  "SSD1306"
#endif
// External SSD1309 breakouts (e.g. Core Electronics CE09964) are 4-pin I²C
// only — no dedicated reset line. PIN_OLED_RESET stays undefined on those
// boards (see hal.h); the reset-pulse block below is then skipped and the
// panel relies on its internal power-on reset.
#ifdef PIN_OLED_RESET
    #define PIN_OLED_RST    PIN_OLED_RESET
#endif
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_PAGES      8

static i2c_master_dev_handle_t s_dev = NULL;
static bool s_show    = false;   // set by display_setup()
static bool s_cleared = true;    // panel is blank (no running screen drawn)
// V2.3.30: OLED contrast / SerLCD backlight in percent (10..100). Set via
// display_set_contrast() — called from display_setup() at boot with the
// configured value, and from http_server.c's /config save handler for live
// updates. Default 80 matches the V2.3.28..V2.3.29 hardcoded init_cmds[]
// contrast register 0xCC so users who never touched the dropdown stay at
// exactly the brightness they already had.
static uint8_t s_brightness_pct = 80;

// V2.3.29: anti-burn-in by alternating SSD1306/9 invert mode at the start
// of each new rotation. V2.4.9: declared unconditionally — multipage code
// is now compiled into all boards (decision is runtime, not compile-time).
// On non-rotation boots this static stays at its initial value FALSE and
// the rotation task that reads it is never spawned.
static bool s_oled_inverted = false;

// V2.4.9: runtime resolved display mode + multipage-active flag, set by
// display_setup(). Read by display_is_multipage() / display_mode_str()
// from any task context — written once at boot before any reader can run.
static display_mode_t s_resolved_mode = DISPLAY_MODE_RADIATION;
static bool           s_is_multipage  = false;

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

// V2.3.29 / V2.4.9: forward declaration — the multi-page rotation task is
// defined further down (after the OLED render functions and dispatcher) but
// xTaskCreate'd inside display_setup() if the resolved mode is rotation.
// V2.4.9 removed the HAL_MULTIPAGE_ROTATION compile-time gate; the task
// function is always compiled in but only spawned when needed.
static void display_task(void *arg);

// V2.3.29: bus 2 init moved to i2c_bus.c. display.c is now a pure
// consumer — gets bus handles via i2c_bus_get_primary() and
// i2c_bus_get_secondary(). Auto-detects the display across both buses
// (primary first since LDO2 is off until the secondary call enables it,
// which lets the dual-bus probe terminate quickly + cheaply if the
// display lives on bus 1). The SerLCD ATmega bootloader's ~500 ms wait
// only fires before the SECONDARY probe — bus 1 has been powered since
// boot, by display_setup time the SerLCD has had hundreds of ms to wake.

// Try the OLED on the given bus. Returns true and binds s_dev on success,
// writing the actual 7-bit address it bound (0x3C or 0x3D) into *addr_out.
// The reset-pulse block is compile-gated by PIN_OLED_RST (defined for
// boards with an onboard reset line — Heltec V2 and Heltec WiFi LoRa 32
// V4-R2); fires once per call (i.e. once per bus), then the address probe
// loop runs. On FeatherS3-D / QT Py the reset block compiles out and this
// is just up to two i2c_master_probe calls.
//
// V2.4.19: probe both 0x3C and 0x3D. Adafruit 326 (and most STEMMA QT
// OLED breakouts) have a solder jumper to switch addresses; some board
// revisions ship with the jumper closed at 0x3D. Pre-V2.4.19 those units
// silently looked like "no display present".
static bool try_oled_on_bus(i2c_master_bus_handle_t bus, uint8_t *addr_out) {
#ifdef PIN_OLED_RST
    // Heltec onboard SSD1306 reset line (GPIO 16). FeatherS3-D / QT Py
    // external 4-pin breakouts have no reset line and PIN_OLED_RST stays
    // undefined → block compiles out. Panel latches reset for ~3 µs
    // minimum; 10 ms either side is plenty.
    // V2.6.22: ESP_ERROR_CHECK — can only fail on an invalid/input-only
    // pin constant, i.e. a board-port mistake; abort loudly at first boot
    // (see the rail-gate blocks in i2c_bus.c for the full rationale).
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_OLED_RST));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_OLED_RST, GPIO_MODE_OUTPUT));
    gpio_set_level(PIN_OLED_RST, 1); vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_OLED_RST, 0); vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_OLED_RST, 1); vTaskDelay(pdMS_TO_TICKS(10));
#endif

    // Try the default address first — most parts answer here and the
    // alternate is only used when a solder jumper has been closed.
    static const uint8_t candidates[] = { SSD1306_ADDR, SSD1306_ADDR_ALT };
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
        uint8_t addr = candidates[i];
        if (i2c_master_probe(bus, addr, 100) != ESP_OK) continue;

        i2c_device_config_t devcfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = addr,
            .scl_speed_hz    = 400000,
        };
        esp_err_t err = i2c_master_bus_add_device(bus, &devcfg, &s_dev);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "i2c_master_bus_add_device(0x%02X) failed: %s",
                     addr, esp_err_to_name(err));
            s_dev = NULL;
            continue;     // try the next candidate
        }
        if (addr_out) *addr_out = addr;
        return true;
    }
    return false;
}

// Apply the SSD1306/9 init sequence. Caller must have just bound s_dev
// via try_oled_on_bus(). Display ends in normal-polarity, cleared state
// at the configured brightness (caller sets contrast separately via
// display_set_contrast()).
static void apply_oled_init_sequence(void) {
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
        0x81, 0xCC,        // contrast — V2.3.30 default 80 % (204/255). The
                           //            display_set_contrast() call from
                           //            display_setup() overrides this with
                           //            the user-configured value.
        0xD9, 0xF1,        // pre-charge period
        0xDB, 0x40,        // VCOMH deselect level
        0xA4,              // entire display follows RAM
        0xA6,              // normal (not inverted)
        0xAF,              // display ON
    };
    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        oled_cmd(init_cmds[i]);
    }

    s_cleared = true;
    oled_clear();
}

// Try to bring up the SerLCD on the given bus. Returns true on success
// and updates the static backend / show / cleared flags.
static bool try_serlcd_on_bus(i2c_master_bus_handle_t bus, bool show_display,
                              uint8_t brightness_pct) {
    if (display_serlcd_init(bus) != ESP_OK) return false;

    s_backend = BACKEND_SERLCD;
    // V2.6.30: fixed identity by construction of the DRIVER, not the
    // hardware — display_serlcd.c probes only OpenLCD's default 0x72 and
    // drives a hardcoded 20x4. (SparkFun also sells 16x2 SerLCDs at the
    // same 0x72 default; one would ACK here but is unsupported and
    // misdriven regardless, so this label is what the driver binds.)
    snprintf(s_backend_desc, sizeof(s_backend_desc), "SerLCD 20x4 @0x72");
    s_show    = show_display;
    s_cleared = !show_display;
    if (!show_display) {
        display_serlcd_clear();
        display_serlcd_set_backlight(0, 0, 0);
    }
    display_set_contrast(brightness_pct);
    return true;
}

bool display_setup(bool show_display, uint8_t brightness_pct, display_mode_t mode) {
    // V2.4.9: store the requested mode. The AUTO resolution happens AFTER
    // the panel is detected (later in this function) because it depends on
    // s_backend and on the per-board OLED chip hint. For explicit
    // RADIATION / ROTATION we can set s_is_multipage immediately —
    // AUTO is finalised at the task_spawn label below.
    s_resolved_mode = mode;
    if (mode == DISPLAY_MODE_RADIATION) {
        s_is_multipage = false;
    } else if (mode == DISPLAY_MODE_ROTATION) {
        s_is_multipage = true;
    }
    // (AUTO case: s_is_multipage stays at its initial false and is
    // overwritten at task_spawn once we know what panel we actually got.)

    // V2.3.29: auto-detect display across BOTH I²C buses. Probe primary
    // first since LDO2 is off until the secondary call enables it — lets
    // the dual-bus probe terminate quickly + cheaply if the display lives
    // on bus 1. The SerLCD ATmega bootloader's ~500 ms wait only fires
    // before the SECONDARY probe (bus 1 has been powered since boot, by
    // display_setup time the SerLCD has had hundreds of ms to wake).
    //
    // Boards without a secondary bus (Heltec, QT Py): i2c_bus_get_secondary()
    // returns NULL and the bus-2 branch is silently skipped.
    //
    // Heltec WiFi LoRa 32 V4-R2 is the one exception to "probe primary
    // first": its OLED lives EXCLUSIVELY on the secondary bus — the
    // primary/env-sensor bus never has a display on it — so probing
    // primary first would pay a guaranteed probe-timeout plus the SerLCD
    // 500 ms wake delay on every single boot for no possible benefit. This
    // board skips straight to the secondary bus and skips the 500 ms
    // SerLCD wake delay too (try_serlcd_on_bus() still runs on this bus —
    // it just returns quickly since there's no SerLCD to answer).

    i2c_master_bus_handle_t bus;
    const char *bus_label;

#if defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    bus = i2c_bus_get_secondary();
    if (!bus) {
        ESP_LOGW(TAG, "no secondary I²C bus — display disabled");
        return false;
    }
    bus_label = "onboard-OLED-bus";
#else
    bus = i2c_bus_get_primary();
    if (!bus) {
        ESP_LOGW(TAG, "no primary I²C bus — display disabled");
        return false;
    }
    bus_label = "STEMMA1";

    // --- Try primary bus -----------------------------------------------
    if (try_serlcd_on_bus(bus, show_display, brightness_pct)) {
        ESP_LOGI(TAG, "display backend: SerLCD at 0x72 on %s (show=%d brightness=%d%%)",
                 bus_label, show_display, brightness_pct);
        goto task_spawn;
    }
    {
        uint8_t oled_addr = 0;
        if (try_oled_on_bus(bus, &oled_addr)) {
            apply_oled_init_sequence();
            s_show    = show_display;
            s_backend = BACKEND_OLED;
            // V2.6.30: fold in the probed address — 0x3C vs 0x3D tells the
            // fleet operator whether the breakout's SA0 jumper is closed.
            snprintf(s_backend_desc, sizeof(s_backend_desc),
                     OLED_CHIP_NAME " 128x64 @0x%02X", oled_addr);
            display_set_contrast(brightness_pct);
            ESP_LOGI(TAG, "display backend: %s at 0x%02X on %s (show=%d brightness=%d%%)",
                     OLED_CHIP_NAME, oled_addr, bus_label,
                     show_display, brightness_pct);
            goto task_spawn;
        }
    }

    // --- Fall through to secondary bus ---------------------------------
    bus = i2c_bus_get_secondary();
    if (!bus) {
        ESP_LOGW(TAG, "no display on %s, no secondary bus on this board — "
                      "display disabled", bus_label);
        return false;
    }
    bus_label = "STEMMA2";

    // SerLCD ATmega bootloader needs ~500 ms after LDO2 enable. (Bus 1
    // probes above didn't need this — bus 1 has been powered since boot.)
    vTaskDelay(pdMS_TO_TICKS(500));
#endif

    if (try_serlcd_on_bus(bus, show_display, brightness_pct)) {
        i2c_bus_secondary_keep_alive();
        ESP_LOGI(TAG, "display backend: SerLCD at 0x72 on %s (show=%d brightness=%d%%)",
                 bus_label, show_display, brightness_pct);
        goto task_spawn;
    }
    {
        uint8_t oled_addr = 0;
        if (try_oled_on_bus(bus, &oled_addr)) {
            apply_oled_init_sequence();
            s_show    = show_display;
            s_backend = BACKEND_OLED;
            // V2.6.30: same identity capture as the primary-bus site above.
            snprintf(s_backend_desc, sizeof(s_backend_desc),
                     OLED_CHIP_NAME " 128x64 @0x%02X", oled_addr);
            display_set_contrast(brightness_pct);
            i2c_bus_secondary_keep_alive();
            ESP_LOGI(TAG, "display backend: %s at 0x%02X on %s (show=%d brightness=%d%%)",
                     OLED_CHIP_NAME, oled_addr, bus_label,
                     show_display, brightness_pct);
            goto task_spawn;
        }
    }

#if defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    ESP_LOGW(TAG, "no display found on %s — display disabled", bus_label);
#else
    ESP_LOGW(TAG, "no display found on either I2C bus — display disabled");
#endif
    return false;

task_spawn:;
    // V2.4.9: finalise AUTO mode now that the panel has been detected.
    // The rule is panel-based:
    //   - SerLCD backend       → rotation (typical 20x4 char display has
    //                            room for the multi-page content)
    //   - Big OLED (SSD1309)   → rotation (per-board compile-time hint
    //                            — BOARD_FEATHERS3_D ships with a 2.42"
    //                            SSD1309 from Core Electronics)
    //   - Small OLED (SSD1306 / SSD1315) → radiation (Heltec onboard
    //                            0.96", Adafruit 326 plugged into STEMMA
    //                            QT, or Heltec WiFi LoRa 32 V4-R2's onboard
    //                            SSD1315 — assumed same compact 0.96"
    //                            footprint Heltec ships across its
    //                            V2/V3/V4 module line; NOT bench-verified
    //                            for this exact SKU)
    //
    // Caveat: SSD1306, SSD1309, and SSD1315 are register-compatible (we
    // drive all three with the same init sequence) and there's no reliable
    // chip-ID register that differs. So the OLED size discrimination uses
    // the compile-time per-board hint. The edge case — Adafruit 326 (small)
    // plugged into a FeatherS3-D's STEMMA1 in place of the SSD1309 —
    // would auto-pick rotation incorrectly; the user can override via
    // display_mode = RADIATION in /config.
    if (mode == DISPLAY_MODE_AUTO) {
        bool resolved_multipage;
        if (s_backend == BACKEND_SERLCD) {
            resolved_multipage = true;
        } else {
#if defined(BOARD_FEATHERS3_D)
            resolved_multipage = true;   // SSD1309 2.42" — big enough for rotation
#elif defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
            resolved_multipage = false;  // SSD1315 0.96" (assumed) — radiation single page
#else
            resolved_multipage = false;  // SSD1306 0.96" — radiation single page
#endif
        }
        s_is_multipage = resolved_multipage;
    }
    ESP_LOGI(TAG, "display mode: requested=%d resolved=%s",
             (int)mode, display_mode_str());

    // V2.3.29 / V2.4.9: Multi-page rotation task. 4 KB stack covers
    // snprintf into char[16] buffers + i2c_master_transmit ~200 B worst-
    // case. Priority 5 sits below WiFi/lwIP (~14) and TX worker (~10) so
    // display rendering never starves networking. V2.4.9 made the spawn
    // a runtime decision (s_is_multipage) instead of compile-time guard.
    if (s_is_multipage) {
        xTaskCreate(display_task, "display", 4096, NULL, 5, NULL);
    }
    return true;
}

// V2.4.9: query accessors for the resolved layout mode.
bool display_is_multipage(void) {
    return s_is_multipage;
}

const char *display_mode_str(void) {
    switch (s_resolved_mode) {
        case DISPLAY_MODE_AUTO:      return s_is_multipage
                                          ? "auto (resolved: rotation)"
                                          : "auto (resolved: radiation)";
        case DISPLAY_MODE_RADIATION: return "radiation (forced)";
        case DISPLAY_MODE_ROTATION:  return "rotation (forced)";
        default:                     return "unknown";
    }
}

// V2.6.30: panel identity for /status + syslog boot banner (see display.h).
const char *display_backend_str(void) {
    return s_backend_desc;
}

void display_boot_screen(void) {
    if (s_backend == BACKEND_SERLCD) {
        display_serlcd_boot_screen(VERSION_STR);
        s_cleared = false;
        return;
    }
    if (s_backend != BACKEND_OLED) return;
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
    if (secs < 60)        snprintf(out, outsz, "%2ds", secs);
    else if (mins < 60)   snprintf(out, outsz, "%2dm", mins);
    else if (hours < 24)  snprintf(out, outsz, "%2dh", hours);
    else {
        int days = secs / 86400;
        snprintf(out, outsz, "%2dd", days % 100);
    }
}

void display_running(int time_sec, int rad_nsvph, int cpm, bool use_display) {
    if (s_backend == BACKEND_SERLCD) {
        // SerLCD has no radiation/CPM page in V2.3.28. main.c only calls
        // display_running() on non-FeatherS3-D boards, and SerLCD has been
        // tested only on FeatherS3-D dust-sensor deployments — so this
        // path is unreachable in practice. Safety stub.
        (void)time_sec; (void)rad_nsvph; (void)cpm; (void)use_display;
        return;
    }
    if (s_backend != BACKEND_OLED) return;
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

// V2.3.30: live brightness change. Stores the new value in s_brightness_pct
// regardless of show state, so a future show=true (currently reboot-required)
// will pick it up. When s_show is true, immediately applies via the active
// backend's native command — for OLED, the SSD1306/9 contrast register
// (0x81 + value); for SerLCD, the RGB backlight set to white at the level.
// The percent → register / level map is linear (×255/100), which matches
// human perception "well enough" for the 10-step UI dropdown.
//
// V2.3.32: pct=0 means OFF.
//   - OLED  : 0xAE drives the panel into sleep mode (segment + common
//             drivers off, charge pump can stay on; near-zero current).
//             0xAF on next non-zero call wakes it; the existing contrast
//             register and RAM contents are retained, so a re-enable
//             after a few seconds shows the same image instantly.
//   - SerLCD: backlight RGB→0,0,0 turns the white LED off. Pixels are
//             still being driven on the LCD glass, so in strong external
//             light the text would still be faintly readable — that's
//             fine, sealed-tube deployments are dark anyway.
void display_set_contrast(uint8_t pct) {
    if (pct > 100) pct = 100;
    s_brightness_pct = pct;

    if (!s_show) return;

    if (s_backend == BACKEND_OLED) {
        if (pct == 0) {
            oled_cmd(0xAE);                               // display OFF (sleep)
        } else {
            uint8_t level = (uint8_t)((pct * 255) / 100);
            oled_cmd(0xAF);                               // ensure ON (idempotent)
            oled_cmd(0x81);
            oled_cmd(level);
        }
    } else if (s_backend == BACKEND_SERLCD) {
        uint8_t level = (uint8_t)((pct * 255) / 100);     // pct=0 → level=0 → backlight off
        display_serlcd_set_backlight(level, level, level);
    }
}

void display_set_status(int index, int value) {
    if (index < 0 || index >= DSP_STATUS_MAX) return;
    s_status[index] = value;
    // V2.3.29 / V2.4.9: on multi-page boots the display task owns the panel
    // — the radiation-era status line at page 7 would overlay whichever
    // page is currently rendered (PM10 bottom half, Pressure bottom half,
    // etc), so skip the redraw. s_status[] is still updated above so any
    // future backend that wants to surface status (e.g. SerLCD RGB
    // backlight = green/red) can read it.
    if (s_is_multipage) return;
    // Single-page (radiation) path — status line lives on page 7 of the
    // running screen, redrawn every time a subsystem state changes.
    if (s_backend != BACKEND_OLED) return;
    if (!s_dev || !s_show || s_cleared) return;
    redraw_status_line();
}

// ====================================================================
// V2.3.29: multi-page display rotation (FeatherS3-D)
// --------------------------------------------------------------------
// Replaces the previous single-Environment-page display_environment()
// API. main.c now calls display_update_snapshot() once per TX cycle
// (~150 s). A dedicated display_task wakes every 5 s, advances the
// page index, skips unavailable pages (no PM sensor → skip PM pages;
// no env sensor → skip env page), and renders the page via per-backend
// render functions:
//
//   OLED  : render_oled_env / _pm_mass / _pm_number / _uploads / _system
//   SerLCD: display_serlcd_render_env / _pm_mass / _pm_number /
//           _uploads / _system (implemented in display_serlcd.c)
//
// Uploads and System pages are always shown (useful even with no
// sensors). OLED invert mode toggles at the start of each NEW rotation
// (page index wraps to 0) so successive 25-s rotations alternate
// black-on-white vs white-on-black — anti-burn-in without a separate
// timer. First rotation keeps the boot-splash polarity (normal).
//
// V2.4.9: previously gated by HAL_MULTIPAGE_ROTATION; now always compiled.
// display_setup() resolves the mode at boot (auto vs explicit) and decides
// whether to spawn display_task — see s_is_multipage. The task function and
// per-page renderers below are dead code when not spawned (linker may DCE
// some; ESP-IDF builds with -ffunction-sections + --gc-sections do this).
// ====================================================================

static display_snapshot_t s_snap = {0};

void display_update_snapshot(const display_snapshot_t *snap) {
    if (snap) s_snap = *snap;
}

// V2.3.29: page dwell time. Set to 7 s after the 5 s initial setting
// was reported as too quick to comfortably read each page on the bench
// OLED. Same value drives both the boot-splash visibility wait at task
// start and the per-page delay in the rotation loop.
#define PAGE_DWELL_MS 7000

#include "display_serlcd.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "main_status.h"      // V2.4.1 (A4): consolidated status snapshot
#include "transmission.h"

// main.c-defined accessor (separate from main_status — controls per-target
// row visibility on the Uploads page).
extern bool main_target_enabled(int target_id);

typedef enum {
    PAGE_ENV = 0,
    PAGE_PM_MASS,
    PAGE_PM_NUMBER,
    PAGE_UPLOADS,
    PAGE_SYSTEM,
    PAGE_COUNT,
} display_page_t;

// -------------------- OLED per-page render functions ----------------

static void render_oled_env(void) {
    oled_clear();
    s_cleared = false;
    char line[16];
    if (s_snap.env_valid) {
        snprintf(line, sizeof(line), "%5.1f C ", s_snap.env_t_c);
        oled_draw_string_2x(0, 0, line);
        snprintf(line, sizeof(line), "%5.1f %% ", s_snap.env_h_pct);
        oled_draw_string_2x(2, 0, line);
        snprintf(line, sizeof(line), "%4d hPa",
                 (int)(s_snap.env_p_pa / 100.0f + 0.5f));
        oled_draw_string_2x(4, 0, line);
    } else {
        oled_draw_string_2x(0, 0, " --- C  ");
        oled_draw_string_2x(2, 0, " --- %  ");
        oled_draw_string_2x(4, 0, " --- hPa");
    }
    if (s_snap.noise_valid) {
        snprintf(line, sizeof(line), "%5.1f dB", s_snap.noise.laeq);
        oled_draw_string_2x(6, 0, line);
    }
}

static void render_oled_pm_mass(void) {
    oled_clear();
    s_cleared = false;
    if (s_snap.pm_valid) {
        char line[16];
        snprintf(line, sizeof(line), "PM1.0%3d", (int)(s_snap.pm.pm1_0 + 0.5f));
        oled_draw_string_2x(0, 0, line);
        snprintf(line, sizeof(line), "PM2.5%3d", (int)(s_snap.pm.pm2_5 + 0.5f));
        oled_draw_string_2x(2, 0, line);
        snprintf(line, sizeof(line), "PM4.0%3d", (int)(s_snap.pm.pm4_0 + 0.5f));
        oled_draw_string_2x(4, 0, line);
        snprintf(line, sizeof(line), "PM10 %3d", (int)(s_snap.pm.pm10 + 0.5f));
        oled_draw_string_2x(6, 0, line);
    } else {
        oled_draw_string_2x(0, 0, "PM1.0---");
        oled_draw_string_2x(2, 0, "PM2.5---");
        oled_draw_string_2x(4, 0, "PM4.0---");
        oled_draw_string_2x(6, 0, "PM10 ---");
    }
}

// PM number-concentration value formatted into 4 chars after the "nX.X"
// label, so the row stays exactly 8 chars at 2x font:
//   < 1000   :  integer right-aligned via %4d  →  " 234" / "  89" / "   9"
//   1k–9.9k  :  "x.xk" with %3.1fk             →  "3.4k" / "9.9k"
//   ≥ 10k    :  "xxk " with %2dk (clamped 99)  →  "10k " / "99k "
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

static void render_oled_pm_number(void) {
    oled_clear();
    s_cleared = false;
    if (s_snap.pm_valid) {
        char line[16];
        char val[8];
        fmt_n_value(val, sizeof(val), s_snap.pm.nc0_5);
        snprintf(line, sizeof(line), "n0.5%s", val);
        oled_draw_string_2x(0, 0, line);
        fmt_n_value(val, sizeof(val), s_snap.pm.nc1_0);
        snprintf(line, sizeof(line), "n1.0%s", val);
        oled_draw_string_2x(2, 0, line);
        fmt_n_value(val, sizeof(val), s_snap.pm.nc2_5);
        snprintf(line, sizeof(line), "n2.5%s", val);
        oled_draw_string_2x(4, 0, line);
        fmt_n_value(val, sizeof(val), s_snap.pm.nc4_0);
        snprintf(line, sizeof(line), "n4.0%s", val);
        oled_draw_string_2x(6, 0, line);
    } else {
        oled_draw_string_2x(0, 0, "n0.5----");
        oled_draw_string_2x(2, 0, "n1.0----");
        oled_draw_string_2x(4, 0, "n2.5----");
        oled_draw_string_2x(6, 0, "n4.0----");
    }
}

// Truncates (does not round) — never shows 100% if any attempt failed.
static int upload_pct(tx_target_id_t id) {
    tx_target_stats_t st;
    tx_get_stats(id, &st);
    if (st.attempted == 0) return 0;
    return (int)((100ULL * st.succeeded) / st.attempted);
}

// Only renders rows for targets currently enabled in /config — compacted
// from the top. Radmon is intentionally never shown (per page design).
// If ALL four are disabled, display_task skips this page entirely so
// you never see a blank panel here.
static void render_oled_uploads(void) {
    oled_clear();
    s_cleared = false;
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
        snprintf(line, sizeof(line), "%s %3d%% ",
                 entries[i].abbrev, upload_pct(entries[i].id));
        oled_draw_string_2x(row * 2, 0, line);
        row++;
    }
}

static void render_oled_system(void) {
    oled_clear();
    s_cleared = false;
    char line[16];

    // Row 0 (2x): TX cycles. < 100000 → integer "C %5d"; ≥ 100000 →
    // "C%5.1fk" giving "C 99.9k" through "C999.9k" — covers ~4.75 years
    // at 150 s TX interval.
    main_status_t st;
    main_status_snapshot(&st);
    uint32_t cycles = st.cycles;
    if (cycles < 100000) {
        snprintf(line, sizeof(line), "C %5lu", (unsigned long)cycles);
    } else {
        snprintf(line, sizeof(line), "C%5.1fk", cycles / 1000.0f);
    }
    oled_draw_string_2x(0, 0, line);

    // Row 1 (2x): adaptive uptime format.
    //   < 1 day : "%2dh %2dm"  →  " 5h 30m" / "23h 59m" (7 chars)
    //   >= 1 day: "%3dd%2dh"   →  "  3d14h" / "365d23h" (7 chars)
    // The minutes resolution before day 1 is useful for early-boot
    // sanity checks; once uptime tops a day, hours-of-day is more
    // informative than minutes-since-last-hour.
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
    oled_draw_string_2x(2, 0, line);

    // Pages 4-6 at 1x font (16 chars each) — heap stats. Page 7 left
    // blank by the oled_clear() above for visual breathing room.
    uint32_t free_h = (uint32_t)esp_get_free_heap_size();
    uint32_t min_h  = (uint32_t)esp_get_minimum_free_heap_size();
    uint32_t max_h  = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    snprintf(line, sizeof(line), "Free %7lu B", (unsigned long)free_h);
    oled_draw_string(4, 0, line);
    snprintf(line, sizeof(line), "Min  %7lu B", (unsigned long)min_h);
    oled_draw_string(5, 0, line);
    snprintf(line, sizeof(line), "Max  %7lu B", (unsigned long)max_h);
    oled_draw_string(6, 0, line);
}

// -------------------- Dispatcher + display task ---------------------

static void render_page(display_page_t page) {
    if (s_backend == BACKEND_OLED) {
        switch (page) {
            case PAGE_ENV:       render_oled_env();        break;
            case PAGE_PM_MASS:   render_oled_pm_mass();    break;
            case PAGE_PM_NUMBER: render_oled_pm_number();  break;
            case PAGE_UPLOADS:   render_oled_uploads();    break;
            case PAGE_SYSTEM:    render_oled_system();     break;
            default: break;
        }
    } else if (s_backend == BACKEND_SERLCD) {
        switch (page) {
            case PAGE_ENV:       display_serlcd_render_env(&s_snap);        break;
            case PAGE_PM_MASS:   display_serlcd_render_pm_mass(&s_snap);    break;
            case PAGE_PM_NUMBER: display_serlcd_render_pm_number(&s_snap);  break;
            case PAGE_UPLOADS:   display_serlcd_render_uploads();           break;
            case PAGE_SYSTEM:    display_serlcd_render_system();            break;
            default: break;
        }
    }
}

static void display_task(void *arg) {
    (void)arg;
    // Boot splash is drawn once by display_boot_screen() at startup. Let
    // it remain visible for 5 s before rotation kicks off.
    vTaskDelay(pdMS_TO_TICKS(PAGE_DWELL_MS));

    int page_idx = 0;
    bool first_rotation = true;

    while (1) {
        if (!s_show) {
            if (!s_cleared) {
                if (s_backend == BACKEND_OLED) {
                    oled_clear();
                } else if (s_backend == BACKEND_SERLCD) {
                    display_serlcd_clear();
                    display_serlcd_set_backlight(0, 0, 0);
                }
                s_cleared = true;
            }
            vTaskDelay(pdMS_TO_TICKS(PAGE_DWELL_MS));
            continue;
        }
        s_cleared = false;

        // Build rotation list — env / PM pages gated on sensor presence.
        // Uploads + System always shown.
        display_page_t pages[PAGE_COUNT];
        int n_pages = 0;
        if (env_sensor_present()) pages[n_pages++] = PAGE_ENV;
        if (pm_sensor_present()) {
            pages[n_pages++] = PAGE_PM_MASS;
            pages[n_pages++] = PAGE_PM_NUMBER;
        }
        // Uploads page is shown only if at least one of MA/SC/OS/AQ is
        // enabled in /config. Radmon is intentionally excluded from this
        // check (and from the page itself).
        if (main_target_enabled(TX_TARGET_MADAVI)  ||
            main_target_enabled(TX_TARGET_SENSORC) ||
            main_target_enabled(TX_TARGET_OSM)     ||
            main_target_enabled(TX_TARGET_AQI)) {
            pages[n_pages++] = PAGE_UPLOADS;
        }
        pages[n_pages++] = PAGE_SYSTEM;

        if (page_idx >= n_pages) page_idx = 0;

        // OLED invert toggles at the start of EACH new rotation. First
        // rotation stays in initial (normal) polarity — matches the boot
        // splash. Invert is a display-level command (preserves the
        // framebuffer), so each page within the rotation sees the same
        // polarity.
        if (page_idx == 0 && !first_rotation && s_backend == BACKEND_OLED) {
            s_oled_inverted = !s_oled_inverted;
            oled_cmd(s_oled_inverted ? 0xA7 : 0xA6);
        }
        if (page_idx == 0) first_rotation = false;

        render_page(pages[page_idx]);
        page_idx++;

        vTaskDelay(pdMS_TO_TICKS(PAGE_DWELL_MS));
    }
}

#elif HAL_HAS_TFT

// ====================================================================
// V2.6.11: Adafruit ESP32-S3 TFT Feather onboard ST7789 color TFT.
// Mutually exclusive with HAL_HAS_OLED (see hal.h) — this board has no
// I2C OLED/SerLCD path, so there's no probe/fallback chain like the
// branch above: the panel is always fitted, and display_setup() always
// resolves to the 5-page rotation (no radiation single-page layout is
// implemented for this backend — the color landscape panel suits the
// rotation grid, and every deployment of this board so far pairs it
// with at least one sensor). Rendering itself lives in display_tft.c;
// this branch only owns the display_task rotation loop and the public
// display_*() dispatch, mirroring the structure above.
// ====================================================================

#include "display_tft.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "env_sensor.h"     // env_sensor_present() for page-skip logic
#include "pm_sensor.h"      // pm_sensor_present() for page-skip logic
#include "transmission.h"   // main_target_enabled() gating for PAGE_UPLOADS
#include "version.h"

// main.c-defined accessor (separate from main_status — controls per-target
// row visibility on the Uploads page). Same declaration as the HAL_HAS_OLED
// branch above.
extern bool main_target_enabled(int target_id);

static const char *TAG = "display";
static bool s_show = false;
static display_mode_t s_resolved_mode = DISPLAY_MODE_AUTO;
static display_snapshot_t s_snap = {0};
static uint8_t s_brightness_pct = 80;   // V2.6.11 review fix: same
                                        // stored-regardless-of-show-state
                                        // pattern as the OLED backend above
                                        // (display.h's documented contract)

#define PAGE_DWELL_MS 7000

typedef enum {
    PAGE_ENV = 0,
    PAGE_PM_MASS,
    PAGE_PM_NUMBER,
    PAGE_UPLOADS,
    PAGE_SYSTEM,
    PAGE_COUNT,
} display_page_t;

static void render_page(display_page_t page) {
    switch (page) {
        case PAGE_ENV:       display_tft_render_env(&s_snap);       break;
        case PAGE_PM_MASS:   display_tft_render_pm_mass(&s_snap);   break;
        case PAGE_PM_NUMBER: display_tft_render_pm_number(&s_snap); break;
        case PAGE_UPLOADS:   display_tft_render_uploads();          break;
        case PAGE_SYSTEM:    display_tft_render_system();           break;
        default: break;
    }
}

static void display_task(void *arg) {
    (void)arg;
    // Boot splash is drawn once by display_boot_screen() at startup. Let
    // it remain visible for PAGE_DWELL_MS (7 s) before rotation kicks off —
    // same dwell used by the HAL_HAS_OLED branch above.
    vTaskDelay(pdMS_TO_TICKS(PAGE_DWELL_MS));

    int page_idx = 0;

    while (1) {
        if (!s_show) {
            vTaskDelay(pdMS_TO_TICKS(PAGE_DWELL_MS));
            continue;
        }

        // Build rotation list — env / PM pages gated on sensor presence.
        // Uploads + System always shown. Identical gating to the OLED
        // branch's display_task().
        display_page_t pages[PAGE_COUNT];
        int n_pages = 0;
        if (env_sensor_present()) pages[n_pages++] = PAGE_ENV;
        if (pm_sensor_present()) {
            pages[n_pages++] = PAGE_PM_MASS;
            pages[n_pages++] = PAGE_PM_NUMBER;
        }
        if (main_target_enabled(TX_TARGET_MADAVI)  ||
            main_target_enabled(TX_TARGET_SENSORC) ||
            main_target_enabled(TX_TARGET_OSM)     ||
            main_target_enabled(TX_TARGET_AQI)) {
            pages[n_pages++] = PAGE_UPLOADS;
        }
        pages[n_pages++] = PAGE_SYSTEM;

        if (page_idx >= n_pages) page_idx = 0;

        render_page(pages[page_idx]);
        page_idx++;

        vTaskDelay(pdMS_TO_TICKS(PAGE_DWELL_MS));
    }
}

bool display_setup(bool show_display, uint8_t brightness_pct, display_mode_t mode) {
    s_resolved_mode = mode;

    esp_err_t err = display_tft_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "display_tft_init failed: %s — display disabled", esp_err_to_name(err));
        return false;
    }

    s_show = show_display;
    display_set_contrast(brightness_pct);

    xTaskCreate(display_task, "display", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "display backend: ST7789 TFT (show=%d brightness=%d%%)",
             show_display, brightness_pct);
    return true;
}

bool display_is_multipage(void) {
    return true;
}

const char *display_mode_str(void) {
    switch (s_resolved_mode) {
        case DISPLAY_MODE_RADIATION: return "radiation (unsupported on TFT, showing rotation)";
        case DISPLAY_MODE_ROTATION:  return "rotation (forced)";
        default:                     return "auto (resolved: rotation)";
    }
}

// V2.6.30: panel identity (see display.h). Compile-time constant here —
// the TFT Feather's ST7789 is soldered on, no probe/fallback chain exists
// (HAL_HAS_TFT boards never enter the I²C display probe). Deliberately
// reported even if display_tft_init() failed: this string answers "what
// hardware is fitted", not "is it healthy" — the part is present either way.
const char *display_backend_str(void) {
    return "ST7789 TFT 240x135";
}

void display_boot_screen(void) {
    // V2.6.11 review fix: match display_set_contrast()'s s_show gating —
    // skip the splash render/push entirely when the display was configured
    // off, instead of always spending an SPI DMA transfer for a screen the
    // backlight will never light.
    if (!s_show) return;
    display_tft_boot_screen(VERSION_STR);
}

// V2.3.30-equivalent live brightness change for this backend: pct=0 turns
// the backlight off (panel keeps driving pixels — same "sealed-tube
// deployments are dark anyway" reasoning as the OLED branch's comment
// above), any pct>0 turns it fully on. No PWM dimming in this first pass
// (design spec §11, out of scope).
void display_set_contrast(uint8_t pct) {
    if (pct > 100) pct = 100;
    // V2.6.11 review fix: store regardless of show state, matching the
    // OLED backend and display.h's documented contract ("the new value is
    // stored for next show=true"), even though no runtime show=false→true
    // path exists yet (same "currently reboot-required" caveat as OLED).
    s_brightness_pct = pct;

    if (!s_show) return;
    display_tft_set_backlight(pct);
}

void display_running(int time_sec, int rad_nsvph, int cpm, bool use_display) {
    // No radiation single-page layout on this backend — see the comment
    // at the top of this HAL_HAS_TFT branch. Safety stub, same pattern as
    // the SerLCD backend's stub above.
    (void)time_sec; (void)rad_nsvph; (void)cpm; (void)use_display;
}

void display_set_status(int index, int value) {
    // Not implemented for this backend's first pass — the Uploads/System
    // pages already surface live upload and system state (design spec
    // §11, out of scope).
    (void)index; (void)value;
}

void display_update_snapshot(const display_snapshot_t *snap) {
    if (snap) s_snap = *snap;
}

#else  // !HAL_HAS_OLED && !HAL_HAS_TFT

// No-op stubs for boards without any onboard display. Same prototypes as
// above so callers compile unchanged; display_setup() returning false
// signals that no panel is fitted.
bool display_setup(bool show_display, uint8_t brightness_pct, display_mode_t mode) {
    (void)show_display; (void)brightness_pct; (void)mode;
    return false;
}
void display_boot_screen(void) {}
void display_set_contrast(uint8_t pct) { (void)pct; }
void display_running(int time_sec, int rad_nsvph, int cpm, bool use_display) {
    (void)time_sec; (void)rad_nsvph; (void)cpm; (void)use_display;
}
void display_set_status(int index, int value) { (void)index; (void)value; }
void display_update_snapshot(const display_snapshot_t *snap) { (void)snap; }
bool display_is_multipage(void) { return false; }
const char *display_mode_str(void) { return "no display"; }
// V2.6.30: no display hardware on this board class — fixed "none" keeps
// the /status line and syslog banner shape identical across the fleet.
const char *display_backend_str(void) { return "none"; }

#endif  // HAL_HAS_OLED / HAL_HAS_TFT / no-display
