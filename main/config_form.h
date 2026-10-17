#pragma once

/** @file
 *  @brief /config form parser: URL-encoded POST body -> config_t, plus the
 *         cross-field rules that keep a stored config consistent.
 *
 *  V2.8.10 (review 2.1 C3b): moved out of http_server.c::config_post() so it
 *  runs unchanged in the host tests (test/test_main.c). The unit touches no
 *  NVS, no HTTP server and no FreeRTOS: config_post() still owns the body
 *  receive, config_save() and the live-apply calls, and hands this parser a
 *  copy of the saved config to edit.
 *
 *  Board-dependent rules (a checkbox forced off where the hardware is missing)
 *  read a config_form_caps_t instead of `#if HAL_HAS_*`, so the host tests can
 *  run both sides of each rule even though the test binary is built for one
 *  board. config_form_caps_from_hal() below is the only place that maps the
 *  hal.h flags onto that struct.
 */

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "hal.h"   // HAL_HAS_* — read only by config_form_caps_from_hal()

/** @brief Board capabilities the form rules depend on. */
typedef struct {
    bool has_antenna_switch;      ///< HAL_HAS_ANTENNA_SWITCH: u.FL antenna select exists
    bool has_i2c_pinout_switch;   ///< HAL_HAS_I2C_PINOUT_SWITCH: alternate I2C pads exist
    bool has_fuel_gauge;          ///< HAL_HAS_FUEL_GAUGE: MAX17048 battery gauge on board
    bool has_lorawan;             ///< HAL_HAS_LORAWAN: lora_* keys get strict hex checks
    bool has_carrier_rev_switch;  ///< HAL_HAS_CARRIER_REV_SWITCH: revc_carrier may be 3
    bool has_lora_fem;            ///< HAL_HAS_LORA_FEM: Heltec GC1109 front end (28 dBm mode)
    uint8_t carrier_rev_fixed;    ///< HAL_REVC_CARRIER_FIXED, 0 = none: carrier stored on non-switch boards
} config_form_caps_t;

/** @brief What the parser found besides the field values. */
typedef struct {
    bool restart_requested;   ///< the "Save and restart" button was used
    /** Comma-separated keys whose value was invalid or out of range (prior value
     *  kept), "" if none. Only known field keys and fixed text ever land here, so
     *  the caller may echo it into HTML unescaped. Truncated, never overrun. */
    char rejected[160];
} config_form_result_t;

/** @brief This build's capabilities, from hal.h. */
static inline config_form_caps_t config_form_caps_from_hal(void) {
    config_form_caps_t c = {
        .has_antenna_switch    = HAL_HAS_ANTENNA_SWITCH,
        .has_i2c_pinout_switch = HAL_HAS_I2C_PINOUT_SWITCH,
        .has_fuel_gauge        = HAL_HAS_FUEL_GAUGE,
        .has_lorawan           = HAL_HAS_LORAWAN,
        .has_carrier_rev_switch = HAL_HAS_CARRIER_REV_SWITCH,
        .has_lora_fem          = HAL_HAS_LORA_FEM,
#ifdef HAL_REVC_CARRIER_FIXED
        .carrier_rev_fixed     = HAL_REVC_CARRIER_FIXED,
#else
        .carrier_rev_fixed     = 0,
#endif
    };
    return c;
}

/** @brief Parse a /config POST body into `next` and apply the cross-field rules.
 *
 *  The caller loads `*next` with a copy of `*prev` (the saved config) first;
 *  fields the form does not mention keep their saved value. Every bool is
 *  pre-cleared, because a form only POSTs ticked checkboxes. A matched field
 *  whose value is invalid keeps its prior value and is listed in
 *  `res->rejected`. `prev` is read for the two rules that restore a saved
 *  value (FTP power save, MQTT Mode B CA).
 *
 *  @param next  config to edit; caller-owned (config_post() keeps it static,
 *               config_t is ~4 KB and once overflowed the httpd stack, V2.6.4)
 *  @param prev  the currently saved config; not modified
 *  @param body  NUL-terminated URL-encoded body; decoded and split IN PLACE
 *  @param caps  board capabilities, normally config_form_caps_from_hal()
 *  @param res   filled completely on return
 */
void config_apply_form(config_t *next, const config_t *prev, char *body,
                       const config_form_caps_t *caps, config_form_result_t *res);
