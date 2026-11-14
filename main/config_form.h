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
#include <stddef.h>   // size_t (V2.8.26 secret-field API)
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
    bool has_lora_fem;            ///< HAL_HAS_LORA_FEM: Heltec GC1109 front end (high-power mode)
    uint8_t carrier_rev_fixed;    ///< HAL_REVC_CARRIER_FIXED, 0 = none: carrier stored on non-switch boards
} config_form_caps_t;

/** @brief What the parser found besides the field values. */
typedef struct {
    bool restart_requested;   ///< the "Save and restart" button was used
    /** Comma-separated keys whose value was invalid or out of range (prior value
     *  kept), "" if none. Only known field keys and fixed text ever land here, so
     *  the caller may echo it into HTML unescaped. Truncated, never overrun. */
    char rejected[160];
    /** V2.8.28: comma-separated keys nothing matched (the submit keys `save` /
     *  `save_restart` excepted) that are 1–24 characters of [A-Za-z0-9_], "" if
     *  none. Only such names land here, so it may be echoed unescaped. */
    char unknown[160];
    uint8_t unknown_odd;    ///< unknown keys failing that rule (counted only; saturates at 255)
    uint8_t unknown_more;   ///< valid unknown keys that did not fit `unknown` (saturates at 255)
} config_form_result_t;

#define CONFIG_CHANGES_MAX 384   ///< chars of key list in the change summary before "… (+N more)"
#define CONFIG_CHANGES_BUF (CONFIG_CHANGES_MAX + 24)   ///< list + " … (+N more)" + NUL
/** Buffer for config_form_unknown_text(): list + " (+255 with other characters)" + " (+255 more)" + NUL. */
#define CONFIG_UNKNOWN_TEXT_BUF (160 + 29 + 12 + 1)

/** @brief Keys of the fields a save changes (names only, secrets excluded).
 *
 *  Compares every schema field of `prev` and `next` (generated from
 *  config_fields.def) and writes the keys that differ, in schema order, as
 *  "a, b, c"; "" if none. Secret keys are left out: they have their own
 *  summary (config_form_secret_changes). A key is written only if it fits
 *  whole within min(CONFIG_CHANGES_MAX, out_sz - 24); the rest is counted and
 *  the text ends " … (+N more)" (just "… (+N more)" if no key fits).
 *  Never overruns `out_sz`.
 *
 *  @param prev    the saved config
 *  @param next    the config about to be saved
 *  @param out     destination; CONFIG_CHANGES_BUF bytes hold the full summary
 *  @param out_sz  size of `out`
 */
void config_form_changes(const config_t *prev, const config_t *next, char *out, size_t out_sz);

/** @brief The "Unknown fields ignored" text of a parse result.
 *
 *  E.g. "send_mda, foo (+2 with other characters) (+1 more)"; "" if the parse
 *  met no unknown key. Contains only names that passed the [A-Za-z0-9_]{1,24}
 *  rule plus fixed text, so it may be echoed into HTML and the log unescaped.
 *
 *  @param res     a result filled by config_apply_form()
 *  @param out     destination; CONFIG_UNKNOWN_TEXT_BUF bytes always suffice
 *  @param out_sz  size of `out`
 */
void config_form_unknown_text(const config_form_result_t *res, char *out, size_t out_sz);

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

// --- V2.8.26: secret fields --------------------------------------------------
// /config never sends a stored secret back to the browser: its field is always
// empty and only the placeholder says whether a value is set. On save, for
// these keys only: empty = keep the stored value, exactly "-" = clear it (the
// admin password cannot be cleared), anything else = the new value.

#define CONFIG_SECRET_PLACEHOLDER_SET   "******** SECRET SET ********"  ///< placeholder: a value is stored
#define CONFIG_SECRET_PLACEHOLDER_UNSET "not set"                       ///< placeholder: nothing stored
#define CONFIG_SECRET_CLEAR             "-"                             ///< field value that clears a secret

/** @brief Form keys of the secret fields (wifi_pw, ap_pw, lora_appkey, …). */
extern const char *const k_config_secret_keys[];
/** @brief Number of entries in k_config_secret_keys. */
extern const size_t k_config_secret_keys_n;

/** @brief True if `key` is the form key of a secret field. */
bool config_form_is_secret_key(const char *key);

/** @brief Name the secret fields that differ between `prev` and `next`, never
 *         their values: "changed: wifi_pw; cleared: mqtt_pw" (set → empty =
 *         cleared, any other difference = changed). Empty string when nothing
 *         differs. Always NUL-terminates within `out_sz`. */
void config_form_secret_changes(const config_t *prev, const config_t *next,
                                char *out, size_t out_sz);
