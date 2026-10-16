#include "config_form.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "util.h"            // safe_strcpy, url_decode, parse_long_strict, parse_float_strict
#include "lorawan_codec.h"   // lw_hex_decode — header-only, no radio dependency

// V2.8.10 (review 2.1 C3b): everything in this file used to live in
// config.c (defaults + the two schema helpers) and http_server.c::config_post()
// (parse loop + cross-field rules). It moved here unchanged apart from three
// mechanical substitutions, so the host tests can run it:
//   * `#if !HAL_HAS_X` force-offs  ->  `if (!caps->has_x)`
//   * the `#if HAL_HAS_LORAWAN` hex intercepts  ->  `caps->has_lorawan &&`
//   * reads of the saved config `s_cfg->`  ->  `prev->`
// There must be no `#if HAL_HAS_*` in this file: a board-dependent rule that
// is compiled out cannot be tested on the host.
//
// One consequence: the three LoRaWAN EUI/key checks below (lw_hex_decode from
// the header-only lorawan_codec.h, no radio code) are now compiled on every
// board and skipped at run time where caps->has_lorawan is false. Until
// V2.8.9 they were compiled only on the LoRaWAN board. The behaviour on other
// boards is unchanged: those keys still take the plain string copy.

// --- Compile-time defaults ---------------------------------------------------
//
// V2.4.1: all field defaults are declared INLINE in `config_fields.def`
// as the third (or fourth) argument of each X_* macro. Adding a new
// field's default = same line as the field declaration. The previous
// DEF_* macro tier (~50 #define lines) is gone — one source of truth.
//
// V2.8.10: moved here from config.c (it touches no NVS) so the host tests can
// build a "saved" config. config_load() in config.c still calls it first.

void config_defaults(config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));

    // Generated default assignments — see config_fields.def. String fields
    // use the bounded-copy safe_strcpy helper. Numeric / bool fields use
    // direct assign.
    #define X_STR(name, size, key, def)         \
        safe_strcpy(cfg->name, (def), (size));
    #define X_BOOL(name, key, def)              cfg->name = (def);
    #define X_U32(name, key, def, lo, hi)       cfg->name = (def);
    #define X_F32(name, key, def, lo, hi)       cfg->name = (def);
    #define X_U8(name, key, def, lo, hi)        cfg->name = (def);
    #include "config_fields.def"
    #undef X_STR
    #undef X_BOOL
    #undef X_U32
    #undef X_F32
    #undef X_U8
}

// --- Schema-generated POST helpers -------------------------------------------
//
// V2.4.1: the per-field POST dispatch is derived from the schema too. Moved
// here from config.c in V2.8.10 and made file-local: config_apply_form() is now
// their only caller.

/** @brief Pre-clear all bool fields in `next`.
 *
 *  Form submissions only include ticked checkboxes, so every bool must be set
 *  false before parsing and re-enabled on key match. Generated from the schema
 *  so adding a new bool needs no parallel edit here.
 */
static void config_post_preclear_bools(config_t *next) {
    // Form submissions only include ticked checkboxes — every bool must
    // be reset to false before parsing so unticked boxes apply.
    #define X_STR(name, size, key, def)         /* string — not pre-cleared */
    #define X_BOOL(name, key, def)              next->name = false;
    #define X_U32(name, key, def, lo, hi)       /* numeric — kept */
    #define X_F32(name, key, def, lo, hi)       /* numeric — kept */
    #define X_U8(name, key, def, lo, hi)        /* numeric — kept */
    #include "config_fields.def"
    #undef X_STR
    #undef X_BOOL
    #undef X_U32
    #undef X_F32
    #undef X_U8
}

/** @brief Try to apply one form field to `next`.
 *
 *  Returns true if `key` matched a known schema field (regardless of whether
 *  the value passed validation — out-of-range numerics keep their prior
 *  value). Returns false if `key` is unknown — the caller then checks for
 *  non-schema keys (e.g. `save`, `save_restart`) or ignores it.
 *
 *  V2.5.34: when the key matched but the numeric value was out of range (so
 *  the prior value was kept), `*out_rejected` is set true — lets the caller
 *  surface "field not saved" feedback instead of a silent no-op. Untouched on
 *  a clean apply or an unknown key. Pass NULL to ignore.
 *
 *  Specialised validators (e.g. OLED brightness step constraint) are applied
 *  by the caller BEFORE calling this — if the special handler consumed the
 *  key, don't call here.
 */
static bool config_post_apply_field(config_t *next, const char *key, const char *val,
                                    bool *out_rejected) {
    // Generated per-field dispatch. Each branch returns true when the
    // key matches, regardless of whether the value passed validation —
    // out-of-range numerics keep the prior field value. V2.5.34: a
    // matched-but-out-of-range numeric sets *out_rejected (if non-NULL)
    // so the caller can report the silent no-save instead of hiding it.
    // V2.8.5: numerics parse with parse_long_strict / parse_float_strict
    // (util.h). strtol(val, NULL, 10) turned "abc" into 0 and "12abc" into
    // 12 and stored them; a value that is not entirely a number is now
    // reported as rejected like an out-of-range one, prior value kept.
    #define X_STR(name, size, k, def)                            \
        if (strcmp(key, k) == 0) {                               \
            safe_strcpy(next->name, val, (size));                \
            return true;                                         \
        }
    #define X_BOOL(name, k, def)                                 \
        if (strcmp(key, k) == 0) {                               \
            next->name = true;                                   \
            return true;                                         \
        }
    #define X_U32(name, k, def, lo, hi)                          \
        if (strcmp(key, k) == 0) {                               \
            long _v = 0;                                         \
            if (parse_long_strict(val, &_v) &&                   \
                _v >= (long)(lo) && _v <= (long)(hi))            \
                next->name = (uint32_t)_v;                       \
            else if (out_rejected) *out_rejected = true;         \
            return true;                                         \
        }
    #define X_F32(name, k, def, lo, hi)                          \
        if (strcmp(key, k) == 0) {                               \
            float _v = 0.0f;                                     \
            if (parse_float_strict(val, &_v) &&                  \
                _v >= (lo) && _v <= (hi))                        \
                next->name = _v;                                 \
            else if (out_rejected) *out_rejected = true;         \
            return true;                                         \
        }
    #define X_U8(name, k, def, lo, hi)                           \
        if (strcmp(key, k) == 0) {                               \
            long _v = 0;                                         \
            if (parse_long_strict(val, &_v) &&                   \
                _v >= (long)(lo) && _v <= (long)(hi))            \
                next->name = (uint8_t)_v;                        \
            else if (out_rejected) *out_rejected = true;         \
            return true;                                         \
        }
    #include "config_fields.def"
    #undef X_STR
    #undef X_BOOL
    #undef X_U32
    #undef X_F32
    #undef X_U8

    return false;   // no schema field matched
}

// --- Bounded append ------------------------------------------------------------
//
// File-local copy of http_server.c::append_safe (same clamp and the same printf
// format attribute as it and sd_logger.c's copy): returns the updated length,
// stops growing once the buffer is full, and never lets a miscounted `n`
// underflow `sz - n`. Kept per file
// rather than shared through util.h because sd_logger.c already defines its own
// static append_safe and includes util.h.

__attribute__((format(printf, 4, 5)))
static int form_append(char *out, size_t sz, int n, const char *fmt, ...) {
    if (n < 0 || (size_t)n >= sz) return (int)sz;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(out + n, sz - n, fmt, ap);
    va_end(ap);
    if (w < 0) return n;
    n += w;
    if ((size_t)n > sz) n = (int)sz;
    return n;
}

// --- Form parser + cross-field rules -----------------------------------------

void config_apply_form(config_t *next, const config_t *prev, char *body,
                       const config_form_caps_t *caps, config_form_result_t *res) {
    // V2.3.24: which submit button was clicked? "save_restart" key only
    // appears when the user clicked "Save and restart"; plain "save" appears
    // when they clicked "Save" (NVS persists, no reboot triggered). Default
    // is no-restart so a hand-crafted POST that omits both keys also stays
    // running.
    res->restart_requested = false;

    // V2.5.34: collect the form keys of fields that matched but whose value was
    // out of range (so the prior value was kept). Reported on the result page +
    // logged, instead of the prior silent no-save that made an out-of-range entry
    // (e.g. ftp_int > its max) look like nothing happened. Only known field keys
    // ever land here (set inside the schema dispatch), so the contents are a fixed
    // allowlist — safe to echo into the result HTML without escaping.
    char *rejected = res->rejected;
    const size_t rejected_sz = sizeof(res->rejected);
    rejected[0] = 0;
    int rej_n = 0;

    // Start from the current config (the caller's copy in `next`); pre-clear
    // every bool (forms only POST ticked checkboxes, so an absent key means
    // "unticked"). Schema-derived — see config_post_preclear_bools above.
    config_post_preclear_bools(next);

    char *p = body;
    while (*p) {
        char *eq = strchr(p, '=');
        if (!eq) break;
        *eq = 0;
        char *val = eq + 1;
        char *amp = strchr(val, '&');
        if (amp) *amp = 0;
        url_decode(val);

        // V2.3.30: oled_bright needs a STEP validator (0 or 10..100 step 10)
        // that's tighter than the schema's generic [0,100] envelope. Handle
        // it FIRST so the generic dispatch never sees this key.
        // V2.3.32: 0 (OFF) accepted — display_set_contrast interprets as
        // panel-dark (OLED 0xAE) / backlight-off (SerLCD).
        bool oor = false;   // matched a known field but value out of range/step
        if (strcmp(p, "oled_bright") == 0) {
            // V2.8.5: strict parse — "abc" used to read as 0, i.e. panel OFF.
            long v = -1;
            if (parse_long_strict(val, &v) &&
                (v == 0 || (v >= 10 && v <= 100 && (v % 10) == 0))) {
                next->oled_brightness_pct = (uint8_t)v;
            } else {
                oor = true;   // V2.5.34: out-of-step value kept prior — report it
            }
        }
        // V2.6.23 (T8): DevEUI/JoinEUI/AppKey need strict hex-length
        // validation the schema's plain X_STR copy doesn't do — same
        // "intercept before the generic dispatch" pattern as oled_bright
        // above. Empty is always accepted (clears the field back to
        // "not configured"); a non-empty value must decode as exactly
        // the required hex length via lw_hex_decode() or the whole POST
        // value is rejected (prior stored value kept, reported below).
        // V2.8.10: gated by caps->has_lorawan (was #if HAL_HAS_LORAWAN); on
        // other boards these keys fall through to the generic string copy.
        else if (caps->has_lorawan && strcmp(p, "lora_deveui") == 0) {
            uint8_t scratch[8];
            if (val[0] == '\0') {
                next->lorawan_deveui[0] = 0;
            } else if (lw_hex_decode(val, scratch, sizeof(scratch))) {
                safe_strcpy(next->lorawan_deveui, val, sizeof(next->lorawan_deveui));
            } else {
                oor = true;
            }
        }
        else if (caps->has_lorawan && strcmp(p, "lora_joineui") == 0) {
            uint8_t scratch[8];
            if (val[0] == '\0') {
                next->lorawan_joineui[0] = 0;
            } else if (lw_hex_decode(val, scratch, sizeof(scratch))) {
                safe_strcpy(next->lorawan_joineui, val, sizeof(next->lorawan_joineui));
            } else {
                oor = true;
            }
        }
        else if (caps->has_lorawan && strcmp(p, "lora_appkey") == 0) {
            uint8_t scratch[16];
            if (val[0] == '\0') {
                next->lorawan_appkey[0] = 0;
            } else if (lw_hex_decode(val, scratch, sizeof(scratch))) {
                safe_strcpy(next->lorawan_appkey, val, sizeof(next->lorawan_appkey));
            } else {
                oor = true;
            }
        }
        // Generic schema dispatch. Returns true if `p` matched a known
        // field; out-of-range numerics keep prior value and set `oor`.
        else if (!config_post_apply_field(next, p, val, &oor)) {
            // Non-schema control keys.
            if (strcmp(p, "save_restart") == 0) res->restart_requested = true;
            // Plain "save" and any unknown keys are silently ignored.
        }

        // V2.5.34: accumulate any rejected field's key (comma-separated).
        if (oor) {
            rej_n = form_append(rejected, rejected_sz, rej_n,
                                "%s%s", rej_n ? ", " : "", p);
        }

        if (!amp) break;
        p = amp + 1;
    }

    // --- Cross-field invariants — applied after dispatch ----------------
    //
    // The ORDER below is the order config_post() always applied them in; the
    // tube-off and pcnt rules in particular must stay after the dispatch and
    // in this sequence.

    // 802.11b/g channels are always 20 MHz — HT40 only exists under 11n.
    // A disabled checkbox doesn't POST, so the form may send wifi_ht20=0
    // even while the UI showed it ticked; enforce the invariant here so
    // the stored state matches what the user saw.
    if (next->wifi_11bg_only) next->wifi_ht20_only = true;

    // External-antenna switch: silently force-disable on boards without the
    // hardware. Defence-in-depth — the UI already greys the checkbox, but a
    // hand-crafted POST could still set it.
    if (!caps->has_antenna_switch) next->use_external_antenna = false;

    // V2.5.19: same defence-in-depth for the I²C pin-out route — force-disable
    // on boards without the alternate pads (UI already greys it).
    if (!caps->has_i2c_pinout_switch) next->i2c_pinout = false;

    // V2.6.6: same defence-in-depth for the battery-attached checkbox —
    // force-disable on boards without the MAX17048 (UI already greys it).
    if (!caps->has_fuel_gauge) next->batt_present = false;

    // V2.8.10: same defence-in-depth for the Rev C carrier select — only the
    // Rev C socket boards have a second pulse-input pad, so everywhere else the
    // stored value stays at its V2 default whatever a hand-crafted POST says.
    if (!caps->has_carrier_rev_switch) next->revc_carrier = 2;

    // ftp_ps_dis is greyed out (and force-unchecked) in the UI when the
    // global wifi_ps_dis is ticked, so the form won't POST it. Preserve the
    // previously saved value rather than clobbering it to false — that way
    // the user's preference survives a round-trip through "global PS off".
    if (next->wifi_ps_disabled) next->ftp_ps_disabled = prev->ftp_ps_disabled;

    // V2.5.4: the radiation-only upload targets (Radmon, GMCMap, ThingSpeak)
    // are meaningless without the Geiger tube. The UI greys + force-unchecks
    // them when "Enable Geiger tube" is off (syncTube()); enforce the same
    // invariant here so a hand-crafted POST can't enable them, and so the
    // stored config stays honest. Unlike ftp_ps_dis we do NOT preserve the
    // prior choice — "tube off" genuinely turns these off, not just hides them.
    if (!next->tube_enabled) {
        next->send_radmon = false;
        next->send_gmc = false;
        next->send_thingspeak = false;
        next->pcnt_filter = false;   // V2.5.16: width filter needs count pulses
        next->deadtime_guard = false; // V2.5.30: dead-time guard needs count pulses
        next->hv_blank        = false;   // V2.6.29: HV blanking needs count pulses
    }

    // V2.5.30: dead-time guard is mutually exclusive with pcnt_filter — the guard
    // runs in the GMC ISR but pcnt_filter makes the PCNT hardware path (which the
    // guard can't reach) authoritative for the uploaded count. pcnt_filter WINS;
    // mirror the UI's syncGuard() greying so a hand-crafted POST can't set both.
    // (V2.6.29: hv_blank deliberately NOT cleared here — it COMPOSES with
    // pcnt_filter via the subtract mode; see config_fields.def hv_blank.)
    if (next->pcnt_filter) {
        next->deadtime_guard = false;
    }

    // V2.8.5: MQTT TLS Mode B (custom CA) with an empty CA certificate is
    // refused. mqtt_init() will not connect with it (it used to fall back to
    // an UNVERIFIED connection), so storing it would only produce a node that
    // silently stops publishing. Keep the previous mode + CA and report the
    // field like an out-of-range value; Mode D remains the explicit way to
    // connect without verification.
    if (next->mqtt_tls_enable && next->mqtt_tls_mode == 1 &&
        next->mqtt_tls_ca[0] == '\0') {
        next->mqtt_tls_mode = prev->mqtt_tls_mode;
        safe_strcpy(next->mqtt_tls_ca, prev->mqtt_tls_ca,
                    sizeof(next->mqtt_tls_ca));
        // Last writer of `rejected`, so the returned length is not kept.
        (void)form_append(rejected, rejected_sz, rej_n, "%s%s",
                          rej_n ? ", " : "",
                          "mqtt_tls_m (Mode B needs a CA certificate)");
    }
}
