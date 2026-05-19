#pragma once

/** @file
 *  @brief Runtime-editable configuration, persisted in NVS.
 *
 *  Stored under the "geiger" NVS namespace. Missing keys fall back to
 *  compile-time defaults declared alongside each field in
 *  `config_fields.def` — so a wiped NVS always boots into a usable state.
 *
 *  V2.4.1: the struct and the NVS/POST plumbing are now generated from a
 *  single schema in `config_fields.def` via X-macros. The previous design
 *  had the field list duplicated across:
 *    - the struct here in config.h
 *    - `config_defaults()` in config.c
 *    - the NVS load loop in config.c
 *    - the NVS save loop in config.c
 *    - the POST pre-clear-bools loop in http_server.c
 *    - the POST per-field dispatch in http_server.c
 *  Adding one field used to touch ~6 places; now it's one line in the .def.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// V2.4.1 (C3): named upper bounds for the string-field MAX content
// length (NOT including the null terminator). Pre-V2.4.1 these were
// inline magic numbers next to every `char field[33];` / `[65];` /
// `[26];` declaration with the rationale in adjacent comments.
// Each `X_STR` in `config_fields.def` declares `field[CFG_*_MAX + 1]`
// so the buffer always has room for content + NUL.
//
// Values are derived from real spec / protocol limits:
#define CFG_WIFI_SSID_MAX    32   // IEEE 802.11 SSID maximum (32 bytes)
#define CFG_WIFI_PSK_MAX     64   // WPA-PSK: 64 hex chars OR 8..63 ASCII
#define CFG_HOSTNAME_MAX     32   // DNS label / DHCP option-12 practical
#define CFG_AP_NAME_MAX      32   // Same as SSID since the AP IS an SSID
#define CFG_NTP_HOST_MAX     63   // RFC 1123 single-label cap
#define CFG_TZ_POSIX_MAX     47   // POSIX TZ string practical max
#define CFG_AP_PW_MAX        32   // Web admin + AP password — WPA2 min 8
#define CFG_USER_NAME_MAX    32   // Radmon / FTP user
#define CFG_PASSWORD_MAX     64   // Radmon / FTP password
#define CFG_FTP_HOST_MAX     63   // FQDN or "host:port" — RFC 1123 cap
#define CFG_FTP_PATH_MAX     63   // remote dir path
#define CFG_OSM_BOX_MAX      25   // MongoDB ObjectId hex (24 chars)
#define CFG_TOKEN_MAX        64   // OSM / aqi.eco access token (64 hex)
#define CFG_MQTT_HOST_MAX    63   // broker FQDN / IPv4 / "host:port" — RFC 1123
#define CFG_MQTT_PFX_MAX     31   // topic prefix — kept short, full topic is
                                  //   "<pfx>/<chip-id>/state" so MQTT 64-char
                                  //   single-level cap leaves room for both
#define CFG_MQTT_CA_CERT_MAX 2400 // PEM-encoded CA cert for Mode B (custom CA).
                                  //   Sized for typical RSA-4096 self-signed CA
                                  //   (~2.2-2.6 KB PEM) with comfortable headroom.
                                  //   Stored verbatim in NVS; URL-encoded form
                                  //   POST stays well under the per-board form
                                  //   buffer (HAL_CFG_FORM_BUF_SIZE in hal.h).
#define CFG_SYSLOG_HOST_MAX  63   // syslog server FQDN / IPv4 — RFC 1123 cap

typedef struct {
    // Struct members generated from the schema. See `config_fields.def`
    // for the canonical field list (name, NVS key, default, validation).
    #define X_STR(name, size, key, def)            char     name[size];
    #define X_BOOL(name, key, def)                 bool     name;
    #define X_U32(name, key, def, lo, hi)          uint32_t name;
    #define X_F32(name, key, def, lo, hi)          float    name;
    #define X_U8(name, key, def, lo, hi)           uint8_t  name;
    #include "config_fields.def"
    #undef X_STR
    #undef X_BOOL
    #undef X_U32
    #undef X_F32
    #undef X_U8
} config_t;

/** @brief Fill cfg with compile-time defaults. Always safe to call. */
void config_defaults(config_t *cfg);

/** @brief Load from NVS, falling back to defaults for missing keys.
 *
 *  Never fails — unrecoverable NVS issues just leave cfg at defaults.
 */
void config_load(config_t *cfg);

/** @brief Persist cfg to NVS. Returns esp_err from nvs_commit. */
esp_err_t config_save(const config_t *cfg);

/** @brief Pre-clear all bool fields in `next`.
 *
 *  Helper for the HTTP POST handler: form submissions only include
 *  ticked checkboxes, so every bool must be set false before parsing
 *  and re-enabled on key match. Generated from the schema so adding
 *  a new bool needs no parallel edit here.
 */
void config_post_preclear_bools(config_t *next);

/** @brief Try to apply one form field to `next`.
 *
 *  Returns true if `key` matched a known schema field (regardless of
 *  whether the value passed validation — out-of-range numerics are
 *  silently ignored and the field keeps its prior value). Returns
 *  false if `key` is unknown — caller can then check for non-schema
 *  keys (e.g. `save`, `save_restart`) or ignore.
 *
 *  Specialised validators (e.g. OLED brightness step constraint) are
 *  applied by the caller BEFORE calling this — if the special handler
 *  consumed the key, don't call here.
 */
bool config_post_apply_field(config_t *next, const char *key, const char *val);
