#include "config.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "hal.h"   // HAL_HAS_I2C_PINOUT_SWITCH — gate the i2c-route dump line
#include "util.h"

static const char *TAG = "config";
static const char *NS  = "geiger";

// --- Compile-time defaults ---------------------------------------------------
//
// V2.4.1: all field defaults are declared INLINE in `config_fields.def`
// as the third (or fourth) argument of each X_* macro. Adding a new
// field's default = same line as the field declaration. The previous
// DEF_* macro tier (~50 #define lines) is gone — one source of truth.

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

// NVS has no float type. Store the IEEE-754 bit pattern in a u32 so we keep
// the full 32-bit precision without the round-trip lossiness of snprintf/atof.

void config_load(config_t *cfg) {
    config_defaults(cfg);
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace '%s' empty — using compile-time defaults", NS);
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed: %s — using defaults", esp_err_to_name(err));
        return;
    }

    // Generated NVS-read loop. Each field keeps its default on missing
    // (ESP_ERR_NVS_NOT_FOUND — first boot or newly-added schema field).
    // V2.4.1 (B2): unexpected failures (ESP_ERR_NVS_INVALID_LENGTH —
    // stored value larger than the current buffer, e.g. downgrade from
    // a future version that grew a field; or any other non-OK return)
    // now log ESP_LOGW so the user doesn't silently see their saved
    // value reverting to the compile-time default. The buffer is left
    // untouched on every failure (verified against the IDF nvs_get_str
    // contract), so this is purely a visibility fix.
    #define CFG_LOAD_LOG(_r, _key)                                              \
        do {                                                                    \
            if ((_r) != ESP_OK && (_r) != ESP_ERR_NVS_NOT_FOUND) {              \
                ESP_LOGW(TAG, "load '%s': %s (kept default)",                   \
                         (_key), esp_err_to_name(_r));                          \
            }                                                                   \
        } while (0)

    #define X_STR(name, size, key, def)               \
        do {                                          \
            size_t _len = (size);                     \
            esp_err_t _r = nvs_get_str(h, key, cfg->name, &_len); \
            CFG_LOAD_LOG(_r, key);                    \
        } while (0);
    #define X_BOOL(name, key, def)                    \
        do {                                          \
            uint8_t _v;                               \
            esp_err_t _r = nvs_get_u8(h, key, &_v);   \
            if (_r == ESP_OK) cfg->name = (_v != 0); \
            CFG_LOAD_LOG(_r, key);                    \
        } while (0);
    #define X_U32(name, key, def, lo, hi)             \
        do {                                          \
            uint32_t _v;                              \
            esp_err_t _r = nvs_get_u32(h, key, &_v);  \
            if (_r == ESP_OK) cfg->name = _v;         \
            CFG_LOAD_LOG(_r, key);                    \
        } while (0);
    #define X_F32(name, key, def, lo, hi)             \
        do {                                          \
            uint32_t _u;                              \
            esp_err_t _r = nvs_get_u32(h, key, &_u);  \
            if (_r == ESP_OK) memcpy(&cfg->name, &_u, sizeof(float)); \
            CFG_LOAD_LOG(_r, key);                    \
        } while (0);
    #define X_U8(name, key, def, lo, hi)              \
        do {                                          \
            uint8_t _v;                               \
            esp_err_t _r = nvs_get_u8(h, key, &_v);   \
            if (_r == ESP_OK) cfg->name = _v;         \
            CFG_LOAD_LOG(_r, key);                    \
        } while (0);
    #include "config_fields.def"
    #undef X_STR
    #undef X_BOOL
    #undef X_U32
    #undef X_F32
    #undef X_U8
    #undef CFG_LOAD_LOG

    nvs_close(h);

    // V2.5.23: the dump itself moved to config_log_summary() below — one place
    // to edit when a field is added. Print it HERE only when syslog is off:
    // there's no rsyslog server to forward to later, so this boot copy is the
    // only chance. For syslog-ON nodes, main.c calls config_log_summary() right
    // after the UDP client is up, so the dump lands on /log AND the server (a
    // boot copy would predate syslog and never leave the device). One function,
    // one print per boot.
    if (!cfg->syslog_enable) {
        config_log_summary(cfg);
    }
}

// Full config dump — secrets masked, grouped per line for a readable trace.
// DELIBERATELY NOT schema-generated: the hand-grouped shape (fields per line,
// passwords under a single token) reads better than a per-field loop, and the
// maintenance cost is one edit HERE per new field. Printed once per boot — see
// config_load() (syslog off) and main.c (syslog on, after the UDP client is up).
void config_log_summary(const config_t *cfg) {
    ESP_LOGI(TAG, "config loaded (ssid=%s host=%s ap=%s tx=%lums)",
             cfg->wifi_ssid, cfg->wifi_hostname, cfg->ap_name,
             (unsigned long)cfg->tx_interval_ms);

    #define MASK(s) ((s)[0] ? "<set>" : "<empty>")
    ESP_LOGI(TAG, "  wifi:             ssid=%s pw=%s host=%s ap_name=%s",
             cfg->wifi_ssid, MASK(cfg->wifi_password), cfg->wifi_hostname, cfg->ap_name);
    ESP_LOGI(TAG, "  wifi:             11bg_only=%d ht20_only=%d ps_disabled=%d ext_antenna=%d",
             cfg->wifi_11bg_only, cfg->wifi_ht20_only, cfg->wifi_ps_disabled,
             cfg->use_external_antenna);
    ESP_LOGI(TAG, "  madavi:           enabled=%d https=%d",
             cfg->send_madavi, cfg->madavi_https);
    ESP_LOGI(TAG, "  sensor.community: enabled=%d https=%d",
             cfg->send_sensorc, cfg->sensorc_https);
    ESP_LOGI(TAG, "  radmon:           enabled=%d https=%d user=%s pw=%s",
             cfg->send_radmon, cfg->radmon_https,
             cfg->radmon_user[0] ? cfg->radmon_user : "<empty>",
             MASK(cfg->radmon_password));
    ESP_LOGI(TAG, "  ntp:              server1=%s server2=%s server3=%s",
             cfg->ntp_server[0]  ? cfg->ntp_server  : "<empty>",
             cfg->ntp_server2[0] ? cfg->ntp_server2 : "<empty>",
             cfg->ntp_server3[0] ? cfg->ntp_server3 : "<empty>");
    ESP_LOGI(TAG, "  tz:               %s", cfg->tz_posix);
    ESP_LOGI(TAG, "  web:              admin_pw=%s tx_interval=%lums",
             MASK(cfg->ap_password), (unsigned long)cfg->tx_interval_ms);
    ESP_LOGI(TAG, "  heap-guard:       floor=%lukB (0=off)",
             (unsigned long)cfg->heap_guard_floor_kb);
    ESP_LOGI(TAG, "  station:          altitude=%.1fm send_sealevel_pressure=%d",
             (double)cfg->station_altitude_m, cfg->send_sealevel_pressure);
    ESP_LOGI(TAG, "  ftp:              enabled=%d tls=%d host=%s user=%s pw=%s",
             cfg->ftp_enabled, cfg->ftp_tls,
             cfg->ftp_host[0] ? cfg->ftp_host : "<empty>",
             cfg->ftp_user[0] ? cfg->ftp_user : "<empty>",
             MASK(cfg->ftp_password));
    ESP_LOGI(TAG, "  ftp:              path=%s interval=%lumin ps_disabled=%d tls12_only=%d",
             cfg->ftp_path[0] ? cfg->ftp_path : "<empty>",
             (unsigned long)cfg->ftp_interval_min, cfg->ftp_ps_disabled,
             cfg->ftp_tls12_only);
    ESP_LOGI(TAG, "  ui:               speaker_tick=%d led_tick=%d play_sound=%d show_display=%d oled_brightness=%d%%",
             cfg->speaker_tick, cfg->led_tick, cfg->play_sound, cfg->show_display,
             cfg->oled_brightness_pct);
    // display_mode is configured here; resolved (auto→radiation/rotation)
    // value gets logged separately by display_setup() after panel probe.
    const char *disp_mode_str =
        (cfg->display_mode == 0) ? "auto"      :
        (cfg->display_mode == 1) ? "radiation" :
        (cfg->display_mode == 2) ? "rotation"  : "?";
    ESP_LOGI(TAG, "  display:          mode=%lu(%s)",
             (unsigned long)cfg->display_mode, disp_mode_str);
    ESP_LOGI(TAG, "  tube:             enabled=%d", cfg->tube_enabled);
    ESP_LOGI(TAG, "  pcnt-filter:      enabled=%d width=%luns",
             cfg->pcnt_filter, (unsigned long)cfg->pcnt_filter_width_ns);
    // i2c_pinout only does anything where the alternate pads exist
    // (HAL_HAS_I2C_PINOUT_SWITCH); elsewhere it's force-disabled + greyed in the
    // UI, so dump it only where it's actually configurable rather than printing
    // an inert "pinout=0" on the other 4 boards (V2.5.20/L2).
#if HAL_HAS_I2C_PINOUT_SWITCH
    ESP_LOGI(TAG, "  i2c:              pinout=%d (0=onboard/STEMMA)",
             cfg->i2c_pinout);
#endif
    ESP_LOGI(TAG, "  openSenseMap:     enabled=%d box_id=%s",
             cfg->send_osm, cfg->osm_box_id[0] ? cfg->osm_box_id : "<empty>");
    ESP_LOGI(TAG, "  openSenseMap:     access_token=%s",
             MASK(cfg->osm_access_token));
    ESP_LOGI(TAG, "  aqi.eco:          enabled=%d token=%s",
             cfg->send_aqi, MASK(cfg->aqi_token));
    // gmcmap account/geiger IDs are public-ish account identifiers (like
    // radmon_user above), not secrets — shown in clear. ThingSpeak write
    // keys ARE secrets → MASK()ed.
    ESP_LOGI(TAG, "  gmcmap:           enabled=%d account=%s geiger=%s",
             cfg->send_gmc,
             cfg->gmc_account_id[0] ? cfg->gmc_account_id : "<empty>",
             cfg->gmc_geiger_id[0]  ? cfg->gmc_geiger_id  : "<empty>");
    ESP_LOGI(TAG, "  thingspeak:       enabled=%d https=%d key=%s",
             cfg->send_thingspeak, cfg->thingspeak_https,
             MASK(cfg->thingspeak_api_key));
    ESP_LOGI(TAG, "  thingspeak.pm:    enabled=%d https=%d key=%s",
             cfg->send_thingspeak_pm, cfg->thingspeak_pm_https,
             MASK(cfg->thingspeak_pm_api_key));
    ESP_LOGI(TAG, "  mqtt:             enabled=%d broker=%s:%lu user=%s pw=%s",
             cfg->mqtt_enable,
             cfg->mqtt_broker[0] ? cfg->mqtt_broker : "<empty>",
             (unsigned long)cfg->mqtt_port,
             cfg->mqtt_user[0] ? cfg->mqtt_user : "<empty>",
             MASK(cfg->mqtt_password));
    ESP_LOGI(TAG, "  mqtt:             topic_prefix=%s ha_discovery=%d",
             cfg->mqtt_topic_prefix[0] ? cfg->mqtt_topic_prefix : "<empty>",
             cfg->mqtt_ha_discovery);
    // mqtt_tls_mode legend: 0=A Mozilla CA bundle / 1=B custom CA / 2=D skip-verify
    const char *mqtt_tls_str =
        (cfg->mqtt_tls_mode == 0) ? "A:bundle"  :
        (cfg->mqtt_tls_mode == 1) ? "B:custom"  :
        (cfg->mqtt_tls_mode == 2) ? "D:skip"    : "?";
    ESP_LOGI(TAG, "  mqtt.tls:         enabled=%d mode=%lu(%s) ca=%s",
             cfg->mqtt_tls_enable,
             (unsigned long)cfg->mqtt_tls_mode, mqtt_tls_str,
             cfg->mqtt_tls_ca[0] ? "<set>" : "<empty>");
    ESP_LOGI(TAG, "  syslog:           enabled=%d host=%s port=%lu",
             cfg->syslog_enable,
             cfg->syslog_host[0] ? cfg->syslog_host : "<empty>",
             (unsigned long)cfg->syslog_port);
    #undef MASK
}

esp_err_t config_save(const config_t *cfg) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open RW failed: %s", esp_err_to_name(err));
        return err;
    }

    // Generated NVS-write loop. Float fields stored as u32 bit pattern
    // (no NVS native float type). Errors short-circuit to `out:` so we
    // always commit (or skip) cleanly and close the handle.
    #define X_STR(name, size, key, def)               \
        err = nvs_set_str(h, key, cfg->name);         \
        if (err) goto out;
    #define X_BOOL(name, key, def)                    \
        do {                                          \
            uint8_t _u = cfg->name ? 1 : 0;           \
            err = nvs_set_u8(h, key, _u);             \
            if (err) goto out;                        \
        } while (0);
    #define X_U32(name, key, def, lo, hi)             \
        err = nvs_set_u32(h, key, cfg->name);         \
        if (err) goto out;
    #define X_F32(name, key, def, lo, hi)             \
        do {                                          \
            uint32_t _u;                              \
            memcpy(&_u, &cfg->name, sizeof(float));   \
            err = nvs_set_u32(h, key, _u);            \
            if (err) goto out;                        \
        } while (0);
    #define X_U8(name, key, def, lo, hi)              \
        err = nvs_set_u8(h, key, cfg->name);          \
        if (err) goto out;
    #include "config_fields.def"
    #undef X_STR
    #undef X_BOOL
    #undef X_U32
    #undef X_F32
    #undef X_U8

    err = nvs_commit(h);
out:
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "config saved");
    } else {
        ESP_LOGE(TAG, "config_save failed: %s", esp_err_to_name(err));
    }
    return err;
}

// --- HTTP POST helpers (consumed by http_server.c::config_post) ------------
//
// V2.4.1: the per-field POST dispatch is also derived from the schema.
// Keeping the helpers HERE (in config.c) rather than http_server.c keeps
// schema-touching code in one translation unit — http_server.c just calls
// `config_post_preclear_bools()` then loops over the form body calling
// `config_post_apply_field()` for each k=v.

void config_post_preclear_bools(config_t *next) {
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

bool config_post_apply_field(config_t *next, const char *key, const char *val) {
    // Generated per-field dispatch. Each branch returns true when the
    // key matches, regardless of whether the value passed validation —
    // out-of-range numerics keep the prior field value silently.
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
            long _v = strtol(val, NULL, 10);                     \
            if (_v >= (long)(lo) && _v <= (long)(hi))            \
                next->name = (uint32_t)_v;                       \
            return true;                                         \
        }
    #define X_F32(name, k, def, lo, hi)                          \
        if (strcmp(key, k) == 0) {                               \
            float _v = strtof(val, NULL);                        \
            if (_v >= (lo) && _v <= (hi))                        \
                next->name = _v;                                 \
            return true;                                         \
        }
    #define X_U8(name, k, def, lo, hi)                           \
        if (strcmp(key, k) == 0) {                               \
            long _v = strtol(val, NULL, 10);                     \
            if (_v >= (long)(lo) && _v <= (long)(hi))            \
                next->name = (uint8_t)_v;                        \
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
