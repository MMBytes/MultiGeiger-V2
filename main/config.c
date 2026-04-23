#include "config.h"

#include <string.h>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "config";
static const char *NS  = "geiger";

// --- Compile-time defaults ---------------------------------------------------
// Change a DEF_* here and reflash to seed a blank NVS; /config POST is the
// runtime path for everything else.

// WiFi creds default to empty — first-boot users configure them via the
// 2-minute AP window. ap_name + wifi_hostname also default empty so main.c
// can fill them with chip-id-derived defaults after reading the factory MAC.
// All other defaults stay populated so a clean device still transmits
// correctly once WiFi is configured.
#define DEF_WIFI_SSID       ""
#define DEF_WIFI_PASSWORD   ""
#define DEF_WIFI_HOSTNAME   ""
#define DEF_AP_NAME         ""
#define DEF_SEND_MADAVI     true
#define DEF_MADAVI_HTTPS    true
#define DEF_SEND_SENSORC    true
#define DEF_SENSORC_HTTPS   true
#define DEF_SEND_RADMON     false
#define DEF_RADMON_HTTPS    true
#define DEF_RADMON_USER     ""
#define DEF_RADMON_PASSWORD ""
#define DEF_NTP_SERVER      "pool.ntp.org"
#define DEF_NTP_SERVER2     ""
#define DEF_NTP_SERVER3     ""
#define DEF_TZ_POSIX        "AEST-10AEDT,M10.1.0,M4.1.0/3"
#define DEF_AP_PASSWORD     "ESP32Geiger"
#define DEF_TX_INTERVAL_MS  150000
#define DEF_ALTITUDE_M      0.0f
#define DEF_SEND_SEALEVEL   false
#define DEF_FTP_ENABLED     false
#define DEF_FTP_TLS         false
#define DEF_FTP_HOST        ""
#define DEF_FTP_USER        ""
#define DEF_FTP_PASSWORD    ""
#define DEF_FTP_PATH        ""
#define DEF_FTP_INTERVAL_MIN 60
#define DEF_SPEAKER_TICK    false
#define DEF_LED_TICK        true
#define DEF_PLAY_SOUND      false
#define DEF_SHOW_DISPLAY    true
#define DEF_WIFI_11BG_ONLY  false
#define DEF_WIFI_HT20_ONLY  false
#define DEF_WIFI_PS_DISABLED false

void config_defaults(config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    strncpy(cfg->wifi_ssid,       DEF_WIFI_SSID,       sizeof(cfg->wifi_ssid) - 1);
    strncpy(cfg->wifi_password,   DEF_WIFI_PASSWORD,   sizeof(cfg->wifi_password) - 1);
    strncpy(cfg->wifi_hostname,   DEF_WIFI_HOSTNAME,   sizeof(cfg->wifi_hostname) - 1);
    strncpy(cfg->ap_name,         DEF_AP_NAME,         sizeof(cfg->ap_name) - 1);
    cfg->send_madavi   = DEF_SEND_MADAVI;
    cfg->madavi_https  = DEF_MADAVI_HTTPS;
    cfg->send_sensorc  = DEF_SEND_SENSORC;
    cfg->sensorc_https = DEF_SENSORC_HTTPS;
    cfg->send_radmon   = DEF_SEND_RADMON;
    cfg->radmon_https  = DEF_RADMON_HTTPS;
    strncpy(cfg->radmon_user,     DEF_RADMON_USER,     sizeof(cfg->radmon_user) - 1);
    strncpy(cfg->radmon_password, DEF_RADMON_PASSWORD, sizeof(cfg->radmon_password) - 1);
    strncpy(cfg->ntp_server,      DEF_NTP_SERVER,      sizeof(cfg->ntp_server) - 1);
    strncpy(cfg->ntp_server2,     DEF_NTP_SERVER2,     sizeof(cfg->ntp_server2) - 1);
    strncpy(cfg->ntp_server3,     DEF_NTP_SERVER3,     sizeof(cfg->ntp_server3) - 1);
    strncpy(cfg->tz_posix,        DEF_TZ_POSIX,        sizeof(cfg->tz_posix) - 1);
    strncpy(cfg->ap_password,     DEF_AP_PASSWORD,     sizeof(cfg->ap_password) - 1);
    cfg->tx_interval_ms       = DEF_TX_INTERVAL_MS;
    cfg->station_altitude_m   = DEF_ALTITUDE_M;
    cfg->send_sealevel_pressure = DEF_SEND_SEALEVEL;
    cfg->ftp_enabled          = DEF_FTP_ENABLED;
    cfg->ftp_tls              = DEF_FTP_TLS;
    strncpy(cfg->ftp_host,     DEF_FTP_HOST,     sizeof(cfg->ftp_host) - 1);
    strncpy(cfg->ftp_user,     DEF_FTP_USER,     sizeof(cfg->ftp_user) - 1);
    strncpy(cfg->ftp_password, DEF_FTP_PASSWORD, sizeof(cfg->ftp_password) - 1);
    strncpy(cfg->ftp_path,     DEF_FTP_PATH,     sizeof(cfg->ftp_path) - 1);
    cfg->ftp_interval_min     = DEF_FTP_INTERVAL_MIN;
    cfg->speaker_tick         = DEF_SPEAKER_TICK;
    cfg->led_tick             = DEF_LED_TICK;
    cfg->play_sound           = DEF_PLAY_SOUND;
    cfg->show_display         = DEF_SHOW_DISPLAY;
    cfg->wifi_11bg_only       = DEF_WIFI_11BG_ONLY;
    cfg->wifi_ht20_only       = DEF_WIFI_HT20_ONLY;
    cfg->wifi_ps_disabled     = DEF_WIFI_PS_DISABLED;
}

static void load_str(nvs_handle_t h, const char *key, char *buf, size_t bufsz) {
    size_t len = bufsz;
    nvs_get_str(h, key, buf, &len);  // leaves default untouched on failure
}

static void load_bool(nvs_handle_t h, const char *key, bool *out) {
    uint8_t v;
    if (nvs_get_u8(h, key, &v) == ESP_OK) *out = (v != 0);
}

static void load_u32(nvs_handle_t h, const char *key, uint32_t *out) {
    uint32_t v;
    if (nvs_get_u32(h, key, &v) == ESP_OK) *out = v;
}

// NVS has no float type. Store the IEEE-754 bit pattern in a u32 so we keep
// the full 32-bit precision without the round-trip lossiness of snprintf/atof.
static void load_f32(nvs_handle_t h, const char *key, float *out) {
    uint32_t v;
    if (nvs_get_u32(h, key, &v) == ESP_OK) {
        memcpy(out, &v, sizeof(*out));
    }
}

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
    load_str (h, "wifi_ssid",  cfg->wifi_ssid,       sizeof(cfg->wifi_ssid));
    load_str (h, "wifi_pw",    cfg->wifi_password,   sizeof(cfg->wifi_password));
    load_str (h, "wifi_host",  cfg->wifi_hostname,   sizeof(cfg->wifi_hostname));
    load_str (h, "ap_name",    cfg->ap_name,         sizeof(cfg->ap_name));
    load_bool(h, "wifi_11bg",  &cfg->wifi_11bg_only);
    load_bool(h, "wifi_ht20",  &cfg->wifi_ht20_only);
    load_bool(h, "wifi_ps_dis",&cfg->wifi_ps_disabled);
    load_bool(h, "send_mad",   &cfg->send_madavi);
    load_bool(h, "mad_https",  &cfg->madavi_https);
    load_bool(h, "send_sc",    &cfg->send_sensorc);
    load_bool(h, "sc_https",   &cfg->sensorc_https);
    load_bool(h, "send_rad",   &cfg->send_radmon);
    load_bool(h, "rad_https",  &cfg->radmon_https);
    load_str (h, "rad_user",   cfg->radmon_user,     sizeof(cfg->radmon_user));
    load_str (h, "rad_pw",     cfg->radmon_password, sizeof(cfg->radmon_password));
    load_str (h, "ntp",        cfg->ntp_server,      sizeof(cfg->ntp_server));
    load_str (h, "ntp2",       cfg->ntp_server2,     sizeof(cfg->ntp_server2));
    load_str (h, "ntp3",       cfg->ntp_server3,     sizeof(cfg->ntp_server3));
    load_str (h, "tz_posix",   cfg->tz_posix,        sizeof(cfg->tz_posix));
    load_str (h, "ap_pw",      cfg->ap_password,     sizeof(cfg->ap_password));
    load_u32 (h, "tx_int_ms",  &cfg->tx_interval_ms);
    load_f32 (h, "alt_m",      &cfg->station_altitude_m);
    load_bool(h, "send_sl",    &cfg->send_sealevel_pressure);
    load_bool(h, "ftp_en",     &cfg->ftp_enabled);
    load_bool(h, "ftp_tls",    &cfg->ftp_tls);
    load_str (h, "ftp_host",   cfg->ftp_host,     sizeof(cfg->ftp_host));
    load_str (h, "ftp_user",   cfg->ftp_user,     sizeof(cfg->ftp_user));
    load_str (h, "ftp_pw",     cfg->ftp_password, sizeof(cfg->ftp_password));
    load_str (h, "ftp_path",   cfg->ftp_path,     sizeof(cfg->ftp_path));
    load_u32 (h, "ftp_int",    &cfg->ftp_interval_min);
    load_bool(h, "sp_tick",    &cfg->speaker_tick);
    load_bool(h, "led_tick",   &cfg->led_tick);
    load_bool(h, "play_sound", &cfg->play_sound);
    load_bool(h, "show_disp",  &cfg->show_display);
    nvs_close(h);
    ESP_LOGI(TAG, "config loaded (ssid=%s host=%s ap=%s tx=%lums)",
             cfg->wifi_ssid, cfg->wifi_hostname, cfg->ap_name,
             (unsigned long)cfg->tx_interval_ms);
}

esp_err_t config_save(const config_t *cfg) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open RW failed: %s", esp_err_to_name(err));
        return err;
    }
    #define SET_STR(k, v)  do { err = nvs_set_str(h, k, v); if (err) goto out; } while (0)
    #define SET_U8(k, v)   do { uint8_t u = (v) ? 1 : 0; err = nvs_set_u8(h, k, u); if (err) goto out; } while (0)
    #define SET_U32(k, v)  do { err = nvs_set_u32(h, k, v); if (err) goto out; } while (0)
    #define SET_F32(k, v)  do { uint32_t u; memcpy(&u, &(v), sizeof(u)); err = nvs_set_u32(h, k, u); if (err) goto out; } while (0)
    SET_STR("wifi_ssid",  cfg->wifi_ssid);
    SET_STR("wifi_pw",    cfg->wifi_password);
    SET_STR("wifi_host",  cfg->wifi_hostname);
    SET_STR("ap_name",    cfg->ap_name);
    SET_U8 ("wifi_11bg",  cfg->wifi_11bg_only);
    SET_U8 ("wifi_ht20",  cfg->wifi_ht20_only);
    SET_U8 ("wifi_ps_dis",cfg->wifi_ps_disabled);
    SET_U8 ("send_mad",   cfg->send_madavi);
    SET_U8 ("mad_https",  cfg->madavi_https);
    SET_U8 ("send_sc",    cfg->send_sensorc);
    SET_U8 ("sc_https",   cfg->sensorc_https);
    SET_U8 ("send_rad",   cfg->send_radmon);
    SET_U8 ("rad_https",  cfg->radmon_https);
    SET_STR("rad_user",   cfg->radmon_user);
    SET_STR("rad_pw",     cfg->radmon_password);
    SET_STR("ntp",        cfg->ntp_server);
    SET_STR("ntp2",       cfg->ntp_server2);
    SET_STR("ntp3",       cfg->ntp_server3);
    SET_STR("tz_posix",   cfg->tz_posix);
    SET_STR("ap_pw",      cfg->ap_password);
    SET_U32("tx_int_ms",  cfg->tx_interval_ms);
    SET_F32("alt_m",      cfg->station_altitude_m);
    SET_U8 ("send_sl",    cfg->send_sealevel_pressure);
    SET_U8 ("ftp_en",     cfg->ftp_enabled);
    SET_U8 ("ftp_tls",    cfg->ftp_tls);
    SET_STR("ftp_host",   cfg->ftp_host);
    SET_STR("ftp_user",   cfg->ftp_user);
    SET_STR("ftp_pw",     cfg->ftp_password);
    SET_STR("ftp_path",   cfg->ftp_path);
    SET_U32("ftp_int",    cfg->ftp_interval_min);
    SET_U8 ("sp_tick",    cfg->speaker_tick);
    SET_U8 ("led_tick",   cfg->led_tick);
    SET_U8 ("play_sound", cfg->play_sound);
    SET_U8 ("show_disp",  cfg->show_display);
    err = nvs_commit(h);
    #undef SET_STR
    #undef SET_U8
    #undef SET_U32
    #undef SET_F32
out:
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "config saved");
    } else {
        ESP_LOGE(TAG, "config_save failed: %s", esp_err_to_name(err));
    }
    return err;
}
