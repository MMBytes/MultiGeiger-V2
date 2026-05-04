#pragma once

/** @file
 *  @brief Runtime-editable configuration, persisted in NVS.
 *
 *  Stored under the "geiger" NVS namespace. Missing keys fall back to
 *  compile-time defaults (DEF_* in config.c) so the device always boots
 *  with usable settings, even with a wiped NVS.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    // WiFi station credentials. Compile-time defaults cover first boot.
    char     wifi_ssid[33];
    char     wifi_password[65];

    // DHCP hostname (DHCP option 12 in the router lease table).
    // Empty = auto-filled at boot with "MultiGeiger<chip-id-decimal>"
    // (no hyphen — TP-Link DHCP silently drops DISCOVERs with hyphens).
    char     wifi_hostname[33];

    // AP-mode SSID broadcast during the 2-minute boot window and when no STA
    // credentials are configured. Empty = auto-filled at boot with
    // "esp32-<chip-id-decimal>".
    char     ap_name[33];

    // Radio capability limits applied to the STA interface before connect.
    //   wifi_11bg_only: disables 802.11n (radio falls back to 11b/g only).
    //   wifi_ht20_only: forces 20 MHz channel bandwidth (disables HT40).
    // Both default false = full capability. Compatibility knob for APs that
    // mishandle HT40 or 11n management frames.
    bool     wifi_11bg_only;
    bool     wifi_ht20_only;

    // When true, WiFi modem sleep is disabled (WIFI_PS_NONE). When false
    // (default), minimum modem sleep is used (WIFI_PS_MIN_MODEM), matching
    // the upstream MultiGeiger behaviour and reducing power draw slightly.
    bool     wifi_ps_disabled;

    // When true, route the WiFi RF chain to the u.FL external-antenna
    // connector (FeatherS3-D and similar boards with an onboard SPDT RF
    // switch). When false (default), the onboard PCB antenna is used.
    // No effect on boards without HAL_HAS_ANTENNA_SWITCH; the /config UI
    // greys the checkbox out in that case.
    bool     use_external_antenna;

    // Upload targets — per-target enable plus HTTPS toggle.
    bool     send_madavi;
    bool     madavi_https;
    bool     send_sensorc;
    bool     sensorc_https;
    bool     send_radmon;
    bool     radmon_https;
    char     radmon_user[33];
    char     radmon_password[65];

    // Up to three NTP servers. Empty strings are skipped.
    char     ntp_server[64];
    char     ntp_server2[64];
    char     ntp_server3[64];

    // POSIX TZ string (see `man tzset`). Drives localtime() and strftime()
    // for log timestamps, the OLED time readout, and FTP upload filenames.
    // Examples:
    //   AEST-10AEDT,M10.1.0,M4.1.0/3   Sydney
    //   CET-1CEST,M3.5.0,M10.5.0/3     Germany
    //   UTC0                           UTC (no DST)
    char     tz_posix[48];

    // Web UI auth (user is always "admin"). Used by /config, /log and
    // /update. Default: ESP32Geiger.
    char     ap_password[33];

    // TX cycle interval (milliseconds).
    uint32_t tx_interval_ms;

    // Station altitude above sea level (metres). Used only to compute the
    // pressure-at-sealevel value sent to sensor.community.
    float    station_altitude_m;

    // When true, sensor.community receives pressure_sealevel in addition to
    // the raw pressure field. When false (default), the altitude-derived
    // value is omitted.
    bool     send_sealevel_pressure;

    // FTP log upload — periodically ships the in-memory log ring to a LAN
    // FTP server (e.g. router USB share). Passive mode.
    //   ftp_user/ftp_password empty = anonymous.
    //   ftp_path: remote directory; filename auto-generated as geiger_<chip>_<ts>.log.
    //   ftp_tls : explicit TLS (AUTH TLS on port 21). Certificate NOT verified
    //             (most LAN FTP servers use self-signed certs).
    bool     ftp_enabled;
    bool     ftp_tls;
    char     ftp_host[64];
    char     ftp_user[33];
    char     ftp_password[65];
    char     ftp_path[64];
    uint32_t ftp_interval_min;

    // When true, WiFi modem sleep is forced off for the duration of each FTP
    // transfer and restored to WIFI_PS_MIN_MODEM at the end. Workaround for
    // boards where DTIM-delayed TCP ACKs stall FTPS uploads (observed on the
    // older sensor with degraded WiFi RX). Default false — newer boards do
    // not need it. Has no effect when wifi_ps_disabled is true (radio is
    // already always-on); the /config UI greys this box out in that case.
    bool     ftp_ps_disabled;

    // User-facing feedback knobs.
    //   speaker_tick: piezo click on every counted GM pulse.
    //   led_tick    : onboard LED flash on every counted GM pulse.
    //   play_sound  : short boot chirp confirming the speaker is wired up.
    //   show_display: drive the OLED (boot splash + running stats).
    bool     speaker_tick;
    bool     led_tick;
    bool     play_sound;
    bool     show_display;
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
