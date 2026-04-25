#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "applog.h"
#include "env_sensor.h"
#include "config.h"
#include "display.h"
#include "http_server.h"
#include "log_ftp.h"
#include "ntp.h"
#include "speaker.h"
#include "transmission.h"
#include "tube.h"
#include "version.h"

static const char *TAG = "v2_main";

// Runtime configuration — loaded from NVS at boot with compile-time defaults
// as fallback (see config.c). Editable via the /config endpoint.
static config_t g_cfg;

// Hardware-derived identity, always populated at boot from the factory MAC.
// Never stored in NVS; never user-editable. g_chip_id is the canonical
// "esp32-<decimal>" string used as X-Sensor header and default AP SSID.
// g_chip_num is just the decimal portion, used to build the default
// "MultiGeiger-<decimal>" DHCP hostname.
static char      g_chip_id[20];
static uint32_t  g_chip_num;

static EventGroupHandle_t s_events;
#define EV_GOT_IP       BIT0
#define EV_DISCONNECTED BIT1

// --- Soak diagnostics (carried over) ---
static int64_t  t_attempt_start_us = 0;
static int64_t  t_sta_connected_us = 0;
static int64_t  t_last_got_ip_us   = 0;
static uint32_t n_attempts = 0, n_connects = 0, n_got_ip = 0, n_disconnects = 0;
static uint32_t last_disconnect_reason = 0;
static float    last_dhcp_s = 0.0f, last_assoc_s = 0.0f;

// --- NTP/TX state ---
static bool     ntp_started = false;
static uint32_t tx_cycles   = 0;

// --- Strict single-mode WiFi:
//   boot .. AP_WINDOW_US:     AP only (STA not started — radio unshared)
//   AP_WINDOW_US onward:       STA only (AP stopped — no fallback AP)
// If STA fails to obtain its first IP within STA_STARTUP_TIMEOUT_US of the
// switch, reboot to re-enter the AP window. Watchdog disarms permanently
// after the first GOT_IP — subsequent disconnects retry STA forever.
#define AP_WINDOW_US           (120 * 1000000LL)
#define STA_STARTUP_TIMEOUT_US (600 * 1000000LL)   // 10 min
static int64_t  boot_time_us       = 0;
static int64_t  sta_transition_us  = 0;
static bool     g_have_sta_creds   = false;
static volatile bool g_sta_connect_allowed = false;

static void mark_attempt(void) {
    t_attempt_start_us = esp_timer_get_time();
    n_attempts++;
}

// Apply user-configured radio capability limits to the STA interface.
// Called right before esp_wifi_start() when the AP→STA switch happens;
// the APIs take effect on the next association, so ordering matters.
// 11b/g-only disables 802.11n; HT20-only caps channel bandwidth at 20 MHz.
static void apply_radio_limits_sta(void) {
    uint8_t proto = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G;
    if (!g_cfg.wifi_11bg_only) proto |= WIFI_PROTOCOL_11N;
    esp_err_t r = esp_wifi_set_protocol(WIFI_IF_STA, proto);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "set_protocol(0x%02x) failed: %s", proto, esp_err_to_name(r));
    } else {
        ESP_LOGI(TAG, "STA protocol = 0x%02x (11bg_only=%d)",
                 proto, g_cfg.wifi_11bg_only);
    }
    wifi_bandwidth_t bw = g_cfg.wifi_ht20_only ? WIFI_BW20 : WIFI_BW40;
    r = esp_wifi_set_bandwidth(WIFI_IF_STA, bw);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "set_bandwidth(%d) failed: %s", bw, esp_err_to_name(r));
    } else {
        ESP_LOGI(TAG, "STA bandwidth = %s (ht20_only=%d)",
                 (bw == WIFI_BW20) ? "20MHz" : "40MHz", g_cfg.wifi_ht20_only);
    }
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base != WIFI_EVENT) return;
    switch (id) {
    case WIFI_EVENT_STA_START:
        if (!g_sta_connect_allowed) {
            ESP_LOGI(TAG, "STA_START (holding during boot AP window — no connect)");
            display_set_status(DSP_STATUS_WIFI, DSP_WIFI_AP);
        } else {
            mark_attempt();
            ESP_LOGI(TAG, "STA_START, calling connect (attempt #%lu)", (unsigned long)n_attempts);
            display_set_status(DSP_STATUS_WIFI, DSP_WIFI_CONNECTING);
            esp_wifi_connect();
        }
        break;
    case WIFI_EVENT_STA_CONNECTED: {
        wifi_event_sta_connected_t *e = (wifi_event_sta_connected_t *)data;
        t_sta_connected_us = esp_timer_get_time();
        last_assoc_s = (t_sta_connected_us - t_attempt_start_us) / 1e6f;
        n_connects++;
        ESP_LOGI(TAG, "STA_CONNECTED #%lu: ch=%d auth=%d bssid=" MACSTR " assoc=%.3fs",
                 (unsigned long)n_connects, e->channel, e->authmode,
                 MAC2STR(e->bssid), last_assoc_s);
        break;
    }
    case WIFI_EVENT_STA_DISCONNECTED: {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        last_disconnect_reason = e->reason;
        n_disconnects++;
        ESP_LOGW(TAG, "STA_DISCONNECTED #%lu: reason=%d",
                 (unsigned long)n_disconnects, e->reason);
        display_set_status(DSP_STATUS_WIFI, DSP_WIFI_ERROR);
        xEventGroupSetBits(s_events, EV_DISCONNECTED);
        break;
    }
    case WIFI_EVENT_AP_STACONNECTED: {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "AP client JOIN: " MACSTR " aid=%d",
                 MAC2STR(e->mac), e->aid);
        break;
    }
    case WIFI_EVENT_AP_STADISCONNECTED: {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "AP client LEAVE: " MACSTR " aid=%d",
                 MAC2STR(e->mac), e->aid);
        break;
    }
    default: break;
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base != IP_EVENT || id != IP_EVENT_STA_GOT_IP) return;
    ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
    int64_t now = esp_timer_get_time();
    last_dhcp_s = (now - t_sta_connected_us) / 1e6f;
    t_last_got_ip_us = now;
    n_got_ip++;
    esp_netif_dns_info_t d1 = { 0 }, d2 = { 0 };
    esp_netif_get_dns_info(e->esp_netif, ESP_NETIF_DNS_MAIN,   &d1);
    esp_netif_get_dns_info(e->esp_netif, ESP_NETIF_DNS_BACKUP, &d2);
    ESP_LOGI(TAG, "GOT_IP #%lu: " IPSTR " gw=" IPSTR
             " dns=" IPSTR " dns2=" IPSTR " dhcp=%.3fs",
             (unsigned long)n_got_ip, IP2STR(&e->ip_info.ip),
             IP2STR(&e->ip_info.gw),
             IP2STR(&d1.ip.u_addr.ip4), IP2STR(&d2.ip.u_addr.ip4),
             last_dhcp_s);
    display_set_status(DSP_STATUS_WIFI, DSP_WIFI_CONNECTED);
    xEventGroupSetBits(s_events, EV_GOT_IP);
}

static bool wifi_up(void) {
    wifi_ap_record_t ap;
    return esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
}

static void build_tx_context(tx_context_t *ctx,
                             uint32_t dt_ms, uint32_t counts, uint32_t hv_pulses,
                             uint32_t min_us, uint32_t max_us,
                             bool bme_valid, float bme_t, float bme_h, float bme_p) {
    memset(ctx, 0, sizeof(*ctx));
    uint32_t cpm = 0;
    if (dt_ms > 0) {
        cpm = (uint32_t)(((uint64_t)counts * 60000ULL) / dt_ms);
    }
    ctx->dt_ms      = dt_ms;
    ctx->gm_counts  = counts;
    ctx->cpm        = cpm;
    ctx->hv_pulses  = hv_pulses;
    ctx->min_micro  = (min_us == UINT32_MAX) ? 0 : min_us;
    ctx->max_micro  = max_us;
    ctx->sw_version = VERSION_STR;
    ctx->chip_id    = g_chip_id;

    wifi_ap_record_t ap_rec = { 0 };
    ctx->rssi = (esp_wifi_sta_get_ap_info(&ap_rec) == ESP_OK) ? ap_rec.rssi : -127;

    ctx->bme_valid            = bme_valid;
    ctx->bme_temperature_c    = bme_t;
    ctx->bme_humidity_pct     = bme_h;
    ctx->bme_pressure_pa      = bme_p;
    ctx->station_altitude_m   = g_cfg.station_altitude_m;
    ctx->send_sealevel_pressure = g_cfg.send_sealevel_pressure;

    ctx->madavi = (tx_target_t){
        .enabled = g_cfg.send_madavi,
        .url_http  = "http://api-rrd.madavi.de/data.php",
        .url_https = "https://api-rrd.madavi.de/data.php",
        .use_https = g_cfg.madavi_https,
        .use_insecure = false,
    };
    ctx->sensorc = (tx_target_t){
        .enabled = g_cfg.send_sensorc,
        .url_http  = "http://api.sensor.community/v1/push-sensor-data/",
        .url_https = "https://api.sensor.community/v1/push-sensor-data/",
        .use_https = g_cfg.sensorc_https,
        .use_insecure = false,
    };
    ctx->radmon = (tx_target_t){
        .enabled = g_cfg.send_radmon,
        .url_http  = "http://radmon.org/radmon.php",
        .url_https = "https://radmon.org/radmon.php",
        .use_https = g_cfg.radmon_https,
        .use_insecure = false,
    };
    ctx->radmon_user     = g_cfg.radmon_user;
    ctx->radmon_password = g_cfg.radmon_password;
}

static void do_tx_cycle(void) {
    uint32_t counts, dt_ms, min_us, max_us, hv_pulses;
    bool hv_error;
    tube_read(&counts, &dt_ms, &min_us, &max_us, &hv_pulses, &hv_error);

    float cps = (dt_ms > 0) ? (counts * 1000.0f / dt_ms) : 0.0f;
    float usvph = cps * SI22G_CPS_TO_USVPH;
    uint32_t cpm = (dt_ms > 0) ? (uint32_t)(((uint64_t)counts * 60000ULL) / dt_ms) : 0;
    wifi_ap_record_t ap_rec = { 0 };
    int rssi = (esp_wifi_sta_get_ap_info(&ap_rec) == ESP_OK) ? ap_rec.rssi : -127;
    ESP_LOGI(TAG, "CYCLE #%lu: dt=%lums counts=%lu cpm=%lu %.3fµSv/h "
             "hv_pulses=%lu hv_err=%d min_us=%lu max_us=%lu rssi=%ddBm",
             (unsigned long)++tx_cycles, (unsigned long)dt_ms,
             (unsigned long)counts, (unsigned long)cpm, usvph,
             (unsigned long)hv_pulses, hv_error,
             (unsigned long)(min_us == UINT32_MAX ? 0 : min_us),
             (unsigned long)max_us, rssi);

    display_set_status(DSP_STATUS_HV, hv_error ? DSP_HV_ERROR : DSP_HV_OK);
    int time_sec   = (int)(esp_timer_get_time() / 1000000LL);
    int rad_nsvph  = (int)(usvph * 1000.0f);
    display_running(time_sec, rad_nsvph, (int)cpm, g_cfg.show_display);

    float bme_t = 0, bme_h = 0, bme_p = 0;
    bool  bme_valid = false;
    if (env_sensor_present()) {
        if (env_sensor_read(&bme_t, &bme_h, &bme_p) == ESP_OK) {
            bme_valid = true;
            ESP_LOGI(TAG, "%s: T=%.2f°C  H=%.2f%%  P=%.2fhPa",
                     env_sensor_name(), bme_t, bme_h, bme_p / 100.0f);
            env_sensor_heat_periodic((uint32_t)(esp_timer_get_time() / 1000), bme_h);
        } else {
            ESP_LOGW(TAG, "%s: read failed", env_sensor_name());
        }
    }

    if (!wifi_up()) {
        ESP_LOGW(TAG, "skipping TX: WiFi down");
        return;
    }
    if (!ntp_time_valid()) {
        ESP_LOGW(TAG, "skipping TX: time not valid (no NTP sync yet)");
        return;
    }

    tx_context_t ctx;
    build_tx_context(&ctx, dt_ms, counts, hv_pulses, min_us, max_us,
                     bme_valid, bme_t, bme_h, bme_p);
    tx_transmit(&ctx);
}

void app_main(void) {
    // Install vprintf hook first so the very first ESP_LOGx below (and
    // everything after — WiFi, HTTP, sensors, TX) lands in the /log buffer.
    applog_init();

    ESP_LOGI(TAG, "%s (IDF %s)", VERSION_STR, esp_get_idf_version());

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    config_load(&g_cfg);

    // Chip ID is always derived from the factory MAC — never user-edited,
    // never stored in NVS. Format: "esp32-<decimal>" where <decimal> is the
    // last 3 MAC bytes packed as (byte5 = LSB, byte3 = MSB).
    {
        uint8_t m[6];
        esp_read_mac(m, ESP_MAC_WIFI_STA);
        g_chip_num = ((uint32_t)m[5]) | ((uint32_t)m[4] << 8) | ((uint32_t)m[3] << 16);
        snprintf(g_chip_id, sizeof(g_chip_id), "esp32-%lu", (unsigned long)g_chip_num);
        ESP_LOGI(TAG, "chip_id: %s (from MAC %02x:%02x:%02x:%02x:%02x:%02x)",
                 g_chip_id, m[0], m[1], m[2], m[3], m[4], m[5]);
    }

    // Fill auto defaults for ap_name + wifi_hostname if the user hasn't
    // chosen their own. These are stored as-is once the user saves the
    // config, so subsequent boots keep the user's values.
    if (g_cfg.ap_name[0] == 0) {
        snprintf(g_cfg.ap_name, sizeof(g_cfg.ap_name), "%s", g_chip_id);
    }
    if (g_cfg.wifi_hostname[0] == 0) {
        snprintf(g_cfg.wifi_hostname, sizeof(g_cfg.wifi_hostname),
                 "MultiGeiger%lu", (unsigned long)g_chip_num);
    }
    ESP_LOGI(TAG, "ap_name: '%s'  wifi_hostname: '%s'",
             g_cfg.ap_name, g_cfg.wifi_hostname);

    // Probe for environmental sensors before WiFi — independent of network,
    // failure is non-fatal (env_sensor_present() gates readings later).
    // env_sensor_init() also owns the I2C bus used by the OLED below.
    env_sensor_init();

    // OLED shares the env_sensor I2C bus — bring it up now so the
    // boot splash is visible while WiFi/NTP/etc. come up.
    display_setup(g_cfg.show_display);
    display_boot_screen();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    esp_netif_t *ap_netif  = esp_netif_create_default_wifi_ap();
    assert(ap_netif);

    // Set the DHCP hostname on the STA netif BEFORE esp_wifi_start(). Once
    // DHCP issues a DISCOVER this is the name in option 12 — visible in the
    // router's lease table. Safe to call on the AP netif too for symmetry.
    esp_netif_set_hostname(sta_netif, g_cfg.wifi_hostname);
    esp_netif_set_hostname(ap_netif,  g_cfg.wifi_hostname);

    s_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_ip_event, NULL));

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    g_have_sta_creds = (g_cfg.wifi_ssid[0] != 0);
    const bool have_sta_creds = g_have_sta_creds;

    // Boot AP: always up for AP_WINDOW_US so a fresh device is configurable
    // at http://192.168.4.1/config even before STA is set. SSID comes from
    // cfg.ap_name (defaulted to g_chip_id above when empty).
    wifi_config_t apc = { 0 };
    int ap_ssid_len = snprintf((char *)apc.ap.ssid, sizeof(apc.ap.ssid),
                               "%s", g_cfg.ap_name);
    apc.ap.ssid_len      = (ap_ssid_len > 0) ? ap_ssid_len : 0;
    apc.ap.channel       = 1;
    apc.ap.max_connection = 4;
    if (strlen(g_cfg.ap_password) >= 8) {
        strncpy((char *)apc.ap.password, g_cfg.ap_password, sizeof(apc.ap.password) - 1);
        apc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        apc.ap.authmode = WIFI_AUTH_OPEN;
        ESP_LOGW(TAG, "AP password <8 chars — falling back to OPEN AP");
    }

    // Boot in AP-only mode. STA config is deferred until the AP window
    // closes in the main loop, so the radio is never shared.
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &apc));
    ESP_ERROR_CHECK(esp_wifi_set_ps(g_cfg.wifi_ps_disabled ? WIFI_PS_NONE : WIFI_PS_MIN_MODEM));

    // Tube runs from boot regardless of WiFi state — counts accumulate in ISR.
    tube_setup();
    speaker_setup(g_cfg.play_sound, g_cfg.led_tick, g_cfg.speaker_tick);
    tx_setup();
    http_server_start(&g_cfg, g_chip_id);
    log_ftp_init(g_chip_id, &g_cfg);

    ESP_LOGI(TAG, "AP up: SSID=%s auth=%d (2-min boot window)",
             (char *)apc.ap.ssid, apc.ap.authmode);
    if (!have_sta_creds) {
        ESP_LOGW(TAG, "no WiFi SSID configured — AP-only. "
                 "Join %s and browse to http://192.168.4.1/config",
                 (char *)apc.ap.ssid);
    }
    ESP_LOGI(TAG, "esp_wifi_start()");
    ESP_ERROR_CHECK(esp_wifi_start());
    boot_time_us = esp_timer_get_time();

    const TickType_t tx_interval = pdMS_TO_TICKS(g_cfg.tx_interval_ms);
    TickType_t next_tx = xTaskGetTickCount() + tx_interval;

    while (1) {
        TickType_t now = xTaskGetTickCount();
        TickType_t wait = (next_tx > now) ? (next_tx - now) : 1;
        // Cap wait so post-loop polls (restart flag, NTP, AP window, FTP)
        // run at least once a second. Without this cap the wait can be the
        // full tx_interval (~150 s) and a restart flag set by /config or
        // /update only fires after the next TX cycle.
        if (wait > pdMS_TO_TICKS(1000)) wait = pdMS_TO_TICKS(1000);
        EventBits_t bits = xEventGroupWaitBits(
            s_events, EV_GOT_IP | EV_DISCONNECTED, pdTRUE, pdFALSE, wait);

        if (bits & EV_GOT_IP) {
            if (!ntp_started) {
                ntp_setup(g_cfg.ntp_server, g_cfg.ntp_server2, g_cfg.ntp_server3,
                          g_cfg.tz_posix);
                ntp_started = true;
            }
        }
        if (bits & EV_DISCONNECTED) {
            if (!g_sta_connect_allowed) {
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            mark_attempt();
            ESP_LOGI(TAG, "retry connect (attempt #%lu)", (unsigned long)n_attempts);
            esp_wifi_connect();
            continue;
        }

        ntp_poll();

        // End of boot AP window: stop the AP and switch to STA-only.
        // Radio is never shared — AP is fully down before STA starts.
        if (!g_sta_connect_allowed && g_have_sta_creds &&
            (esp_timer_get_time() - boot_time_us) > AP_WINDOW_US) {
            ESP_LOGI(TAG, "AP window closed — stopping AP and switching to STA");

            ESP_ERROR_CHECK(esp_wifi_stop());
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

            wifi_config_t wc = { 0 };
            strncpy((char *)wc.sta.ssid,     g_cfg.wifi_ssid,     sizeof(wc.sta.ssid) - 1);
            strncpy((char *)wc.sta.password, g_cfg.wifi_password, sizeof(wc.sta.password) - 1);
            wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
            // All-channel scan + strongest-first. Default fast_scan picks
            // the first BSSID it finds matching the SSID, which on mesh /
            // multi-AP networks (e.g. Deco) can be a far node when a much
            // stronger one is in the same room. Costs ~1 s extra at connect.
            wc.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
            wc.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));

            apply_radio_limits_sta();

            g_sta_connect_allowed = true;
            sta_transition_us = esp_timer_get_time();
            ESP_ERROR_CHECK(esp_wifi_start());
            // STA_START event handler will call esp_wifi_connect() because
            // g_sta_connect_allowed is now true.
        }

        // Startup-only watchdog: if STA never obtains its first IP within
        // STA_STARTUP_TIMEOUT_US of the AP→STA switch, reboot to re-enter
        // the AP window. Disarms permanently once n_got_ip > 0 — subsequent
        // disconnects retry STA forever without rebooting.
        if (g_sta_connect_allowed && n_got_ip == 0 && sta_transition_us > 0 &&
            (esp_timer_get_time() - sta_transition_us) > STA_STARTUP_TIMEOUT_US) {
            ESP_LOGW(TAG, "STA failed to obtain first IP within %llds — rebooting",
                     (long long)(STA_STARTUP_TIMEOUT_US / 1000000));
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        }

        // Defer config-save / OTA restart until the TX worker has drained
         // its current job — killing an HTTPS POST mid-handshake would lose
         // a cycle of data and also risks a dirty TLS close. Also suppress
         // starting NEW TX cycles once a restart is pending so we don't
         // enqueue work we'd just cut short.
        if (http_server_restart_requested()) {
            if (tx_is_idle()) {
                ESP_LOGW(TAG, "restart requested — TX idle, rebooting in 2s");
                vTaskDelay(pdMS_TO_TICKS(2000));
                esp_restart();
            }
            static bool defer_logged = false;
            if (!defer_logged) {
                ESP_LOGW(TAG, "restart requested — deferring until TX cycle completes");
                defer_logged = true;
            }
        } else if (xTaskGetTickCount() >= next_tx) {
            do_tx_cycle();
            next_tx = xTaskGetTickCount() + tx_interval;
        }

        log_ftp_loop((uint32_t)(esp_timer_get_time() / 1000));
    }
}
