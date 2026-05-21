#include <string.h>
#include <stdio.h>
#include <inttypes.h>   // PRIu64 for the reconnect counters
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "als.h"
#include "veml7700.h"
#include "applog.h"
#include "coredump.h"
#include "hal.h"
#include "env_sensor.h"
#include "i2c_bus.h"            // V2.3.29: bus lifecycle (replaces env_sensor_get_i2c_bus)
#include "pm_sensor.h"
#include "noise_sensor.h"
#include "config.h"
#include "display.h"
#include "http_server.h"
#include "log_ftp.h"
#include "main_status.h"
#include "mqtt.h"
#include "neopixel.h"
#include "net_arp.h"            // V2.4.19: gratuitous ARP after WiFi reconnect
#include "ntp.h"
#include "periodic.h"           // V2.4.19: 24h housekeeping (PSA refresh + safety-net ARP)
#include "speaker.h"
#include "syslog.h"
#include "transmission.h"
#include "tube.h"
#include "util.h"
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
// V2.4.1 (A9): bit set by main_request_restart() to wake the main loop
// from xEventGroupWaitBits — drops restart latency from up to 1 s
// (the previous polled-flag pattern bounded by the loop's wait cap) to
// the FreeRTOS context-switch time (~µs). Persistent state lives in
// g_restart_requested below since the event bit is consumed by the
// xClearOnExit=pdTRUE wait — see main loop policy.
#define EV_RESTART      BIT2
static volatile bool g_restart_requested = false;

void main_request_restart(void) {
    g_restart_requested = true;
    xEventGroupSetBits(s_events, EV_RESTART);
}

// V2.4.17: sticky flag set by the OTA POST teardown path. When true, the
// main-loop poll skips MQTT and syslog re-init — without this the poll
// re-inits both within ~1 s of mqtt_stop()/syslog_stop(), undoing the
// V2.4.13 OTA teardown during the bulk of the OTA receive loop.
// V2.4.14's FTPS teardown deliberately does NOT set this — that path
// wants MQTT to auto-restart after the upload completes.
static volatile bool g_services_suspended = false;

void main_suspend_services(void) {
    g_services_suspended = true;
}

bool main_services_suspended(void) {
    return g_services_suspended;
}

// --- Soak diagnostics (carried over) ---
static int64_t  t_attempt_start_us = 0;
static int64_t  t_sta_connected_us = 0;
static int64_t  t_last_got_ip_us   = 0;
// Future-V2.4.23: uint32_t → uint64_t. Was uint32_t which wraps at 4.3 B —
// theoretically reachable on a node with marginal WiFi over years. Wrap was
// harmless (display rolls over) but the audit flagged it as "do once". 64 bits
// gives 584 million years of headroom at 1 ns increments; zero perf cost.
// `main_status_t.reconnects` stays uint32_t (consumers don't care about
// post-wrap precision) — truncating cast on assignment.
static uint64_t n_attempts = 0, n_connects = 0, n_got_ip = 0, n_disconnects = 0;
static uint32_t last_disconnect_reason = 0;
static float    last_dhcp_s = 0.0f, last_assoc_s = 0.0f;

// V2.4.19: set in the main loop when EV_GOT_IP fires; cleared after the
// post-reconnect gratuitous ARP is actually sent. Decoupled from the
// event so we can defer the ARP send to a tick when tx_is_idle() — see
// the bottom of the main loop for the consumer and the rationale.
static bool s_arp_after_reconnect_pending = false;

// --- NTP/TX state ---
static bool     ntp_started = false;
// V2.4.13: mqtt-init tracking moved into mqtt.c (mqtt_is_initialized()).
// The old static `mqtt_started` flag here was opaque to mqtt_stop(), so the
// OTA teardown path couldn't signal "please re-init MQTT on next tick" — by
// pulling the source of truth into mqtt.c, mqtt_stop() can flip it directly
// and main.c's poll re-arms within ~1 s on a failed OTA.
static uint32_t tx_cycles   = 0;

// --- Cached last-cycle snapshot (for the status page) ---
//
// All fields written ONLY by the main task in do_tx_cycle() (single writer);
// read lock-free by the HTTP server task. 32-bit scalar fields are
// word-aligned and torn-tolerant on Xtensa LX6/LX7 — no lock needed.
// A momentary inconsistency between cpm/usvph and bme_t/h/p is acceptable
// — they're for at-a-glance status, not for accounting.
//
// V2.4.1 (B1): the 64-bit `g_last_cycle_at` IS NOT torn-tolerant on a
// 32-bit core. A naive int64_t store is two 32-bit stores; a reader on
// another task could see the high half of one write and the low half of
// the next (manifest: momentary year-2038-ish garbage timestamp on the
// status page for a few µs around each TX cycle). Protected by a
// dedicated spinlock for the read/write pair below. The other scalars
// don't need the lock — keeping them out preserves the existing zero-
// cost reads on the hot status-page path.
static uint32_t g_last_dt_ms      = 0;
static uint32_t g_last_counts     = 0;
static uint32_t g_last_cpm        = 0;
static float    g_last_usvph      = 0.0f;
static uint32_t g_last_hv_pulses  = 0;
static bool     g_last_hv_error   = false;
static bool     g_last_bme_valid  = false;   // OR of the three below
static bool     g_last_bme_t_valid = false;   // V2.4.12
static bool     g_last_bme_h_valid = false;   // V2.4.12
static bool     g_last_bme_p_valid = false;   // V2.4.12
static float    g_last_bme_t      = 0.0f;
static float    g_last_bme_h      = 0.0f;
static float    g_last_bme_p      = 0.0f;
static int64_t  g_last_cycle_at   = 0;     // wall-clock unix epoch (0 = never)
static uint32_t g_last_cycle_ms   = 0;     // monotonic uptime ms (0 = never)
static portMUX_TYPE g_last_cycle_at_mux = portMUX_INITIALIZER_UNLOCKED;

// Public snapshot getter — V2.4.1 (A4): replaces 13 individual extern
// functions previously hand-declared at the top of http_server.c and
// display.c. Reader gets every field in one call; the int64_t
// `last_cycle_at` is read under the spinlock to pair with the write in
// `do_tx_cycle` (B1 atomic fix). 32-bit fields are word-aligned and
// torn-tolerant on Xtensa, so they don't take a lock — the snapshot is
// not inter-field consistent (e.g. cpm could be from cycle N and env_t
// from cycle N+1) but each scalar is individually intact.
void main_status_snapshot(main_status_t *out) {
    out->cycles         = tx_cycles;
    out->last_dt_ms     = g_last_dt_ms;
    out->last_cpm       = g_last_cpm;
    out->last_usvph     = g_last_usvph;
    out->last_hv_pulses = g_last_hv_pulses;
    out->last_hv_error  = g_last_hv_error;
    out->have_env       = g_last_bme_valid;
    out->have_env_t     = g_last_bme_t_valid;
    out->have_env_h     = g_last_bme_h_valid;
    out->have_env_p     = g_last_bme_p_valid;
    out->env_t          = g_last_bme_t;
    out->env_h          = g_last_bme_h;
    out->env_p          = g_last_bme_p;
    portENTER_CRITICAL(&g_last_cycle_at_mux);
    out->last_cycle_at  = g_last_cycle_at;
    portEXIT_CRITICAL(&g_last_cycle_at_mux);
    out->last_cycle_ms  = g_last_cycle_ms;
    out->reconnects     = (uint32_t)n_disconnects;   // truncating cast — see counter decl
}

// V2.3.29: per-target enable accessor used by the multi-page display
// task to hide disabled rows from the Uploads page (and skip the page
// entirely when nothing is enabled). Mirrors http_server.c's
// target_enabled() pattern but exposes the g_cfg.send_* flags through
// a single function so display.c / display_serlcd.c don't need a config
// pointer.
bool main_target_enabled(int target_id) {
    switch (target_id) {
        case TX_TARGET_MADAVI:  return g_cfg.send_madavi;
        case TX_TARGET_SENSORC: return g_cfg.send_sensorc;
        case TX_TARGET_RADMON:  return g_cfg.send_radmon;
        case TX_TARGET_OSM:     return g_cfg.send_osm;
        case TX_TARGET_AQI:     return g_cfg.send_aqi;
        default: return false;
    }
}

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

// Drive the onboard SPDT RF switch to route the WiFi front-end to either the
// PCB chip antenna (default) or the u.FL external connector. Called once at
// boot, before esp_wifi_init(); the switch is a passive analog routing
// element, so changing it after WiFi has associated would drop the link.
//
// On boards without HAL_HAS_ANTENNA_SWITCH (e.g. Heltec WiFi Kit 32 V2) this
// function is a no-op — the config flag is force-cleared at the http_server
// level and the UI greys out the checkbox.
static void apply_antenna_routing(void) {
#if HAL_HAS_ANTENNA_SWITCH
    bool external = g_cfg.use_external_antenna;
  #if HAL_ANTENNA_SELECT_VERIFIED
    int level = (external == ANTENNA_SELECT_HIGH_IS_EXTERNAL) ? 1 : 0;
    gpio_reset_pin(PIN_ANTENNA_SELECT);
    gpio_set_direction(PIN_ANTENNA_SELECT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_ANTENNA_SELECT, level);
    ESP_LOGI(TAG, "antenna routing: %s (gpio %d = %d)",
             external ? "EXTERNAL u.FL" : "internal PCB",
             PIN_ANTENNA_SELECT, level);
  #else
    // Pin / polarity not yet verified against board schematic — skip the
    // gpio write so we don't drive an unrelated line. Still log intent so
    // the user can verify the config field is being read correctly.
    ESP_LOGW(TAG, "antenna routing: %s requested, but HAL_ANTENNA_SELECT_VERIFIED=0 — gpio write SKIPPED",
             external ? "EXTERNAL u.FL" : "internal PCB");
  #endif
#endif
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
            ESP_LOGI(TAG, "STA_START, calling connect (attempt #%" PRIu64 ")", n_attempts);
            display_set_status(DSP_STATUS_WIFI, DSP_WIFI_CONNECTING);
            esp_wifi_connect();
        }
        break;
    case WIFI_EVENT_STA_CONNECTED: {
        wifi_event_sta_connected_t *e = (wifi_event_sta_connected_t *)data;
        t_sta_connected_us = esp_timer_get_time();
        last_assoc_s = (t_sta_connected_us - t_attempt_start_us) / 1e6f;
        n_connects++;
        ESP_LOGI(TAG, "STA_CONNECTED #%" PRIu64 ": ch=%d auth=%d bssid=" MACSTR " assoc=%.3fs",
                 n_connects, e->channel, e->authmode,
                 MAC2STR(e->bssid), last_assoc_s);
        break;
    }
    case WIFI_EVENT_STA_DISCONNECTED: {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        last_disconnect_reason = e->reason;
        n_disconnects++;
        ESP_LOGW(TAG, "STA_DISCONNECTED #%" PRIu64 ": reason=%d",
                 n_disconnects, e->reason);
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
    ESP_LOGI(TAG, "GOT_IP #%" PRIu64 ": " IPSTR " gw=" IPSTR
             " dns=" IPSTR " dns2=" IPSTR " dhcp=%.3fs",
             n_got_ip, IP2STR(&e->ip_info.ip),
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
                             bool bme_valid, float bme_t, float bme_h, float bme_p,
                             bool pm_valid, const pm_sample_t *pm,
                             bool noise_valid, const noise_sample_t *noise) {
    memset(ctx, 0, sizeof(*ctx));
    uint32_t cpm = 0;
    if (dt_ms > 0) {
        cpm = (uint32_t)(((uint64_t)counts * 60000ULL) / dt_ms);
    }
    ctx->dt_ms        = dt_ms;
    ctx->gm_counts    = counts;
    ctx->cpm          = cpm;
    ctx->hv_pulses    = hv_pulses;
    ctx->min_micro    = (min_us == UINT32_MAX) ? 0 : min_us;
    ctx->max_micro    = max_us;
    ctx->sw_version   = VERSION_STR;
    ctx->chip_id      = g_chip_id;
    ctx->tube_enabled = g_cfg.tube_enabled;
    ctx->pm_valid     = pm_valid;
    if (pm_valid && pm) ctx->pm = *pm;
    ctx->noise_valid  = noise_valid;
    if (noise_valid && noise) ctx->noise = *noise;

    wifi_ap_record_t ap_rec = { 0 };
    ctx->rssi = (esp_wifi_sta_get_ap_info(&ap_rec) == ESP_OK) ? ap_rec.rssi : -127;

    ctx->bme_valid            = bme_valid;
    ctx->bme_temperature_c    = bme_t;
    ctx->bme_humidity_pct     = bme_h;
    ctx->bme_pressure_pa      = bme_p;
    ctx->station_altitude_m   = g_cfg.station_altitude_m;
    ctx->send_sealevel_pressure = g_cfg.send_sealevel_pressure;

    // V2.4.1 (C9): URLs moved to transmission.c. main.c just passes the
    // per-cycle config flags; the helper fills the URL pair + insecure=false.
    tx_target_configure(&ctx->madavi,  TX_TARGET_MADAVI,  g_cfg.send_madavi,  g_cfg.madavi_https);
    tx_target_configure(&ctx->sensorc, TX_TARGET_SENSORC, g_cfg.send_sensorc, g_cfg.sensorc_https);
    tx_target_configure(&ctx->radmon,  TX_TARGET_RADMON,  g_cfg.send_radmon,  g_cfg.radmon_https);
    ctx->radmon_user     = g_cfg.radmon_user;
    ctx->radmon_password = g_cfg.radmon_password;

    ctx->send_osm         = g_cfg.send_osm;
    ctx->osm_use_insecure = false;
    ctx->osm_box_id       = g_cfg.osm_box_id;
    ctx->osm_access_token = g_cfg.osm_access_token;

    ctx->send_aqi         = g_cfg.send_aqi;
    ctx->aqi_use_insecure = false;
    ctx->aqi_token        = g_cfg.aqi_token;
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

    // Two CYCLE log line shapes — the long one (Geiger active) carries radiation
    // metrics, the short one (tube disabled) keeps just dt + rssi so logs stay
    // readable on PM-only deployments.
    if (g_cfg.tube_enabled) {
        ESP_LOGI(TAG, "CYCLE #%lu: dt=%lums counts=%lu cpm=%lu %.3fµSv/h "
                 "hv_pulses=%lu hv_err=%d min_us=%lu max_us=%lu rssi=%ddBm",
                 (unsigned long)++tx_cycles, (unsigned long)dt_ms,
                 (unsigned long)counts, (unsigned long)cpm, usvph,
                 (unsigned long)hv_pulses, hv_error,
                 (unsigned long)(min_us == UINT32_MAX ? 0 : min_us),
                 (unsigned long)max_us, rssi);
    } else {
        ESP_LOGI(TAG, "CYCLE #%lu: dt=%lums (tube disabled) rssi=%ddBm",
                 (unsigned long)++tx_cycles, (unsigned long)dt_ms, rssi);
    }

    // Cache for /status — see g_last_* declarations + accessors above.
    g_last_dt_ms     = dt_ms;
    g_last_counts    = counts;
    g_last_cpm       = cpm;
    g_last_usvph     = usvph;
    g_last_hv_pulses = hv_pulses;
    g_last_hv_error  = hv_error;
    // V2.4.1 (B1): 64-bit store on a 32-bit core isn't atomic — wrap to
    // pair with the spinlock'd read in main_status_last_cycle_at.
    portENTER_CRITICAL(&g_last_cycle_at_mux);
    g_last_cycle_at  = (int64_t)time(NULL);
    portEXIT_CRITICAL(&g_last_cycle_at_mux);
    g_last_cycle_ms  = (uint32_t)(esp_timer_get_time() / 1000LL);

    // Display: when the tube is disabled the radiation/CPM areas show zero
    // (placeholder — the OLED driver still draws layout). HV status is forced
    // OK because the HV pump isn't running, so there's nothing to fail.
    display_set_status(DSP_STATUS_HV,
                       (!g_cfg.tube_enabled) ? DSP_HV_OK :
                       (hv_error ? DSP_HV_ERROR : DSP_HV_OK));
    int time_sec   = (int)(esp_timer_get_time() / 1000000LL);
    int rad_nsvph  = g_cfg.tube_enabled ? (int)(usvph * 1000.0f) : 0;
    int cpm_disp   = g_cfg.tube_enabled ? (int)cpm : 0;
    // V2.3.29 / V2.4.9: multi-page rotation owns the panel when active.
    // Suppress the radiation-page draw to avoid a briefly-flashed page
    // being overwritten by the next rotation tick. When rotation is OFF
    // (single-page radiation mode), render the radiation page here per
    // TX cycle. Decision was compile-time HAL_MULTIPAGE_ROTATION pre-
    // V2.4.9; now runtime via display_is_multipage().
    if (!display_is_multipage()) {
        display_running(time_sec, rad_nsvph, cpm_disp, g_cfg.show_display);
    } else {
        (void)time_sec; (void)rad_nsvph; (void)cpm_disp;
    }

    float bme_t = 0, bme_h = 0, bme_p = 0;
    bool  bme_valid = false;
    bool  have_t = false, have_h = false, have_p = false;
    if (env_sensor_present()) {
        // V2.3.26: per-sensor raw values logged alongside the fused result so
        // any divergence (flaky SHT45 silently failing → BMP390 fallback fills
        // T but not H, etc.) is immediately visible.
        // V2.4.12: per-field validity (have_t/h/p) so the MQTT publish layer
        // can suppress fields that this cycle didn't actually measure (e.g.
        // SHT45-only setup has no pressure source — pre-V2.4.12 it published
        // env_p=0.0 every cycle and HA showed a 0.00 hPa entity).
        char env_raw[160];
        if (env_sensor_read(&bme_t, &bme_h, &bme_p, &have_t, &have_h, &have_p,
                            env_raw, sizeof(env_raw)) == ESP_OK) {
            bme_valid = true;
            ESP_LOGI(TAG, "%s %s: T=%.2f°C  H=%.2f%%  P=%.2fhPa",
                     env_raw, env_sensor_name(), bme_t, bme_h, bme_p / 100.0f);
            env_sensor_heat_periodic((uint32_t)(esp_timer_get_time() / 1000), bme_h);
        } else {
            ESP_LOGW(TAG, "%s: read failed (%s)", env_sensor_name(), env_raw);
        }
    }

    // Cache for /status. valid mirrors the local — false suppresses the env
    // block on the page until we have at least one good sample.
    g_last_bme_valid = bme_valid;
    g_last_bme_t_valid = bme_valid && have_t;
    g_last_bme_h_valid = bme_valid && have_h;
    g_last_bme_p_valid = bme_valid && have_p;
    if (bme_valid) {
        g_last_bme_t = bme_t;
        g_last_bme_h = bme_h;
        g_last_bme_p = bme_p;
    }

    // Particulate-matter sample — uploaded to Madavi (combined env body) and
    // sensor.community (X-PIN 12 POST) when pm_valid. Driver's data-ready
    // poll keeps this safe even if the sensor missed an internal 1 Hz tick
    // (drops to ESP_FAIL after ~1 s).
    pm_sample_t pm = { 0 };
    bool pm_valid = false;
    if (pm_sensor_present()) {
        if (pm_sensor_read(&pm) == ESP_OK) {
            pm_valid = true;
            ESP_LOGI(TAG,
                     "%s: PM1.0=%.1f PM2.5=%.1f PM4.0=%.1f PM10=%.1f µg/m³  "
                     "NC0.5=%.1f NC1=%.1f NC2.5=%.1f NC4=%.1f NC10=%.1f /cm³  "
                     "typ_size=%.2fµm",
                     pm_sensor_name(),
                     pm.pm1_0, pm.pm2_5, pm.pm4_0, pm.pm10,
                     pm.nc0_5, pm.nc1_0, pm.nc2_5, pm.nc4_0, pm.nc10,
                     pm.typ_size_um);
        } else {
            ESP_LOGW(TAG, "%s: read failed", pm_sensor_name());
        }

        // Refresh device-status bits — fan / laser self-diagnosis. ESP_LOGE
        // for hard faults (fan or laser failed) so they jump out in /log;
        // ESP_LOGW for the soft "fan speed drift" warning. Cleanly-OK
        // status is silent (avoid spamming logs every cycle when fine).
        pm_sensor_status_t st;
        if (pm_sensor_read_status(&st) == ESP_OK) {
            if (st.fan_fail || st.laser_fail) {
                ESP_LOGE(TAG,
                         "%s STATUS FAULT: fan_fail=%d laser_fail=%d "
                         "fan_speed_warn=%d (raw=0x%08lx)",
                         pm_sensor_name(),
                         st.fan_fail, st.laser_fail, st.fan_speed_warn,
                         (unsigned long)st.raw);
            } else if (st.fan_speed_warn) {
                ESP_LOGW(TAG, "%s fan speed warning (raw=0x%08lx)",
                         pm_sensor_name(), (unsigned long)st.raw);
            }
        }
    }

    // Noise sample — DNMS LAeq window finalised at the end of the PREVIOUS
    // cycle by noise_sensor_trigger() (or, on first boot, by the trigger in
    // noise_sensor_init). Read here, then re-trigger below to start the next
    // window. Window length ≈ TX cycle interval (~150 s) — a meaningful LAeq
    // for the upload period, not just for the in-cycle work time.
    noise_sample_t noise = { 0 };
    bool noise_valid = false;
    if (noise_sensor_present()) {
        if (noise_sensor_read(&noise) == ESP_OK) {
            noise_valid = true;
            ESP_LOGI(TAG, "%s: LAeq=%.1fdB(A)  min=%.1fdB(A)  max=%.1fdB(A)",
                     noise_sensor_name(), noise.laeq, noise.la_min, noise.la_max);
        } else {
            ESP_LOGW(TAG, "%s: read failed (or first cycle still integrating)",
                     noise_sensor_name());
        }
    }

    // V2.3.29 / V2.4.9: feed the multi-page display task with this
    // cycle's sensor readings IF rotation is active. The task wakes
    // every 5 s, rotates through Env / PM Mass / PM Number / Uploads /
    // System (skipping pages whose sensors aren't fitted), and renders
    // independently. Dynamic data not in the snapshot (uptime, free
    // heap, TX cycles, upload counters) is read live at render time so
    // the relevant pages update every 5 s, not every 150 s.
    //
    // V2.4.9 made the call runtime-gated (was compile-time
    // HAL_MULTIPAGE_ROTATION). On radiation-only boots
    // display_update_snapshot is a harmless write to a static struct
    // that no reader consumes — but skipping it saves a few cycles per
    // TX loop.
    if (display_is_multipage()) {
        display_snapshot_t snap = {
            .env_valid   = bme_valid,
            .env_t_c     = bme_t,
            .env_h_pct   = bme_h,
            .env_p_pa    = bme_p,
            .pm_valid    = pm_valid,
            .pm          = pm,
            .noise_valid = noise_valid,
            .noise       = noise,
        };
        display_update_snapshot(&snap);
    }

    if (!wifi_up()) {
        ESP_LOGW(TAG, "skipping TX: WiFi down");
        // Still trigger the next DNMS window so we don't lose the cycle's
        // worth of integration time waiting for WiFi.
        if (noise_sensor_present()) noise_sensor_trigger();
        return;
    }
    if (!ntp_time_valid()) {
        ESP_LOGW(TAG, "skipping TX: time not valid (no NTP sync yet)");
        if (noise_sensor_present()) noise_sensor_trigger();
        return;
    }

    // V2.4.12: publish MQTT BEFORE handing off to tx_transmit. Both calls
    // are non-blocking enqueues (mqtt → esp-mqtt task; tx_transmit → CPU1
    // worker), so the two log streams race once both are queued. Pre-V2.4.12
    // the order was tx → mqtt, which meant the "mqtt: publish ok" line
    // landed in the middle of Madavi's TLS handshake log — visually
    // confusing in /log. Calling mqtt_publish_state first gives the LAN
    // broker a head start: the PUBACK typically lands before Madavi's first
    // log line (~1.7 s for cert validation alone), so the boot/cycle trace
    // reads cleanly as:
    //   CYCLE #N → heap → mqtt publish ok → Sending to Madavi → ...
    //
    // Built from the g_last_* cache written at line ~385 via
    // main_status_snapshot() so the JSON includes uptime / cycles /
    // reconnects without re-reading state here.
    {
        main_status_t snap;
        main_status_snapshot(&snap);
        mqtt_publish_state(&snap, pm_valid, &pm, noise_valid, &noise);
    }

    tx_context_t ctx;
    build_tx_context(&ctx, dt_ms, counts, hv_pulses, min_us, max_us,
                     bme_valid, bme_t, bme_h, bme_p,
                     pm_valid, &pm,
                     noise_valid, &noise);
    tx_transmit(&ctx);

    // Start the next LAeq window so the next cycle's read covers the full
    // ~150 s interval. Issue this AFTER tx_transmit (which is non-blocking —
    // it just enqueues the snapshot) so we don't add the I²C round-trip to
    // the user-facing TX latency.
    if (noise_sensor_present()) {
        if (noise_sensor_trigger() != ESP_OK) {
            ESP_LOGW(TAG, "%s: trigger for next window failed", noise_sensor_name());
        }
    }
}

void app_main(void) {
    // Install vprintf hook first so the very first ESP_LOGx below (and
    // everything after — WiFi, HTTP, sensors, TX) lands in the /log buffer.
    applog_init();

    ESP_LOGI(TAG, "%s (IDF %s)", VERSION_STR, esp_get_idf_version());
    // V2.3.16-pre2: explicit board variant in serial log so the variant is
    // unambiguous from the very first line without having to grep cipher/
    // partition lines further down. BOARD_NAME comes from hal.h
    // (one of "heltec_v2", "heltec_v2_4mb", "feathers3_d").
    ESP_LOGI(TAG, "BOARD: %s", BOARD_NAME);

    // V2.4.18: probe the coredump partition. Logs whether a previous
    // panic left a dump and parses the summary for cheap status-page
    // reads. Runs BEFORE WiFi/HTTP/sensors so the boot log line lands
    // in /log + syslog (once syslog is up later) regardless of how the
    // operator inspects the device.
    coredump_init();

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

    // Chip / module identification — distinguishes silicon between board variants.
    // For Heltec WiFi Kit 32 V2: chip.model == ESP32, EmbFlash NO, 8 MB flash
    // (D0WDQ6 + separate flash chip). For UM FeatherS3-D: chip.model == ESP32-S3,
    // EmbFlash YES, EmbPSRAM YES, 16 MB flash. Other combinations indicate either
    // a wrong-board flash or an unknown variant — log loudly and continue.
    {
        esp_chip_info_t chip;
        esp_chip_info(&chip);
        uint32_t flash_size = 0;
        esp_flash_get_size(NULL, &flash_size);
        const char *model =
            (chip.model == CHIP_ESP32)   ? "ESP32"   :
            (chip.model == CHIP_ESP32S2) ? "ESP32-S2":
            (chip.model == CHIP_ESP32S3) ? "ESP32-S3":
            (chip.model == CHIP_ESP32C3) ? "ESP32-C3": "?";
        ESP_LOGI(TAG, "chip: %s rev=v%d.%d cores=%d feat=[%s%s%s%s%s] flash=%luMB",
                 model,
                 chip.revision / 100, chip.revision % 100,
                 chip.cores,
                 (chip.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi "     : "",
                 (chip.features & CHIP_FEATURE_BLE)      ? "BLE "      : "",
                 (chip.features & CHIP_FEATURE_BT)       ? "BT "       : "",
                 (chip.features & CHIP_FEATURE_EMB_FLASH)? "EmbFlash " : "",
                 (chip.features & CHIP_FEATURE_EMB_PSRAM)? "EmbPSRAM " : "",
                 (unsigned long)(flash_size / (1024 * 1024)));
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

    // V2.3.29: dual-bus device probing.
    //
    // i2c_bus.c owns both buses (primary always-on, secondary lazy +
    // sheddable). For each sensor module we try the primary bus first;
    // if no device was found, we ask for the secondary bus (which
    // lazily enables LDO2 on FeatherS3-D, returns NULL on Heltec / QT Py)
    // and probe again. On a hit, mark the secondary bus as kept-alive
    // so i2c_bus_finalize() below doesn't tear it down.
    //
    // Display does its own dual-bus auto-detect inside display_setup()
    // (and calls i2c_bus_secondary_keep_alive() itself if it lands on
    // the secondary). After all init, i2c_bus_finalize() drops LDO2 if
    // nothing — sensor or display — ended up on STEMMA2.
    i2c_master_bus_handle_t bus1 = i2c_bus_get_primary();

    // Helper macro: try a sensor's init on bus 1; if no device bound,
    // try bus 2; if a device was found there, keep the bus alive.
    #define PROBE_ON_BOTH_BUSES(init_fn, present_fn, bus1)                  \
        do {                                                                \
            init_fn(bus1);                                                  \
            if (!present_fn()) {                                            \
                i2c_master_bus_handle_t _b2 = i2c_bus_get_secondary();      \
                if (_b2) {                                                  \
                    init_fn(_b2);                                           \
                    if (present_fn()) i2c_bus_secondary_keep_alive();       \
                }                                                           \
            }                                                               \
        } while (0)

    PROBE_ON_BOTH_BUSES(env_sensor_init,   env_sensor_present,   bus1);
    PROBE_ON_BOTH_BUSES(pm_sensor_init,    pm_sensor_present,    bus1);
    PROBE_ON_BOTH_BUSES(noise_sensor_init, noise_sensor_present, bus1);
    PROBE_ON_BOTH_BUSES(veml7700_init,     veml7700_present,     bus1);

    #undef PROBE_ON_BOTH_BUSES

    // V2.3.29: ALS-PT19 ambient-light sensor (FeatherS3-D only — analog,
    // ADC1_CH3 on GPIO 4). On other boards als_init() is a no-op stub.
    als_init();

    // Display: probes both buses internally, marks bus 2 kept-alive
    // itself if it lands there.
    display_setup(g_cfg.show_display, g_cfg.oled_brightness_pct,
                  (display_mode_t)g_cfg.display_mode);
    display_boot_screen();

    // End-of-init: if the secondary bus was lazily enabled but no
    // consumer (sensor or display) bound to it, drop LDO2 to save the
    // ~5–10 mA quiescent + NeoPixel idle current.
    i2c_bus_finalize();

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

    // Route the WiFi front-end to the desired antenna BEFORE bringing the
    // radio up. The SPDT switch is a passive RF element — flipping it after
    // the link is associated would drop the connection.
    apply_antenna_routing();

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
        safe_strcpy((char *)apc.ap.password, g_cfg.ap_password, sizeof(apc.ap.password));
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
    // When tube_enabled is false the call still configures HV_FET as a static
    // LOW output but skips ISR install + HV gptimer; speaker pulse callback
    // can still be registered (it just never fires without ISR pulses).
    tube_setup(g_cfg.tube_enabled);
    speaker_setup(g_cfg.play_sound, g_cfg.led_tick, g_cfg.speaker_tick);

    // NeoPixel (boards with HAL_HAS_NEOPIXEL only — stubs out elsewhere).
    // Init the pixel after the tube ISR is wired so the pulse-tick callback
    // registration in neopixel_register_pulse_tick() finds a live tube.
    // Both calls are unconditional — neopixel.c handles the no-NeoPixel case
    // with no-op stubs, so no #if HAL_HAS_NEOPIXEL guard needed here.
    neopixel_init();
    if (g_cfg.tube_enabled) {
        neopixel_register_pulse_tick();
    }
    tx_setup();
    http_server_start(&g_cfg, g_chip_id);
    log_ftp_init(g_chip_id, &g_cfg);
    // V2.4.2: MQTT 3.1.1 publish-only client. No-op if disabled / no broker.
    // Has to start AFTER http_server so failures don't block /config access.
    //
    // V2.4.11: deferred start — wait until MQTT can plausibly succeed before
    // calling mqtt_init(). Two preconditions must both hold:
    //   (1) STA has an IP (n_got_ip > 0). The broker lives on the LAN; AP
    //       mode has no route to it. Without this gate, MQTT spams ~5×
    //       ESP-TLS "select() timeout" errors per 25 s for the whole 2-min
    //       AP boot window.
    //   (2) Wall clock is sane (ntp_time_valid()). TLS cert NotBefore/After
    //       validation fails with a 1970 clock.
    //
    // V2.4.12 fix: the original V2.4.11 gated only on (2). ESP32's RTC
    // survives soft reboots (OTA, watchdog, esp_restart), so after the first
    // cold boot the clock is already past 2025 and ntp_time_valid() returns
    // true immediately — defeating the gate. Adding (1) fixes both reboot
    // modes: cold boot waits for SNTP, soft reboot waits for STA GOT_IP.
    //
    // Exception: if MQTT is disabled or broker is empty, still call mqtt_init
    // now so the "disabled" log line appears in the boot trace — same boot
    // diagnostics behaviour as before. mqtt_is_initialized() prevents the
    // loop from calling it a second time (sticky flag inside mqtt.c).
    if (!g_cfg.mqtt_enable || g_cfg.mqtt_broker[0] == 0) {
        mqtt_init(&g_cfg, g_chip_id);  // logs "disabled (...)" + returns
    } else {
        ESP_LOGI(TAG, "MQTT deferred until STA has IP + NTP synced (broker=%s:%lu)",
                 g_cfg.mqtt_broker, (unsigned long)g_cfg.mqtt_port);
    }

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
        // Cap wait so post-loop polls (NTP, AP window, STA watchdog, FTP)
        // run at least once a second. The restart flag is now event-driven
        // (V2.4.1 A9 — EV_RESTART wakes us within µs) but the other periodic
        // checks still need the 1 s tick.
        if (wait > pdMS_TO_TICKS(1000)) wait = pdMS_TO_TICKS(1000);
        EventBits_t bits = xEventGroupWaitBits(
            s_events,
            EV_GOT_IP | EV_DISCONNECTED | EV_RESTART,
            pdTRUE,    // clear matched bits — EV_RESTART persistence lives in g_restart_requested
            pdFALSE,
            wait);

        if (bits & EV_GOT_IP) {
            if (!ntp_started) {
                ntp_setup(g_cfg.ntp_server, g_cfg.ntp_server2, g_cfg.ntp_server3,
                          g_cfg.tz_posix);
                ntp_started = true;
            }
            // V2.4.19: queue a gratuitous ARP for the next idle tick so the
            // upstream AP / mesh bridge learns this sensor's MAC on the
            // fresh association. The actual send is deferred — see the
            // consumer at the bottom of this loop for the why.
            s_arp_after_reconnect_pending = true;
        }
        if (bits & EV_DISCONNECTED) {
            if (!g_sta_connect_allowed) {
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            mark_attempt();
            ESP_LOGI(TAG, "retry connect (attempt #%" PRIu64 ")", n_attempts);
            esp_wifi_connect();
            continue;
        }

        ntp_poll();

        // V2.4.12: start MQTT only once both preconditions hold (see boot
        // section comment above). n_got_ip>0 means STA has reached the LAN
        // at least once (sticky — only ever increments); ntp_time_valid()
        // means the wall clock is past 2025-01-01 (sane for TLS cert
        // validation, whether from fresh SNTP or RTC carryover).
        // V2.4.13: source of truth moved to mqtt_is_initialized() so the
        // /update OTA-teardown path can flip it back to false via mqtt_stop()
        // and this poll will re-init MQTT on the next tick after a failed
        // OTA. On a successful OTA the device reboots, so the re-init never
        // runs — but the failure path is the one we care about for recovery.
        // V2.4.17: skip re-init if the OTA teardown set the suspended flag.
        // Without this, an OTA in progress would see MQTT/syslog re-init
        // ~1 s after the teardown, defeating the V2.4.13 heap-freeing intent.
        if (!mqtt_is_initialized() && !main_services_suspended() &&
            n_got_ip > 0 && ntp_time_valid()) {
            ESP_LOGI(TAG, "STA has IP + clock sane — starting MQTT client");
            mqtt_init(&g_cfg, g_chip_id);
        }

        // V2.4.15: bring up syslog UDP client once STA has IP. No NTP gate
        // (we don't need a synced clock — rsyslog uses receive time as a
        // fallback when our timestamp is missing/wrong, and a sane device
        // clock is best-effort polish in the message). syslog_init is a
        // no-op when syslog_enable=false or syslog_host is empty.
        // syslog_is_initialized() flips true only on successful socket
        // open, so the poll naturally retries if the first attempt failed
        // (e.g. DNS not ready) on subsequent ticks.
        // V2.4.17: same suspension gate as MQTT above — OTA teardown stays
        // sticky until reboot.
        if (g_cfg.syslog_enable && g_cfg.syslog_host[0] &&
            !syslog_is_initialized() && !main_services_suspended() &&
            n_got_ip > 0) {
            ESP_LOGI(TAG, "STA has IP — starting syslog UDP client");
            syslog_init(g_cfg.syslog_host,
                        (uint16_t)g_cfg.syslog_port,
                        g_cfg.wifi_hostname);
        }

        // End of boot AP window: stop the AP and switch to STA-only.
        // Radio is never shared — AP is fully down before STA starts.
        if (!g_sta_connect_allowed && g_have_sta_creds &&
            (esp_timer_get_time() - boot_time_us) > AP_WINDOW_US) {
            ESP_LOGI(TAG, "AP window closed — stopping AP and switching to STA");

            ESP_ERROR_CHECK(esp_wifi_stop());
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

            wifi_config_t wc = { 0 };
            safe_strcpy((char *)wc.sta.ssid,     g_cfg.wifi_ssid,     sizeof(wc.sta.ssid));
            safe_strcpy((char *)wc.sta.password, g_cfg.wifi_password, sizeof(wc.sta.password));
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
        // enqueue work we'd just cut short. V2.4.1 (A9): g_restart_requested
        // is set by main_request_restart() (was http_server's static flag);
        // EV_RESTART wakes us immediately when set.
        if (g_restart_requested) {
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

        {
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            periodic_loop(now_ms);       // V2.4.19: 24h PSA refresh + safety-net ARP
            log_ftp_loop(now_ms);
        }

        // V2.4.19: deferred gratuitous ARP after each WiFi reconnect.
        //
        // The `s_arp_after_reconnect_pending` flag is set in the
        // `bits & EV_GOT_IP` block above on every GOT_IP event. We fire
        // the ARP from here (rather than inline at the event) for two
        // reasons:
        //   1. tx_is_idle() gating: the TX worker on CPU1 may be mid-
        //      handshake with Madavi/sensor.community; deferring lets
        //      us coalesce with the worker's natural idle window.
        //   2. Settling time: lwIP itself emits one gratuitous ARP on
        //      GOT_IP. Firing ours on a later tick (1 s typical) gives
        //      the upstream AP a second, well-separated chance to learn
        //      the path — more useful than two back-to-back ARPs that
        //      could both be lost in the same airtime collision.
        //
        // FTP gating is structural — log_ftp_loop runs on this main
        // task; if FTP were uploading, this line wouldn't have been
        // reached.
        //
        // The 24h safety-net ARP (for sensors that go days without a
        // reconnect) is piggy-backed on the PSA crypto refresh in
        // log_ftp.c — same TX-idle gate, no extra timer.
        if (s_arp_after_reconnect_pending && wifi_up() && tx_is_idle()) {
            net_arp_send_gratuitous();
            s_arp_after_reconnect_pending = false;
        }
    }
}
