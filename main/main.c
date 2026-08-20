#include <string.h>
#include <stdio.h>
#include <inttypes.h>   // PRIu32/PRIu64 for counter format macros
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
#include "fuel_gauge.h"   // V2.6.6: MAX17048 battery fuel gauge (FeatherS3-D)
#include "gnss.h"               // V2.5.8: I²C GNSS (PA1010D / MAX-M10S)
#include "applog.h"
#include "coredump.h"
#include "hal.h"
#include "env_sensor.h"
#include "i2c_bus.h"            // V2.3.29: bus lifecycle (replaces env_sensor_get_i2c_bus)
#include "pm_sensor.h"
#include "noise_sensor.h"
#include "sgp41.h"
#include "config.h"
#include "diag.h"               // V2.4.28: I²C sensor-read-error counter
#include "display.h"
#include "http_server.h"
#include "log_ftp.h"
#include "lorawan.h"            // V2.6.23: SX1262 LoRaWAN uplink (Heltec V4-R2 only)
#include "main_status.h"
#include "mqtt.h"
#include "neopixel.h"
#include "led.h"
#include "net_arp.h"            // V2.4.19: gratuitous ARP after WiFi reconnect
#include "ntp.h"
#include "sysinfo.h"   // chip_model_str (shared model-string ladder)
#include "periodic.h"           // V2.4.19: 24h housekeeping (PSA refresh + safety-net ARP)
#include "history.h"            // V2.5.6: in-RAM CPM history + rolling averages
#include "sd_card.h"            // V2.6.19: standalone mode — microSD FAT32 mount
#include "sd_logger.h"          // V2.6.19: standalone mode — per-cycle CSV logger
#include "speaker.h"
#include "syslog.h"
#include "telemetry.h"          // V2.6.19: standalone mode — neutral column registry
#include "transmission.h"
#include "tube.h"
#include "tube_logic.h"   // V2.6.31: pcnt_blank_wide (width-aware blank subtract)
#include "tube_pcnt.h"   // V2.5.16: optional PCNT pulse-width comb diagnostic
#include "util.h"
#include "version.h"

static const char *TAG = "v2_main";

// Runtime configuration — loaded from NVS at boot with compile-time defaults
// as fallback (see config.c). Editable via the /config endpoint.
// V2.6.23: no longer `static` — lorawan.cpp reads this by the `extern
// config_t g_cfg;` declaration in config.h (see that comment for why; every
// other module instead gets a pointer/values handed to it explicitly).
config_t g_cfg;

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

// V2.4.24: OTA-in-progress flag. Non-sticky (unlike g_services_suspended).
// Set on entry to update_post, cleared on every return path. Read by the
// main loop's TX-cycle scheduler to skip do_tx_cycle while an OTA upload
// is consuming the WiFi link.
static volatile bool g_ota_in_progress = false;

void main_ota_begin(void) {
    g_ota_in_progress = true;
}

void main_ota_end(void) {
    g_ota_in_progress = false;
}

bool main_ota_in_progress(void) {
    return g_ota_in_progress;
}

// --- Soak diagnostics (carried over) ---
// t_attempt_start_us is written by mark_attempt() from BOTH the main task
// (retry/roaming-safety-net paths) and the WiFi event task (STA_START), then
// read back on the event task in on_wifi_event to compute last_assoc_s — a
// genuine cross-task int64_t (two-instruction store on 32-bit Xtensa, torn-
// read hazard). Guarded with its own spinlock rather than demoted to 32-bit
// (as n_attempts/sync_tv_sec were) because the consumer needs sub-second
// precision; follows the g_last_cycle_at_mux precedent below.
static int64_t  t_attempt_start_us = 0;
static portMUX_TYPE t_attempt_start_mux = portMUX_INITIALIZER_UNLOCKED;
// t_sta_connected_us: written and read only within on_wifi_event/on_ip_event,
// both on the single default-event-loop task — never touched by the main
// task, so no cross-task hazard and no lock needed.
static int64_t  t_sta_connected_us = 0;
// uint32_t — written on the WiFi event task, read on the main task; on 32-bit
// Xtensa a uint64_t store is two instructions (torn-read hazard). Wraps after
// 4.3 billion events (~136 years at 1/s) — sufficient for display use.
static uint32_t n_attempts = 0, n_connects = 0, n_got_ip = 0;
// uint32_t — written on the WiFi event task, read on the TX task; on 32-bit
// Xtensa a uint64_t store is two instructions (torn-read hazard). Wraps after
// 4.3 billion disconnects (~136 years at 1/s) — sufficient for display use.
static uint32_t n_disconnects = 0;
static uint32_t last_disconnect_reason = 0;
static float    last_dhcp_s = 0.0f, last_assoc_s = 0.0f;

// V2.4.19: set in the main loop when EV_GOT_IP fires; cleared after the
// post-reconnect gratuitous ARP is actually sent. Decoupled from the
// event so we can defer the ARP send to a tick when tx_is_idle() — see
// the bottom of the main loop for the consumer and the rationale.
static bool s_arp_after_reconnect_pending = false;

// V2.6.19 (final review A2): boot-time latch of the two standalone-SD
// checkboxes. /config's plain "Save" commits g_cfg live (only "Save and
// restart" reboots), but g_cfg.standalone_sd/standalone_ap_on are BOTH
// starred reboot-required in the UI — the mode's arming (sd_logger_init(),
// gnss_set_clock_source(), the boot SD mount) only ever runs once, at boot.
// Reading g_cfg.standalone_sd live in do_tx_cycle()/the radio-off branch let
// a plain Save on a running networked node kill the radio (esp_wifi_stop+
// deinit fires within ~1 s) while the GPS clock source was never armed, so
// sd_logger_cycle() waits at its clock-sync gate forever: radio dead, zero
// rows ever logged, until a physical power-cycle. Latching both flags here
// and using ONLY the latch below (never g_cfg.standalone_sd/standalone_ap_on
// directly) makes a live Save a pure no-op until reboot, matching the
// starred-field contract the /config result page already promises.
//
// V2.6.23: s_standalone_ap_on_latched is now honored in ALL modes, not just
// alongside standalone-SD (was: the AP-window-close branch only consulted
// it when s_standalone_sd_latched was also true, so a networked node with
// STA creds could never keep the AP up). See the AP-window-close block
// below — this latch now short-circuits both the standalone-SD radio-off
// branch AND the switch-to-STA branch, so any node (SD-standalone,
// LoRaWAN-standalone, or a plain networked node the user wants permanently
// reachable at 192.168.4.1) can hold the AP forever.
static bool s_standalone_sd_latched    = false;
static bool s_standalone_ap_on_latched = false;

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
static uint32_t g_last_cpm        = 0;
static float    g_last_usvph      = 0.0f;
// V2.5.16: PCNT width-filter state for /status — whether the last cycle's CPM
// was width-filtered, the pre-filter CPM, and the active filter width.
static bool     g_last_filtering    = false;
static uint32_t g_last_cpm_raw      = 0;
static uint32_t g_last_filter_width_ns = 0;
static uint32_t g_last_hv_pulses        = 0;   // cumulative since boot (MQTT)
static uint32_t g_last_hv_pulses_delta  = 0;   // V2.4.27: per-cycle delta (status + legacy HTTPS)
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
    out->last_hv_pulses       = g_last_hv_pulses;
    out->last_hv_pulses_delta = g_last_hv_pulses_delta;
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
    out->i2c_errors     = diag_i2c_errors();         // V2.4.28
    out->pcnt_filtering = g_last_filtering;          // V2.5.16
    out->last_cpm_raw   = g_last_cpm_raw;
    out->pcnt_filter_width_ns = g_last_filter_width_ns;
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
        case TX_TARGET_OSM_STAGING: return g_cfg.send_osm_staging;
        case TX_TARGET_AQI:     return g_cfg.send_aqi;
        default: return false;
    }
}

// --- Strict single-mode WiFi:
//   boot .. AP_WINDOW_US:     AP only (STA not started — radio unshared)
//   AP_WINDOW_US onward:       STA only (AP stopped — no fallback AP), UNLESS
//                              a client is still connected to the AP, in which
//                              case the switch is held until AP_CLIENT_GRACE_US
//                              after the last client leaves (see below).
// If STA fails to obtain its first IP within STA_STARTUP_TIMEOUT_US of the
// switch, reboot to re-enter the AP window. Watchdog disarms permanently
// after the first GOT_IP — subsequent disconnects retry STA forever.
#define AP_WINDOW_US           (120 * 1000000LL)
// V2.6.23: once the AP window elapses, don't close the AP while a client is
// still associated — hold it, then close this long after the LAST client
// leaves (so a browser that briefly drops between page loads doesn't trip an
// instant close). Deliberately short: the AP is only meant for a config
// session, and holding it defers the STA switch / telemetry.
#define AP_CLIENT_GRACE_US     (5 * 1000000LL)
#define STA_STARTUP_TIMEOUT_US (600 * 1000000LL)   // 10 min
#if CONFIG_ESP_WIFI_ENABLE_ROAMING_APP
// V2.5.34 safety-net (PSRAM/roaming builds only): post-association we delegate
// reconnects to the experimental roaming app. If it ever stalls, force our own
// esp_wifi_connect() after this long with no IP. Deliberately long (minutes,
// not seconds) so the app is left to fully own ordinary roams/reconnects —
// a router reboot can keep it retrying for 1-2 min before it succeeds on its
// own (observed 2026-07-01), and firing this mid-retry only adds a spurious
// disconnect/reconnect. This is a last-resort backstop for a genuine stall,
// not a speed guarantee.
#define ROAM_RECONNECT_SAFETY_NET_US (300 * 1000000LL)   // 5 min
#endif
static int64_t  boot_time_us       = 0;
static int64_t  sta_transition_us  = 0;
static bool     g_have_sta_creds   = false;
static volatile bool g_sta_connect_allowed = false;

static void mark_attempt(void) {
    portENTER_CRITICAL(&t_attempt_start_mux);
    t_attempt_start_us = esp_timer_get_time();
    portEXIT_CRITICAL(&t_attempt_start_mux);
    n_attempts++;
}

// Maximum WIFI_BW* legal for a given protocol bitmap, per esp_wifi.h's
// esp_wifi_set_bandwidth()/set_bandwidths() @attention 1+2: WIFI_BW40 needs
// WIFI_PROTOCOL_11N in the mask and neither WIFI_PROTOCOL_11AC nor
// WIFI_PROTOCOL_11AX — both setters return ESP_ERR_INVALID_ARG otherwise.
static wifi_bandwidth_t max_legal_bw(uint8_t proto_mask) {
    if ((proto_mask & WIFI_PROTOCOL_11N) &&
        !(proto_mask & (WIFI_PROTOCOL_11AC | WIFI_PROTOCOL_11AX))) {
        return WIFI_BW40;
    }
    return WIFI_BW20;
}

// Apply user-configured radio capability limits to the STA interface.
// Called right before esp_wifi_start() when the AP→STA switch happens;
// the APIs take effect on the next association, so ordering matters.
// 11b/g-only disables 802.11n; HT20-only caps channel bandwidth at 20 MHz.
//
// V2.6.17: dual-band chips (ESP32-C5) default to WIFI_BAND_MODE_AUTO, under
// which the single-band esp_wifi_set_protocol()/set_bandwidth() calls below
// are documented to return ESP_ERR_NOT_SUPPORTED — confirmed live on a C5
// board (both calls failed, silently leaving the flags un-applied). The
// plural per-band esp_wifi_set_protocols()/set_bandwidths() are required
// instead in that mode. 11bg_only/ht20_only are 2.4 GHz-only concepts (5 GHz
// has no 11b/g), so the 5 GHz side is always given the fullest protocol set.
//
// On CONFIG_SOC_WIFI_HE_SUPPORT chips (the C5 today), IDF's own unrestricted
// 2.4 GHz default already includes WIFI_PROTOCOL_11AX (esp_wifi.h's
// esp_wifi_set_protocol() doc comment). Because the singular call used to
// fail outright here, that AX-inclusive default was never actually
// overridden — every C5 has been running on it, unmonitored, since its
// board port shipped. Gating 11AX on this capability macro (rather than
// leaving it out, as the pre-existing 2.4GHz-only boards' value does)
// preserves that already-proven default instead of silently downgrading
// HE-capable chips to HT (802.11n) the first time this code path actually
// takes effect.
//
// bw.ghz_2g is NOT a bare ht20_only ternary: confirmed live on a C5 that
// requesting WIFI_BW40 against a mask carrying WIFI_PROTOCOL_11AX fails the
// same way a bg-only (no-11N) mask does — set_bandwidths(2g=WIFI_BW40,
// 5g=WIFI_BW20) was rejected outright with ESP_ERR_INVALID_ARG, leaving
// BOTH bands' bandwidth unset. max_legal_bw() derives the ceiling from the
// same protocol mask just requested above, so it can never ask for a value
// the driver will refuse for THIS mask, on THIS or any future HE-capable
// board — ht20_only still narrows further when checked. On an HE chip with
// it unchecked, 2.4 GHz is capped at 20 MHz anyway (11AX is in the mask),
// making the checkbox a no-op there; disclosed via a /config caveat
// (http_server.c) rather than silently promising 40 MHz. 5 GHz's bandwidth
// now goes through the same helper instead of a hardcoded WIFI_BW20 — it
// still always resolves to BW20 (its protocol set always carries 11AC/
// 11AX), but the code now derives that instead of just asserting it.
static void apply_radio_limits_sta(void) {
    wifi_band_mode_t band_mode = WIFI_BAND_MODE_2G_ONLY;
    esp_wifi_get_band_mode(&band_mode);

    if (band_mode == WIFI_BAND_MODE_AUTO) {
        wifi_protocols_t proto = { 0 };
        proto.ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G;
        if (!g_cfg.wifi_11bg_only) {
            proto.ghz_2g |= WIFI_PROTOCOL_11N;
#if CONFIG_SOC_WIFI_HE_SUPPORT
            proto.ghz_2g |= WIFI_PROTOCOL_11AX;
#endif
        }
        proto.ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N |
                        WIFI_PROTOCOL_11AC | WIFI_PROTOCOL_11AX;
        esp_err_t r = esp_wifi_set_protocols(WIFI_IF_STA, &proto);
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "set_protocols(2g=0x%02x, 5g=0x%02x) failed: %s",
                     proto.ghz_2g, proto.ghz_5g, esp_err_to_name(r));
        } else {
            ESP_LOGI(TAG, "STA protocols = 2g:0x%02x 5g:0x%02x (11bg_only=%d)",
                     proto.ghz_2g, proto.ghz_5g, g_cfg.wifi_11bg_only);
        }

        wifi_bandwidths_t bw = { 0 };
        bw.ghz_2g = g_cfg.wifi_ht20_only ? WIFI_BW20 : max_legal_bw(proto.ghz_2g);
        bw.ghz_5g = max_legal_bw(proto.ghz_5g);
        r = esp_wifi_set_bandwidths(WIFI_IF_STA, &bw);
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "set_bandwidths(2g=%d, 5g=%d) failed: %s",
                     bw.ghz_2g, bw.ghz_5g, esp_err_to_name(r));
        } else {
            ESP_LOGI(TAG, "STA bandwidths = 2g:%s 5g:20MHz (ht20_only=%d)",
                     (bw.ghz_2g == WIFI_BW20) ? "20MHz" : "40MHz", g_cfg.wifi_ht20_only);
        }
        return;
    }

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
    // V2.6.22: ESP_ERROR_CHECK — can only fail on an invalid/input-only
    // pin constant, i.e. a board-port mistake; abort loudly at first boot
    // (see the rail-gate blocks in i2c_bus.c for the full rationale).
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_ANTENNA_SELECT));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_ANTENNA_SELECT, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(PIN_ANTENNA_SELECT, level));
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
            ESP_LOGI(TAG, "STA_START, calling connect (attempt #%" PRIu32 ")", n_attempts);
            display_set_status(DSP_STATUS_WIFI, DSP_WIFI_CONNECTING);
            esp_wifi_connect();
        }
        break;
    case WIFI_EVENT_STA_CONNECTED: {
        wifi_event_sta_connected_t *e = (wifi_event_sta_connected_t *)data;
        t_sta_connected_us = esp_timer_get_time();
        portENTER_CRITICAL(&t_attempt_start_mux);
        int64_t attempt_start_us = t_attempt_start_us;
        portEXIT_CRITICAL(&t_attempt_start_mux);
        last_assoc_s = (t_sta_connected_us - attempt_start_us) / 1e6f;
        n_connects++;
        ESP_LOGI(TAG, "STA_CONNECTED #%" PRIu32 ": ch=%d auth=%d bssid=" MACSTR " assoc=%.3fs",
                 n_connects, e->channel, e->authmode,
                 MAC2STR(e->bssid), last_assoc_s);
        break;
    }
    case WIFI_EVENT_STA_DISCONNECTED: {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        last_disconnect_reason = e->reason;
        n_disconnects++;
        ESP_LOGW(TAG, "STA_DISCONNECTED #%" PRIu32 ": reason=%d",
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
    n_got_ip++;
    esp_netif_dns_info_t d1 = { 0 }, d2 = { 0 };
    esp_netif_get_dns_info(e->esp_netif, ESP_NETIF_DNS_MAIN,   &d1);
    esp_netif_get_dns_info(e->esp_netif, ESP_NETIF_DNS_BACKUP, &d2);
    ESP_LOGI(TAG, "GOT_IP #%" PRIu32 ": " IPSTR " gw=" IPSTR
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
    // V2.5.6: rolling 5-/15-min CPM means from history.c (GMC ACPM + ThingSpeak
    // field3/4). Ramp-up: before the first minute sample, fall back to the
    // current per-cycle cpm so uploads never send 0.
    {
        history_snapshot_t h;
        history_get(&h);
        ctx->roll_valid = (h.min_count > 0);
        ctx->cpm5  = ctx->roll_valid ? h.cpm5  : cpm;
        ctx->cpm15 = ctx->roll_valid ? h.cpm15 : cpm;
    }
    ctx->hv_pulses    = hv_pulses;
    ctx->min_micro    = (min_us == UINT32_MAX) ? 0 : min_us;
    ctx->max_micro    = max_us;
    ctx->sw_version   = VERSION_STR;
    ctx->chip_id      = g_chip_id;
    ctx->tube_enabled = g_cfg.tube_enabled;
    ctx->tube_type    = g_cfg.tube_type;   // V2.6.1: drives the dose conversion factor
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

    // V2.5.18: heap-guard floor travels with the per-cycle snapshot so the
    // fragmentation auto-reboot can be evaluated on the TX worker right where
    // the per-cycle heap line is logged (relocated from periodic_loop).
    ctx->heap_guard_floor_kb = g_cfg.heap_guard_floor_kb;
    ctx->heap_guard_confirm_cycles = g_cfg.heap_guard_confirm_cycles;  // V2.5.33

    // V2.4.1 (C9): URLs moved to transmission.c. main.c just passes the
    // per-cycle config flags; the helper fills the URL pair + insecure=false.
    tx_target_configure(&ctx->madavi,  TX_TARGET_MADAVI,  g_cfg.send_madavi,  g_cfg.madavi_https);
    tx_target_configure(&ctx->sensorc, TX_TARGET_SENSORC, g_cfg.send_sensorc, g_cfg.sensorc_https);
    tx_target_configure(&ctx->radmon,  TX_TARGET_RADMON,  g_cfg.send_radmon,  g_cfg.radmon_https);
    // V2.5.20 (review R8): credentials copied BY VALUE — see tx_context_t.
    safe_strcpy(ctx->radmon_user,     g_cfg.radmon_user,     sizeof(ctx->radmon_user));
    safe_strcpy(ctx->radmon_password, g_cfg.radmon_password, sizeof(ctx->radmon_password));

    // V2.5.5: OSM + aqi.eco unified to tx_target_t (were bare send_*/use_insecure
    // bools). HTTPS-only; the per-box/per-token URL is built dynamically in
    // send_osm()/send_aqi(), so url_* come out NULL via the s_target_urls
    // {NULL,NULL} rows. tx_target_configure sets use_insecure=false, matching
    // the prior hardcoded literal.
    tx_target_configure(&ctx->osm, TX_TARGET_OSM, g_cfg.send_osm, /*use_https=*/true);
    safe_strcpy(ctx->osm_box_id,       g_cfg.osm_box_id,       sizeof(ctx->osm_box_id));
    safe_strcpy(ctx->osm_access_token, g_cfg.osm_access_token, sizeof(ctx->osm_access_token));

    // V2.5.26: openSenseMap STAGING — HTTPS-only beta target, independent creds.
    tx_target_configure(&ctx->osm_staging, TX_TARGET_OSM_STAGING,
                        g_cfg.send_osm_staging, /*use_https=*/true);
    safe_strcpy(ctx->osm_staging_box_id, g_cfg.osm_staging_box_id,
                sizeof(ctx->osm_staging_box_id));
    safe_strcpy(ctx->osm_staging_token,  g_cfg.osm_staging_token,
                sizeof(ctx->osm_staging_token));

    tx_target_configure(&ctx->aqi, TX_TARGET_AQI, g_cfg.send_aqi, /*use_https=*/true);
    safe_strcpy(ctx->aqi_token, g_cfg.aqi_token, sizeof(ctx->aqi_token));

    // V2.5.1: GMCMap (HTTP-only) + ThingSpeak (HTTPS-capable).
    tx_target_configure(&ctx->gmc, TX_TARGET_GMC, g_cfg.send_gmc, false);
    safe_strcpy(ctx->gmc_account_id, g_cfg.gmc_account_id, sizeof(ctx->gmc_account_id));
    safe_strcpy(ctx->gmc_geiger_id,  g_cfg.gmc_geiger_id,  sizeof(ctx->gmc_geiger_id));

    tx_target_configure(&ctx->thingspeak, TX_TARGET_THINGSPEAK,
                        g_cfg.send_thingspeak, g_cfg.thingspeak_https);
    safe_strcpy(ctx->thingspeak_api_key, g_cfg.thingspeak_api_key,
                sizeof(ctx->thingspeak_api_key));

    // V2.5.4: ThingSpeak PM — second, independent channel for the SPS30.
    tx_target_configure(&ctx->thingspeak_pm, TX_TARGET_THINGSPEAK_PM,
                        g_cfg.send_thingspeak_pm, g_cfg.thingspeak_pm_https);
    safe_strcpy(ctx->thingspeak_pm_api_key, g_cfg.thingspeak_pm_api_key,
                sizeof(ctx->thingspeak_pm_api_key));
}

// V2.6.19: per-cycle tube snapshot consumed by the telemetry read callbacks
// (standalone CSV columns). Written once per cycle by do_tx_cycle on the
// main service task; read by sd_logger_cycle in the SAME call frame — no
// concurrency. "Tube " header prefix keeps the sorted CSV grouped (spec §5).
typedef struct {
    bool     valid;
    uint32_t cpm;
    float    usvph;
    uint32_t counts;
    uint32_t hv_pulses;
    uint32_t dt_ms;
} tube_tm_cache_t;

static tube_tm_cache_t s_tube_tm;

static bool tm_tube_cpm(char *cell, size_t cap, void *arg) {
    (void)arg;
    if (!s_tube_tm.valid) return false;
    snprintf(cell, cap, "%lu", (unsigned long)s_tube_tm.cpm);
    return true;
}
static bool tm_tube_dose(char *cell, size_t cap, void *arg) {
    (void)arg;
    if (!s_tube_tm.valid) return false;
    snprintf(cell, cap, "%.4f", (double)s_tube_tm.usvph);
    return true;
}
static bool tm_tube_counts(char *cell, size_t cap, void *arg) {
    (void)arg;
    if (!s_tube_tm.valid) return false;
    snprintf(cell, cap, "%lu", (unsigned long)s_tube_tm.counts);
    return true;
}
static bool tm_tube_hv(char *cell, size_t cap, void *arg) {
    (void)arg;
    if (!s_tube_tm.valid) return false;
    snprintf(cell, cap, "%lu", (unsigned long)s_tube_tm.hv_pulses);
    return true;
}
static bool tm_tube_window(char *cell, size_t cap, void *arg) {
    (void)arg;
    if (!s_tube_tm.valid) return false;
    snprintf(cell, cap, "%lu", (unsigned long)s_tube_tm.dt_ms);
    return true;
}

static void do_tx_cycle(void) {
    uint32_t counts_raw, dt_ms, min_us, max_us, hv_pulses;
    bool hv_error;
    tube_read(&counts_raw, &dt_ms, &min_us, &max_us, &hv_pulses, &hv_error);

    // V2.5.12: capture the raw-edge profile right next to tube_read so the
    // count window and raw-edge window align (so raw_edges >= counts). Dumped
    // in the DIAG log line below when the tube is enabled.
    uint32_t diag_raw_edges = 0;
    uint32_t diag_guard_removed = 0;   // V2.5.30: edges dropped by the dead-time guard
    uint32_t diag_hv_coincident = 0;   // V2.6.9: counted edges landing in the HV-pulse coincidence window
    uint32_t diag_hv_blanked               = 0;   // V2.6.29: would-be counts dropped by the HV blanking window
    uint32_t diag_hist[TUBE_DIAG_NBUCKETS] = {0};
    tube_get_diag(&diag_raw_edges, &diag_guard_removed, &diag_hv_coincident,
                  &diag_hv_blanked, diag_hist);

    // V2.5.16: snapshot the parallel PCNT width-comb ONCE here (the read
    // advances each unit's per-cycle delta base, so re-reading would zero the
    // second consumer's delta) so the same snapshot drives BOTH the optional
    // width FILTER and the PCNT diag log line below. pc[NWIDTHS-1] = count
    // surviving the widest (configured filter width, default 4 µs) glitch filter.
    uint32_t pc[TUBE_PCNT_NWIDTHS] = {0};
    bool pcnt_on = tube_pcnt_active();
    if (pcnt_on) tube_pcnt_read(pc);

    // V2.5.16: when pcnt_filter is on, the authoritative count for this cycle is
    // the width-filtered PCNT count — it drops the 1-4 µs marginal-pulse
    // population behind the Feather/Heltec gap (see reference_radiation_data_
    // analysis). counts_raw (the ISR dead-time-gated count) is kept as the
    // pre-filter reference and logged on the FILTER line. NOTE the PCNT path
    // applies a WIDTH gate only — it has no dead-time/spacing gate. On THIS
    // hardware the sub-190 µs ringing the ISR rejects is <250 ns wide (measured,
    // 2026-06-08), so the 4 µs width filter is EXPECTED to drop it too — but
    // that's the thing under test, not a guarantee: a genuine >4 µs-wide pulse
    // arriving inside the 190 µs window would be counted by PCNT and not by the
    // ISR (so the filtered count can occasionally exceed counts_raw — the
    // `removed`/`drop` guards handle that without underflow). Everything
    // downstream (cps/cpm/usvph, /status, MQTT, uploads,
    // build_tx_context) derives from `counts`, so this one substitution filters
    // the whole pipeline. Rolling cpm5/cpm15 are ALSO filtered now: history.c
    // samples the PCNT filtered monotonic total via the same `filtering`
    // decision passed to history_tick() (V2.5.16).
    bool filtering = g_cfg.pcnt_filter && pcnt_on;
    uint32_t counts = filtering ? pc[TUBE_PCNT_NWIDTHS - 1] : counts_raw;

    // V2.6.29: PCNT subtract mode — pcnt_filter and hv_blank may now run
    // TOGETHER. The PCNT hardware path has no notion of HV timing, so on
    // boards whose phantom pulses pass the width filter (S3 family, >=4 µs)
    // the phantoms sit inside the filtered count and must be subtracted out.
    // V2.6.31: subtract only the phantoms STILL IN the PCNT count. The
    // phantom's width is temperature-dependent (field data 2026-08-01:
    // >=4 µs cool, <4 µs above ~15 °C tube temp), so the V2.6.29 full
    // hv_blanked subtraction double-subtracted warm-hour phantoms the width
    // filter had already dropped (−6 CPM at summer temps). pcnt_blank_wide()
    // (tube_logic.h, host-tested) derives the wide-phantom share from the
    // width filter's own removal tally. Strictly gated on `filtering`: in
    // plain mode (pcnt off) counts_raw already excludes blanked edges at the
    // ISR, and with blanking off diag_hv_blanked is 0 — either way this block
    // is a no-op. The monotonic twin feeds history.c's rolling cpm5/cpm15
    // (per-cycle lump cadence — see the note in history_tick).
    uint32_t blanked_wide = 0;
    if (filtering && diag_hv_blanked > 0) {
        blanked_wide = pcnt_blank_wide(pc[TUBE_PCNT_NWIDTHS - 1], counts_raw,
                                       diag_hv_blanked);
        counts       = (counts >= blanked_wide) ? counts - blanked_wide : 0;
        tube_note_blanked_wide(blanked_wide);
    }

    // V2.4.27: hv_pulses returned by tube_read() is cumulative-since-boot
    // (see tube.h). Derive the per-cycle delta here so the legacy HTTPS
    // upload paths (sensor.community / Madavi / Radmon) carry the same
    // semantic as the V1.x firmware did — V1.x's transmit() function
    // computed `delta = current - last; last = current;` before sending.
    // The V2.0 rewrite uploaded the cumulative value raw, which broke
    // historical CSV-archive analyses (radiation.txt). MQTT keeps
    // publishing the cumulative value because HA expects
    // `total_increasing` semantic on this entity.
    static uint32_t s_last_uploaded_hv_pulses = 0;
    uint32_t hv_pulses_delta = hv_pulses - s_last_uploaded_hv_pulses;
    s_last_uploaded_hv_pulses = hv_pulses;

    float cps = (dt_ms > 0) ? (counts * 1000.0f / dt_ms) : 0.0f;
    float usvph = cps * tube_cps_to_usvph(g_cfg.tube_type);
    uint32_t cpm = (dt_ms > 0) ? (uint32_t)(((uint64_t)counts * 60000ULL) / dt_ms) : 0;
    wifi_ap_record_t ap_rec = { 0 };
    int rssi = (esp_wifi_sta_get_ap_info(&ap_rec) == ESP_OK) ? ap_rec.rssi : -127;

    // Two CYCLE log line shapes — the long one (Geiger active) carries radiation
    // metrics, the short one (tube disabled) keeps just dt + rssi so logs stay
    // readable on PM-only deployments. Both now trail the connected AP's BSSID +
    // channel (V2.5.34): per-cycle rssi alone can't reveal a node stuck on a weak
    // or wrong mesh BSSID after a reconnect — the BSSID makes it visible in syslog
    // (and lets us watch the roaming app actually move the node, see below).
    uint32_t i2c_errs = diag_i2c_errors();   // V2.4.28: cumulative since boot

    // Uptime for CYCLE log lines — NTP-accurate when synced, crystal fallback.
    char cycle_uptime[20];
    format_uptime_hm(ntp_uptime_s(), cycle_uptime, sizeof(cycle_uptime));

    if (g_cfg.tube_enabled) {
        ESP_LOGI(TAG, "CYCLE #%lu: dt=%lums counts=%lu cpm=%lu %.3fµSv/h "
                 "hv_pulses=%lu (cum=%lu) hv_err=%d min_us=%lu max_us=%lu "
                 "rssi=%ddBm i2c_err=%lu bssid=" MACSTR " ch=%d "
                 "disconnects=%lu uptime=%s",
                 (unsigned long)++tx_cycles, (unsigned long)dt_ms,
                 (unsigned long)counts, (unsigned long)cpm, usvph,
                 (unsigned long)hv_pulses_delta, (unsigned long)hv_pulses, hv_error,
                 (unsigned long)(min_us == UINT32_MAX ? 0 : min_us),
                 (unsigned long)max_us, rssi, (unsigned long)i2c_errs,
                 MAC2STR(ap_rec.bssid), ap_rec.primary,
                 (unsigned long)n_disconnects, cycle_uptime);

        // V2.5.16: when filtering, the CYCLE line above carries the POST-filter
        // count/cpm (what's uploaded); surface the PRE-filter ISR values here so
        // the unfiltered reading stays in the log. removed = how many ISR counts
        // the width filter dropped this cycle.
        if (filtering) {
            uint32_t cpm_raw = (dt_ms > 0)
                ? (uint32_t)(((uint64_t)counts_raw * 60000ULL) / dt_ms) : 0;
            // V2.6.29: `removed` = the WIDTH filter's own effect. The ISR
            // reference must be blank-corrected: counts_raw EXCLUDES blanked
            // phantoms while pc[widest] still INCLUDES them (they pass the
            // width filter), so the comparable "what the ISR would have
            // counted" is counts_raw + hv_blanked — verified live on the trial
            // field node's CYCLE #2 (counts_raw=205, pcnt=223, hv_blanked=18: the naive
            // formula clamps real width removals to 0 by up to hv_blanked).
            // Reduces to the pre-blank formula when hv_blanked==0.
            // V2.6.31: the subtracted amount is now blanked_wide (width-aware,
            // see pcnt_blank_wide) — report BOTH the raw blank tally and the
            // wide share actually subtracted: their difference is the narrow
            // phantom population, i.e. a free per-cycle phantom-width-vs-
            // temperature telemetry channel (log parsers: clause shape changed
            // from "hv_blanked N subtracted" to "hv_blanked N, N wide
            // subtracted" in V2.6.31).
            uint32_t pcnt_counts = pc[TUBE_PCNT_NWIDTHS - 1];
            uint32_t isr_ref     = counts_raw + diag_hv_blanked;
            uint32_t removed     = (isr_ref >= pcnt_counts)
                                       ? (isr_ref - pcnt_counts)
                                       : 0;
            ESP_LOGI(TAG, "FILTER: pcnt_filter ON @%luns — CYCLE counts/cpm are "
                          "POST-filter; pre-filter counts=%lu cpm=%lu (removed %lu, "
                          "hv_blanked %lu, %lu wide subtracted)",
                     (unsigned long)tube_pcnt_width_ns(TUBE_PCNT_NWIDTHS - 1),
                     (unsigned long)counts_raw, (unsigned long)cpm_raw,
                     (unsigned long)removed, (unsigned long)diag_hv_blanked,
                     (unsigned long)blanked_wide);
        }

        // V2.5.12: raw-edge profiler dump. rejected = edges suppressed by the
        // dead-time gate (ringing/double-counts) — always the RAW ISR count, so
        // it stays meaningful whether or not the width filter is on. edt_us bins
        // show where edge-to-edge spacing lands: low bins = ringing/noise, top
        // two = real.
        uint32_t diag_rejected =
            (diag_raw_edges >= counts_raw) ? (diag_raw_edges - counts_raw) : 0;
        // V2.5.30: guard_removed = REAL counts the dead-time guard suppressed this
        // cycle (only edges past the 190µs gate — its true marginal effect, so
        // counts_without_guard = counts + guard_removed). It is a SUBSET of
        // `rejected`, NOT an orthogonal column — do not sum the two. 0 when off.
        // Placed after rejected so the trailing edt_us block stays positionally
        // last for log parsers. V2.6.9: hv_coincident is placed after
        // guard_removed, before edt_us, for the same reason — it's a COUNTED
        // subset (of `counts`, not of raw_edges-counts like the two before
        // it), tested against hv_pulses/cum on the CYCLE line above: a real
        // Poisson tube shows this near-zero; HV-pickup shows it tracking
        // hv_pulses ~1:1 (radiation_overcounting_independent_review.md).
        // V2.6.29: hv_blanked slots after hv_coincident (edt_us stays last) —
        // would-be counts the HV blanking window dropped (NOT in `counts`,
        // disjoint from guard_removed, but — like guard_removed — a SUBSET of
        // `rejected` above, since rejected = raw_edges − counts = gate rejects
        // + guard_removed + hv_blanked; don't sum it with rejected.
        // counts_without_blank = counts + hv_blanked). With blanking active on
        // an affected board, expect hv_blanked ~= hv_pulses while
        // hv_coincident collapses toward 0.
        ESP_LOGI(TAG, "DIAG: raw_edges=%lu rejected=%lu guard_removed=%lu hv_coincident=%lu "
                      "hv_blanked=%lu "
                      "edt_us[<50|<190|<500|<1k|<5k|<50k|<500k|>=]=%lu %lu %lu %lu %lu %lu %lu %lu",
                 (unsigned long)diag_raw_edges, (unsigned long)diag_rejected,
                 (unsigned long)diag_guard_removed, (unsigned long)diag_hv_coincident,
                 (unsigned long)diag_hv_blanked,
                 (unsigned long)diag_hist[0], (unsigned long)diag_hist[1],
                 (unsigned long)diag_hist[2], (unsigned long)diag_hist[3],
                 (unsigned long)diag_hist[4], (unsigned long)diag_hist[5],
                 (unsigned long)diag_hist[6], (unsigned long)diag_hist[7]);

#if TUBE_REJLOG_ENABLE
        // V2.7.2: sub-b1 reject profiler dump. The DIAG histogram above bins
        // every rejected edge into "<190µs" and that bin is where the Rev B/C
        // phantom population lives — so the one number that identifies the
        // mechanism is exactly the one bucketing destroys. One line per record
        // rather than an accumulated buffer: the population is ~5/cycle, and a
        // per-record line needs no length arithmetic to be correct.
        //   edt = µs to the previous edge  -> position on the count-node
        //         recovery ramp (τ = 190.4µs, and it is NOT R5·C4 — see
        //         tube.h for why the divider tap adds in series); the
        //         population's UPPER cutoff gives the disturbance amplitude
        //         (the 190µs bucket edge is the dead-time gate, not the
        //         cutoff, and its matching 190 is a coincidence — see tube.h).
        //   dt  = µs to the last COUNTED pulse (differs from edt only inside a
        //         multi-edge burst, so edt!=dt flags a burst rather than a
        //         single phantom).
        //   tph = µs since the last recharge_tick ISR entry, 0..~100µs. UNIFORM
        //         acquits the 10kHz timer ISR; CLUSTERED convicts it.
        //   hvg = µs since the last HV FET turn-off.
        {
            static tube_rejlog_rec_t rej[TUBE_REJLOG_N];
            uint32_t                 rej_drop = 0;
            uint32_t                 rej_n    = tube_get_rejlog(rej, TUBE_REJLOG_N, &rej_drop);
            ESP_LOGI(TAG, "REJLOG: n=%lu dropped=%lu",
                     (unsigned long)rej_n, (unsigned long)rej_drop);
            for (uint32_t i = 0; i < rej_n; i++) {
                ESP_LOGI(TAG, "REJ: i=%lu ts=%lu edt=%lu dt=%lu tph=%lu hvg=%lu",
                         (unsigned long)i,
                         (unsigned long)rej[i].ts_us,
                         (unsigned long)rej[i].edt_us,
                         (unsigned long)rej[i].dt_us,
                         (unsigned long)rej[i].tick_phase_us,
                         (unsigned long)rej[i].hv_gap_us);
            }
        }
#endif

        // V2.5.16: PCNT width-comb dump (reuses the snapshot taken up top — do
        // NOT re-read, that would advance the delta base and zero this dump).
        // pc[0] is unfiltered
        // (≈ DIAG raw_edges, same cycle); pc[1..3] reject pulses narrower than
        // TUBE_PCNT_WIDTHS_NS[i]. (pc[0]-pc[N]) = pulses in that width band. When
        // pcnt_filter is on, pc[NWIDTHS-1] is the count the CYCLE line reports.
        if (pcnt_on) {
            uint32_t drop = (pc[0] >= pc[TUBE_PCNT_NWIDTHS - 1])
                                ? (pc[0] - pc[TUBE_PCNT_NWIDTHS - 1]) : 0;
            ESP_LOGI(TAG, "PCNT: w_ns[%lu|%lu|%lu|%lu]=%lu %lu %lu %lu "
                     "(off≈raw_edges; off-widest=%lu narrow)",
                     (unsigned long)tube_pcnt_width_ns(0), (unsigned long)tube_pcnt_width_ns(1),
                     (unsigned long)tube_pcnt_width_ns(2), (unsigned long)tube_pcnt_width_ns(3),
                     (unsigned long)pc[0], (unsigned long)pc[1],
                     (unsigned long)pc[2], (unsigned long)pc[3],
                     (unsigned long)drop);
        }
    } else {
        ESP_LOGI(TAG, "CYCLE #%lu: dt=%lums (tube disabled) rssi=%ddBm i2c_err=%lu "
                 "bssid=" MACSTR " ch=%d disconnects=%lu uptime=%s",
                 (unsigned long)++tx_cycles, (unsigned long)dt_ms, rssi,
                 (unsigned long)i2c_errs, MAC2STR(ap_rec.bssid), ap_rec.primary,
                 (unsigned long)n_disconnects, cycle_uptime);
    }

    // One-line LoRaWAN health next to every CYCLE line (user request during
    // the first live-gateway bench): the per-uplink "uplink OK" lines land
    // seconds AFTER this cycle's snapshot is enqueued, so this line
    // summarizes the lifetime counters as-of the previous cycles — a
    // long-scrollback health read without grepping per-uplink lines.
    // Compile-time no-op on boards without the radio (stub returns DISABLED).
    if (HAL_HAS_LORAWAN && g_cfg.lorawan_enabled) {
        lorawan_status_t lst;
        lorawan_get_status(&lst);
        ESP_LOGI(TAG, "LORA: state=%s up=%lu fail=%lu duty_skip=%lu joins=%lu last_err=%d",
                 lorawan_state_name(lst.state), (unsigned long)lst.uplinks_sent,
                 (unsigned long)lst.failed, (unsigned long)lst.duty_skipped,
                 (unsigned long)lst.join_attempts, (int)lst.last_error);
    }

    // Cache for /status — see g_last_* declarations + accessors above.
    g_last_dt_ms     = dt_ms;
    g_last_cpm       = cpm;
    g_last_usvph     = usvph;
    g_last_filtering = filtering;          // V2.5.16: /status filter indicator
    g_last_cpm_raw   = (dt_ms > 0)
        ? (uint32_t)(((uint64_t)counts_raw * 60000ULL) / dt_ms) : 0;
    g_last_filter_width_ns = filtering ? tube_pcnt_width_ns(TUBE_PCNT_NWIDTHS - 1) : 0;
    g_last_hv_pulses       = hv_pulses;        // cumulative — MQTT
    g_last_hv_pulses_delta = hv_pulses_delta;  // per-cycle — status page + HTTPS uploads
    g_last_hv_error  = hv_error;
    // V2.4.1 (B1): 64-bit store on a 32-bit core isn't atomic — wrap to
    // pair with the spinlock'd read in main_status_last_cycle_at.
    int64_t cycle_at = (int64_t)time(NULL);
    portENTER_CRITICAL(&g_last_cycle_at_mux);
    g_last_cycle_at  = cycle_at;
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
        // V2.6.32: rad_nsvph/cpm_disp now feed the snapshot below (TFT
        // rotation Radiation page); uptime is read live at render time.
        (void)time_sec;
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
            diag_i2c_error_inc();   // V2.4.28
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

    // GNSS line — emitted only when a receiver is bound (gnss_present()), so
    // boards without a GPS module log nothing here. Display-only data (V2.5.11):
    // the snapshot is whatever gnss_poll() last drained on the main task, so no
    // I²C is touched from this TX path. Two shapes mirror the /status card:
    // a full fix line, or an "acquiring" line before RMC goes valid.
    if (gnss_present()) {
        gnss_fix_t gf;
        gnss_get_fix(&gf);
        if (gf.valid) {
            char utc[32] = "—";
            if (gf.utc > 0) {
                struct tm tm_utc;
                gmtime_r(&gf.utc, &tm_utc);
                strftime(utc, sizeof(utc), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
            }
            ESP_LOGI(TAG,
                     "%s: Fix: %s, %u satellites, HDOP %.1f "
                     "Position: %.6f, %.6f Altitude: %.0f m MSL UTC: %s",
                     gnss_chip_name(), gf.fix_3d ? "3D" : "2D",
                     (unsigned)gf.sats, (double)gf.hdop,
                     gf.lat, gf.lon, (double)gf.alt_m, utc);
        } else {
            ESP_LOGI(TAG, "%s: Fix: acquiring (%u satellites visible)",
                     gnss_chip_name(), (unsigned)gf.sats);
        }
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
            diag_i2c_error_inc();   // V2.4.28
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
            diag_i2c_error_inc();   // V2.4.28 — note: first-cycle integration
                                    // returns ESP_FAIL, so the first cycle
                                    // increments once on every boot. Stable
                                    // afterward.
        }
    }

    // V2.6.15: SGP41 NOx index — unlike noise/PM above, this is a
    // continuously-updated background value (see sgp41.h), not something
    // triggered per TX cycle. We just sample whatever the background task
    // has most recently cached. Not yet valid for the first ~10 s
    // (conditioning) plus ~45 s (algorithm's initial blackout) after boot.
    int32_t nox_index = 0;
    bool    nox_valid = (sgp41_get_nox_index(&nox_index) == ESP_OK);
    if (sgp41_present()) {
        if (nox_valid) {
            ESP_LOGI(TAG, "SGP41: NOx index=%ld", (long)nox_index);
        } else if (sgp41_had_valid_reading()) {
            // V2.6.15: was working, cache has since aged out (no fresh
            // sample within NOX_STALE_US) — not a boot state.
            ESP_LOGI(TAG, "SGP41: NOx index unavailable (sensor unresponsive)");
        } else {
            ESP_LOGI(TAG, "SGP41: NOx index not yet available (conditioning/warming up)");
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
            .rad_valid   = g_cfg.tube_enabled,
            .rad_nsvph   = rad_nsvph,
            .rad_cpm     = cpm_disp,
        };
        display_update_snapshot(&snap);
    }

    // V2.6.19 standalone mode: the cycle's output is a CSV row, not an
    // upload. Skips the WiFi/NTP gates (meaningless offline), MQTT, and
    // tx_transmit entirely. DNMS window re-trigger still happens (below)
    // so noise integration keeps covering full cycles.
    // Final review A2: gate on the boot-time LATCH, not the live g_cfg
    // field — see the latch's declaration comment.
    if (s_standalone_sd_latched) {
        s_tube_tm = (tube_tm_cache_t){
            .valid     = g_cfg.tube_enabled,
            .cpm       = cpm,
            .usvph     = usvph,
            .counts    = counts,
            .hv_pulses = hv_pulses_delta,
            .dt_ms     = dt_ms,
        };
        // Bypassing tx_run() (transmission.c) below this branch also skips
        // its per-cycle heap log lines — standalone had zero heap visibility
        // in /log until this was added, matching what tx_run() logs every
        // networked TX cycle (as one combined line here). Ring-buffer/serial
        // only, not the CSV.
        diag_log_heap_standalone();

        // Final review A3: env_sensor_read() above already ran the fused
        // cascade (priority short-circuit — skips a chip once an earlier
        // one already satisfied its fields), which is exactly right for the
        // single fused reading networked firmware publishes. But the CSV
        // wants every ATTACHED chip's own column (spec §4.1), and a column
        // whose chip got skipped this cycle never refreshes its own
        // telemetry cache (populated only inside that chip's own *_read()):
        // permanently empty, or worse, frozen on a stale value from the one
        // cycle its higher-priority sibling happened to fail. One extra full
        // I2C pass, standalone-only, main task, 150 s cadence — acceptable
        // per spec; the fused path above is untouched.
        env_sensor_refresh_all_for_telemetry();
        sd_logger_cycle();
        if (noise_sensor_present()) {
            if (noise_sensor_trigger() != ESP_OK) {
                ESP_LOGW(TAG, "%s: trigger for next window failed", noise_sensor_name());
            }
        }
        return;
    }

    // V2.6.23: LoRaWAN uplink — non-blocking enqueue into a depth-1 mailbox
    // (xQueueOverwrite: freshest snapshot wins; see lorawan_transmit()). No-op
    // stub on boards without the radio; gated at runtime by lorawan_enabled
    // inside lorawan_setup(). Reuses this cycle's counting window verbatim:
    // the payload carries counts + dt, so CPM is derived server-side whatever
    // the interval (spec §5).
    //
    // Placed ahead of the wifi_up()/NTP early-returns deliberately: LoRaWAN
    // is the standalone-deployment uplink (spec §0) — it must fire with WiFi
    // down and with no NTP sync (the payload carries counts+dt, no wall
    // time). Only the WiFi TX pipeline below is gated on connectivity. (The
    // one earlier return this still sits after is the standalone-SD branch,
    // unreachable on the only LoRaWAN board — no SD slot — but a future
    // SD+LoRa board would need this enqueue lifted above that branch too.)
    if (g_cfg.lorawan_enabled) {
        lorawan_snapshot_t lsnap = {
            .gm_counts     = counts,
            .dt_ms         = dt_ms,
            .tube_nbr      = (uint8_t)g_cfg.tube_type,   // same 0-3 table V1.9/ttn2luft uses
            .env_valid     = bme_valid,
            .temperature_c = bme_t,
            .humidity_pct  = bme_h,
            .pressure_pa   = bme_p,
        };
        lorawan_transmit(&lsnap);
    }

    // "WiFi TX" wording + LoRaWAN suffix: these two gates only skip the
    // WiFi-based upload pipeline below — the LoRaWAN enqueue above already
    // fired. The old bare "skipping TX" read as "nothing was sent at all" on
    // LoRaWAN-standalone nodes (user-reported during the first live-gateway
    // bench). Suffix is compile-gated on HAL_HAS_LORAWAN so a non-radio
    // board with a stray lorawan_enabled=true NVS byte can't claim an
    // uplink path it doesn't have.
    if (!wifi_up()) {
        ESP_LOGW(TAG, "skipping WiFi TX: WiFi down%s",
                 (HAL_HAS_LORAWAN && g_cfg.lorawan_enabled) ? " (LoRaWAN uplink unaffected)" : "");
        // Still trigger the next DNMS window so we don't lose the cycle's
        // worth of integration time waiting for WiFi.
        if (noise_sensor_present()) noise_sensor_trigger();
        return;
    }
    if (!ntp_time_valid()) {
        ESP_LOGW(TAG, "skipping WiFi TX: time not valid (no NTP sync yet)%s",
                 (HAL_HAS_LORAWAN && g_cfg.lorawan_enabled) ? " (LoRaWAN uplink unaffected)" : "");
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
        mqtt_publish_state(&snap, pm_valid, &pm, noise_valid, &noise,
                           nox_valid, nox_index);
    }

    // V2.4.27: hv_pulses_delta (not cumulative) — see comment in do_tx_cycle
    // above. The legacy CSV archives (sensor.community / Madavi / Radmon)
    // expect per-cycle delta, matching V1.x firmware behaviour.
    tx_context_t ctx;
    build_tx_context(&ctx, dt_ms, counts, hv_pulses_delta, min_us, max_us,
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

    // V2.6.21: apply TZ before anything logs or timestamps a file — standalone
    // boards never reach ntp_setup() (no STA, so no GOT_IP), so without this
    // they never got past the UTC default despite tz_posix being configured.
    // Harmless on networked boards too: ntp_setup() re-applies the same value
    // once GOT_IP fires.
    ntp_set_timezone(g_cfg.tz_posix);

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
        const char *model = chip_model_str(chip.model);
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
    //
    // Heltec WiFi LoRa 32 V4-R2 is the one exception: its secondary bus is
    // permanently and exclusively the onboard OLED, never a general-purpose
    // STEMMA-style connector — see PROBE_ON_BOTH_BUSES below, which skips
    // the secondary-bus fallback entirely on this board.
    //
    // V2.5.19: select the primary-bus pin route from config BEFORE the first
    // i2c_bus_get_primary() below caches the bus. No-op except on QT Py
    // (HAL_HAS_I2C_PINOUT_SWITCH); reboot-required by construction.
    i2c_bus_set_primary_pinout(g_cfg.i2c_pinout);
    i2c_master_bus_handle_t bus1 = i2c_bus_get_primary();

    // V2.6.6: MAX17048 fuel gauge is a fixed onboard part on STEMMA1/
    // primary — unlike the pluggable env/PM/noise sensors below, it never
    // needs the secondary-bus fallback probe.
    fuel_gauge_init(bus1);
    // V2.6.6: seed the user-confirmed battery-presence flag from config —
    // see fuel_gauge.h for why this can't be auto-detected via VCELL.
    // Live-reapplied on every /config Save in http_server.c::config_post.
    fuel_gauge_set_user_present(g_cfg.batt_present);

    // Helper macro: try a sensor's init on bus 1; if no device bound,
    // try bus 2; if a device was found there, keep the bus alive.
    //
    // Heltec WiFi LoRa 32 V4-R2: the secondary bus is the onboard OLED,
    // never a pluggable sensor connector, so the fallback probe is
    // compiled out entirely for this board — it would otherwise probe the
    // display's bus on every sensor driver, every boot, for no reason.
#if defined(BOARD_HELTEC_WIFI_LORA32_V4_R2)
    #define PROBE_ON_BOTH_BUSES(init_fn, present_fn, bus1)                  \
        do {                                                                \
            init_fn(bus1);                                                  \
        } while (0)
#else
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
#endif

    PROBE_ON_BOTH_BUSES(env_sensor_init,   env_sensor_present,   bus1);
    PROBE_ON_BOTH_BUSES(pm_sensor_init,    pm_sensor_present,    bus1);
    PROBE_ON_BOTH_BUSES(noise_sensor_init, noise_sensor_present, bus1);
    // V2.6.15: SGP41 VOC+NOx sensor — probed after env_sensor_init so its
    // background sampling task (started inside sgp41_init on a hit) can
    // find SHT45 already bound for RH/T compensation reads.
    PROBE_ON_BOTH_BUSES(sgp41_init,        sgp41_present,        bus1);

    // V2.5.10: GNSS vs ambient-light auto-detect (no config toggle). The
    // PA1010D GNSS breakout sits at I²C 0x10 — the SAME address as the
    // VEML7700 — so they can't coexist on a bus. gnss_init() resolves this by
    // sniffing for live NMEA at 0x10 (and binding u-blox 0x42 unambiguously);
    // a VEML7700 fails the sniff, so we fall through to the light-sensor probe
    // only when no GNSS was found. Whichever module is physically present just
    // works — nothing to configure.
    PROBE_ON_BOTH_BUSES(gnss_init, gnss_present, bus1);
    if (!gnss_present()) {
        PROBE_ON_BOTH_BUSES(veml7700_init, veml7700_present, bus1);
    }

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

    // V2.6.19: standalone SD-logging mode (spec §2). Tube columns register
    // here (not in a driver — the values are computed per-cycle in
    // do_tx_cycle), gated on tube_enabled: an env-only standalone logger
    // gets a CSV with no tube columns at all. Registration itself is
    // harmless when standalone is off (registry has no consumer then).
    if (g_cfg.tube_enabled) {
        telemetry_register("Tube CPM",           tm_tube_cpm,    NULL);
        telemetry_register("Tube Dose [uSv/h]",  tm_tube_dose,   NULL);
        telemetry_register("Tube GM Counts",     tm_tube_counts, NULL);
        telemetry_register("Tube HV Pulses",     tm_tube_hv,     NULL);
        telemetry_register("Tube Window [ms]",   tm_tube_window, NULL);
    }
    // Final review A2: latch here, once, at boot — g_cfg.standalone_sd/
    // standalone_ap_on themselves must NOT be read again anywhere past this
    // point (do_tx_cycle and the AP-window-close branch below both use the
    // latch instead). See the latch's declaration comment for why.
    s_standalone_sd_latched    = HAL_HAS_SD_CARD && g_cfg.standalone_sd;
    s_standalone_ap_on_latched = g_cfg.standalone_ap_on;
    if (s_standalone_sd_latched) {
        sd_logger_init(g_chip_id);
        gnss_set_clock_source(true);   // no NTP offline — GPS is the clock (spec §6)
        if (sd_card_mount() != ESP_OK) {
            ESP_LOGW(TAG, "standalone: SD mount failed at boot — will retry each cycle");
        }
        if (!gnss_present()) {
            ESP_LOGW(TAG, "standalone: no GNSS receiver found — "
                          "no clock source, CSV logging will never start!");
        }
        // Final review A1: the SD-failure alert (sd_logger.c's fail_cycle(),
        // via neopixel_set_alert()) needs a live NeoPixel even when led_tick
        // is off or the tube is disabled (spec §8 test 6's exact env-only
        // rig) — init it here, unconditionally, so it's never a silent
        // no-op on a standalone node. Guarded against double-init below
        // (neopixel_init() is not safe to call twice — see that call site).
        neopixel_init();
    }

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
    // V2.5.20 (review R5): ssid_len must be the bytes actually IN the buffer.
    // The previous snprintf-based copy used the UNtruncated return length, so
    // a 32-char ap_name into the 32-byte ssid field reported len 32 while the
    // buffer held 31 chars + NUL — the beacon then carried a trailing 0x00 as
    // its 32nd SSID byte.
    safe_strcpy((char *)apc.ap.ssid, g_cfg.ap_name, sizeof(apc.ap.ssid));
    apc.ap.ssid_len      = (uint8_t)strlen((const char *)apc.ap.ssid);
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

    // V2.5.16: optional PCNT pulse-width filter+comb (off by default). Brought
    // up after tube_setup so the count pin is already an input with the GMC ISR
    // live — PCNT only adds GPIO-matrix taps and never disturbs the ISR. When
    // pcnt_filter is on, do_tx_cycle uses the widest-tooth count as the
    // authoritative CPM (and logs the full comb + pre-filter values). Tube-gated:
    // pointless without count pulses.
    // V2.6.31: also brought up in blanking-only mode (hv_blank without
    // pcnt_filter) as a pure DIAGNOSTIC: the comb is passive parallel hardware
    // — counts/cpm stay ISR-based (the `filtering` substitution stays strictly
    // pcnt_filter-gated) — but the PCNT w_ns log line keeps flowing, which is
    // the phantom-width-vs-temperature telemetry the 2026-08-01 field analysis
    // ran on.
    if (g_cfg.tube_enabled && (g_cfg.pcnt_filter || g_cfg.hv_blank)) {
        tube_pcnt_init(g_cfg.pcnt_filter_width_ns);
    }

    // V2.5.30: optional dead-time guard / burst-collapse (off by default — the
    // deadtime_guard checkbox is unchecked). A retriggerable refractory on top of
    // the 190µs ISR gate that collapses 1-5ms afterpulse/re-trigger trains to one.
    // config_effective_guard_us() returns 0 when off OR when pcnt_filter supersedes.
    // Diagnostic only (alters dead-time loss; doesn't reach the genuine ~40% of
    // the board gap) — see config_fields.def. Tube-gated. Set here at boot;
    // also live-applied from config_post on /config Save (review #4).
    if (g_cfg.tube_enabled) {
        tube_set_guard_us(config_effective_guard_us(&g_cfg));
        // V2.6.29: optional HV blanking window (off by default) — phantom-pulse
        // suppression for the Rev B/C coupling defect, keyed on the recharge
        // state machine's FET turn-off stamp. Same opt-in/live-apply discipline
        // as the guard, but it COMPOSES with pcnt_filter (subtract mode in
        // do_tx_cycle/history.c) instead of being superseded by it; see
        // config_fields.def.
        tube_set_blank_us(config_effective_blank_us(&g_cfg));
    }

    history_init();   // V2.5.6: CPM history ring (sampler primed on first tick)
    speaker_setup(g_cfg.play_sound, g_cfg.led_tick, g_cfg.speaker_tick);

    // Per-GM-pulse visual feedback. led_init() is unconditional: it drives the
    // plain user LED (XIAO GPIO21) to a deterministic OFF so an active-low pin
    // isn't left floating — cheap, and a no-op stub on boards without it.
    // V2.5.20/L1: neopixel_init() is now gated on led_tick too, so the WS2812
    // power rail (PIN_NEOPIXEL_POWER) is NOT energised when the flash is
    // disabled. Both inits run before registration so the pulse-tick callback
    // finds initialised hardware. The single tube callback slot is claimed by
    // exactly one of: neopixel.c (HAL_HAS_NEOPIXEL, no HAL_HAS_SPEAKER — QT Py,
    // SparkFun C5), led.c (plain user LED with no speaker/NeoPixel — XIAO,
    // Heltec V4 R2), or speaker.c itself (HAL_HAS_SPEAKER, most boards). On
    // every speaker board that also has a NeoPixel (SparkFun S3, Feather V2,
    // and since V2.6.24 the dual-LED TFT Feather / #5477 too), speaker.c keeps
    // the callback and drives the NeoPixel directly via neopixel_notify_pulse()
    // since the slot can't be shared -- see neopixel_register_pulse_tick()'s
    // doc comment. Whichever module(s) don't claim it are stubs for that call.
    led_init();
    if (g_cfg.tube_enabled && g_cfg.led_tick) {
        // Final review A1: standalone mode may already have brought the
        // NeoPixel up above (for the SD alert, independent of led_tick) —
        // neopixel_init() is not safe to call twice (would re-create the RMT
        // TX channel on the same already-claimed GPIO), so skip the repeat
        // init in that case; only the pulse-tick worker/callback still needs
        // registering here.
        if (!s_standalone_sd_latched) {
            neopixel_init();
        }
        neopixel_register_pulse_tick();
        led_register_pulse_tick();
    }
    tx_setup();
    lorawan_setup();
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
    // cold boot the clock is already past 2026 and ntp_time_valid() returns
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

#if CONFIG_ESP_WIFI_ENABLE_ROAMING_APP
    // When we defer a disconnect to the roaming app, the time we started waiting
    // for it to bring the link back (0 = currently connected). Drives the
    // reconnect safety-net below. app_main never returns, so a plain local persists.
    int64_t t_roam_defer_us = 0;
#endif

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
            // V2.4.30: nudge esp-mqtt to reconnect immediately on this
            // (re)association instead of waiting up to reconnect_timeout_ms
            // (30 s) for its internal retry timer. No-op on the first GOT_IP
            // (client not yet inited — mqtt_init runs later this iteration)
            // and when already connected. Closes the ~10 s tail-lag seen
            // after a router reboot where STA held an IP but MQTT sat idle.
            mqtt_kick_reconnect();
#if CONFIG_ESP_WIFI_ENABLE_ROAMING_APP
            t_roam_defer_us = 0;   // back online — disarm the reconnect safety-net
#endif
        }
        if (bits & EV_DISCONNECTED) {
            if (!g_sta_connect_allowed) {
                continue;
            }
#if CONFIG_ESP_WIFI_ENABLE_ROAMING_APP
            // V2.5.34: on PSRAM boards the ESP-IDF roaming app owns reconnection —
            // BUT only ONCE we've associated at least once. Its disconnect hook
            // (roam_sta_disconnected) calls esp_wifi_connect() itself (and on a
            // low-RSSI roam re-scans to the strongest BSSID); driving connect from
            // here too would give the supplicant two reconnect owners — a connect
            // race (the "multiple lifecycle owners → emergent timing" trap). HOWEVER
            // the roaming app's allow_reconnect flag is false until the FIRST
            // successful STA_CONNECTED, so before that it ignores disconnects. If we
            // also did nothing, a failed first connect (AP down/slow at boot) would
            // be retried by NOBODY until the 10-min startup watchdog reboots — worse
            // than the pre-roaming behaviour. So defer only after n_connects>0 (the
            // exact point allow_reconnect flips true); until then keep our own retry,
            // which can't race the app (it's idle pre-association). mark_attempt() on
            // BOTH paths so n_attempts tracks reconnect activity either way (review
            // #3) — when deferring it timestamps the cycle for the next assoc-time.
            if (n_connects > 0) {
                mark_attempt();
                if (t_roam_defer_us == 0) t_roam_defer_us = esp_timer_get_time();
                ESP_LOGI(TAG, "disconnected — reconnect owned by roaming app (attempt #%" PRIu32 ")",
                         n_attempts);
                continue;
            }
#endif
            vTaskDelay(pdMS_TO_TICKS(500));
            mark_attempt();
            // V2.6.15: esp_wifi_connect() can be rejected with ESP_ERR_WIFI_CONN
            // when the driver's own internal auth/SAE retry from the previous
            // cycle is still in flight — observed repeatedly during a WPA3-SAE
            // retry storm on the C5's 5GHz link, where each internal SAE round
            // took several seconds and every 500ms-later retry call here was
            // bounced. Harmless (the driver keeps retrying on its own schedule
            // regardless), but logging it as a fresh "retry connect" attempt was
            // misleading. Log what actually happened instead.
            esp_err_t connect_err = esp_wifi_connect();
            if (connect_err == ESP_ERR_WIFI_CONN) {
                ESP_LOGI(TAG, "retry connect (attempt #%" PRIu32 "): driver already reconnecting, skipped",
                         n_attempts);
            } else {
                ESP_LOGI(TAG, "retry connect (attempt #%" PRIu32 ")", n_attempts);
            }
            continue;
        }

        ntp_poll();
        gnss_poll();   // V2.5.8: drain GNSS I²C + GPS-primary clock discipline (no-op if absent)

        // V2.4.12: start MQTT only once both preconditions hold (see boot
        // section comment above). n_got_ip>0 means STA has reached the LAN
        // at least once (sticky — only ever increments); ntp_time_valid()
        // means the wall clock is past 2026-01-01 (sane for TLS cert
        // validation, whether from fresh SNTP or RTC carryover).
        // V2.4.13: source of truth moved to mqtt_is_initialized() so the
        // /update OTA-teardown path can flip it back to false via mqtt_stop()
        // and this poll will re-init MQTT on the next tick after a failed
        // OTA. On a successful OTA the device reboots, so the re-init never
        // runs — but the failure path is the one we care about for recovery.
        // V2.4.17: skip re-init if the OTA teardown set the suspended flag.
        // Without this, an OTA in progress would see MQTT/syslog re-init
        // ~1 s after the teardown, defeating the V2.4.13 heap-freeing intent.
        // V2.5.10: also defer the (re)start when an FTPS upload is imminent.
        // The 24h PSA refresh stops MQTT and relies on this poll to restart
        // it; if an FTPS upload (which also stops MQTT) is due on the next
        // tick, restarting here just burns a full TLS connect + HA-discovery
        // publish that FTP tears down ~180 ms later. Skipping it lets MQTT
        // come back up exactly once, after the upload. (Diagnosed from a
        // PSA+FTP schedule alignment on esp32-5963724.)
        // V2.5.13: on the heap-tight Heltec (no PSRAM), defer the MQTT TLS handshake
        // + HA-discovery burst until the first TX cycle's HTTPS uploads have FINISHED.
        // Otherwise the two TLS storms overlap at boot (2-3 concurrent TLS contexts),
        // cratering the DMA-capable heap (min_free 280-1664 B) and triggering an MQTT
        // connect->disconnect->reconnect churn (observed on esp32-12276328). Two terms:
        // `tx_cycles >= 1` gates past boot; `tx_is_idle()` ensures the CPU1 worker
        // (which runs the uploads ASYNC, after do_tx_cycle's non-blocking tx_transmit)
        // isn't mid-cycle — so MQTT starts in the ~168 s gap BETWEEN cycle #1's uploads
        // and cycle #2, with no concurrent upload TLS. Race-free: do_tx_cycle bumps the
        // counter AND enqueues the worker in one pass on this same task. `tx_cycles` is
        // monotonic and the worker idles >90% of the time, so the OTA-/FTPS-teardown
        // MQTT re-inits (same poll) are unaffected (and also no longer collide with an
        // in-flight upload). PSRAM boards start MQTT as soon as IP+clock are ready.
        if (!mqtt_is_initialized() && !main_services_suspended() &&
            n_got_ip > 0 && ntp_time_valid() && !log_ftp_imminent()
#ifdef BOARD_HELTEC_V2
            && tx_cycles >= 1 && tx_is_idle()
#endif
           ) {
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
            // V2.5.23: now that the UDP client is up, emit the full config dump
            // so it reaches the rsyslog server (the boot-time copy predates
            // syslog and never leaves the device). One-shot by construction:
            // this block is gated on !syslog_is_initialized() above, so it can't
            // re-enter once syslog is up. Syslog-OFF nodes dumped it at boot.
            if (syslog_is_initialized()) {
                config_log_summary(&g_cfg);

                // V2.6.16: raw wifi_ap_record_t + negotiated phymode dump —
                // same association info the driver's own "wifi:" trace lines
                // print (RSSI, channel, security, phy caps), but queried via
                // the public esp_wifi API so we control the timing and it
                // reaches syslog. Fields are logged as-is (no enum->string
                // mapping): authmode/pairwise/group/phymode are the raw
                // wifi_auth_mode_t / wifi_cipher_type_t / wifi_phy_mode_t
                // values, b/g/n/lr/a/ac/ax/wps/ftmr/ftmi are the ap_record's
                // capability bitfields.
                wifi_ap_record_t ap = { 0 };
                if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                    wifi_phy_mode_t phymode = 0;
                    esp_wifi_sta_get_negotiated_phymode(&phymode);
                    uint16_t aid = 0;
                    esp_wifi_sta_get_aid(&aid);
                    ESP_LOGI(TAG, "wifi link: bssid=" MACSTR " ch=%d second=%d bandwidth=%d "
                                  "rssi=%d authmode=%d pairwise=%d group=%d "
                                  "b=%d g=%d n=%d lr=%d a=%d ac=%d ax=%d wps=%d ftmr=%d ftmi=%d "
                                  "phymode=%d aid=%d",
                             MAC2STR(ap.bssid), ap.primary, ap.second, ap.bandwidth,
                             ap.rssi, ap.authmode, ap.pairwise_cipher, ap.group_cipher,
                             ap.phy_11b, ap.phy_11g, ap.phy_11n, ap.phy_lr, ap.phy_11a,
                             ap.phy_11ac, ap.phy_11ax, ap.wps, ap.ftm_responder,
                             ap.ftm_initiator, phymode, aid);
                }

                // V2.6.19: STA IP/gateway/netmask/DNS dump alongside the "wifi
                // link:" RF details above. Read fresh here (not reused from the
                // GOT_IP handler's own log line) because that one fires before
                // syslog_init() and never reaches the server — same reasoning
                // as the config: dump this block already does.
                esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
                if (sta_netif) {
                    esp_netif_ip_info_t ip = { 0 };
                    esp_netif_dns_info_t dns1 = { 0 }, dns2 = { 0 };
                    esp_netif_get_ip_info(sta_netif, &ip);
                    esp_netif_get_dns_info(sta_netif, ESP_NETIF_DNS_MAIN,   &dns1);
                    esp_netif_get_dns_info(sta_netif, ESP_NETIF_DNS_BACKUP, &dns2);
                    ESP_LOGI(TAG, "wifi ip: IP=" IPSTR " Gateway=" IPSTR
                                  " Network Mask=" IPSTR " DNS=" IPSTR ", " IPSTR,
                             IP2STR(&ip.ip), IP2STR(&ip.gw), IP2STR(&ip.netmask),
                             IP2STR(&dns1.ip.u_addr.ip4), IP2STR(&dns2.ip.u_addr.ip4));
                }
            }
        }

        // End of boot AP window: stop the AP and switch to STA-only.
        // Radio is never shared — AP is fully down before STA starts.
        //
        // V2.6.19 standalone (spec §2): never switch to STA. Sub-modes:
        //   standalone_ap_on=1 → AP + httpd stay up forever (field
        //     monitoring via /status; costs ~50-80 mA);
        //   standalone_ap_on=0 → radio fully off at window close. Note the
        //     existing branch below requires g_have_sta_creds — standalone
        //     must NOT: a credential-less standalone node still needs its
        //     radio turned off (the stock behaviour would hold AP forever).
        // Final review A2: both reads below are the boot-time LATCH, not
        // live g_cfg — a plain /config Save toggling either checkbox on a
        // running node must have zero effect until reboot (see the latch's
        // declaration comment).
        static bool s_radio_off = false;

        // V2.6.23: keep the boot AP up while a client is actively connected,
        // so the window close never yanks /config out from under someone
        // mid-configuration. We ASK THE DRIVER for the associated-station
        // count (esp_wifi_ap_get_sta_list) instead of maintaining our own
        // connect/disconnect counter — the driver is the single source of
        // truth and can't desync from a coalesced/missed event. Evaluated
        // only after the AP window elapsed and only while the AP is still up
        // and closeable (not already switched to STA, not standalone-radio-
        // off). Once the last client leaves we wait out AP_CLIENT_GRACE_US
        // before closing. Applies to BOTH timed-close branches below; the
        // permanent keep-AP-on branch never closes so it doesn't consult it.
        static int64_t s_ap_last_client_us = 0;
        static bool    s_ap_deferring      = false;
        bool ap_hold_for_client = false;
        // Only evaluate when a TIMED close is actually reachable this boot:
        // not after the STA switch / radio-off (interface gone), not in
        // permanent keep-AP-on mode (never closes), and only when some close
        // branch can fire — standalone-SD, or a normal node with STA creds.
        // (Without this a connecting client would log a spurious "deferring
        // close" in the always-on-AP / no-creds modes where nothing closes.)
        if (!g_sta_connect_allowed && !s_radio_off &&
            !s_standalone_ap_on_latched && (s_standalone_sd_latched || g_have_sta_creds) &&
            (esp_timer_get_time() - boot_time_us) > AP_WINDOW_US) {
            wifi_sta_list_t apsta;
            if (esp_wifi_ap_get_sta_list(&apsta) == ESP_OK && apsta.num > 0) {
                s_ap_last_client_us = esp_timer_get_time();
                ap_hold_for_client  = true;
            } else if (s_ap_last_client_us != 0 &&
                       (esp_timer_get_time() - s_ap_last_client_us) <= AP_CLIENT_GRACE_US) {
                ap_hold_for_client = true;   // within grace after last client left
            }
            if (ap_hold_for_client && !s_ap_deferring) {
                ESP_LOGI(TAG, "AP window elapsed but client connected — deferring close");
                s_ap_deferring = true;
            } else if (!ap_hold_for_client && s_ap_deferring) {
                ESP_LOGI(TAG, "AP client gone (+grace) — proceeding with window close");
                s_ap_deferring = false;
            }
        }

        if (s_standalone_ap_on_latched) {
            // V2.6.23: generalized keep-AP-on (was SD-standalone-only in
            // V2.6.19). AP + httpd stay up for the life of the boot so a
            // field node — SD-standalone OR LoRaWAN-standalone OR any node
            // the user wants permanently reachable — can always be
            // reconfigured. Costs ~50-80 mA; STA never starts.
        } else if (s_standalone_sd_latched) {
            if (!s_radio_off && !ap_hold_for_client &&
                (esp_timer_get_time() - boot_time_us) > AP_WINDOW_US) {
                ESP_LOGI(TAG, "AP window closed — standalone mode, radio off");
                ESP_ERROR_CHECK(esp_wifi_stop());
                ESP_ERROR_CHECK(esp_wifi_deinit());
                s_radio_off = true;
            }
        } else if (!g_sta_connect_allowed && g_have_sta_creds && !ap_hold_for_client &&
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

#if CONFIG_ESP_WIFI_ENABLE_ROAMING_APP
        // V2.5.34 reconnect safety-net: post-association (n_got_ip>0, so the
        // startup watchdog above is disarmed) we hand reconnects to the roaming
        // app. If it ever stalls — fails to bring the link back — nobody else
        // would, so after ROAM_RECONNECT_SAFETY_NET_US with no IP force one
        // esp_wifi_connect() ourselves and re-arm to retry. Cleared on GOT_IP.
        // The multi-minute window is deliberate: it lets the app own ordinary
        // roams/reconnects end to end (a router reboot alone can take 1-2 min
        // of its own retries to resolve) without this backstop racing it and
        // forcing a spurious disconnect mid-retry; it only fires on a genuine
        // stall (defence-in-depth for the experimental roaming app).
        if (g_sta_connect_allowed && n_got_ip > 0 && t_roam_defer_us > 0 &&
            (esp_timer_get_time() - t_roam_defer_us) > ROAM_RECONNECT_SAFETY_NET_US) {
            ESP_LOGW(TAG, "roaming app did not reconnect within %llds — forcing connect",
                     (long long)(ROAM_RECONNECT_SAFETY_NET_US / 1000000));
            mark_attempt();
            esp_wifi_connect();
            t_roam_defer_us = esp_timer_get_time();   // re-arm for the next interval
        }
#endif

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
            // V2.4.24: skip the scheduled TX cycle while an OTA upload is
            // in progress — frees WiFi airtime for the OTA POST instead
            // of competing with it via three TLS handshakes to Madavi /
            // sensor.community / Radmon. Non-sticky: as soon as
            // update_post returns (success or failure), the next main
            // tick fires the deferred TX cycle. We deliberately do NOT
            // advance next_tx in the skip case so the cycle fires
            // immediately on resume rather than after a fresh tx_interval.
            if (!main_ota_in_progress()) {
                do_tx_cycle();
                next_tx = xTaskGetTickCount() + tx_interval;
            }
        }

        {
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            periodic_loop(now_ms);  // V2.4.19: 24h PSA refresh + ARP (heap-guard moved to tx_run, V2.5.18)
            // V2.5.6: 60s CPM history sampler. V2.5.16: feed it the SAME
            // filtered-vs-raw decision as the per-cycle count so cpm5/cpm15
            // (GMC ACPM, ThingSpeak f3/f4) track the filtered CPM when the
            // width filter is on.
            history_tick(now_ms, g_cfg.pcnt_filter && tube_pcnt_active());
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
