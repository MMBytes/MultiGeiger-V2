/** @file
 *  @brief MQTT 3.1.1 publish-only client implementation (V2.4.2 Phase 1).
 *
 *  Wraps Espressif's `esp-mqtt` (component name `mqtt`, header
 *  `mqtt_client.h`). We use the simple synchronous-init / async-event
 *  flow: `esp_mqtt_client_init` + `esp_mqtt_client_start` once at boot,
 *  then a single ESP_EVENT_ANY_ID handler updates the connected flag and
 *  publishes the "online" availability message on each (re)connect.
 *
 *  Per `mqtt.h` the contract is non-blocking publish — if the broker
 *  isn't currently connected, `esp_mqtt_client_publish` returns -1 and we
 *  silently drop the message (with a debug log) rather than block the
 *  main task waiting for reconnect. Next TX cycle (~150 s later) tries
 *  again with a fresh sample.
 *
 *  Why no cJSON: the state payload is hand-rolled snprintf because the
 *  whole project deliberately avoids cJSON to keep flash use down — see
 *  CHANGELOG.md V2.3.x history for the pattern. The payload is small
 *  enough (~400 bytes max) that the manual approach is readable.
 */

#include "mqtt.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/ip4_addr.h"
#include "mqtt_client.h"

#include "als.h"
#include "log_ftp.h"
#include "mqtt_discovery.h"
#include "sysinfo.h"
#include "transmission.h"
#include "veml7700.h"
#include "version.h"

// Internal hook from mqtt_discovery.c — used at init to seed the
// tube-enabled predicate without exposing cfg through the discovery
// public API. Declared local so other modules can't accidentally use it.
extern void mqtt_discovery_set_tube_enabled(bool enabled);
#ifdef MQTT_RICH_STATE
extern void mqtt_discovery_set_upload_flags(bool madavi, bool sensorc, bool radmon,
                                            bool osm, bool aqi, bool ftp);
#endif

static const char *TAG = "mqtt";

// --- Module state -----------------------------------------------------------
//
// Singleton — we only ever connect to one broker per device. Holding the
// client handle here keeps mqtt.h's surface narrow (no opaque-pointer
// dance for callers).
static esp_mqtt_client_handle_t s_client = NULL;
static const char *s_chip_id  = NULL;      // pointer borrowed from main.c
static const char *s_prefix   = NULL;      // pointer borrowed from g_cfg
static volatile bool s_connected = false;
static volatile uint32_t s_publish_count = 0;
static bool s_ha_discovery_enabled = false;   // V2.4.3: cached cfg->mqtt_ha_discovery

// V2.4.26: cached upload-target enables so the rich-state JSON only emits
// per-target stat blocks for targets actually configured. Captured at init
// (and again on /config Save → mqtt_init re-entry path) so the publish hot
// path doesn't need to reach into the global cfg.
#ifdef MQTT_RICH_STATE
static bool s_send_madavi  = false;
static bool s_send_sensorc = false;
static bool s_send_radmon  = false;
static bool s_send_osm     = false;
static bool s_send_aqi     = false;
static bool s_ftp_enabled  = false;
#endif

// Future-V2.4.23: serialises s_client lifecycle against publish_state.
//
// The race: mqtt_publish_state() reads s_client, checks non-NULL + connected,
// then calls esp_mqtt_client_publish(s_client, ...). If mqtt_stop() runs on
// another task (httpd, during OTA teardown) between the check and the use,
// it calls esp_mqtt_client_destroy(s_client) and sets s_client=NULL — the
// publish then dereferences a destroyed handle (use-after-free). In practice
// the window is microseconds and the OTA teardown path is rare, so the race
// has never been observed — but it's not strictly synchronised.
//
// Fix: a single mutex serialises {init, stop, publish_state}. Created lazily
// in mqtt_init() — applog_init runs before mqtt_init in app_main so there's
// no ordering concern with our other early-boot mutexes. We do NOT take this
// mutex in mqtt_is_initialized / mqtt_is_connected / mqtt_publish_count
// because those are single-word reads, torn-tolerant on 32-bit Xtensa.
static SemaphoreHandle_t s_state_mux = NULL;
// V2.4.13: sticky "init was called" flag, independent of s_client. Survives
// the disabled-by-config no-op return so main.c's poll doesn't re-call init.
// Cleared by mqtt_stop() so the poll re-arms after the OTA teardown.
static bool s_initialized = false;

// Topic scratch — built once at init from prefix + chip_id, reused for
// every publish to avoid repeated snprintf cost in the hot path. Sized
// generously: prefix up to 31 + chip-id up to 19 + "/availability" = 13
// + NUL → 64 bytes is plenty.
#define TOPIC_BUF_SZ  96
static char s_topic_state[TOPIC_BUF_SZ];
static char s_topic_avail[TOPIC_BUF_SZ];

// --- Event handler ----------------------------------------------------------
//
// esp-mqtt callbacks run on a dedicated mqtt task — we keep work brief
// (set flag, single publish). No locking needed: s_connected is a
// single-byte bool, torn-tolerant on Xtensa.
static void on_mqtt_event(void *arg, esp_event_base_t base,
                          int32_t id, void *data) {
    (void)arg; (void)base;
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        ESP_LOGI(TAG, "CONNECTED to broker");
        // Flip availability to "online" (retained — HA shows the device
        // as available immediately, even before the first state publish).
        // Retain matters here: the LWT we registered is "offline" retained,
        // so a fresh subscriber connecting between our connect and our
        // first state publish would otherwise see the stale "offline".
        esp_mqtt_client_publish(s_client, s_topic_avail,
                                "online", 0, /*qos*/ 1, /*retain*/ 1);
        // V2.4.3: republish HA Discovery config payloads on every
        // (re)connect. They're retained on the broker so HA will pick them
        // up even if it joins later, AND so a wiped broker or broker
        // restart doesn't leave us in a state where HA never sees our
        // entities. Cheap — broker just overwrites identical retained
        // payloads at zero observable cost.
        if (s_ha_discovery_enabled) {
            mqtt_discovery_publish_all(s_client, s_chip_id, s_prefix);
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "DISCONNECTED");
        break;
    case MQTT_EVENT_ERROR:
        // esp-mqtt fills in `error_handle` with detail. We don't try to
        // decode every transport error — just log enough to diagnose
        // misconfig (wrong broker, bad creds) from /log.
        if (e && e->error_handle) {
            ESP_LOGW(TAG, "ERROR: type=%d connect_return=%d "
                          "esp_tls_last=0x%x esp_transport_sock_errno=%d",
                     e->error_handle->error_type,
                     e->error_handle->connect_return_code,
                     e->error_handle->esp_tls_last_esp_err,
                     e->error_handle->esp_transport_sock_errno);
        } else {
            ESP_LOGW(TAG, "ERROR: (no detail)");
        }
        break;
    default:
        // PUBLISHED / SUBSCRIBED / DATA / BEFORE_CONNECT — not interesting
        // for a publish-only client. Avoid log spam by ignoring.
        break;
    }
}

// --- Public API -------------------------------------------------------------

void mqtt_init(const config_t *cfg, const char *chip_id) {
    // Lazy mutex creation. Runs once on first mqtt_init (cold boot)
    // or after a deliberate mqtt_stop (re-init never destroys the mux).
    if (!s_state_mux) {
        s_state_mux = xSemaphoreCreateMutex();
        if (!s_state_mux) {
            ESP_LOGE(TAG, "xSemaphoreCreateMutex failed — TOCTOU race not guarded");
            // Fall through — pre-mux behaviour is what we had before;
            // failing init entirely would be worse than carrying on.
        }
    }
    if (s_client) {
        ESP_LOGW(TAG, "init called twice — ignoring");
        return;
    }
    // V2.4.13: set the sticky flag FIRST so disabled-by-config branches
    // below still register as "initialized" — keeps main.c's poll from
    // calling init in a loop when MQTT is intentionally off.
    s_initialized = true;
    if (!cfg->mqtt_enable) {
        ESP_LOGI(TAG, "disabled (mqtt_enable=false)");
        return;
    }
    if (cfg->mqtt_broker[0] == 0) {
        ESP_LOGI(TAG, "disabled (broker empty)");
        return;
    }

    s_chip_id = chip_id;
    s_prefix  = cfg->mqtt_topic_prefix;
    s_ha_discovery_enabled = cfg->mqtt_ha_discovery;
    // Seed the tube-enabled predicate in mqtt_discovery so its entity
    // gate function can run without reaching into main.c's cfg singleton.
    mqtt_discovery_set_tube_enabled(cfg->tube_enabled);
#ifdef MQTT_RICH_STATE
    // V2.4.26: cache upload-target enables for the rich-state JSON. Same
    // pattern as tube_enabled — pulled out of cfg here so the publish hot
    // path doesn't need to reach into main.c's cfg singleton, and so the
    // /config Save path's mqtt_stop + mqtt_init cycle re-reads them.
    s_send_madavi  = cfg->send_madavi;
    s_send_sensorc = cfg->send_sensorc;
    s_send_radmon  = cfg->send_radmon;
    s_send_osm     = cfg->send_osm;
    s_send_aqi     = cfg->send_aqi;
    s_ftp_enabled  = cfg->ftp_enabled;
    mqtt_discovery_set_upload_flags(s_send_madavi, s_send_sensorc, s_send_radmon,
                                    s_send_osm, s_send_aqi, s_ftp_enabled);
#endif

    // Pre-build the per-device topic strings so the publish path doesn't
    // re-snprintf every cycle. snprintf return ignored — the buffers are
    // sized for the documented field maxes (CFG_MQTT_PFX_MAX=31 +
    // chip-id 19 + suffix), can't overflow.
    snprintf(s_topic_state, sizeof(s_topic_state),
             "%s/%s/state", s_prefix, s_chip_id);
    snprintf(s_topic_avail, sizeof(s_topic_avail),
             "%s/%s/availability", s_prefix, s_chip_id);

    // Build the broker URI. esp-mqtt accepts "mqtt://host:port" for plain
    // TCP and "mqtts://host:port" for TLS — the scheme alone selects the
    // transport. V2.4.6 adds the TLS path with three configurable trust modes
    // (see config_fields.def for the enum).
    const bool tls = cfg->mqtt_tls_enable;
    char uri[16 + CFG_MQTT_HOST_MAX + 1 + 6];   // "mqtts://" + host + ":" + port
    snprintf(uri, sizeof(uri), "%s%s:%" PRIu32,
             tls ? "mqtts://" : "mqtt://",
             cfg->mqtt_broker, cfg->mqtt_port);

    esp_mqtt_client_config_t mc = { 0 };
    mc.broker.address.uri               = uri;
    mc.session.keepalive                = 60;       // seconds
    mc.session.disable_clean_session    = false;    // start fresh each session
    // V2.4.30: slow esp-mqtt's blind auto-reconnect from the 10 s default to
    // 30 s. During a WiFi outage (router reboot) the client otherwise fires a
    // fresh TCP+TLS connect every ~10-15 s — each fails fast with
    // ENETUNREACH but still builds/tears an esp-tls context, churning heap
    // (negligible on PSRAM boards, real fragmentation pressure on the Heltec
    // V2's tight heap — see [[reference_heltec_v2_heap_oom_and_ota_teardown]]).
    // Recovery latency is NOT affected: mqtt_kick_reconnect() forces an
    // immediate reconnect from main.c's GOT_IP handler the instant STA
    // re-associates, so this slower timer only governs the broker-down-but-
    // WiFi-up case (e.g. broker host reboot), where 30 s is fine.
    mc.network.reconnect_timeout_ms     = 30000;
    mc.session.last_will.topic          = s_topic_avail;
    mc.session.last_will.msg            = "offline";
    mc.session.last_will.msg_len        = 7;
    mc.session.last_will.qos            = 1;
    mc.session.last_will.retain         = 1;
    if (cfg->mqtt_user[0] != 0) {
        mc.credentials.username                  = cfg->mqtt_user;
        mc.credentials.authentication.password   = cfg->mqtt_password;
    }

    // V2.4.6: TLS trust-mode wiring. Only consulted when tls_enable is on.
    // Modes — see config_fields.def for the canonical descriptions:
    //   0 = A  Mozilla CA bundle (the same bundle already used for HTTPS
    //          uploads; covers public CAs like Let's Encrypt)
    //   1 = B  custom CA cert pasted by user (typical home Mosquitto with
    //          a self-signed CA)
    //   2 = D  skip server verification (TLS encrypts, doesn't authenticate
    //          — for trusted-LAN deployments)
    const char *tls_mode_str = "n/a";
    if (tls) {
        switch (cfg->mqtt_tls_mode) {
            case 0:  // Mode A — Mozilla CA bundle
                mc.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
                tls_mode_str = "A (Mozilla CA bundle)";
                break;
            case 1:  // Mode B — custom CA cert from NVS
                if (cfg->mqtt_tls_ca[0] == 0) {
                    ESP_LOGW(TAG, "TLS Mode B selected but mqtt_tls_ca is empty — "
                                  "falling back to skip-verify so connect doesn't loop");
                    mc.broker.verification.skip_cert_common_name_check = true;
                    tls_mode_str = "B (no CA configured — degraded to skip-verify)";
                } else {
                    mc.broker.verification.certificate = cfg->mqtt_tls_ca;
                    // certificate_len = 0 → esp-tls treats input as NUL-terminated
                    // string and infers length, which is what we want for a PEM.
                    tls_mode_str = "B (custom CA cert)";
                }
                break;
            case 2:  // Mode D — skip server verification
                mc.broker.verification.skip_cert_common_name_check = true;
                tls_mode_str = "D (skip verification)";
                break;
            default:
                // Schema bounds the field at [0,2] so we should never get here.
                // Defensive: fall through to Mode A so connect succeeds.
                ESP_LOGW(TAG, "unknown mqtt_tls_mode=%" PRIu32 " — using Mode A",
                         cfg->mqtt_tls_mode);
                mc.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
                tls_mode_str = "A (fallback from unknown mode)";
                break;
        }
    }
    // Client ID — broker enforces uniqueness, so embed the chip id. Mosquitto
    // and most brokers will boot a duplicate client ID, which would cause a
    // permanent flap if two devices accidentally shared an ID.
    char client_id[32];
    snprintf(client_id, sizeof(client_id), "geiger_%s", s_chip_id);
    mc.credentials.client_id            = client_id;

    // Take the state mux around s_client assignment so a concurrent
    // publish_state on another task can't read a half-initialised client.
    if (s_state_mux) xSemaphoreTake(s_state_mux, portMAX_DELAY);
    s_client = esp_mqtt_client_init(&mc);
    if (!s_client) {
        if (s_state_mux) xSemaphoreGive(s_state_mux);
        ESP_LOGE(TAG, "esp_mqtt_client_init failed");
        return;
    }

    esp_err_t r = esp_mqtt_client_register_event(
        s_client, ESP_EVENT_ANY_ID, on_mqtt_event, NULL);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "register_event failed: %s", esp_err_to_name(r));
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        if (s_state_mux) xSemaphoreGive(s_state_mux);
        return;
    }

    r = esp_mqtt_client_start(s_client);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "client_start failed: %s", esp_err_to_name(r));
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        if (s_state_mux) xSemaphoreGive(s_state_mux);
        return;
    }
    if (s_state_mux) xSemaphoreGive(s_state_mux);

    ESP_LOGI(TAG, "started — uri=%s tls_mode=%s state=%s avail=%s",
             uri, tls_mode_str, s_topic_state, s_topic_avail);
}

// --- State publish ----------------------------------------------------------
//
// Build payload like:
//   {"v":"V2.4.2","uptime_ms":12345,"cycles":42,"cpm":18,"usvph":0.115,
//    "rssi":-58,"env_t":22.31,"env_h":58.14,"env_p":101325.0,
//    "pm1":2.1,"pm25":4.5,"pm10":5.2,"nc25":3.1,
//    "noise_laeq":42.3,"lux":350.2}
//
// Field naming follows HA Discovery's natural attribute names (pm1, pm25,
// pm10) so the Phase 2 discovery payloads reference value_template like
// `{{ value_json.pm25 }}` without any per-field aliasing.
//
// Fields absent for sensors not present — e.g. on the Heltec V2 Geiger-only
// build, the payload contains just radiation + RSSI + uptime + cycles.
// Bytes saved that way matter because MQTT brokers charge airtime to the
// keepalive budget and WiFi cycle.
void mqtt_publish_state(const main_status_t *st,
                        bool pm_valid, const pm_sample_t *pm,
                        bool noise_valid, const noise_sample_t *noise) {
    // Hold s_state_mux from the s_client check all the way through the
    // esp_mqtt_client_publish call. Keeps mqtt_stop() (called by OTA
    // teardown on the httpd task) from destroying the handle between
    // our NULL-check and the publish — the original TOCTOU described
    // in the V2.4.22 audit. esp_mqtt_client_publish is a non-blocking
    // enqueue per IDF docs (returns immediately, internal task does
    // the network I/O) so holding a mutex across it is safe.
    if (s_state_mux) xSemaphoreTake(s_state_mux, portMAX_DELAY);
    if (!s_client || !s_connected) {
        ESP_LOGD(TAG, "publish skipped (client=%p connected=%d)",
                 s_client, s_connected);
        if (s_state_mux) xSemaphoreGive(s_state_mux);
        return;
    }

    // V2.4.26 buffer sizing:
    //   - Base 768 B covers all sensor blocks + the always-on system block
    //     (worst case ~365 B on Heltec, ~580 B on FeatherS3-D PM+DNMS).
    //   - +512 B under MQTT_RICH_STATE for the per-target upload stats
    //     (5 targets × 4 fields ≈ 315 B + FTPS ≈ 60 B + slack).
    // Stack-allocated — cycle task has 4 KB+ stack per existing config, so
    // 1280 B on PSRAM boards / 768 B on Heltec both fit with headroom.
#ifdef MQTT_RICH_STATE
    char buf[1280];
#else
    char buf[768];
#endif
    int n = 0;
    int rem;
#define APPEND(...)                                                  \
    do {                                                             \
        rem = (int)sizeof(buf) - n;                                  \
        if (rem <= 1) break;                                         \
        int _w = snprintf(buf + n, rem, __VA_ARGS__);                \
        if (_w < 0) break;                                           \
        n += (_w < rem) ? _w : (rem - 1);                            \
    } while (0)

    APPEND("{\"v\":\"%s\"", VERSION_STR);
    APPEND(",\"uptime_ms\":%" PRIu32, st->last_cycle_ms);
    APPEND(",\"cycles\":%" PRIu32,    st->cycles);
    APPEND(",\"reconnects\":%" PRIu32, st->reconnects);
    APPEND(",\"i2c_err\":%" PRIu32,   st->i2c_errors);   // V2.4.28

    // Radiation block — only when there's a real reading. Suppressing
    // the keys (rather than emitting zero) lets HA show the entity as
    // unavailable instead of misleading 0 CPM.
    if (st->last_cpm > 0 || st->last_usvph > 0.0f) {
        APPEND(",\"cpm\":%" PRIu32,   st->last_cpm);
        APPEND(",\"usvph\":%.4f",     st->last_usvph);
        APPEND(",\"hv_pulses\":%" PRIu32, st->last_hv_pulses);
        APPEND(",\"hv_err\":%s",      st->last_hv_error ? "true" : "false");
    }

    // Environment (BME280 / SHT45+BMP581 / etc.)
    // V2.4.12: per-field gating — pre-V2.4.12 used the single have_env
    // flag for all three, which made SHT45-only setups publish env_p=0.0
    // every cycle (SHT45 has no pressure channel). HA then showed a
    // phantom 0.00 hPa entity.
    if (st->have_env_t) APPEND(",\"env_t\":%.2f", st->env_t);
    if (st->have_env_h) APPEND(",\"env_h\":%.2f", st->env_h);
    if (st->have_env_p) APPEND(",\"env_p\":%.1f", st->env_p);    // Pa

    // Particulate matter — sample is per-cycle, passed by caller
    if (pm_valid && pm) {
        APPEND(",\"pm1\":%.2f",    pm->pm1_0);
        APPEND(",\"pm25\":%.2f",   pm->pm2_5);
        APPEND(",\"pm4\":%.2f",    pm->pm4_0);
        APPEND(",\"pm10\":%.2f",   pm->pm10);
        APPEND(",\"nc05\":%.2f",   pm->nc0_5);
        APPEND(",\"nc1\":%.2f",    pm->nc1_0);
        APPEND(",\"nc25\":%.2f",   pm->nc2_5);
        APPEND(",\"nc4\":%.2f",    pm->nc4_0);
        APPEND(",\"nc10\":%.2f",   pm->nc10);
        APPEND(",\"ps_typ\":%.2f", pm->typ_size_um);
    }

    // Noise
    if (noise_valid && noise) {
        APPEND(",\"noise_laeq\":%.1f", noise->laeq);
        APPEND(",\"noise_min\":%.1f",  noise->la_min);
        APPEND(",\"noise_max\":%.1f",  noise->la_max);
    }

    // Ambient light — read live (cheap: one ADC sample, ~ms). Two
    // independent ALS implementations may coexist on FeatherS3-D, prefer
    // the calibrated VEML7700 when present; fall back to onboard ALS-PT19.
    float lux = 0.0f;
    bool have_lux = false;
    if (veml7700_present() && veml7700_read(NULL, NULL, &lux) == ESP_OK) {
        have_lux = true;
    } else if (als_present() && als_read(NULL, NULL, &lux) == ESP_OK) {
        have_lux = true;
    }
    if (have_lux) {
        APPEND(",\"lux\":%.1f", lux);
    }

    // --- V2.4.26: system stats (all boards) -------------------------------
    // RSSI + IP from the WiFi STA netif. Both ~free to read (cached by the
    // wifi/netif drivers). IP is emitted as a string for HA-readability.
    {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            APPEND(",\"rssi\":%d", (int)ap.rssi);
        }
        esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (sta) {
            esp_netif_ip_info_t ip = { 0 };
            if (esp_netif_get_ip_info(sta, &ip) == ESP_OK && ip.ip.addr != 0) {
                char ip_s[16];
                esp_ip4addr_ntoa(&ip.ip, ip_s, sizeof(ip_s));
                APPEND(",\"ip\":\"%s\"", ip_s);
            }
        }
        APPEND(",\"heap_free\":%" PRIu32,      esp_get_free_heap_size());
        APPEND(",\"heap_min\":%" PRIu32,       esp_get_minimum_free_heap_size());
        APPEND(",\"heap_max_alloc\":%" PRIu32, (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        APPEND(",\"reset_reason\":\"%s\"",     reset_reason_str(esp_reset_reason()));
    }

#ifdef MQTT_RICH_STATE
    // --- V2.4.26: per-target upload stats (PSRAM boards only) -------------
    // Mirror of the /status page's Uploads block — same accessors, same
    // gating on cfg.send_*. Per-target keys use the same short names used
    // by HA Discovery so val_tpl resolves trivially.
    static const struct {
        tx_target_id_t  id;
        const bool     *enabled;
        const char     *key;     // JSON field prefix, e.g. "madavi"
    } UL_TARGETS[] = {
        { TX_TARGET_MADAVI,  &s_send_madavi,  "madavi" },
        { TX_TARGET_SENSORC, &s_send_sensorc, "sc"     },
        { TX_TARGET_RADMON,  &s_send_radmon,  "radmon" },
        { TX_TARGET_OSM,     &s_send_osm,     "osm"    },
        { TX_TARGET_AQI,     &s_send_aqi,     "aqi"    },
    };
    for (size_t i = 0; i < sizeof(UL_TARGETS)/sizeof(UL_TARGETS[0]); i++) {
        if (!*UL_TARGETS[i].enabled) continue;
        tx_target_stats_t s;
        tx_get_stats(UL_TARGETS[i].id, &s);
        const char *k = UL_TARGETS[i].key;
        APPEND(",\"%s_ok\":%" PRIu32,  k, s.succeeded);
        APPEND(",\"%s_att\":%" PRIu32, k, s.attempted);
        APPEND(",\"%s_rc\":%d",        k, s.last_rc);
        APPEND(",\"%s_breaker\":%d",   k, s.breaker_open_cycles);
    }
    if (s_ftp_enabled) {
        log_ftp_stats_t f;
        log_ftp_get_stats(&f);
        APPEND(",\"ftp_ok\":%s",       f.have_last && f.last_ok ? "true" : "false");
        APPEND(",\"ftp_bytes\":%" PRIu32, f.last_bytes);
        int64_t age_s = 0;
        if (f.have_last && f.last_at > 0) {
            time_t now = time(NULL);
            if (now > (time_t)f.last_at) age_s = (int64_t)now - f.last_at;
        }
        APPEND(",\"ftp_age_s\":%" PRId64, age_s);
    }
#endif

    APPEND("}");
#undef APPEND

    int msg_id = esp_mqtt_client_publish(s_client, s_topic_state,
                                         buf, n, /*qos*/ 0, /*retain*/ 0);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "publish failed (msg_id=%d, bytes=%d)", msg_id, n);
    } else {
        s_publish_count++;
        ESP_LOGI(TAG, "publish ok: %d bytes (#%" PRIu32 ")", n, s_publish_count);
    }
    if (s_state_mux) xSemaphoreGive(s_state_mux);
}

bool mqtt_is_connected(void) {
    return s_connected;
}

uint32_t mqtt_publish_count(void) {
    return s_publish_count;
}

// V2.4.30: force an immediate reconnect attempt. Called from main.c's GOT_IP
// handler so a WiFi re-association (router reboot, channel hop, roam)
// reconnects MQTT promptly instead of idling up to reconnect_timeout_ms (30 s)
// for esp-mqtt's internal retry timer to fire. Without this, after a router
// reboot STA can hold a valid IP for ~10 s while the broker connection sits
// dormant (observed 2026-05-29: GOT_IP at 00:18:59, MQTT reconnect at 00:19:10).
//
// esp_mqtt_client_reconnect() returns ESP_FAIL when the client is already
// connected/connecting (its "invalid state") — exactly when we want to do
// nothing — so we gate on s_connected and don't bother checking the return.
// Mux-guarded against mqtt_stop()'s teardown for the same TOCTOU reason as
// mqtt_publish_state (s_client could be destroyed between the check and use).
void mqtt_kick_reconnect(void) {
    if (s_state_mux) xSemaphoreTake(s_state_mux, portMAX_DELAY);
    if (s_client && !s_connected) {
        esp_mqtt_client_reconnect(s_client);
    }
    if (s_state_mux) xSemaphoreGive(s_state_mux);
}

// V2.4.13: tear down the client to free TLS state — called by /update POST
// handler before the OTA receive loop. Heap freed ≈ 18-25 KB on the Heltec
// V2 (mbedTLS context + session ticket + read/write bufs); negligible on
// FeatherS3-D, where heap was never tight to begin with. Idempotent.
void mqtt_stop(void) {
    // Take s_state_mux for the entire teardown so a concurrent publish_state
    // (main task during a TX cycle) finishes its in-flight enqueue before
    // we destroy the handle. See the s_state_mux declaration above for the
    // race we're guarding against.
    if (s_state_mux) xSemaphoreTake(s_state_mux, portMAX_DELAY);
    if (!s_client) {
        // Either never started (disabled by config) or already stopped.
        // Reset the sticky flag anyway so main.c's poll will retry init —
        // covers the "OTA failed after teardown, want MQTT back" case.
        s_initialized = false;
        s_connected   = false;
        ESP_LOGI(TAG, "stop: no client to stop (already idle)");
        if (s_state_mux) xSemaphoreGive(s_state_mux);
        return;
    }
    ESP_LOGI(TAG, "stopping client to free TLS state");
    // esp_mqtt_client_stop sends a graceful DISCONNECT to the broker and
    // joins the internal task. esp_mqtt_client_destroy frees the mbedTLS
    // session, allocators, and the client struct itself.
    esp_mqtt_client_stop(s_client);
    esp_mqtt_client_destroy(s_client);
    s_client      = NULL;
    s_connected   = false;
    s_initialized = false;
    ESP_LOGI(TAG, "stop: client destroyed");
    if (s_state_mux) xSemaphoreGive(s_state_mux);
}

bool mqtt_is_initialized(void) {
    return s_initialized;
}
