#include "transmission.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_tls.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// V2.3.21: drive the OLED status-line slots for sensor.community, Madavi
// and Radmon (V1 parity — see reference V1 transmission.cpp). OSM and
// aqi.eco intentionally don't get OLED slots; their status is fully
// visible on the / status page from V2.3.18.
#include "display.h"

static const char *TAG = "tx";

// Per-target stats surfaced on /. Updated only from the TX worker task; read
// from the HTTP server task. See transmission.h for field semantics.
//
// V2.4.1 (B1): `last_at` is an int64_t. On the 32-bit Xtensa LX6/LX7 cores
// a naive int64_t store is two 32-bit stores, so a reader on a different
// task can see torn high/low halves (manifest: momentary year-2038-ish
// timestamp on the status page around each TX cycle's record_attempt).
// Protect every read AND write of s_stats[] with a single spinlock — also
// gives the reader a consistent snapshot of attempted/succeeded/last_rc/
// last_at as written by ONE record event, instead of a mix from two
// successive events. Spinlock is ~10ns per access vs the seconds-scale
// network operations nearby — cost is negligible.
static tx_target_stats_t s_stats[TX_TARGET_COUNT] = {0};
static portMUX_TYPE      s_stats_mux = portMUX_INITIALIZER_UNLOCKED;
static const char *s_target_names[TX_TARGET_COUNT] = {
    "Madavi", "sensor.community", "Radmon", "openSenseMap", "aqi.eco"
};

const char *tx_target_name(tx_target_id_t id) {
    if (id < 0 || id >= TX_TARGET_COUNT) return "?";
    return s_target_names[id];
}

// V2.4.1 (C9): static URL table for the three "fixed-URL" upload targets.
// Pre-V2.4.1 these literals lived inline in main.c::build_tx_context, which
// leaked endpoint knowledge to the wrong layer. OSM and aqi.eco are absent
// because their URLs are per-box / per-account and built dynamically inside
// send_osm() / send_aqi() in this file.
static const struct {
    const char *url_http;
    const char *url_https;
} s_target_urls[TX_TARGET_COUNT] = {
    [TX_TARGET_MADAVI]  = { "http://api-rrd.madavi.de/data.php",
                            "https://api-rrd.madavi.de/data.php" },
    [TX_TARGET_SENSORC] = { "http://api.sensor.community/v1/push-sensor-data/",
                            "https://api.sensor.community/v1/push-sensor-data/" },
    [TX_TARGET_RADMON]  = { "http://radmon.org/radmon.php",
                            "https://radmon.org/radmon.php" },
    [TX_TARGET_OSM]     = { NULL, NULL },   // dynamic — see send_osm()
    [TX_TARGET_AQI]     = { NULL, NULL },   // dynamic — see send_aqi()
};

void tx_target_configure(tx_target_t *out, tx_target_id_t id,
                         bool enabled, bool use_https) {
    if (!out) return;
    if (id < 0 || id >= TX_TARGET_COUNT) return;
    out->enabled      = enabled;
    out->use_https    = use_https;
    out->use_insecure = false;
    out->url_http     = s_target_urls[id].url_http;
    out->url_https    = s_target_urls[id].url_https;
}

void tx_get_stats(tx_target_id_t id, tx_target_stats_t *out) {
    if (!out || id < 0 || id >= TX_TARGET_COUNT) {
        if (out) memset(out, 0, sizeof(*out));
        return;
    }
    portENTER_CRITICAL(&s_stats_mux);
    *out = s_stats[id];
    portEXIT_CRITICAL(&s_stats_mux);
}

// V2.4.1 (C6): single record_outcome replaces the V2.3.x
// record_attempt + record_success pair. All field updates land under one
// spinlock acquire, so a concurrent reader can't see attempted++ but
// not-yet-succeeded++ from the same call — fixes a 1-cycle inconsistency
// window that was always present pre-V2.4.1. `ok` should be true iff the
// caller's target-specific success-code check (200 / 201 / both) matched.
static void record_outcome(tx_target_id_t id, int rc, bool ok) {
    portENTER_CRITICAL(&s_stats_mux);
    s_stats[id].attempted++;
    if (ok) s_stats[id].succeeded++;
    s_stats[id].last_rc = rc;
    s_stats[id].last_at = (int64_t)time(NULL);
    portEXIT_CRITICAL(&s_stats_mux);
}

// TX runs on its own CPU1-pinned task so mbedTLS handshakes don't starve
// the CPU0 idle task (which feeds the task watchdog).
#define TX_TASK_STACK_BYTES  16384   // V2.3.22: bumped from 10240. Safety margin for TLS 1.3 handshake stack usage (mbedTLS 4.x cert parsing + key derivation are deeper than 1.2 was).
#define TX_TASK_PRIO         (tskIDLE_PRIORITY + 1)
#define TX_QUEUE_DEPTH       1

static volatile bool s_tx_busy = false;

static QueueHandle_t s_tx_queue;

#define HTTP_TIMEOUT_MS     15000
#define HTTP_MAX_RETRIES    4
#define RESP_BUF_SIZE       512

// Small user-event handler that captures response body for Radmon "OK" check.
typedef struct {
    char   *buf;
    size_t  cap;
    size_t  len;
} resp_ctx_t;

static esp_err_t http_event_cb(esp_http_client_event_t *e) {
    if (e->event_id != HTTP_EVENT_ON_DATA) return ESP_OK;
    resp_ctx_t *r = (resp_ctx_t *)e->user_data;
    if (!r) return ESP_OK;
    size_t n = (r->len + e->data_len < r->cap) ? e->data_len : (r->cap - 1 - r->len);
    if (n > 0) {
        memcpy(r->buf + r->len, e->data, n);
        r->len += n;
        r->buf[r->len] = 0;
    }
    return ESP_OK;
}

// Returns HTTP status code, or -1 on transport error.
//
// V2.3.16 history: temporarily replaced with a cached_client pattern in pre2
// (long-lived handles per target with save_client_session=true). Bench-tested
// 2026-05-10 — the approach didn't pay off in practice because cloud servers
// close idle keep-alive connections faster than our 150 s cycle interval, and
// esp_http_client doesn't actually use the saved TLS session ticket on the
// reconnect (cert validation fires on every retry, indicating a fresh ECDHE
// handshake). Net effect was 2.8× slower cycles (10 s → 28 s) and identical
// PSA pressure. Reverted before V2.3.16 ship; per-cycle init+cleanup is the
// stable baseline. PSA leak mitigation continues via V2.3.15's defensive
// infrastructure (slot bump, nuclear PSA reset on consecutive OOMs and on
// FTPS write stalls).
static int do_request(const char *url, esp_http_client_method_t method,
                      const char *content_type, const char *x_sensor,
                      const char *x_pin, const char *body,
                      resp_ctx_t *resp, bool insecure) {
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = HTTP_TIMEOUT_MS,
        .method = method,
        .event_handler = http_event_cb,
        .user_data = resp,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
    };
    if (strncmp(url, "https", 5) == 0) {
        cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
        if (!insecure) {
            cfg.crt_bundle_attach = esp_crt_bundle_attach;
        } else {
            cfg.skip_cert_common_name_check = true;
        }
    }
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE(TAG, "http_client_init failed for %s", url);
        return -1;
    }
    if (content_type) esp_http_client_set_header(client, "Content-Type", content_type);
    if (x_sensor)     esp_http_client_set_header(client, "X-Sensor", x_sensor);
    if (x_pin)        esp_http_client_set_header(client, "X-PIN", x_pin);
    esp_http_client_set_header(client, "Connection", "close");
    if (body) esp_http_client_set_post_field(client, body, strlen(body));

    esp_err_t err = esp_http_client_perform(client);
    int status = (err == ESP_OK) ? esp_http_client_get_status_code(client) : -1;
    if (err != ESP_OK) {
        // Truncate at '?' to avoid logging query-string credentials.
        const char *q = strchr(url, '?');
        int url_len = q ? (int)(q - url) : (int)strlen(url);
        ESP_LOGW(TAG, "perform error: %s (%.*s)", esp_err_to_name(err), url_len, url);
    }
    esp_http_client_cleanup(client);
    return status;
}

static bool wifi_up(void) {
    wifi_ap_record_t ap;
    return esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
}

static void tx_run(tx_context_t *c);

static void tx_task(void *arg) {
    (void)arg;
    tx_context_t ctx;
    while (1) {
        if (xQueueReceive(s_tx_queue, &ctx, portMAX_DELAY) == pdTRUE) {
            s_tx_busy = true;
            tx_run(&ctx);
            s_tx_busy = false;
        }
    }
}

void tx_setup(void) {
    s_tx_queue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(tx_context_t));
    configASSERT(s_tx_queue);
    BaseType_t ok = xTaskCreatePinnedToCore(
        tx_task, "tx", TX_TASK_STACK_BYTES, NULL, TX_TASK_PRIO, NULL, 1);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "transmission ready (cert bundle = esp_crt_bundle_attach, worker on CPU1)");
}

// --- Keep-alive push helpers (shared by Madavi + sensor.community) ----------

// Perform a POST on an already-initialised client. Updates X-PIN (if provided)
// + body; leaves URL, Content-Type, X-Sensor and the cert bundle in place from
// the first call. Pass x_pin=NULL for targets that don't use it (Madavi).
static int do_request_on_client(esp_http_client_handle_t client,
                                const char *x_pin, const char *body) {
    if (x_pin) esp_http_client_set_header(client, "X-PIN", x_pin);
    esp_http_client_set_post_field(client, body, strlen(body));
    esp_err_t err = esp_http_client_perform(client);
    int status = (err == ESP_OK) ? esp_http_client_get_status_code(client) : -1;
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "perform error: %s", esp_err_to_name(err));
    }
    return status;
}

static int post_with_retry(esp_http_client_handle_t client,
                           const char *target, const char *label,
                           const char *x_pin, const char *body) {
    for (int i = 0; i < HTTP_MAX_RETRIES; i++) {
        if (!wifi_up()) return -2;
        int rc = do_request_on_client(client, x_pin, body);
        if (rc > 0 && rc != 408 && rc < 500) return rc;
        ESP_LOGW(TAG, "%s[%s] rc=%d (retry %d/%d)",
                 target, label, rc, i + 1, HTTP_MAX_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ESP_LOGW(TAG, "%s[%s]: all retries exhausted", target, label);
    return -1;
}

// Open a keep-alive HTTP(S) client for a sensor-push target. Content-Type
// and X-Sensor are set here; per-POST headers (X-PIN) are set by caller
// between perform() calls. No "Connection: close" — keeping the socket alive
// lets a second POST reuse the TLS session within ONE CYCLE (~3-5 s saving
// for the second/third/fourth POST; first POST always pays handshake cost).
//
// V2.3.16 history: was briefly replaced by a long-lived cached client across
// cycles. Reverted — see do_request() comment for the bench-test analysis.
static esp_http_client_handle_t open_push_client(const char *url, bool use_insecure,
                                                 const char *chip_id) {
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = HTTP_TIMEOUT_MS,
        .method = HTTP_METHOD_POST,
        .event_handler = http_event_cb,
        .user_data = NULL,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
        .keep_alive_enable = true,
    };
    if (strncmp(url, "https", 5) == 0) {
        cfg.transport_type = HTTP_TRANSPORT_OVER_SSL;
        if (!use_insecure) {
            cfg.crt_bundle_attach = esp_crt_bundle_attach;
        } else {
            cfg.skip_cert_common_name_check = true;
        }
    }
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return NULL;
    esp_http_client_set_header(client, "Content-Type", "application/json; charset=UTF-8");
    esp_http_client_set_header(client, "X-Sensor", chip_id);
    return client;
}

// --- Madavi -----------------------------------------------------------------
// Up to two POSTs over one keep-alive TLS session:
//   1. Geiger body (Si22G_* + signal) — only when tube_enabled.
//   2. Environmental body — only when env or PM sensor data is available.
//      Combines BME280_* (T/H/P), SPS30_* (PM mass + number conc + size),
//      and the Geiger pulse stats (samples / min_micro / max_micro) on the
//      back of which the Madavi side writes the pulse-stats RRDs.
// Madavi routes by field-name prefix (not X-PIN), so combining BME + PM in
// one body is the canonical form (matches dusty-code's approach).

static void build_madavi_geiger_body(const tx_context_t *c, char *buf, size_t cap) {
    snprintf(buf, cap,
        "{\n"
        " \"software_version\": \"%s\",\n"
        " \"sensordatavalues\": [\n"
        "  {\"value_type\": \"Si22G_counts_per_minute\", \"value\": \"%lu\"},\n"
        "  {\"value_type\": \"Si22G_hv_pulses\", \"value\": \"%lu\"},\n"
        "  {\"value_type\": \"Si22G_counts\", \"value\": \"%lu\"},\n"
        "  {\"value_type\": \"Si22G_sample_time_ms\", \"value\": \"%lu\"},\n"
        "  {\"value_type\": \"signal\", \"value\": \"%d\"}\n"
        " ]\n}",
        c->sw_version,
        (unsigned long)c->cpm, (unsigned long)c->hv_pulses,
        (unsigned long)c->gm_counts, (unsigned long)c->dt_ms,
        (int)c->rssi);
}

// Build the Madavi "environmental" body (renamed from thp). Combines every
// non-radiation source into one POST, plus pulse-stats and signal at the end.
// Caller must ensure at least one of bme_valid / pm_valid / pulse-stats is
// present; an empty body is harmless but wastes a POST.
static void build_madavi_env_body(const tx_context_t *c, char *buf, size_t cap) {
    bool have_pulse_stats = c->tube_enabled && (c->gm_counts > 1);
    int n = snprintf(buf, cap,
        "{\n"
        " \"software_version\": \"%s\",\n"
        " \"sensordatavalues\": [\n",
        c->sw_version);

    bool first = true;
    #define COMMA() do { if (!first) n += snprintf(buf + n, cap - n, ",\n"); first = false; } while (0)

    if (c->bme_valid) {
        // V2.3.29 fix: bme_valid is set when ANY sub-sensor in the env
        // cascade reports a reading. With only an SHT45 fitted (no Bosch
        // pressure chip), bme_pressure_pa stays at its zero-init value
        // and the previous unconditional emit sent "BME280_pressure: 0.00"
        // — Madavi accepted it but graphed a 0 Pa flatline forever.
        //
        // Sentinel-based field selection picks the right Madavi field
        // names based on which actual fields have data:
        //   - have_p (>1 hPa)  → full Bosch trio "BME280_*" → Madavi
        //                         routes to data-{ID}-bme280-highres.rrd
        //   - !have_p, have_h  → DHT-style "temperature"/"humidity" →
        //                         routes to data-{ID}-dht-highres.rrd
        //                         (slight semantic lie — graph labels as
        //                         "DHT" — but values are correct)
        //   - T only           → "temperature" only (e.g. broken H read)
        // Atmospheric pressure on Earth never goes below 1 hPa even at
        // the edge of space, so 100 Pa is a safe "no sensor" sentinel.
        // Humidity 0.01 % RH never occurs in practice; default 0 means
        // sensor absent. Bug only affects Madavi here — sensor.community,
        // OSM, aqi.eco have their own body builders that need separate
        // fixes if the same problem matters for them.
        bool have_p = (c->bme_pressure_pa > 100.0f);
        bool have_h = (c->bme_humidity_pct > 0.01f);

        COMMA();
        if (have_p) {
            // Full Bosch-family trio (BME280, BMP581, BMP390, BME688
            // — all relabelled to BME280_* since that's what Madavi's
            // hardcoded value_type whitelist recognises).
            n += snprintf(buf + n, cap - n,
                "  {\"value_type\": \"BME280_temperature\", \"value\": \"%.2f\"},\n"
                "  {\"value_type\": \"BME280_humidity\", \"value\": \"%.2f\"},\n"
                "  {\"value_type\": \"BME280_pressure\", \"value\": \"%.2f\"}",
                c->bme_temperature_c, c->bme_humidity_pct, c->bme_pressure_pa);
        } else if (have_h) {
            // SHT45 (or similar T+H-only) without a paired Bosch chip.
            // Use unprefixed names → Madavi's DHT RRD.
            n += snprintf(buf + n, cap - n,
                "  {\"value_type\": \"temperature\", \"value\": \"%.2f\"},\n"
                "  {\"value_type\": \"humidity\", \"value\": \"%.2f\"}",
                c->bme_temperature_c, c->bme_humidity_pct);
        } else {
            // Edge case: only T is valid (e.g. SHT45 H read failed
            // mid-cycle, leaving cached T but H=0). Emit T alone.
            n += snprintf(buf + n, cap - n,
                "  {\"value_type\": \"temperature\", \"value\": \"%.2f\"}",
                c->bme_temperature_c);
        }
    }
    if (c->pm_valid) {
        // SPS30_* prefix matches Madavi's field-prefix routing convention
        // — keeps the PM RRDs separate from BME280_* in the back-end.
        COMMA();
        n += snprintf(buf + n, cap - n,
            "  {\"value_type\": \"SPS30_P0\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_P2\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_P4\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_P1\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_N05\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_N1\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_N25\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_N4\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_N10\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_TS\", \"value\": \"%.3f\"}",
            c->pm.pm1_0, c->pm.pm2_5, c->pm.pm4_0, c->pm.pm10,
            c->pm.nc0_5, c->pm.nc1_0, c->pm.nc2_5, c->pm.nc4_0, c->pm.nc10,
            c->pm.typ_size_um);

        // HACK (Madavi-only): also emit SDS_P1 / SDS_P2 with the same PM10 /
        // PM2.5 values. Madavi's data.php is a 2017-vintage backend that
        // recognises SDS011 (and BME280) only — there is no $has_sps30 check,
        // so all the SPS30_* fields above are silently dropped from the RRDs
        // and graphs. Mapping our PM10 → SDS_P1 and PM2.5 → SDS_P2 makes
        // Madavi's $has_sds011 fire, which creates an SDS011-typed RRD per
        // device with the right values plotted. Field-name convention follows
        // dusty-code's SDS011 driver (sds011.cpp:524-526) and matches Madavi's
        // data.php:49 detection: `isset($values["SDS_P1"]) && isset($values["SDS_P2"])`.
        // This is a Madavi-specific workaround — sensor.community gets the
        // proper SPS30 fields on X-PIN 12; openSenseMap and aqi.eco continue
        // to receive SPS30_* only (their parsers handle the prefix natively).
        // Mirror of the existing BME280_* relabel hack for SHT45+BMP581 data.
        n += snprintf(buf + n, cap - n,
            ",\n"
            "  {\"value_type\": \"SDS_P1\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SDS_P2\", \"value\": \"%.2f\"}",
            c->pm.pm10, c->pm.pm2_5);
    }
    if (c->noise_valid) {
        // DNMS_noise_* prefix is the canonical airrohr / dusty naming —
        // Madavi's prefix-based routing requires it; sensor.community uses
        // the same naming on its dedicated X-PIN 15 POST.
        COMMA();
        n += snprintf(buf + n, cap - n,
            "  {\"value_type\": \"DNMS_noise_LAeq\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"DNMS_noise_LA_min\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"DNMS_noise_LA_max\", \"value\": \"%.2f\"}",
            c->noise.laeq, c->noise.la_min, c->noise.la_max);
    }
    if (have_pulse_stats) {
        COMMA();
        n += snprintf(buf + n, cap - n,
            "  {\"value_type\": \"samples\", \"value\": \"%lu\"},\n"
            "  {\"value_type\": \"min_micro\", \"value\": \"%lu\"},\n"
            "  {\"value_type\": \"max_micro\", \"value\": \"%lu\"}",
            (unsigned long)c->gm_counts,
            (unsigned long)c->min_micro,
            (unsigned long)c->max_micro);
    }
    COMMA();
    n += snprintf(buf + n, cap - n,
        "  {\"value_type\": \"signal\", \"value\": \"%d\"}\n"
        " ]\n}",
        (int)c->rssi);
    #undef COMMA
}

static int send_madavi(const tx_context_t *c) {
    bool have_env = c->bme_valid || c->pm_valid || c->noise_valid;
    if (!c->tube_enabled && !have_env) return 0;

    const char *url = c->madavi.use_https ? c->madavi.url_https : c->madavi.url_http;
    esp_http_client_handle_t client =
        open_push_client(url, c->madavi.use_insecure, c->chip_id);
    if (!client) {
        ESP_LOGE(TAG, "madavi: http_client_init failed");
        return -1;
    }

    // Body buffer sized for the worst case: BME (3 fields) + SPS30 (10 fields)
    // + pulse stats (3 fields) + signal + boilerplate. ~1200 bytes is generous.
    char body[1280];

    int rc_g = 0;
    if (c->tube_enabled) {
        build_madavi_geiger_body(c, body, sizeof(body));
        rc_g = post_with_retry(client, "Madavi", "geiger", NULL, body);
    }

    // Environmental POST — runs whenever any non-radiation source is live.
    // Skipped when the geiger POST failed at transport level so we don't pile
    // a second POST onto an already-broken TLS session.
    int rc_e = 0;
    if (have_env && (!c->tube_enabled || rc_g > 0)) {
        build_madavi_env_body(c, body, sizeof(body));
        rc_e = post_with_retry(client, "Madavi", "env", NULL, body);
    }

    esp_http_client_cleanup(client);

    if (c->tube_enabled && have_env) {
        ESP_LOGI(TAG, "Madavi: geiger rc=%d, env rc=%d", rc_g, rc_e);
        if (rc_g == 200 && rc_e == 200) return 200;
        return (rc_g != 200) ? rc_g : rc_e;
    }
    if (c->tube_enabled) {
        ESP_LOGI(TAG, "Madavi: geiger rc=%d (tube only)", rc_g);
        return rc_g;
    }
    ESP_LOGI(TAG, "Madavi: env rc=%d (tube disabled)", rc_e);
    return rc_e;
}

// --- sensor.community -------------------------------------------------------
// Up to four separate POSTs over one keep-alive TLS session — sensor.community
// routes by X-PIN header, so each sensor class needs its own POST. PIN values
// verified against the authoritative SENSOR_TYPES dict in
// devices.sensor.community/webapp/default_settings.py AND airrohr-firmware/ext_def.h:
//   X-PIN 19 — radiation (Si22G;  lowercase Luftdaten field names)
//   X-PIN 11 — BME280    (lowercase temperature/humidity/pressure; pressure in hPa)
//   X-PIN  1 — SPS30     (SPS30_* prefixed Luftdaten field names — PIN 1 is the
//                         shared "particulate matter" pin used by every PM sensor:
//                         SDS011, PMS-series, HPM, NPM, IPS-7100, HM3301, SPS30.
//                         The receiving server disambiguates by the SPS30_* prefix
//                         within the body.)
//   X-PIN 15 — DNMS      (DNMS_noise_* prefixed; canonical DNMS_API_PIN)
// Mixing fields across PIN classes in one POST returns 400 from the SC API.
// All POSTs share ONE underlying TCP+TLS connection thanks to keep-alive — we
// set the X-PIN header per-perform but never close the socket.

static void build_sensorc_geiger_body(const tx_context_t *c, char *buf, size_t cap) {
    float msi = (c->cpm / 60.0f) * SI22G_CPS_TO_USVPH / 1000.0f;  // mSv/h
    snprintf(buf, cap,
        "{\n"
        " \"software_version\": \"%s\",\n"
        " \"sensordatavalues\": [\n"
        "  {\"value_type\": \"counts_per_minute\", \"value\": \"%lu\"},\n"
        "  {\"value_type\": \"hv_pulses\", \"value\": \"%lu\"},\n"
        "  {\"value_type\": \"counts\", \"value\": \"%lu\"},\n"
        "  {\"value_type\": \"sample_time_ms\", \"value\": \"%lu\"},\n"
        "  {\"value_type\": \"radiation_msi\", \"value\": \"%.6f\"}\n"
        " ]\n}",
        c->sw_version,
        (unsigned long)c->cpm, (unsigned long)c->hv_pulses,
        (unsigned long)c->gm_counts, (unsigned long)c->dt_ms, msi);
}

// X-PIN 15 body for hbitter DNMS noise sensor. Canonical airrohr value
// (DNMS_API_PIN 15 from airrohr-firmware/defines.h) — keep DNMS_noise_*
// prefixed names identical between SC and Madavi (unlike Si22G/BME which
// use unprefixed names on SC, prefixed on Madavi).
static void build_sensorc_noise_body(const tx_context_t *c, char *buf, size_t cap) {
    snprintf(buf, cap,
        "{\n"
        " \"software_version\": \"%s\",\n"
        " \"sensordatavalues\": [\n"
        "  {\"value_type\": \"DNMS_noise_LAeq\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"DNMS_noise_LA_min\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"DNMS_noise_LA_max\", \"value\": \"%.2f\"}\n"
        " ]\n}",
        c->sw_version,
        c->noise.laeq, c->noise.la_min, c->noise.la_max);
}

// X-PIN 1 body for Sensirion SPS30. PIN 1 is the SHARED "particulate matter"
// pin used by every PM sensor in sensor.community's registry (SDS011, PMS-series,
// HPM, NPM, IPS-7100, HM3301, SPS30 — verified against
// devices.sensor.community/webapp/default_settings.py SENSOR_TYPES and against
// airrohr-firmware/ext_def.h's SPS30_API_PIN).
//
// IMPORTANT: on PIN 1 the body uses CANONICAL UNPREFIXED Luftdaten field names
// (P0/P2/P4/P1 for mass, N05/N1/N25/N4/N10 for number concentration, TS for
// typical particle size) regardless of which physical PM sensor produced the
// data. The server tracks WHICH device sent the values via the X-Sensor header
// (the chip ID) — NOT via field-name prefix. Verified by reading airrohr-
// firmware: every PM sensor (SDS011, PMS, HPM, SPS30) stores values internally
// as <SENSOR>_<FIELD> (e.g. SPS30_P0) but strips that prefix before posting:
//   sendSensorCommunity(result, SPS30_API_PIN, SENSORS_SPS30, "SPS30_");
//                                                              ^^^^^^^
//                                                       prefix to strip
// V2.3.2 → V2.3.15-pre1 wrongly sent SPS30_-prefixed names on PIN 1 → server
// returned HTTP 400 on every PM POST because it didn't recognise the fields.
// Surfaced in the wild on the dust node 10.11.12.72 once SPS30 was actually
// wired and pm_valid started being true. V2.3.15-pre2 strips the prefix here.
//
// Note Madavi / OSM / aqi.eco use DIFFERENT routing (field-name prefix matters
// for THOSE targets) — see build_madavi_env_body and build_luftdaten_body —
// and so they correctly keep the SPS30_ prefix. Only sensor.community on PIN 1
// uses the unprefixed schema.
//
// typ_size_um carries 3 decimals because the value range (~0.3..1.0 µm typical)
// needs the precision; mass and number concentrations are fine at 2 decimals.
static void build_sensorc_pm_body(const tx_context_t *c, char *buf, size_t cap) {
    snprintf(buf, cap,
        "{\n"
        " \"software_version\": \"%s\",\n"
        " \"sensordatavalues\": [\n"
        "  {\"value_type\": \"P0\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"P2\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"P4\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"P1\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"N05\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"N1\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"N25\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"N4\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"N10\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"TS\", \"value\": \"%.3f\"}\n"
        " ]\n}",
        c->sw_version,
        c->pm.pm1_0, c->pm.pm2_5, c->pm.pm4_0, c->pm.pm10,
        c->pm.nc0_5, c->pm.nc1_0, c->pm.nc2_5, c->pm.nc4_0, c->pm.nc10,
        c->pm.typ_size_um);
}

static void build_sensorc_bme_body(const tx_context_t *c, char *buf, size_t cap) {
    float p_hpa = c->bme_pressure_pa / 100.0f;
    int n = snprintf(buf, cap,
        "{\n"
        " \"software_version\": \"%s\",\n"
        " \"sensordatavalues\": [\n"
        "  {\"value_type\": \"temperature\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"humidity\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"pressure\", \"value\": \"%.2f\"}",
        c->sw_version,
        c->bme_temperature_c, c->bme_humidity_pct, p_hpa);
    if (c->send_sealevel_pressure) {
        // Barometric reduction to sea level: P0 = P * (1 - h * 0.0000226)^-5.257
        float p_sl = p_hpa * powf(1.0f - c->station_altitude_m * 0.0000226f, -5.257f);
        n += snprintf(buf + n, cap - n,
            ",\n"
            "  {\"value_type\": \"altitude\", \"value\": \"%.1f\"},\n"
            "  {\"value_type\": \"pressure_sealevel\", \"value\": \"%.2f\"}",
            c->station_altitude_m, p_sl);
    }
    snprintf(buf + n, cap - n, "\n ]\n}");
}

// True if a previously-attempted POST in the same TLS session failed at
// transport level (rc <= 0). When that happens later POSTs in the same
// session usually fail too, so we skip them rather than waste retries.
static inline bool prior_ok(int rc) {
    return rc == 0 /* not attempted */ || rc > 0 /* attempted, got a status */;
}

static int send_sensorc(const tx_context_t *c) {
    if (!c->tube_enabled && !c->bme_valid && !c->pm_valid && !c->noise_valid) return 0;

    const char *url = c->sensorc.use_https ? c->sensorc.url_https : c->sensorc.url_http;
    esp_http_client_handle_t client =
        open_push_client(url, c->sensorc.use_insecure, c->chip_id);
    if (!client) {
        ESP_LOGE(TAG, "sensor.community: http_client_init failed");
        return -1;
    }

    char body[600];

    int rc_g = 0;
    if (c->tube_enabled) {
        build_sensorc_geiger_body(c, body, sizeof(body));
        rc_g = post_with_retry(client, "sensor.community", "geiger", "19", body);
    }

    int rc_b = 0;
    if (c->bme_valid && prior_ok(rc_g)) {
        build_sensorc_bme_body(c, body, sizeof(body));
        rc_b = post_with_retry(client, "sensor.community", "bme", "11", body);
    }

    int rc_p = 0;
    if (c->pm_valid && prior_ok(rc_g) && prior_ok(rc_b)) {
        build_sensorc_pm_body(c, body, sizeof(body));
        rc_p = post_with_retry(client, "sensor.community", "pm",  "1", body);
    }

    int rc_n = 0;
    if (c->noise_valid && prior_ok(rc_g) && prior_ok(rc_b) && prior_ok(rc_p)) {
        build_sensorc_noise_body(c, body, sizeof(body));
        rc_n = post_with_retry(client, "sensor.community", "noise", "15", body);
    }

    esp_http_client_cleanup(client);

    ESP_LOGI(TAG, "sensor.community: geiger rc=%d, bme rc=%d, pm rc=%d, noise rc=%d "
             "(ran g=%d b=%d p=%d n=%d)",
             rc_g, rc_b, rc_p, rc_n,
             c->tube_enabled, c->bme_valid, c->pm_valid, c->noise_valid);

    // Aggregate: the first ran-but-not-201 wins; if everything that ran was
    // 201, return 201. If nothing ran, return 0 (handled at top).
    int worst = 201;
    if (c->tube_enabled && rc_g != 201)                                       worst = rc_g;
    else if (c->bme_valid && prior_ok(rc_g) && rc_b != 201)                   worst = rc_b;
    else if (c->pm_valid  && prior_ok(rc_g) && prior_ok(rc_b) && rc_p != 201) worst = rc_p;
    else if (c->noise_valid && prior_ok(rc_g) && prior_ok(rc_b) &&
             prior_ok(rc_p) && rc_n != 201)                                   worst = rc_n;
    return worst;
}

// --- Radmon -----------------------------------------------------------------
// GET /radmon.php?function=submit&user=X&password=Y&value=CPM&unit=CPM
// Success = HTTP 200 + body contains "OK".

static int send_radmon(const tx_context_t *c) {
    if (!c->radmon_user[0] || !c->radmon_password[0]) {
        ESP_LOGW(TAG, "Radmon: credentials empty, skipping.");
        return -3;
    }
    const char *base = c->radmon.use_https ? c->radmon.url_https : c->radmon.url_http;
    char url[256];
    snprintf(url, sizeof(url),
             "%s?function=submit&user=%s&password=%s&value=%lu&unit=CPM",
             base, c->radmon_user, c->radmon_password, (unsigned long)c->cpm);

    for (int i = 0; i < HTTP_MAX_RETRIES; i++) {
        if (!wifi_up()) return -2;
        char buf[RESP_BUF_SIZE];
        resp_ctx_t resp = { .buf = buf, .cap = sizeof(buf), .len = 0 };
        buf[0] = 0;
        int rc = do_request(url, HTTP_METHOD_GET, NULL, c->chip_id, NULL, NULL,
                            &resp, c->radmon.use_insecure);
        if (rc == 200) {
            if (strstr(buf, "OK")) return 200;
            ESP_LOGW(TAG, "Radmon rejected: %s", buf);
            return -1;
        }
        if (rc > 0 && rc != 408 && rc < 500) return rc;  // 4xx (auth) — don't retry
        ESP_LOGW(TAG, "Radmon rc=%d (retry %d/%d)", rc, i + 1, HTTP_MAX_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    return -1;
}

// --- Combined Luftdaten body (used by openSenseMap + aqi.eco) ---------------
// Both targets accept a Luftdaten-style sensordatavalues array. openSenseMap
// gets the full sensor bundle (Si22G_* radiation, BME280_* T/H/P, SPS30_* PM,
// DNMS_noise_*, pulse stats, signal) because per-box channel mappings can
// route each field where the user wants. aqi.eco has a fixed hardcoded
// VALUE_MAPPING in updater.php that only knows specific value_type aliases,
// so for aqi.eco we trim out everything it doesn't map (radiation, pulse
// stats, SPS30_TS, DNMS min/max) and add NAMF-style alias duplicates for
// T/H/P (SHT3X_*, BMP_*) so the server can pick whichever alias it prefers.
//
// V2.3.25: also strip "esp32-" prefix from chip_id when emitting esp8266id
// for aqi.eco — the server's devices.esp8266_id column is bigint, so a
// non-numeric value silently breaks the per-cycle UPDATE and surfaces as
// HTTP 500 with an empty body. Took a full evening of probing to identify
// (see reference_aqi_eco.md).
//
// openSenseMap needs the URL query `?luftdaten=1` to switch its parser into
// Luftdaten compatibility mode; aqi.eco expects the body wrapped with an
// `esp8266id` field at the top.
static void build_luftdaten_body(const tx_context_t *c, char *buf, size_t cap,
                                 bool prefix_aqi_id) {
    int n = 0;
    if (prefix_aqi_id) {
        // aqi.eco: strip "esp32-" so the value parses as bigint server-side.
        const char *aqi_id = c->chip_id;
        if (strncmp(aqi_id, "esp32-", 6) == 0) aqi_id += 6;
        n += snprintf(buf + n, cap - n,
            "{\n"
            " \"esp8266id\": \"%s\",\n"
            " \"software_version\": \"%s\",\n"
            " \"sensordatavalues\": [\n",
            aqi_id, c->sw_version);
    } else {
        n += snprintf(buf + n, cap - n,
            "{\n"
            " \"software_version\": \"%s\",\n"
            " \"sensordatavalues\": [\n",
            c->sw_version);
    }

    bool first = true;
    #define COMMA() do { if (!first) n += snprintf(buf + n, cap - n, ",\n"); first = false; } while (0)

    // Radiation block — Si22G_* are not in aqi.eco's VALUE_MAPPING (no
    // radiation column), so skip the entire block for aqi.eco. openSenseMap
    // can route Si22G data to a configured per-box channel.
    if (c->tube_enabled && !prefix_aqi_id) {
        float msi = (c->cpm / 60.0f) * SI22G_CPS_TO_USVPH / 1000.0f;
        COMMA();
        n += snprintf(buf + n, cap - n,
            "  {\"value_type\": \"Si22G_counts_per_minute\", \"value\": \"%lu\"},\n"
            "  {\"value_type\": \"Si22G_hv_pulses\", \"value\": \"%lu\"},\n"
            "  {\"value_type\": \"Si22G_counts\", \"value\": \"%lu\"},\n"
            "  {\"value_type\": \"Si22G_sample_time_ms\", \"value\": \"%lu\"},\n"
            "  {\"value_type\": \"Si22G_radiation_msi\", \"value\": \"%.6f\"}",
            (unsigned long)c->cpm, (unsigned long)c->hv_pulses,
            (unsigned long)c->gm_counts, (unsigned long)c->dt_ms, msi);
        if (c->gm_counts > 1) {
            n += snprintf(buf + n, cap - n,
                ",\n"
                "  {\"value_type\": \"samples\", \"value\": \"%lu\"},\n"
                "  {\"value_type\": \"min_micro\", \"value\": \"%lu\"},\n"
                "  {\"value_type\": \"max_micro\", \"value\": \"%lu\"}",
                (unsigned long)c->gm_counts,
                (unsigned long)c->min_micro,
                (unsigned long)c->max_micro);
        }
    }
    if (c->bme_valid) {
        COMMA();
        n += snprintf(buf + n, cap - n,
            "  {\"value_type\": \"BME280_temperature\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"BME280_humidity\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"BME280_pressure\", \"value\": \"%.2f\"}",
            c->bme_temperature_c, c->bme_humidity_pct, c->bme_pressure_pa);
        if (prefix_aqi_id) {
            // V2.3.25 NAMF-style alias duplicates — aqi.eco's VALUE_MAPPING
            // lists SHT3X_* first in temperature/humidity alias arrays and
            // BMP_pressure as an accepted alias for pressure. Sending the
            // same value under multiple aliases maximises compatibility
            // against any future server-side preference change. Same numeric
            // value, so picking SHT3X vs BME280 doesn't affect the data.
            n += snprintf(buf + n, cap - n,
                ",\n"
                "  {\"value_type\": \"SHT3X_temperature\", \"value\": \"%.2f\"},\n"
                "  {\"value_type\": \"SHT3X_humidity\", \"value\": \"%.2f\"},\n"
                "  {\"value_type\": \"BMP_pressure\", \"value\": \"%.2f\"}",
                c->bme_temperature_c, c->bme_humidity_pct, c->bme_pressure_pa);
        }
    }
    if (c->pm_valid) {
        COMMA();
        n += snprintf(buf + n, cap - n,
            "  {\"value_type\": \"SPS30_P0\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_P2\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_P4\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_P1\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_N05\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_N1\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_N25\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_N4\", \"value\": \"%.2f\"},\n"
            "  {\"value_type\": \"SPS30_N10\", \"value\": \"%.2f\"}",
            c->pm.pm1_0, c->pm.pm2_5, c->pm.pm4_0, c->pm.pm10,
            c->pm.nc0_5, c->pm.nc1_0, c->pm.nc2_5, c->pm.nc4_0, c->pm.nc10);
        // SPS30_TS (typical particle size) — not in aqi.eco's VALUE_MAPPING.
        // Kept for openSenseMap which can route it to a per-box channel.
        if (!prefix_aqi_id) {
            n += snprintf(buf + n, cap - n,
                ",\n"
                "  {\"value_type\": \"SPS30_TS\", \"value\": \"%.3f\"}",
                c->pm.typ_size_um);
        }
    }
    if (c->noise_valid) {
        // DNMS_noise_LAeq is the only noise alias in aqi.eco's VALUE_MAPPING
        // (canonical column = noise_level). LA_min/LA_max have no
        // destination columns there — kept for openSenseMap only.
        COMMA();
        n += snprintf(buf + n, cap - n,
            "  {\"value_type\": \"DNMS_noise_LAeq\", \"value\": \"%.2f\"}",
            c->noise.laeq);
        if (!prefix_aqi_id) {
            n += snprintf(buf + n, cap - n,
                ",\n"
                "  {\"value_type\": \"DNMS_noise_LA_min\", \"value\": \"%.2f\"},\n"
                "  {\"value_type\": \"DNMS_noise_LA_max\", \"value\": \"%.2f\"}",
                c->noise.la_min, c->noise.la_max);
        }
    }
    COMMA();
    n += snprintf(buf + n, cap - n,
        "  {\"value_type\": \"signal\", \"value\": \"%d\"}\n"
        " ]\n}",
        (int)c->rssi);
    #undef COMMA
}

// --- openSenseMap -----------------------------------------------------------
// Single HTTPS POST per cycle to ingress.opensensemap.org. The path
// /boxes/<BOX_ID>/data?luftdaten=1 routes the Luftdaten body into the box's
// existing channels (mapping is configured per-box in the openSenseMap UI).
// Success = HTTP 201; any other status counts as a soft failure.
static int send_osm(const tx_context_t *c) {
    if (!c->osm_box_id || c->osm_box_id[0] == 0) {
        ESP_LOGW(TAG, "openSenseMap: box_id empty, skipping");
        return -3;
    }
    char url[160];
    snprintf(url, sizeof(url),
             "https://ingress.opensensemap.org/boxes/%s/data?luftdaten=1",
             c->osm_box_id);

    char body[1600];
    build_luftdaten_body(c, body, sizeof(body), false);

    // V2.3.16: optional Bearer-token auth. The OSM Luftdaten path
    // historically accepted unauthenticated POSTs; their dashboard now lets
    // box owners opt-in to require an Authorization header per box. Build a
    // small client wrapper here so we can attach the header on the per-cycle
    // client (do_request's fixed signature doesn't take an auth header). When
    // the token is empty the Authorization header is never sent — keeps
    // backward compat with unauthenticated boxes.
    bool have_token = c->osm_access_token && c->osm_access_token[0];
    char authz[80];
    if (have_token) snprintf(authz, sizeof(authz), "Bearer %s", c->osm_access_token);

    for (int i = 0; i < HTTP_MAX_RETRIES; i++) {
        if (!wifi_up()) return -2;
        // Inline init+perform+cleanup so we can attach the optional auth
        // header. Mirrors do_request() but with one extra set_header call.
        esp_http_client_config_t cfg = {
            .url = url,
            .timeout_ms = HTTP_TIMEOUT_MS,
            .method = HTTP_METHOD_POST,
            .event_handler = http_event_cb,
            .buffer_size = 1024,
            .buffer_size_tx = 1024,
            .transport_type = HTTP_TRANSPORT_OVER_SSL,
        };
        if (!c->osm_use_insecure) cfg.crt_bundle_attach = esp_crt_bundle_attach;
        else                       cfg.skip_cert_common_name_check = true;

        esp_http_client_handle_t client = esp_http_client_init(&cfg);
        if (!client) {
            ESP_LOGE(TAG, "openSenseMap: http_client_init failed");
            return -1;
        }
        esp_http_client_set_header(client, "Content-Type", "application/json; charset=UTF-8");
        esp_http_client_set_header(client, "X-Sensor", c->chip_id);
        if (have_token) esp_http_client_set_header(client, "Authorization", authz);
        esp_http_client_set_header(client, "Connection", "close");
        esp_http_client_set_post_field(client, body, strlen(body));

        esp_err_t err = esp_http_client_perform(client);
        int rc = (err == ESP_OK) ? esp_http_client_get_status_code(client) : -1;
        if (err != ESP_OK) ESP_LOGW(TAG, "openSenseMap perform error: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);

        if (rc == 201 || rc == 200) return rc;
        if (rc > 0 && rc != 408 && rc < 500) {
            ESP_LOGW(TAG, "openSenseMap rc=%d (4xx) — not retrying", rc);
            return rc;
        }
        ESP_LOGW(TAG, "openSenseMap rc=%d (retry %d/%d)", rc, i + 1, HTTP_MAX_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ESP_LOGW(TAG, "openSenseMap: all retries exhausted");
    return -1;
}

// --- aqi.eco ----------------------------------------------------------------
// Single HTTPS POST per cycle to api.aqi.eco/update/<TOKEN>. Body is the
// Luftdaten bundle wrapped with an `esp8266id` field — server uses this to
// match the device against the user's account. Success = HTTP 200.
static int send_aqi(const tx_context_t *c) {
    if (!c->aqi_token || c->aqi_token[0] == 0) {
        ESP_LOGW(TAG, "aqi.eco: token empty, skipping");
        return -3;
    }
    char url[160];
    snprintf(url, sizeof(url), "https://api.aqi.eco/update/%s", c->aqi_token);

    char body[1700];   // slightly larger than OSM body — extra esp8266id field
    build_luftdaten_body(c, body, sizeof(body), true);

    for (int i = 0; i < HTTP_MAX_RETRIES; i++) {
        if (!wifi_up()) return -2;
        int rc = do_request(url, HTTP_METHOD_POST,
                            "application/json; charset=UTF-8",
                            c->chip_id, NULL, body, NULL, c->aqi_use_insecure);
        if (rc == 200 || rc == 201) return rc;
        if (rc > 0 && rc != 408 && rc < 500) {
            ESP_LOGW(TAG, "aqi.eco rc=%d (4xx) — not retrying", rc);
            return rc;
        }
        ESP_LOGW(TAG, "aqi.eco rc=%d (retry %d/%d)", rc, i + 1, HTTP_MAX_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    ESP_LOGW(TAG, "aqi.eco: all retries exhausted");
    return -1;
}

// --- Orchestrator -----------------------------------------------------------

bool tx_is_idle(void) {
    return !s_tx_busy && uxQueueMessagesWaiting(s_tx_queue) == 0;
}

void tx_transmit(const tx_context_t *c) {
    // Shallow-copy and enqueue — all const char* fields point to static literals.
    // Queue depth = 1, non-blocking: if the worker is still busy with the
    // previous cycle, drop the new one (150 s TX interval vs. <15 s worker time
    // makes overlap unlikely, but we don't want main to block on TLS).
    if (xQueueSend(s_tx_queue, c, 0) != pdTRUE) {
        ESP_LOGW(TAG, "TX queue full — dropping this cycle");
    }
}

static void tx_run(tx_context_t *c) {
    // Per-target consecutive-fail streak — internal only. The "skip remaining"
    // counters were promoted to s_stats[i].breaker_open_cycles so the status
    // page can show them; updates here keep them in sync (single-writer).
    //
    // V2.4.1 (B5): these are function-static which means they survive across
    // calls. SAFE ONLY because TX_QUEUE_DEPTH == 1 and exactly one tx_task
    // exists — tx_run is never reentered. If either invariant ever changes
    // (deeper queue OR a second worker on the other core), promote to
    // module-static with a per-target spinlock OR pass these in via the
    // tx_context_t. Adding a comment rather than restructuring today —
    // single-worker queue is a long-standing design choice.
    static int madavi_fail_streak  = 0;
    static int sensorc_fail_streak = 0;
    static int radmon_fail_streak  = 0;
    static int osm_fail_streak     = 0;
    static int aqi_fail_streak     = 0;

    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_free  = esp_get_minimum_free_heap_size();
    uint32_t max_alloc = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    // V2.3.17: include min_free so the per-cycle log captures the lifetime
    // watermark too. Lets us correlate transient-peak events with what was
    // happening across the codebase, not just FTPS-specific moments
    // (which the FTPS pre/post heap log already covers).
    ESP_LOGI(TAG, "free heap before TX: %lu bytes / min_free=%lu / max_alloc=%lu bytes",
             (unsigned long)free_heap, (unsigned long)min_free, (unsigned long)max_alloc);

    // Madavi and sensor.community accept a mix of radiation + env (THP) + PM +
    // noise payloads, so they're called whenever ANY source is live. Radmon is
    // radiation-only, so it's gated strictly on tube_enabled.
    bool any_payload = c->tube_enabled || c->bme_valid || c->pm_valid || c->noise_valid;

    if (c->madavi.enabled && any_payload) {
        if (s_stats[TX_TARGET_MADAVI].breaker_open_cycles > 0) {
            s_stats[TX_TARGET_MADAVI].breaker_open_cycles--;
            ESP_LOGI(TAG, "Madavi: breaker open (%d cycles left)",
                     s_stats[TX_TARGET_MADAVI].breaker_open_cycles);
            display_set_status(DSP_STATUS_MADAVI, DSP_SRV_ERROR);
        } else {
            ESP_LOGI(TAG, "Sending to Madavi (%s)", c->madavi.use_https ? "https" : "http");
            display_set_status(DSP_STATUS_MADAVI, DSP_SRV_SENDING);
            int rc = send_madavi(c);
            bool ok = (rc == 200);
            record_outcome(TX_TARGET_MADAVI, rc, ok);
            ESP_LOGI(TAG, "Madavi: %s (rc=%d)", ok ? "ok" : "error", rc);
            display_set_status(DSP_STATUS_MADAVI, ok ? DSP_SRV_IDLE : DSP_SRV_ERROR);
            if (ok) {
                madavi_fail_streak = 0;
            } else if (rc == -1) {  // only full retry exhaustion counts
                madavi_fail_streak++;
                if (madavi_fail_streak >= TX_CB_FAIL_THRESHOLD) {
                    s_stats[TX_TARGET_MADAVI].breaker_open_cycles = TX_CB_SKIP_CYCLES;
                    ESP_LOGW(TAG, "Madavi: %d fails, breaker open for %d cycles",
                             madavi_fail_streak, TX_CB_SKIP_CYCLES);
                    madavi_fail_streak = 0;
                }
            }
        }
    } else if (c->madavi.enabled) {
        ESP_LOGI(TAG, "Madavi: skipping (no payload — tube disabled and no env/PM sensor)");
        display_set_status(DSP_STATUS_MADAVI, DSP_SRV_IDLE);
    } else {
        display_set_status(DSP_STATUS_MADAVI, DSP_SRV_OFF);
    }

    if (c->sensorc.enabled && any_payload) {
        if (s_stats[TX_TARGET_SENSORC].breaker_open_cycles > 0) {
            s_stats[TX_TARGET_SENSORC].breaker_open_cycles--;
            ESP_LOGI(TAG, "sensor.community: breaker open (%d cycles left)",
                     s_stats[TX_TARGET_SENSORC].breaker_open_cycles);
            display_set_status(DSP_STATUS_SCOMM, DSP_SRV_ERROR);
        } else {
            ESP_LOGI(TAG, "Sending to sensor.community (%s)", c->sensorc.use_https ? "https" : "http");
            display_set_status(DSP_STATUS_SCOMM, DSP_SRV_SENDING);
            int rc = send_sensorc(c);
            bool ok = (rc == 201);
            record_outcome(TX_TARGET_SENSORC, rc, ok);
            ESP_LOGI(TAG, "sensor.community: %s (rc=%d)", ok ? "ok" : "error", rc);
            display_set_status(DSP_STATUS_SCOMM, ok ? DSP_SRV_IDLE : DSP_SRV_ERROR);
            if (ok) {
                sensorc_fail_streak = 0;
            } else if (rc == -1) {  // only full retry exhaustion counts
                sensorc_fail_streak++;
                if (sensorc_fail_streak >= TX_CB_FAIL_THRESHOLD) {
                    s_stats[TX_TARGET_SENSORC].breaker_open_cycles = TX_CB_SKIP_CYCLES;
                    ESP_LOGW(TAG, "sensor.community: %d fails, breaker open for %d cycles",
                             sensorc_fail_streak, TX_CB_SKIP_CYCLES);
                    sensorc_fail_streak = 0;
                }
            }
        }
    } else if (c->sensorc.enabled) {
        ESP_LOGI(TAG, "sensor.community: skipping (no payload — tube disabled and no env/PM sensor)");
        display_set_status(DSP_STATUS_SCOMM, DSP_SRV_IDLE);
    } else {
        display_set_status(DSP_STATUS_SCOMM, DSP_SRV_OFF);
    }

    if (c->radmon.enabled && c->tube_enabled) {
        if (s_stats[TX_TARGET_RADMON].breaker_open_cycles > 0) {
            s_stats[TX_TARGET_RADMON].breaker_open_cycles--;
            ESP_LOGI(TAG, "Radmon: breaker open (%d cycles left)",
                     s_stats[TX_TARGET_RADMON].breaker_open_cycles);
            display_set_status(DSP_STATUS_RADMON, DSP_SRV_ERROR);
        } else {
            ESP_LOGI(TAG, "Sending to Radmon (%s)", c->radmon.use_https ? "https" : "http");
            display_set_status(DSP_STATUS_RADMON, DSP_SRV_SENDING);
            int rc = send_radmon(c);
            bool ok = (rc == 200);
            record_outcome(TX_TARGET_RADMON, rc, ok);
            ESP_LOGI(TAG, "Radmon: %s (rc=%d)", ok ? "ok" : "error", rc);
            display_set_status(DSP_STATUS_RADMON, ok ? DSP_SRV_IDLE : DSP_SRV_ERROR);
            if (ok) {
                radmon_fail_streak = 0;
            } else if (rc == -1) {  // only full retry exhaustion counts
                radmon_fail_streak++;
                if (radmon_fail_streak >= TX_CB_FAIL_THRESHOLD) {
                    s_stats[TX_TARGET_RADMON].breaker_open_cycles = TX_CB_SKIP_CYCLES;
                    ESP_LOGW(TAG, "Radmon: %d fails, breaker open for %d cycles",
                             radmon_fail_streak, TX_CB_SKIP_CYCLES);
                    radmon_fail_streak = 0;
                }
            }
        }
    } else if (c->radmon.enabled) {
        ESP_LOGI(TAG, "Radmon: skipping (tube disabled — Radmon is radiation-only)");
        display_set_status(DSP_STATUS_RADMON, DSP_SRV_IDLE);
    } else {
        display_set_status(DSP_STATUS_RADMON, DSP_SRV_OFF);
    }

    if (c->send_osm && any_payload) {
        if (s_stats[TX_TARGET_OSM].breaker_open_cycles > 0) {
            s_stats[TX_TARGET_OSM].breaker_open_cycles--;
            ESP_LOGI(TAG, "openSenseMap: breaker open (%d cycles left)",
                     s_stats[TX_TARGET_OSM].breaker_open_cycles);
        } else {
            ESP_LOGI(TAG, "Sending to openSenseMap (https)");
            int rc = send_osm(c);
            bool ok = (rc == 201 || rc == 200);
            record_outcome(TX_TARGET_OSM, rc, ok);
            ESP_LOGI(TAG, "openSenseMap: %s (rc=%d)", ok ? "ok" : "error", rc);
            if (ok) {
                osm_fail_streak = 0;
            } else if (rc == -1) {
                osm_fail_streak++;
                if (osm_fail_streak >= TX_CB_FAIL_THRESHOLD) {
                    s_stats[TX_TARGET_OSM].breaker_open_cycles = TX_CB_SKIP_CYCLES;
                    ESP_LOGW(TAG, "openSenseMap: %d fails, breaker open for %d cycles",
                             osm_fail_streak, TX_CB_SKIP_CYCLES);
                    osm_fail_streak = 0;
                }
            }
        }
    } else if (c->send_osm) {
        ESP_LOGI(TAG, "openSenseMap: skipping (no payload)");
    }

    if (c->send_aqi && any_payload) {
        if (s_stats[TX_TARGET_AQI].breaker_open_cycles > 0) {
            s_stats[TX_TARGET_AQI].breaker_open_cycles--;
            ESP_LOGI(TAG, "aqi.eco: breaker open (%d cycles left)",
                     s_stats[TX_TARGET_AQI].breaker_open_cycles);
        } else {
            ESP_LOGI(TAG, "Sending to aqi.eco (https)");
            int rc = send_aqi(c);
            bool ok = (rc == 200 || rc == 201);
            record_outcome(TX_TARGET_AQI, rc, ok);
            ESP_LOGI(TAG, "aqi.eco: %s (rc=%d)", ok ? "ok" : "error", rc);
            if (ok) {
                aqi_fail_streak = 0;
            } else if (rc == -1) {
                aqi_fail_streak++;
                if (aqi_fail_streak >= TX_CB_FAIL_THRESHOLD) {
                    s_stats[TX_TARGET_AQI].breaker_open_cycles = TX_CB_SKIP_CYCLES;
                    ESP_LOGW(TAG, "aqi.eco: %d fails, breaker open for %d cycles",
                             aqi_fail_streak, TX_CB_SKIP_CYCLES);
                    aqi_fail_streak = 0;
                }
            }
        }
    } else if (c->send_aqi) {
        ESP_LOGI(TAG, "aqi.eco: skipping (no payload)");
    }
}
