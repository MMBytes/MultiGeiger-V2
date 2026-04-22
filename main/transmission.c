#include "transmission.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
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

static const char *TAG = "tx";

// TX runs on its own CPU1-pinned task so mbedTLS handshakes don't starve
// the CPU0 idle task (which feeds the task watchdog).
#define TX_TASK_STACK_BYTES  10240
#define TX_TASK_PRIO         (tskIDLE_PRIORITY + 1)
#define TX_QUEUE_DEPTH       1

static volatile bool s_tx_busy = false;

static QueueHandle_t s_tx_queue;

#define HTTP_TIMEOUT_MS     20000
#define HTTP_MAX_RETRIES    4
#define RESP_BUF_SIZE       512

// BME280 sanity — if reported temperature exceeds 60 C, treat the sensor as
// stuck (e.g. bad I²C line) and suppress the THP branch for that cycle.
// After 50 consecutive suppressed cycles (~2 h at 150 s interval) reboot.
#define TEMP_SANITY_MAX          60.0f
#define TEMP_SANITY_REBOOT_CYCLES 50

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
        ESP_LOGW(TAG, "perform error: %s (%s)", esp_err_to_name(err), url);
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
    for (int i = 0; i <= HTTP_MAX_RETRIES; i++) {
        if (!wifi_up()) return -2;
        int rc = do_request_on_client(client, x_pin, body);
        if (rc > 0 && rc != 408 && rc < 500) return rc;
        ESP_LOGW(TAG, "%s[%s] rc=%d (retry %d/%d)",
                 target, label, rc, i + 1, HTTP_MAX_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    return -1;
}

// Open a keep-alive HTTP(S) client for a sensor-push target. Content-Type
// and X-Sensor are set here; per-POST headers (X-PIN) are set by caller
// between perform() calls. No "Connection: close" — keeping the socket alive
// lets a second POST reuse the TLS session (~3–5 s handshake savings).
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
// Two POSTs: geiger (Si22G_* + signal) and THP (BME280 T/H + pulse stats +
// signal). Madavi routes by field-name prefix (not X-PIN); the pulse-stats
// RRDs are written only on the THP request path, so samples / min_micro /
// max_micro ride with the BME POST.
//
// BME280_pressure carries free-heap bytes (V1.16 TEMP HACK for remote memory
// monitoring) — real pressure goes to sensor.community, not here. Skip THP
// entirely when BME is absent.
//
// Both POSTs share one TLS session via keep-alive (single client init/cleanup).

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

static void build_madavi_thp_body(const tx_context_t *c, char *buf, size_t cap) {
    uint32_t free_heap = esp_get_free_heap_size();
    bool have_pulse_stats = (c->gm_counts > 1);
    int n = snprintf(buf, cap,
        "{\n"
        " \"software_version\": \"%s\",\n"
        " \"sensordatavalues\": [\n"
        "  {\"value_type\": \"BME280_temperature\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"BME280_humidity\", \"value\": \"%.2f\"},\n"
        "  {\"value_type\": \"BME280_pressure\", \"value\": \"%lu\"}",
        c->sw_version,
        c->bme_temperature_c, c->bme_humidity_pct,
        (unsigned long)free_heap);
    if (have_pulse_stats) {
        n += snprintf(buf + n, cap - n,
            ",\n"
            "  {\"value_type\": \"samples\", \"value\": \"%lu\"},\n"
            "  {\"value_type\": \"min_micro\", \"value\": \"%lu\"},\n"
            "  {\"value_type\": \"max_micro\", \"value\": \"%lu\"}",
            (unsigned long)c->gm_counts,
            (unsigned long)c->min_micro,
            (unsigned long)c->max_micro);
    }
    n += snprintf(buf + n, cap - n,
        ",\n"
        "  {\"value_type\": \"signal\", \"value\": \"%d\"}\n"
        " ]\n}",
        (int)c->rssi);
}

static int send_madavi(const tx_context_t *c) {
    const char *url = c->madavi.use_https ? c->madavi.url_https : c->madavi.url_http;
    esp_http_client_handle_t client =
        open_push_client(url, c->madavi.use_insecure, c->chip_id);
    if (!client) {
        ESP_LOGE(TAG, "madavi: http_client_init failed");
        return -1;
    }

    char body[700];

    build_madavi_geiger_body(c, body, sizeof(body));
    int rc_g = post_with_retry(client, "Madavi", "geiger", NULL, body);

    int rc_t = rc_g;
    if (c->bme_valid) {
        build_madavi_thp_body(c, body, sizeof(body));
        rc_t = post_with_retry(client, "Madavi", "thp", NULL, body);
    }

    esp_http_client_cleanup(client);

    if (c->bme_valid) {
        ESP_LOGI(TAG, "Madavi: geiger rc=%d, thp rc=%d", rc_g, rc_t);
    }
    if (!c->bme_valid) return rc_g;
    if (rc_g == 200 && rc_t == 200) return 200;
    return (rc_g != 200) ? rc_g : rc_t;
}

// --- sensor.community -------------------------------------------------------
// Two separate POSTs: X-PIN 19 for radiation, X-PIN 11 for BME280. The API
// routes by pin header; mixing fields in one POST returns 400. BME POST uses
// lowercase field names (temperature/humidity/pressure) and pressure in hPa
// (= Pa / 100). Altitude and sealevel values are gated on the checkbox.
//
// Both POSTs share ONE TLS session: we create the client once, perform twice
// (swapping X-PIN + body between calls), then cleanup. No `Connection: close`
// header so the underlying socket stays alive across perform()s — the second
// POST avoids its own TCP+TLS handshake (~3–5 s on this hardware).

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

static int send_sensorc(const tx_context_t *c) {
    const char *url = c->sensorc.use_https ? c->sensorc.url_https : c->sensorc.url_http;
    esp_http_client_handle_t client =
        open_push_client(url, c->sensorc.use_insecure, c->chip_id);
    if (!client) {
        ESP_LOGE(TAG, "sensor.community: http_client_init failed");
        return -1;
    }

    char body[600];

    build_sensorc_geiger_body(c, body, sizeof(body));
    int rc_g = post_with_retry(client, "sensor.community", "geiger", "19", body);

    int rc_b = rc_g;
    if (c->bme_valid) {
        build_sensorc_bme_body(c, body, sizeof(body));
        rc_b = post_with_retry(client, "sensor.community", "bme", "11", body);
    }

    esp_http_client_cleanup(client);

    if (c->bme_valid) {
        ESP_LOGI(TAG, "sensor.community: geiger rc=%d, bme rc=%d", rc_g, rc_b);
    }
    if (!c->bme_valid) return rc_g;
    if (rc_g == 201 && rc_b == 201) return 201;
    return (rc_g != 201) ? rc_g : rc_b;
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

    for (int i = 0; i <= HTTP_MAX_RETRIES; i++) {
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
    static int radmon_fail_streak = 0;
    static int radmon_skip_remaining = 0;
    static int thp_suppressed = 0;

    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t max_alloc = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "free heap before TX: %lu bytes / max_alloc=%lu bytes",
             (unsigned long)free_heap, (unsigned long)max_alloc);

    // BME280 sanity — suppress THP when temperature is stuck high. Reboots
    // after too many consecutive suppressed cycles. Geiger side unaffected.
    if (c->bme_valid && c->bme_temperature_c > TEMP_SANITY_MAX) {
        thp_suppressed++;
        ESP_LOGW(TAG, "BME280 temp %.1f C > sanity limit %.0f C — skipping THP (%d/%d)",
                 c->bme_temperature_c, TEMP_SANITY_MAX,
                 thp_suppressed, TEMP_SANITY_REBOOT_CYCLES);
        if (thp_suppressed >= TEMP_SANITY_REBOOT_CYCLES) {
            ESP_LOGE(TAG, "BME280 stuck high for %d cycles, rebooting.", thp_suppressed);
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart();
        }
        c->bme_valid = false;
    } else if (c->bme_valid) {
        thp_suppressed = 0;
    }

    if (c->madavi.enabled) {
        ESP_LOGI(TAG, "→ Madavi (%s)", c->madavi.use_https ? "https" : "http");
        int rc = send_madavi(c);
        ESP_LOGI(TAG, "Madavi: %s (rc=%d)", rc == 200 ? "ok" : "error", rc);
    }

    if (c->sensorc.enabled) {
        ESP_LOGI(TAG, "→ sensor.community (%s)", c->sensorc.use_https ? "https" : "http");
        int rc = send_sensorc(c);
        ESP_LOGI(TAG, "sensor.community: %s (rc=%d)", rc == 201 ? "ok" : "error", rc);
    }

    if (c->radmon.enabled) {
        if (radmon_skip_remaining > 0) {
            radmon_skip_remaining--;
            ESP_LOGI(TAG, "Radmon: breaker open (%d cycles left)", radmon_skip_remaining);
        } else {
            ESP_LOGI(TAG, "→ Radmon (%s)", c->radmon.use_https ? "https" : "http");
            int rc = send_radmon(c);
            bool ok = (rc == 200);
            ESP_LOGI(TAG, "Radmon: %s (rc=%d)", ok ? "ok" : "error", rc);
            if (ok) {
                radmon_fail_streak = 0;
            } else if (rc != -3) {  // -3 = no creds; don't count
                radmon_fail_streak++;
                if (radmon_fail_streak >= RADMON_FAIL_THRESHOLD) {
                    radmon_skip_remaining = RADMON_SKIP_CYCLES;
                    ESP_LOGW(TAG, "Radmon: %d fails → breaker open for %d cycles",
                             radmon_fail_streak, RADMON_SKIP_CYCLES);
                    radmon_fail_streak = 0;
                }
            }
        }
    }
}
