#include "sgp41.h"

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "i2c_bus.h"
#include "sensirion_crc.h"
#include "sensirion_gas_index_algorithm.h"
#include "sht45.h"
#include "telemetry.h"

static const char *TAG = "sgp41";

#define SGP41_ADDR                 0x59

// Commands (Sensirion datasheet / Adafruit_SGP41 reference driver).
#define CMD_EXECUTE_CONDITIONING   0x2612
#define CMD_MEASURE_RAW_SIGNALS    0x2619
#define CMD_GET_SERIAL_NUMBER      0x3682

// Default (compensation-disabled) ticks: 50% RH, 25 degC. Used during
// conditioning and whenever no live T/H reading is available.
#define DEFAULT_RH_TICKS            0x8000
#define DEFAULT_T_TICKS             0x6666

// V2.6.15: bumped 50 -> 70 ms, same rationale as bme280.c's 55->70 bump.
// At CONFIG_FREERTOS_HZ=100 (10 ms tick), pdMS_TO_TICKS(50) = 5 ticks, but
// vTaskDelay's worst case is (N-1) full tick periods -> as little as ~40 ms
// actual sleep, below the datasheet's 50 ms max conversion time.
// pdMS_TO_TICKS(70) = 7 ticks -> 60..70 ms actual, a guaranteed >=60 ms floor.
#define CONDITIONING_DELAY_MS       70    // datasheet: wait after cmd before read
#define MEASUREMENT_DELAY_MS        70

#define CONDITIONING_SECONDS        10    // 10 x 1 Hz per Sensirion reference example
#define SAMPLE_PERIOD_MS           1000

// V2.6.15: log every 30th consecutive measurement failure after the first,
// instead of every one -- avoids ~1 log line/s of WARN spam if the sensor
// drops off the bus. Pure log throttle, unrelated to cache staleness below.
#define FAILURE_LOG_INTERVAL        30
// One-shot informational log when a failure streak crosses this length
// (roughly matches NOX_STALE_US below at the 1 Hz sample rate).
#define FAILURE_STREAK_NOTE         15

// No fresh, publishable NOx index for this long -> treat the cache as
// stale and withhold it from callers, rather than serving an arbitrarily
// old reading as if it were current.
#define NOX_STALE_US                (15LL * 1000 * 1000)

static i2c_master_dev_handle_t s_dev  = NULL;
static bool                    s_ready = false;

static GasIndexAlgorithmParams s_nox_params;

static SemaphoreHandle_t s_cache_mux = NULL;
static int32_t           s_last_nox_index = 0;
// V2.6.15: timestamp of the last cache write (esp_timer_get_time(), us
// since boot), -1 = never valid. Replaces separate "valid" and "ever
// valid" booleans: staleness is fundamentally a data-age question, so the
// getters compute it directly from age instead of the writer proactively
// flipping a flag on a failure-count threshold. See sgp41_get_nox_index()
// and sgp41_had_valid_reading().
static int64_t            s_last_good_us = -1;

static uint16_t humidity_to_ticks(float rh_pct) {
    if (rh_pct < 0.0f)   rh_pct = 0.0f;
    if (rh_pct > 100.0f) rh_pct = 100.0f;
    return (uint16_t)(rh_pct * 65535.0f / 100.0f + 0.5f);
}

static uint16_t temperature_to_ticks(float t_c) {
    if (t_c < -45.0f) t_c = -45.0f;
    if (t_c > 130.0f) t_c = 130.0f;
    return (uint16_t)((t_c + 45.0f) * 65535.0f / 175.0f + 0.5f);
}

// Write a command with an optional 1- or 2-word payload, each word followed
// by its own CRC8 byte per the Sensirion I2C protocol convention.
static esp_err_t write_cmd(uint16_t cmd, const uint16_t *words, size_t n_words) {
    uint8_t buf[2 + 2 * 3];
    size_t  i = 0;
    buf[i++] = (uint8_t)(cmd >> 8);
    buf[i++] = (uint8_t)(cmd & 0xFF);
    for (size_t w = 0; w < n_words; w++) {
        buf[i++] = (uint8_t)(words[w] >> 8);
        buf[i++] = (uint8_t)(words[w] & 0xFF);
        const uint8_t wbuf[2] = { buf[i - 2], buf[i - 1] };
        buf[i++] = sensirion_crc8(wbuf, 2);
    }
    return i2c_master_transmit(s_dev, buf, i, 100);
}

// Read n_words 16-bit words, each followed by a CRC8 byte, verifying every
// word's checksum before accepting it.
static esp_err_t read_words(uint16_t *out, size_t n_words) {
    uint8_t buf[3 * 3];   // largest response is 3 words (serial number)
    if (n_words * 3 > sizeof(buf)) return ESP_ERR_INVALID_ARG;
    esp_err_t err = i2c_master_receive(s_dev, buf, n_words * 3, 100);
    if (err != ESP_OK) return err;
    for (size_t w = 0; w < n_words; w++) {
        const uint8_t *p = &buf[w * 3];
        if (sensirion_crc8(p, 2) != p[2]) return ESP_FAIL;
        out[w] = ((uint16_t)p[0] << 8) | p[1];
    }
    return ESP_OK;
}

static esp_err_t sgp41_get_serial(uint16_t serial[3]) {
    esp_err_t err = write_cmd(CMD_GET_SERIAL_NUMBER, NULL, 0);
    if (err != ESP_OK) return err;
    // V2.6.15: precise busy-wait, same fix as sht45.c's sht45_read_serial().
    // pdMS_TO_TICKS(1) truncates to 0 ticks at CONFIG_FREERTOS_HZ=100 (10 ms
    // tick), making vTaskDelay(0) a no-op yield instead of a real delay.
    esp_rom_delay_us(2000);
    return read_words(serial, 3);
}

// Conditioning read: only sraw_voc is meaningful during this window (per
// Sensirion's reference example, sraw_nox stays unused/0 until the switch
// to measureRawSignals after the 10 s window).
static esp_err_t sgp41_execute_conditioning(uint16_t *sraw_voc) {
    const uint16_t payload[2] = { DEFAULT_RH_TICKS, DEFAULT_T_TICKS };
    esp_err_t err = write_cmd(CMD_EXECUTE_CONDITIONING, payload, 2);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(CONDITIONING_DELAY_MS));
    return read_words(sraw_voc, 1);
}

static esp_err_t sgp41_measure_raw_signals(uint16_t rh_ticks, uint16_t t_ticks,
                                           uint16_t *sraw_voc, uint16_t *sraw_nox) {
    const uint16_t payload[2] = { rh_ticks, t_ticks };
    esp_err_t err = write_cmd(CMD_MEASURE_RAW_SIGNALS, payload, 2);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_DELAY_MS));
    uint16_t results[2];
    err = read_words(results, 2);
    if (err != ESP_OK) return err;
    *sraw_voc = results[0];
    *sraw_nox = results[1];
    return ESP_OK;
}

// Background sampling task: 10 s x 1 Hz conditioning, then measure+algorithm
// at 1 Hz forever. vTaskDelayUntil keeps the long-run average at 1 Hz
// regardless of the ~50 ms I2C conversion delay each iteration absorbs —
// the Gas Index Algorithm is only validated for steady 1 s / 10 s sampling
// intervals (sensirion_gas_index_algorithm.h), so cadence drift matters.
static void sgp41_task(void *arg) {
    (void)arg;

    TickType_t wake = xTaskGetTickCount();
    for (int i = 0; i < CONDITIONING_SECONDS; i++) {
        uint16_t sraw_voc = 0;
        esp_err_t err = sgp41_execute_conditioning(&sraw_voc);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "conditioning tick %d/%d failed: %s",
                     i + 1, CONDITIONING_SECONDS, esp_err_to_name(err));
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
    ESP_LOGI(TAG, "SGP41 conditioning complete (%d s) — switching to measurement",
             CONDITIONING_SECONDS);

    uint32_t consec_fail = 0;
    for (;;) {
        uint16_t rh_ticks = DEFAULT_RH_TICKS;
        uint16_t t_ticks  = DEFAULT_T_TICKS;
        float t_c, h_pct;
        if (sht45_present() && sht45_read(&t_c, &h_pct) == ESP_OK) {
            rh_ticks = humidity_to_ticks(h_pct);
            t_ticks  = temperature_to_ticks(t_c);
        }

        uint16_t sraw_voc, sraw_nox;
        esp_err_t err = sgp41_measure_raw_signals(rh_ticks, t_ticks, &sraw_voc, &sraw_nox);
        if (err == ESP_OK) {
            consec_fail = 0;
            int32_t nox_index = 0;
            GasIndexAlgorithm_process(&s_nox_params, (int32_t)sraw_nox, &nox_index);
            // 0 = still in the algorithm's 45 s initial-blackout period — not
            // a real reading yet, don't publish it.
            if (nox_index > 0) {
                xSemaphoreTake(s_cache_mux, portMAX_DELAY);
                s_last_nox_index = nox_index;
                s_last_good_us   = esp_timer_get_time();
                xSemaphoreGive(s_cache_mux);
            }
        } else {
            consec_fail++;
            // Throttle to the first failure, then every Nth, instead of one
            // WARN per second for as long as the sensor stays wedged.
            if (consec_fail == 1 || consec_fail % FAILURE_LOG_INTERVAL == 0) {
                ESP_LOGW(TAG, "measure_raw_signals failed (%u consecutive): %s",
                         (unsigned)consec_fail, esp_err_to_name(err));
            }
            // Log-only -- sgp41_get_nox_index() independently withholds the
            // cache once it's older than NOX_STALE_US, no cache write needed
            // here. WARN is deliberate: 15 s unresponsive is worth flagging.
            if (consec_fail == FAILURE_STREAK_NOTE) {
                ESP_LOGW(TAG, "SGP41 unresponsive for %d consecutive samples",
                         FAILURE_STREAK_NOTE);
            }
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

// V2.6.19: CSV read callback. No new cache needed here — sgp41_get_nox_index()
// already withholds the value once the background task's cache ages past
// NOX_STALE_US, so this simply forwards that staleness decision.
static bool tm_read_nox(char *cell, size_t cap, void *arg) {
    (void)arg;
    int32_t nox;
    if (sgp41_get_nox_index(&nox) != ESP_OK) return false;
    snprintf(cell, cap, "%ld", (long)nox);
    return true;
}

esp_err_t sgp41_init(i2c_master_bus_handle_t bus) {
    if (s_ready) return ESP_OK;
    if (!bus) return ESP_ERR_INVALID_ARG;

    esp_err_t err = i2c_master_probe(bus, SGP41_ADDR, 50);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SGP41 not found at 0x%02X (probe: %s)",
                 SGP41_ADDR, esp_err_to_name(err));
        return ESP_OK;   // absence is non-fatal, matches env/pm/noise sensor pattern
    }

    err = i2c_add_device(bus, SGP41_ADDR, 100000, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_bus_add_device: %s", esp_err_to_name(err));
        return ESP_OK;
    }

    // Verify with a real read, same rationale as sht45.c: a chip that ACKs
    // the probe but returns an all-zero or all-0xFFFF serial is either
    // absent, dead, or counterfeit.
    uint16_t serial[3] = { 0 };
    err = sgp41_get_serial(serial);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SGP41 serial read failed: %s", esp_err_to_name(err));
        i2c_dev_teardown(&s_dev);
        return ESP_OK;
    }
    bool serial_invalid = (serial[0] == serial[1]) && (serial[1] == serial[2]) &&
                          (serial[0] == 0x0000 || serial[0] == 0xFFFF);
    if (serial_invalid) {
        ESP_LOGW(TAG, "SGP41 serial implausible (0x%04x%04x%04x) — treating as absent",
                 serial[0], serial[1], serial[2]);
        i2c_dev_teardown(&s_dev);
        return ESP_OK;
    }
    ESP_LOGI(TAG, "SGP41 ready at 0x%02X, serial=0x%04x%04x%04x",
             SGP41_ADDR, serial[0], serial[1], serial[2]);

    // V2.6.15: a failed mutex allocation must not silently fall back to the
    // unserialized cache access this mutex exists to prevent.
    if (!s_cache_mux) s_cache_mux = xSemaphoreCreateMutex();
    if (!s_cache_mux) {
        ESP_LOGE(TAG, "mutex creation failed — treating SGP41 as absent");
        i2c_dev_teardown(&s_dev);
        return ESP_OK;
    }
    GasIndexAlgorithm_init(&s_nox_params, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);

    BaseType_t ok = xTaskCreate(sgp41_task, "sgp41", 4096, NULL,
                                tskIDLE_PRIORITY + 2, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "sampling task creation failed");
        i2c_dev_teardown(&s_dev);
        return ESP_OK;
    }

    s_ready = true;

    // V2.6.19: register our CSV column once. env_sensor's dual-bus probe can
    // call sgp41_init() a second time; the s_tm_registered guard (mirrors
    // sht45.c) is what actually prevents a double registration in that case.
    static bool s_tm_registered = false;
    if (!s_tm_registered) {
        s_tm_registered = true;
        telemetry_register("SGP41 NOx Index", tm_read_nox, NULL);
    }
    return ESP_OK;
}

bool sgp41_present(void) {
    return s_ready;
}

esp_err_t sgp41_get_nox_index(int32_t *out) {
    if (!out) return ESP_ERR_INVALID_ARG;
    if (!s_ready || !s_cache_mux) return ESP_FAIL;
    xSemaphoreTake(s_cache_mux, portMAX_DELAY);
    int64_t good_us = s_last_good_us;
    int32_t idx      = s_last_nox_index;
    xSemaphoreGive(s_cache_mux);
    if (good_us < 0 || (esp_timer_get_time() - good_us) > NOX_STALE_US) return ESP_FAIL;
    *out = idx;
    return ESP_OK;
}

bool sgp41_had_valid_reading(void) {
    if (!s_ready || !s_cache_mux) return false;
    xSemaphoreTake(s_cache_mux, portMAX_DELAY);
    bool ever = (s_last_good_us >= 0);
    xSemaphoreGive(s_cache_mux);
    return ever;
}
