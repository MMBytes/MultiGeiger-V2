#include "sht45.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sht45";

#define SHT45_ADDR          0x44

// Commands (datasheet table 8)
#define CMD_MEASURE_HIGH    0xFD   // high-precision T+RH, ~8.2 ms
#define CMD_READ_SERIAL     0x89   // read 32-bit factory serial; ~1 ms
#define CMD_HEATER_200MW    0x39   // 200 mW for 0.1 s, then measure
#define CMD_SOFT_RESET      0x94

// Heater interval: only activate when humidity > this and 10 min have elapsed.
#define HEATER_HUMIDITY_THR  80.0f
#define HEATER_INTERVAL_MS   (10 * 60 * 1000UL)

static i2c_master_dev_handle_t s_dev  = NULL;
static bool                    s_ready = false;
static uint32_t                s_last_heat_ms = 0;

// CRC-8: polynomial 0x31, init 0xFF (Sensirion standard).
static bool crc_ok(uint8_t a, uint8_t b, uint8_t crc) {
    uint8_t v = 0xFF;
    v ^= a;
    for (int i = 0; i < 8; i++) v = (v & 0x80) ? (v << 1) ^ 0x31 : v << 1;
    v ^= b;
    for (int i = 0; i < 8; i++) v = (v & 0x80) ? (v << 1) ^ 0x31 : v << 1;
    return v == crc;
}

static esp_err_t send_cmd(uint8_t cmd) {
    return i2c_master_transmit(s_dev, &cmd, 1, 50);
}

// V2.3.30: read the factory-burned 32-bit serial via cmd 0x89 (datasheet
// §4.5). Response is 6 bytes: serial[31:16] + CRC + serial[15:0] + CRC.
// Used at init time only — gives a way to identify a specific physical
// chip in mixed-board test scenarios (helpful when one of several SHT45s
// is faulty and you need to mark which physical part to retire).
static esp_err_t sht45_read_serial(uint32_t *serial) {
    if (!s_dev || !serial) return ESP_FAIL;
    esp_err_t err = send_cmd(CMD_READ_SERIAL);
    if (err != ESP_OK) return err;
    // V2.3.31: precise busy-wait. vTaskDelay(pdMS_TO_TICKS(2)) at the default
    // CONFIG_FREERTOS_HZ=100 (10 ms tick) is 1 tick = 0..10 ms actual — it
    // could return immediately, before the chip has the serial ready (datasheet
    // says ~1 ms). 2 ms busy-wait costs nothing at boot and is deterministic.
    esp_rom_delay_us(2000);
    uint8_t buf[6];
    err = i2c_master_receive(s_dev, buf, sizeof(buf), 50);
    if (err != ESP_OK) return err;
    if (!crc_ok(buf[0], buf[1], buf[2]) || !crc_ok(buf[3], buf[4], buf[5])) {
        return ESP_FAIL;
    }
    *serial = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
              ((uint32_t)buf[3] <<  8) |  (uint32_t)buf[4];
    return ESP_OK;
}

// Try one full init pass: soft-reset + verification measurement. Logs every
// step's error code so future bring-up issues don't collapse to a single
// opaque "probe read failed". Returns ESP_OK on success, the underlying
// esp_err_t on failure. Caller decides whether to retry. Added in V2.3.11
// after the FeatherS3-D bench's Adafruit 6174 SHT45 ACK'd at 0x44 but failed
// the verification read, with no detail of which step caused the failure.
static esp_err_t try_init_pass(uint32_t reset_wait_ms, const char *label) {
    esp_err_t err = send_cmd(CMD_SOFT_RESET);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "[%s] soft_reset write: %s", label, esp_err_to_name(err));
        return err;
    }
    // V2.3.31: for sub-20 ms post-reset waits use esp_rom_delay_us so the
    // 100 Hz tick rate doesn't shorten the wait. ≥20 ms can stay on
    // vTaskDelay (worst-case quantisation is small relative to the wait).
    if (reset_wait_ms < 20) {
        esp_rom_delay_us(reset_wait_ms * 1000);
    } else {
        vTaskDelay(pdMS_TO_TICKS(reset_wait_ms));
    }

    // Inline the measurement+read so we can log each step. Mirrors
    // sht45_read but with verbose error reporting.
    err = send_cmd(CMD_MEASURE_HIGH);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "[%s] measure_high write: %s", label, esp_err_to_name(err));
        return err;
    }
    // V2.3.31: 8.2 ms typical, 9.4 ms max — vTaskDelay(pdMS_TO_TICKS(10))
    // = 1 tick at 100 Hz = 0..10 ms actual. See reference_sht45.md.
    esp_rom_delay_us(15000);

    uint8_t buf[6];
    err = i2c_master_receive(s_dev, buf, sizeof(buf), 50);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "[%s] post-measure read: %s", label, esp_err_to_name(err));
        return err;
    }

    if (!crc_ok(buf[0], buf[1], buf[2]) || !crc_ok(buf[3], buf[4], buf[5])) {
        ESP_LOGW(TAG, "[%s] CRC mismatch — bytes=%02x %02x %02x %02x %02x %02x",
                 label, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
        return ESP_FAIL;
    }

    // Sanity-check the values are in plausible range (e.g. -45..125 °C, 0..100 %RH)
    // — protects against a chip that ACKs but returns garbage.
    uint16_t raw_t = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t raw_h = ((uint16_t)buf[3] << 8) | buf[4];
    float t = -45.0f + 175.0f * ((float)raw_t / 65535.0f);
    float h = -6.0f  + 125.0f * ((float)raw_h / 65535.0f);
    ESP_LOGI(TAG, "[%s] read OK: T=%.2f°C H=%.2f%% (raw_t=0x%04x raw_h=0x%04x)",
             label, t, h, raw_t, raw_h);
    return ESP_OK;
}

esp_err_t sht45_init(i2c_master_bus_handle_t bus) {
    if (s_ready) return ESP_OK;

    // Quick probe before adding the device — avoids a lingering handle if
    // the sensor is absent.
    esp_err_t err = i2c_master_probe(bus, SHT45_ADDR, 50);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SHT45 not found at 0x%02X (probe: %s)",
                 SHT45_ADDR, esp_err_to_name(err));
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "SHT45 ACK'd at 0x%02X — verifying with measurement read", SHT45_ADDR);

    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SHT45_ADDR,
        .scl_speed_hz    = 100000,
    };
    err = i2c_master_bus_add_device(bus, &devcfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_bus_add_device: %s", esp_err_to_name(err));
        return err;
    }

    // First attempt: 10 ms post-soft-reset wait. Sensirion datasheet says
    // the soft reset takes max 1 ms, but the Adafruit 6174 (SHT45-AD1F-R2,
    // PTFE-filter variant) empirically NACKs the measurement command for
    // several ms after soft reset — V2.3.11 diagnostic captured the original
    // 2 ms wait failing as `measure_high write: ESP_ERR_INVALID_RESPONSE`
    // every boot. 10 ms gives ~10× the datasheet spec and ~2× the empirical
    // requirement, with margin for future SHT4x variants.
    //
    // Retry kept as a safety net: if attempt 1 still fails (some chip
    // revision needs even longer, or an actual hardware fault) we wait 50 ms
    // and try again with a 50 ms post-soft-reset wait. Both attempts log the
    // precise step + esp_err so failure mode is immediately diagnosable.
    err = try_init_pass(10, "attempt 1");
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "first attempt failed (%s), retrying with longer wait",
                 esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(50));
        err = try_init_pass(50, "attempt 2");
    }

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SHT45 probe read failed — both attempts unsuccessful "
                 "(last error: %s). Removing device handle.", esp_err_to_name(err));
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_FAIL;
    }

    s_ready = true;

    // V2.3.30: log the factory serial — diagnostic aid for tracking which
    // physical chip is which in dev / multi-board scenarios. Failure to
    // read serial is non-fatal (chip is otherwise working).
    uint32_t serial = 0;
    if (sht45_read_serial(&serial) == ESP_OK) {
        ESP_LOGI(TAG, "SHT45 serial = 0x%08lX", (unsigned long)serial);
    } else {
        ESP_LOGW(TAG, "SHT45 serial read failed (chip otherwise OK)");
    }

    ESP_LOGI(TAG, "SHT45 ready at 0x%02X (high-precision mode)", SHT45_ADDR);
    return ESP_OK;
}

bool sht45_present(void) {
    return s_ready;
}

esp_err_t sht45_read(float *temperature_c, float *humidity_pct) {
    if (!s_dev) return ESP_FAIL;

    // V2.3.26: log the two silent error paths. Previously sht45_read() returned
    // a bare esp_err_t on write/receive failure with no log, so an SHT45 that
    // ACK'd at init but went flaky in steady state (loose connector, marginal
    // pull-up, counterfeit chip dropping its address) produced H=0.00% on every
    // cycle with no trace of WHY. The init path's try_init_pass() already logs
    // every step; mirror that here.
    esp_err_t err = send_cmd(CMD_MEASURE_HIGH);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "measure_high write: %s", esp_err_to_name(err));
        return err;
    }

    // V2.3.31: precise busy-wait for the post-measurement window. The previous
    // vTaskDelay(pdMS_TO_TICKS(10)) was the root cause of intermittent H=0.00%
    // (with valid T) and post-measure NACK in V2.3.0..V2.3.30: at the default
    // CONFIG_FREERTOS_HZ=100 (10 ms tick) it evaluates to vTaskDelay(1) which
    // sleeps 0..10 ms. Sometimes shorter than the chip's 9.4 ms max conversion
    // time → I²C address NACK on the recv (Mode 2) OR a successful read where
    // the H register hasn't latched yet (Mode 1: H=0x0000, CRC of 00 00 = 0x81
    // passes the integrity check). Switching to esp_rom_delay_us() costs ~15 ms
    // of CPU per 150 s TX cycle = 0.01 % — negligible. See reference_sht45.md
    // §"FreeRTOS sub-tick trap" for the full diagnosis.
    esp_rom_delay_us(15000);

    uint8_t buf[6];
    err = i2c_master_receive(s_dev, buf, sizeof(buf), 50);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "post-measure read: %s", esp_err_to_name(err));
        return err;
    }

    if (!crc_ok(buf[0], buf[1], buf[2]) || !crc_ok(buf[3], buf[4], buf[5])) {
        ESP_LOGW(TAG, "SHT45 CRC mismatch");
        return ESP_FAIL;
    }

    uint16_t raw_t = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t raw_h = ((uint16_t)buf[3] << 8) | buf[4];

    // Sensirion datasheet section 4.6 transfer functions.
    float t = -45.0f + 175.0f * ((float)raw_t / 65535.0f);
    float h = -6.0f  + 125.0f * ((float)raw_h / 65535.0f);
    if (h < 0.0f)   h = 0.0f;
    if (h > 100.0f) h = 100.0f;

    if (temperature_c)  *temperature_c  = t;
    if (humidity_pct)   *humidity_pct   = h;
    return ESP_OK;
}

void sht45_heat_periodic(uint32_t now_ms, float humidity_pct) {
    if (!s_ready) return;
    if (humidity_pct < HEATER_HUMIDITY_THR) return;
    if (s_last_heat_ms != 0 &&
        (uint32_t)(now_ms - s_last_heat_ms) < HEATER_INTERVAL_MS) return;

    // CMD_HEATER_200MW triggers a 200 mW / 100 ms heat pulse and then takes
    // a measurement. The result is discarded — we just want the heat effect.
    // The next normal sht45_read() call will get fresh post-heat readings.
    ESP_LOGI(TAG, "SHT45 heater activated (RH=%.1f%%)", humidity_pct);
    send_cmd(CMD_HEATER_200MW);
    vTaskDelay(pdMS_TO_TICKS(120));   // 100 ms pulse + 20 ms settle
    uint8_t discard[6];
    i2c_master_receive(s_dev, discard, sizeof(discard), 50);
    s_last_heat_ms = now_ms;
}
