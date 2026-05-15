#include "sps30.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sps30";

#define SPS30_ADDR              0x69
#define SPS30_BUS_HZ            100000

// Two-byte (16-bit BE) command codes — see Sensirion SPS30 I²C protocol PDF
// (rev 0.95, "I²C transfer types" section) for the full list.
#define CMD_START_MEASUREMENT   0x0010
#define CMD_STOP_MEASUREMENT    0x0104
#define CMD_READ_DATA_READY     0x0202
#define CMD_READ_MEASURED       0x0300
#define CMD_READ_DEVICE_STATUS  0xD206
#define CMD_START_FAN_CLEAN     0x5607
#define CMD_RESET               0xD304
#define CMD_READ_SERIAL         0xD033   // V2.3.30: 32-byte ASCII serial (null-padded)

// Wire format selector for CMD_START_MEASUREMENT.
//   0x03 0x00 — big-endian IEEE-754 float (60 bytes per read)
//   0x05 0x00 — big-endian uint16 (20 bytes per read)
// Float mode keeps the full driver-side precision and matches what dusty-code
// uses for sensor.community / Madavi uploads.
#define FLOAT_MODE_HI           0x03
#define FLOAT_MODE_LO           0x00

static i2c_master_dev_handle_t s_dev   = NULL;
static bool                    s_ready = false;

// CRC-8: polynomial 0x31, init 0xFF, no reflection, no XOR-out (Sensirion
// standard — same as SHT45 / BMP390). Computed per 16-bit word with the high
// byte fed first, then the low byte.
static uint8_t crc8(const uint8_t *p, size_t n) {
    uint8_t c = 0xFF;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int b = 0; b < 8; b++) {
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x31) : (uint8_t)(c << 1);
        }
    }
    return c;
}

// Write a 2-byte command with no parameters.
static esp_err_t send_cmd(uint16_t cmd) {
    uint8_t buf[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

// Write a 2-byte command followed by one 16-bit parameter word (with its CRC).
// Used only for CMD_START_MEASUREMENT in this driver.
static esp_err_t send_cmd_arg(uint16_t cmd, uint8_t arg_hi, uint8_t arg_lo) {
    uint8_t buf[5];
    buf[0] = (uint8_t)(cmd >> 8);
    buf[1] = (uint8_t)(cmd & 0xFF);
    buf[2] = arg_hi;
    buf[3] = arg_lo;
    buf[4] = crc8(&buf[2], 2);
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

// Read N bytes after a previously-sent command. Sensirion's protocol is split-
// transaction: write command, optional delay, then read.
static esp_err_t recv(uint8_t *buf, size_t n) {
    return i2c_master_receive(s_dev, buf, n, 100);
}

// V2.3.30: read the factory serial (cmd 0xD033). Sensirion's wire format
// returns 48 bytes = 16 word-pairs, where each pair is 2 ASCII chars
// followed by 1 CRC byte (32 ASCII total + 16 CRC = 48). The serial is
// null-padded, typically 8-16 chars on Sensirion-shipped SPS30s.
//
// outsz must be ≥ 33 (32 chars + null). On success, *out is null-terminated.
// Returns ESP_OK only if all 16 CRCs validate AND the recv succeeded.
static esp_err_t sps30_read_serial(char *out, size_t outsz) {
    if (!s_dev || !out || outsz < 33) return ESP_ERR_INVALID_ARG;
    esp_err_t err = send_cmd(CMD_READ_SERIAL);
    if (err != ESP_OK) return err;
    // V2.3.31: busy-wait. vTaskDelay(pdMS_TO_TICKS(5)) at 100 Hz tick = 1 tick
    // = 0..10 ms actual; the 0 ms case races the chip's max-5-ms response time.
    esp_rom_delay_us(5000);

    uint8_t buf[48];
    err = recv(buf, sizeof(buf));
    if (err != ESP_OK) return err;

    size_t out_idx = 0;
    for (size_t i = 0; i < sizeof(buf); i += 3) {
        if (crc8(&buf[i], 2) != buf[i + 2]) return ESP_FAIL;
        out[out_idx++] = (char)buf[i];
        out[out_idx++] = (char)buf[i + 1];
    }
    out[out_idx] = '\0';
    // Trim trailing nulls / non-printables (Sensirion null-pads short serials).
    for (size_t i = 0; i < out_idx; i++) {
        if (out[i] == '\0') break;
        if (out[i] < 0x20 || out[i] > 0x7E) { out[i] = '\0'; break; }
    }
    return ESP_OK;
}

esp_err_t sps30_init(i2c_master_bus_handle_t bus) {
    if (s_ready) return ESP_OK;

    if (i2c_master_probe(bus, SPS30_ADDR, 50) != ESP_OK) {
        ESP_LOGW(TAG, "SPS30 not found at 0x%02X", SPS30_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SPS30_ADDR,
        .scl_speed_hz    = SPS30_BUS_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &devcfg, &s_dev);
    if (err != ESP_OK) return err;

    // Soft reset clears any in-progress state from a previous boot. Datasheet
    // requires ≥100 ms before the next command; we use 120 ms to be safe.
    if (send_cmd(CMD_RESET) != ESP_OK) {
        ESP_LOGW(TAG, "SPS30 reset NACK — continuing anyway");
    }
    vTaskDelay(pdMS_TO_TICKS(120));

    // Start continuous measurement in float wire-format mode.
    err = send_cmd_arg(CMD_START_MEASUREMENT, FLOAT_MODE_HI, FLOAT_MODE_LO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPS30 start_measurement: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return err;
    }

    // First measurement is available ~20 ms after start_measurement; allow
    // 30 ms before the first read to be conservative.
    vTaskDelay(pdMS_TO_TICKS(30));

    s_ready = true;

    // V2.3.30: log the factory serial — diagnostic aid for tracking
    // physical chips. Read AFTER start_measurement so a partial bus
    // reset / re-init flow doesn't leave the chip half-configured if
    // the serial read happens to fail. Failure is non-fatal.
    char serial[33] = {0};
    if (sps30_read_serial(serial, sizeof(serial)) == ESP_OK) {
        ESP_LOGI(TAG, "SPS30 serial = \"%s\"", serial);
    } else {
        ESP_LOGW(TAG, "SPS30 serial read failed (chip otherwise OK)");
    }

    ESP_LOGI(TAG, "SPS30 ready at 0x%02X (continuous float mode, fan ON, "
             "auto-clean every 7 days)", SPS30_ADDR);
    return ESP_OK;
}

bool sps30_present(void) {
    return s_ready;
}

// Poll CMD_READ_DATA_READY until the flag word reads 0x0001. Returns ESP_OK
// when ready, ESP_ERR_TIMEOUT if no fresh sample appears within ~1 s.
static esp_err_t wait_data_ready(void) {
    for (int i = 0; i < 10; i++) {
        if (send_cmd(CMD_READ_DATA_READY) != ESP_OK) return ESP_FAIL;
        esp_rom_delay_us(5000);   // V2.3.31: precise wait — see file header
        uint8_t buf[3];
        if (recv(buf, sizeof(buf)) != ESP_OK) return ESP_FAIL;
        if (crc8(buf, 2) != buf[2]) return ESP_FAIL;
        if (buf[1] == 0x01) return ESP_OK;     // low byte = ready flag
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t sps30_read(pm_sample_t *out) {
    if (!s_ready || !s_dev) return ESP_FAIL;
    if (!out) return ESP_ERR_INVALID_ARG;

    esp_err_t err = wait_data_ready();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SPS30 data not ready: %s", esp_err_to_name(err));
        return err;
    }

    if (send_cmd(CMD_READ_MEASURED) != ESP_OK) return ESP_FAIL;
    // V2.3.31: busy-wait — vTaskDelay(pdMS_TO_TICKS(5)) at 100 Hz = 0..10 ms.
    esp_rom_delay_us(5000);

    // Float mode: 10 floats × (2 bytes data + 1 byte CRC) × 2 words = 60 bytes.
    // Layout per float: [hi_word_b0, hi_word_b1, hi_crc, lo_word_b0, lo_word_b1, lo_crc].
    uint8_t buf[60];
    if (recv(buf, sizeof(buf)) != ESP_OK) return ESP_FAIL;

    float values[10];
    for (int i = 0; i < 10; i++) {
        const uint8_t *p = buf + i * 6;
        if (crc8(p + 0, 2) != p[2]) {
            ESP_LOGW(TAG, "SPS30 CRC fail on word %d", 2 * i);
            return ESP_FAIL;
        }
        if (crc8(p + 3, 2) != p[5]) {
            ESP_LOGW(TAG, "SPS30 CRC fail on word %d", 2 * i + 1);
            return ESP_FAIL;
        }
        // Reassemble big-endian 32-bit IEEE-754 float.
        uint32_t bits = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                        ((uint32_t)p[3] << 8)  |  (uint32_t)p[4];
        memcpy(&values[i], &bits, sizeof(values[i]));
    }

    out->pm1_0 = values[0];
    out->pm2_5 = values[1];
    out->pm4_0 = values[2];
    out->pm10  = values[3];
    out->nc0_5 = values[4];
    out->nc1_0 = values[5];
    out->nc2_5 = values[6];
    out->nc4_0 = values[7];
    out->nc10  = values[8];
    out->typ_size_um = values[9];
    return ESP_OK;
}

esp_err_t sps30_start_fan_cleaning(void) {
    if (!s_ready) return ESP_FAIL;
    return send_cmd(CMD_START_FAN_CLEAN);
}

esp_err_t sps30_read_device_status(uint32_t *status_out) {
    if (!s_ready || !s_dev) return ESP_FAIL;
    if (!status_out) return ESP_ERR_INVALID_ARG;

    if (send_cmd(CMD_READ_DEVICE_STATUS) != ESP_OK) return ESP_FAIL;
    // V2.3.31: busy-wait — vTaskDelay(pdMS_TO_TICKS(5)) at 100 Hz = 0..10 ms.
    esp_rom_delay_us(5000);

    // 4-byte status word as 2 words × (2 data bytes + 1 CRC) = 6 bytes.
    uint8_t buf[6];
    if (recv(buf, sizeof(buf)) != ESP_OK) return ESP_FAIL;
    if (crc8(buf + 0, 2) != buf[2]) {
        ESP_LOGW(TAG, "SPS30 status CRC fail (word 0)");
        return ESP_FAIL;
    }
    if (crc8(buf + 3, 2) != buf[5]) {
        ESP_LOGW(TAG, "SPS30 status CRC fail (word 1)");
        return ESP_FAIL;
    }
    *status_out = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                  ((uint32_t)buf[3] << 8)  |  (uint32_t)buf[4];
    return ESP_OK;
}
