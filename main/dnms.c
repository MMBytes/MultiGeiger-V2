#include "dnms.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "dnms";

#define DNMS_ADDR               0x55
#define DNMS_BUS_HZ             100000

// Two-byte (16-bit BE) command codes — see hbitter/DNMS firmware and the
// canonical airrohr-firmware/dnms_i2c.h. All commands are write-only on the
// command itself; responses are split-transaction (write cmd, read N bytes).
#define CMD_RESET               0x0001
#define CMD_READ_VERSION        0x0002
#define CMD_CALCULATE_LEQ       0x0003
#define CMD_READ_DATA_READY     0x0004
#define CMD_READ_LEQ            0x0005

// Response sizes. All counted in raw bytes ON THE WIRE (data words + interleaved
// CRC bytes), NOT in payload bytes.
//   READ_VERSION: 9 words × (2 data + 1 CRC) = 27 bytes total → 18 chars after
//                 CRC stripping (NUL-terminated to 18+1 = 19 in the cache).
//   READ_DATA_READY: 1 word × (2 data + 1 CRC) = 3 bytes → 1 uint16 after stripping.
//   READ_LEQ: 6 words × (2 data + 1 CRC) = 18 bytes → 3 floats (12 bytes) after stripping.
#define VERSION_LEN_CHARS       18
#define VERSION_WIRE_LEN        27   // 9 words × 3 bytes
#define DATA_READY_WIRE_LEN     3
#define READ_LEQ_WIRE_LEN       18   // 6 words × 3 bytes

static i2c_master_dev_handle_t s_dev   = NULL;
static bool                    s_ready = false;
static char                    s_version[VERSION_LEN_CHARS + 1] = {0};

// CRC-8: polynomial 0x31, init 0xFF, no reflection, no XOR-out — Sensirion
// standard, identical to SHT45 / SPS30. Computed per 16-bit word with the
// high byte fed first, then the low byte.
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

// Read N bytes after a previously-sent command (split-transaction).
static esp_err_t recv(uint8_t *buf, size_t n) {
    return i2c_master_receive(s_dev, buf, n, 100);
}

// Strip per-word CRC from a wire payload: every 3 bytes = 2 data + 1 CRC.
// Returns ESP_OK if all CRCs match, ESP_FAIL on first mismatch. data_out
// receives the concatenated data bytes (length = wire_len * 2 / 3).
static esp_err_t strip_crc(const uint8_t *wire, size_t wire_len, uint8_t *data_out) {
    if (wire_len % 3 != 0) return ESP_ERR_INVALID_SIZE;
    size_t out_idx = 0;
    for (size_t i = 0; i < wire_len; i += 3) {
        if (crc8(&wire[i], 2) != wire[i + 2]) {
            ESP_LOGW(TAG, "DNMS CRC fail at wire offset %u", (unsigned)i);
            return ESP_FAIL;
        }
        data_out[out_idx++] = wire[i];
        data_out[out_idx++] = wire[i + 1];
    }
    return ESP_OK;
}

esp_err_t dnms_init(i2c_master_bus_handle_t bus) {
    if (s_ready) return ESP_OK;

    if (i2c_master_probe(bus, DNMS_ADDR, 50) != ESP_OK) {
        ESP_LOGW(TAG, "DNMS not found at 0x%02X", DNMS_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = DNMS_ADDR,
        .scl_speed_hz    = DNMS_BUS_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &devcfg, &s_dev);
    if (err != ESP_OK) return err;

    // Soft reset clears any in-progress state from a previous boot. The Teensy
    // takes ~1 s to reinitialise its I²S microphone path and start sampling
    // (canonical wait per airrohr-firmware/initDNMS).
    if (send_cmd(CMD_RESET) != ESP_OK) {
        ESP_LOGW(TAG, "DNMS reset NACK — continuing anyway");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Read the firmware version string and verify it starts with "DNMS".
    // This catches a wrong-part populate at 0x55 (some Adafruit STEMMA
    // breakouts also use 0x55 — DPS310, AHT20 family).
    if (send_cmd(CMD_READ_VERSION) != ESP_OK) {
        ESP_LOGE(TAG, "DNMS READ_VERSION write failed");
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t wire[VERSION_WIRE_LEN];
    if (recv(wire, sizeof(wire)) != ESP_OK) {
        ESP_LOGE(TAG, "DNMS READ_VERSION read failed");
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_FAIL;
    }
    uint8_t version_bytes[VERSION_LEN_CHARS];
    if (strip_crc(wire, sizeof(wire), version_bytes) != ESP_OK) {
        ESP_LOGE(TAG, "DNMS READ_VERSION CRC failed");
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_FAIL;
    }
    if (version_bytes[0] != 'D' || version_bytes[1] != 'N' ||
        version_bytes[2] != 'M' || version_bytes[3] != 'S') {
        ESP_LOGW(TAG, "Device at 0x%02X is not DNMS (version starts 0x%02X 0x%02X 0x%02X 0x%02X)",
                 DNMS_ADDR, version_bytes[0], version_bytes[1],
                 version_bytes[2], version_bytes[3]);
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_FAIL;
    }

    memcpy(s_version, version_bytes, VERSION_LEN_CHARS);
    s_version[VERSION_LEN_CHARS] = 0;

    s_ready = true;
    ESP_LOGI(TAG, "DNMS ready at 0x%02X (version: %s)", DNMS_ADDR, s_version);
    return ESP_OK;
}

bool dnms_present(void) {
    return s_ready;
}

const char *dnms_get_version(void) {
    return s_ready ? s_version : "";
}

esp_err_t dnms_trigger(void) {
    if (!s_ready) return ESP_FAIL;
    return send_cmd(CMD_CALCULATE_LEQ);
}

esp_err_t dnms_data_ready(bool *ready) {
    if (!s_ready || !s_dev || !ready) return ESP_FAIL;

    if (send_cmd(CMD_READ_DATA_READY) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t wire[DATA_READY_WIRE_LEN];
    if (recv(wire, sizeof(wire)) != ESP_OK) return ESP_FAIL;

    uint8_t data[2];
    if (strip_crc(wire, sizeof(wire), data) != ESP_OK) return ESP_FAIL;

    // Low byte holds the ready flag (16-bit word, MSB always 0 in canonical
    // implementations). Non-zero = result available.
    *ready = (data[1] != 0);
    return ESP_OK;
}

esp_err_t dnms_read_leq(noise_sample_t *out) {
    if (!s_ready || !s_dev) return ESP_FAIL;
    if (!out) return ESP_ERR_INVALID_ARG;

    if (send_cmd(CMD_READ_LEQ) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t wire[READ_LEQ_WIRE_LEN];
    if (recv(wire, sizeof(wire)) != ESP_OK) return ESP_FAIL;

    // 6 words × 3 bytes = 18 wire bytes → 12 data bytes = 3 floats.
    uint8_t data[12];
    if (strip_crc(wire, sizeof(wire), data) != ESP_OK) return ESP_FAIL;

    float values[3];
    for (int i = 0; i < 3; i++) {
        // Each float = 4 bytes big-endian IEEE 754, assembled from two
        // adjacent (already-CRC-stripped) words at offset i*4.
        const uint8_t *p = data + i * 4;
        uint32_t bits = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                        ((uint32_t)p[2] << 8)  |  (uint32_t)p[3];
        memcpy(&values[i], &bits, sizeof(values[i]));
    }

    out->laeq   = values[0];
    out->la_min = values[1];
    out->la_max = values[2];
    return ESP_OK;
}
