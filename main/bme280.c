#include "bme280.h"

#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bme280";

#define I2C_FREQ_HZ    100000

#define BME_ADDR_A     0x76
#define BME_ADDR_B     0x77

// --- Registers ---------------------------------------------------------------
#define REG_CAL_TP     0x88   // 26 bytes: T1..T3, P1..P9, then pad to 0xA1
#define REG_CAL_H1     0xA1
#define REG_ID         0xD0   // should read 0x60
#define REG_RESET      0xE0   // write 0xB6 for soft reset
#define REG_CAL_H2     0xE1   // 7 bytes: H2..H6 (H4/H5 pack across E4/E5/E6)
#define REG_CTRL_HUM   0xF2
#define REG_STATUS     0xF3
#define REG_CTRL_MEAS  0xF4
#define REG_CONFIG     0xF5
#define REG_DATA       0xF7   // 8 bytes: P(3) T(3) H(2)

#define CHIP_ID_BME280 0x60

// --- Oversampling / filter settings -----------------------------------------
// Forced-mode profile, matching BMP390 / BME688 driver pattern in this project.
// Mode bits in ctrl_meas are set per-read by bme280_read() (sleep at init,
// forced for each conversion, then auto-return to sleep).
//   ctrl_hum:        osrs_h = x2 (010)
//   ctrl_meas BASE:  osrs_t = x8 (100) | osrs_p = x4 (011) | mode bits = 00 sleep
//   ctrl_meas READ:  CTRL_MEAS_BASE | 0x01 (forced — datasheet 5.4.5)
//   config:          t_sb irrelevant in forced mode | filter = OFF (000) | spi3w_en = 0
// IIR filter is off: at our 150 s read interval the filter time-constant becomes
// pure latency rather than useful smoothing (oversampling already does multi-
// sample averaging within each forced conversion). Matches Bosch's "Weather
// monitoring" recipe (datasheet table 7) and the BMP390/BME688 drivers.
#define CTRL_HUM_VAL    0x02
#define CTRL_MEAS_BASE  ((0x04 << 5) | (0x03 << 2))   // mode = sleep; OR 0x01 to force
#define CONFIG_VAL      0x00                          // filter off, spi3w off

// --- State -------------------------------------------------------------------
static i2c_master_bus_handle_t  s_bus   = NULL;
static i2c_master_dev_handle_t  s_dev   = NULL;
static bool                     s_ready = false;

// Raw Bosch calibration. Names match the datasheet so the compensation
// formulas below read like the reference implementation.
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t  dig_H1, dig_H3;
static int16_t  dig_H2, dig_H4, dig_H5;
static int8_t   dig_H6;

// --- Low-level I2C helpers ---------------------------------------------------

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t n) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, 100);
}

// --- Init --------------------------------------------------------------------

static esp_err_t attach_dev(uint8_t addr) {
    if (s_dev) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
}

static esp_err_t probe_and_attach(uint8_t *found_addr, bool skip_addr_77) {
    const uint8_t candidates[2] = { BME_ADDR_A, BME_ADDR_B };
    for (int i = 0; i < 2; i++) {
        if (skip_addr_77 && candidates[i] == BME_ADDR_B) continue;
        if (i2c_master_probe(s_bus, candidates[i], 100) == ESP_OK) {
            esp_err_t err = attach_dev(candidates[i]);
            if (err != ESP_OK) return err;
            uint8_t id = 0;
            if (read_regs(REG_ID, &id, 1) == ESP_OK && id == CHIP_ID_BME280) {
                *found_addr = candidates[i];
                return ESP_OK;
            }
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t read_calibration(void) {
    // T1..T3, P1..P9 live at 0x88..0x9F (24 bytes, little-endian pairs)
    uint8_t tp[24];
    esp_err_t err = read_regs(REG_CAL_TP, tp, sizeof(tp));
    if (err != ESP_OK) return err;

    dig_T1 = (uint16_t)(tp[0]  | (tp[1]  << 8));
    dig_T2 = (int16_t) (tp[2]  | (tp[3]  << 8));
    dig_T3 = (int16_t) (tp[4]  | (tp[5]  << 8));
    dig_P1 = (uint16_t)(tp[6]  | (tp[7]  << 8));
    dig_P2 = (int16_t) (tp[8]  | (tp[9]  << 8));
    dig_P3 = (int16_t) (tp[10] | (tp[11] << 8));
    dig_P4 = (int16_t) (tp[12] | (tp[13] << 8));
    dig_P5 = (int16_t) (tp[14] | (tp[15] << 8));
    dig_P6 = (int16_t) (tp[16] | (tp[17] << 8));
    dig_P7 = (int16_t) (tp[18] | (tp[19] << 8));
    dig_P8 = (int16_t) (tp[20] | (tp[21] << 8));
    dig_P9 = (int16_t) (tp[22] | (tp[23] << 8));

    err = read_regs(REG_CAL_H1, &dig_H1, 1);
    if (err != ESP_OK) return err;

    uint8_t h[7];
    err = read_regs(REG_CAL_H2, h, sizeof(h));
    if (err != ESP_OK) return err;
    dig_H2 = (int16_t)(h[0] | (h[1] << 8));
    dig_H3 = h[2];
    // dig_H4: 12-bit signed, H4[11:4] in 0xE4, H4[3:0] in low nibble of 0xE5
    dig_H4 = (int16_t)(((int16_t)(int8_t)h[3] << 4) | (h[4] & 0x0F));
    // dig_H5: 12-bit signed, H5[3:0] in high nibble of 0xE5, H5[11:4] in 0xE6
    dig_H5 = (int16_t)(((int16_t)(int8_t)h[5] << 4) | (h[4] >> 4));
    dig_H6 = (int8_t)h[6];
    return ESP_OK;
}

esp_err_t bme280_init(i2c_master_bus_handle_t bus, bool skip_addr_77) {
    if (s_ready) return ESP_OK;

    // Bus is owned by env_sensor — just record the handle for our helpers.
    s_bus = bus;

    uint8_t addr = 0;
    esp_err_t err = probe_and_attach(&addr, skip_addr_77);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BME280 not found on I2C (tried %s)",
                 skip_addr_77 ? "0x76 only" : "0x76 and 0x77");
        return err;
    }
    ESP_LOGI(TAG, "BME280 found at 0x%02X (chip ID 0x60 verified)", addr);

    // Soft reset, give the chip its 2 ms settling time, then poll the
    // calibration-copying bit until the NVM is ready (datasheet 5.4.1).
    err = write_reg(REG_RESET, 0xB6);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(5));
    for (int i = 0; i < 20; i++) {
        uint8_t status = 0;
        if (read_regs(REG_STATUS, &status, 1) == ESP_OK && !(status & 0x01)) break;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    err = read_calibration();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "calibration read failed: %s", esp_err_to_name(err));
        return err;
    }

    // ctrl_hum must be written BEFORE ctrl_meas (datasheet 5.4.3).
    // Mode bits left at 00 (sleep) here — bme280_read() flips them to forced per call.
    if ((err = write_reg(REG_CTRL_HUM,  CTRL_HUM_VAL))   != ESP_OK) return err;
    if ((err = write_reg(REG_CTRL_MEAS, CTRL_MEAS_BASE)) != ESP_OK) return err;
    if ((err = write_reg(REG_CONFIG,    CONFIG_VAL))     != ESP_OK) return err;

    s_ready = true;
    ESP_LOGI(TAG, "BME280 ready (osrs T=x8 P=x4 H=x2, filter off, forced mode)");
    return ESP_OK;
}

bool bme280_present(void) {
    return s_ready;
}

i2c_master_bus_handle_t bme280_get_i2c_bus(void) {
    return s_bus;
}

// --- Compensation (Bosch reference, datasheet 8.2) --------------------------

static int32_t  compensate_T_int32(int32_t adc_T, int32_t *t_fine_out) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                    ((int32_t)dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    if (t_fine_out) *t_fine_out = t_fine;
    return (t_fine * 5 + 128) >> 8;  // °C * 100
}

static uint32_t compensate_P_int64(int32_t adc_P, int32_t t_fine) {
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dig_P6;
    var2 += ((var1 * (int64_t)dig_P5) << 17);
    var2 += ((int64_t)dig_P4) << 35;
    var1  = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1  = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0) return 0;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)p;  // Q24.8 Pa
}

static uint32_t compensate_H_int32(int32_t adc_H, int32_t t_fine) {
    int32_t v = t_fine - 76800;
    v = ((((adc_H << 14) - ((int32_t)dig_H4 << 20) - ((int32_t)dig_H5 * v)) +
          16384) >> 15) *
        (((((((v * (int32_t)dig_H6) >> 10) *
             (((v * (int32_t)dig_H3) >> 11) + 32768)) >> 10) + 2097152) *
           (int32_t)dig_H2 + 8192) >> 14);
    v = v - (((((v >> 15) * (v >> 15)) >> 7) * (int32_t)dig_H1) >> 4);
    if (v < 0) v = 0;
    if (v > 419430400) v = 419430400;
    return (uint32_t)(v >> 12);  // %RH * 1024
}

// --- Read --------------------------------------------------------------------

esp_err_t bme280_read(float *t_out, float *h_out, float *p_out) {
    if (!s_ready) return ESP_FAIL;

    // Trigger one forced-mode conversion. Writing the mode bits to ctrl_meas
    // wakes the chip, runs T/P/H acquisition with the configured oversampling,
    // then returns to sleep automatically.
    esp_err_t err = write_reg(REG_CTRL_MEAS, CTRL_MEAS_BASE | 0x01);
    if (err != ESP_OK) return err;

    // T x8 + P x4 + H x2 worst-case measurement time per datasheet 9.1:
    //   t_meas = 1.25 + 2.3*8 + (2.3*4 + 0.575) + (2.3*2 + 0.575) ≈ 30.6 ms
    // Wait 35 ms for comfortable margin.
    vTaskDelay(pdMS_TO_TICKS(35));

    uint8_t d[8];
    err = read_regs(REG_DATA, d, sizeof(d));
    if (err != ESP_OK) return err;

    int32_t adc_P = (int32_t)(((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | (d[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | (d[5] >> 4));
    int32_t adc_H = (int32_t)(((uint32_t)d[6] << 8)  |  (uint32_t)d[7]);

    // Sensor reports 0x80000 on T/P and 0x8000 on H when a channel is
    // disabled or still settling. We configured all three, so treat these
    // as a transient I/O hiccup and surface an error rather than compute.
    if (adc_T == 0x80000 || adc_P == 0x80000 || adc_H == 0x8000) return ESP_FAIL;

    int32_t t_fine;
    int32_t  T  = compensate_T_int32(adc_T, &t_fine);
    uint32_t P  = compensate_P_int64(adc_P, t_fine);
    uint32_t H  = compensate_H_int32(adc_H, t_fine);

    if (t_out) *t_out = T / 100.0f;
    if (p_out) *p_out = P / 256.0f;
    if (h_out) *h_out = H / 1024.0f;
    return ESP_OK;
}
