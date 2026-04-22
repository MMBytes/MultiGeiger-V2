#include "bme280.h"

#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bme280";

// --- Board wiring (Heltec Wireless Stick V2) --------------------------------
#define PIN_SDA        4
#define PIN_SCL       15
#define I2C_PORT       I2C_NUM_0
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
// ctrl_hum:  osrs_h = x2 (010)
// ctrl_meas: osrs_t = x8 (100) | osrs_p = x4 (011) | mode = normal (11)
// config:    t_sb = 1000 ms (101) | filter = coef 4 (011) | spi3w_en = 0
#define CTRL_HUM_VAL   0x02
#define CTRL_MEAS_VAL  ((0x04 << 5) | (0x03 << 2) | 0x03)
#define CONFIG_VAL     ((0x05 << 5) | (0x03 << 2) | 0x00)

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

static esp_err_t probe_and_attach(uint8_t *found_addr) {
    const uint8_t candidates[2] = { BME_ADDR_A, BME_ADDR_B };
    for (int i = 0; i < 2; i++) {
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

esp_err_t bme280_init(void) {
    if (s_ready) return ESP_OK;

    // Build the master bus on the Heltec V2 I2C pins. Internal pull-ups on
    // as a belt-and-braces — the PCB already has 4.7k externals on J_I2C.
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port         = I2C_PORT,
        .sda_io_num       = PIN_SDA,
        .scl_io_num       = PIN_SCL,
        .clk_source       = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t addr = 0;
    err = probe_and_attach(&addr);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BME280 not found on I2C (tried 0x76, 0x77)");
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
    if ((err = write_reg(REG_CTRL_HUM,  CTRL_HUM_VAL))  != ESP_OK) return err;
    if ((err = write_reg(REG_CTRL_MEAS, CTRL_MEAS_VAL)) != ESP_OK) return err;
    if ((err = write_reg(REG_CONFIG,    CONFIG_VAL))    != ESP_OK) return err;

    s_ready = true;
    ESP_LOGI(TAG, "BME280 ready (osrs T=x8 P=x4 H=x2, filter=4, t_sb=1000ms, normal mode)");
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

    uint8_t d[8];
    esp_err_t err = read_regs(REG_DATA, d, sizeof(d));
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
