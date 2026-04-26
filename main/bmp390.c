#include "bmp390.h"

#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bmp390";

#define BMP390_ADDR         0x77
#define CHIP_ID_BMP390      0x60

// Register map (BMP390 datasheet section 5)
#define REG_CHIP_ID         0x00
#define REG_ERR             0x02
#define REG_DATA_0          0x04   // 6 bytes: P(3 bytes) T(3 bytes)
#define REG_PWR_CTRL        0x1B   // press_en | temp_en | mode[5:4]
#define REG_OSR             0x1C   // osr_p[2:0] | osr_t[5:3]
#define REG_CONFIG          0x1F   // iir_filter[3:1]
#define REG_CALIB           0x31   // 21 bytes of NVM calibration

// Operating profile
//   PWR_CTRL: bits[1:0] = press_en + temp_en, bits[5:4] = forced mode (01)
//   OSR:      osr_p = 011 (x8) at bits[2:0]; osr_t = 001 (x2) at bits[5:3]
//   CONFIG:   iir_filter = 000 (off) at bits[3:1] — at our 150 s read interval
//             the filter time-constant is pure latency; oversampling already
//             does the per-read averaging. Matches Bosch's "Weather monitoring"
//             ultra-low-power preset (datasheet table 10) and the project's
//             other env-sensor drivers.
#define PWR_CTRL_FORCED     0x13   // press_en | temp_en | forced
#define OSR_VAL             0x0B   // (001 << 3) | 011
#define CONFIG_VAL          0x00   // iir_filter off

static i2c_master_dev_handle_t s_dev   = NULL;
static bool                    s_ready = false;

// Floating-point calibration coefficients — derived from NVM once at init.
// Names follow the Bosch compensation algorithm (datasheet 8.5).
static double par_T1, par_T2, par_T3;
static double par_P1, par_P2, par_P3, par_P4;
static double par_P5, par_P6, par_P7, par_P8, par_P9, par_P10, par_P11;

// --- Low-level I2C helpers ---------------------------------------------------

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t n) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, 100);
}

// --- Calibration -------------------------------------------------------------

static esp_err_t load_calibration(void) {
    uint8_t d[21];
    esp_err_t err = read_regs(REG_CALIB, d, sizeof(d));
    if (err != ESP_OK) return err;

    // NVM values, little-endian. Types per datasheet table 10.
    uint16_t nvm_T1  = (uint16_t)(d[0]  | ((uint16_t)d[1]  << 8));
    uint16_t nvm_T2  = (uint16_t)(d[2]  | ((uint16_t)d[3]  << 8));
    int8_t   nvm_T3  = (int8_t)d[4];
    int16_t  nvm_P1  = (int16_t)(d[5]  | ((uint16_t)d[6]  << 8));
    int16_t  nvm_P2  = (int16_t)(d[7]  | ((uint16_t)d[8]  << 8));
    int8_t   nvm_P3  = (int8_t)d[9];
    int8_t   nvm_P4  = (int8_t)d[10];
    uint16_t nvm_P5  = (uint16_t)(d[11] | ((uint16_t)d[12] << 8));
    uint16_t nvm_P6  = (uint16_t)(d[13] | ((uint16_t)d[14] << 8));
    int8_t   nvm_P7  = (int8_t)d[15];
    int8_t   nvm_P8  = (int8_t)d[16];
    int16_t  nvm_P9  = (int16_t)(d[17] | ((uint16_t)d[18] << 8));
    int8_t   nvm_P10 = (int8_t)d[19];
    int8_t   nvm_P11 = (int8_t)d[20];

    // Scale factors from datasheet table 10 (pow(2, N) equivalents).
    par_T1  = (double)nvm_T1  / 2.52e-2;      // * 2^8   = / 2^-8
    par_T2  = (double)nvm_T2  / 1.07374182e9; // / 2^30
    par_T3  = (double)nvm_T3  / 2.81474977e14;// / 2^48

    par_P1  = ((double)nvm_P1  - 16384.0) / 1048576.0;  // / 2^20
    par_P2  = ((double)nvm_P2  - 16384.0) / 536870912.0;// / 2^29
    par_P3  = (double)nvm_P3  / 4.29496730e9;  // / 2^32
    par_P4  = (double)nvm_P4  / 1.37438953e11; // / 2^37
    par_P5  = (double)nvm_P5  / 0.125;          // * 2^3 = / 2^-3
    par_P6  = (double)nvm_P6  / 64.0;           // / 2^6
    par_P7  = (double)nvm_P7  / 256.0;          // / 2^8
    par_P8  = (double)nvm_P8  / 32768.0;        // / 2^15
    par_P9  = (double)nvm_P9  / 2.81474977e14;  // / 2^48
    par_P10 = (double)nvm_P10 / 2.81474977e14;  // / 2^48
    par_P11 = (double)nvm_P11 / 3.68934882e19;  // / 2^65

    return ESP_OK;
}

// --- Init --------------------------------------------------------------------

esp_err_t bmp390_init(i2c_master_bus_handle_t bus) {
    if (s_ready) return ESP_OK;

    if (i2c_master_probe(bus, BMP390_ADDR, 50) != ESP_OK) {
        ESP_LOGW(TAG, "BMP390 not found at 0x%02X", BMP390_ADDR);
        return ESP_ERR_NOT_FOUND;
    }

    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BMP390_ADDR,
        .scl_speed_hz    = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &devcfg, &s_dev);
    if (err != ESP_OK) return err;

    uint8_t chip_id = 0;
    err = read_regs(REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK || chip_id != CHIP_ID_BMP390) {
        ESP_LOGW(TAG, "BMP390 chip ID mismatch: got 0x%02X (want 0x%02X)",
                 chip_id, CHIP_ID_BMP390);
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    err = load_calibration();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP390 calibration read failed: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return err;
    }

    // Configure oversampling and IIR filter. OSR and CONFIG are written once;
    // PWR_CTRL is written per-measurement to trigger forced-mode conversions.
    if ((err = write_reg(REG_OSR,    OSR_VAL))    != ESP_OK) return err;
    if ((err = write_reg(REG_CONFIG, CONFIG_VAL)) != ESP_OK) return err;

    s_ready = true;
    ESP_LOGI(TAG, "BMP390 ready at 0x%02X (P x8, T x2, IIR off, forced mode)",
             BMP390_ADDR);
    return ESP_OK;
}

bool bmp390_present(void) {
    return s_ready;
}

// --- Compensation (Bosch datasheet section 8.5, floating-point path) ---------

static double compensate_temperature(int32_t adc_T, double *t_lin_out) {
    double pd1 = (double)adc_T - par_T1;
    double pd2 = pd1 * par_T2;
    double t_lin = pd2 + (pd1 * pd1) * par_T3;
    if (t_lin_out) *t_lin_out = t_lin;
    return t_lin;   // °C
}

static double compensate_pressure(int32_t adc_P, double t_lin) {
    double pd1 = par_P6 * t_lin;
    double pd2 = par_P7 * (t_lin * t_lin);
    double pd3 = par_P8 * (t_lin * t_lin * t_lin);
    double out1 = par_P5 + pd1 + pd2 + pd3;

    pd1  = par_P2 * t_lin;
    pd2  = par_P3 * (t_lin * t_lin);
    pd3  = par_P4 * (t_lin * t_lin * t_lin);
    double out2 = (double)adc_P * (par_P1 + pd1 + pd2 + pd3);

    pd1  = (double)adc_P * (double)adc_P;
    pd2  = par_P9 + par_P10 * t_lin;
    pd3  = pd1 * pd2;
    double pd4  = pd3 + (double)adc_P * (double)adc_P * (double)adc_P * par_P11;

    return out1 + out2 + pd4;   // Pa
}

// --- Read --------------------------------------------------------------------

esp_err_t bmp390_read(float *temperature_c, float *pressure_pa) {
    if (!s_ready) return ESP_FAIL;

    // Trigger a single forced-mode conversion.
    esp_err_t err = write_reg(REG_PWR_CTRL, PWR_CTRL_FORCED);
    if (err != ESP_OK) return err;

    // P x8 + T x2 worst-case measurement time is ~10.6 ms; 20 ms is safe.
    vTaskDelay(pdMS_TO_TICKS(20));

    // 6 bytes: press[0..2] (xlsb, lsb, msb), temp[3..5] (xlsb, lsb, msb).
    uint8_t d[6];
    err = read_regs(REG_DATA_0, d, sizeof(d));
    if (err != ESP_OK) return err;

    int32_t adc_P = (int32_t)(d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16));
    int32_t adc_T = (int32_t)(d[3] | ((uint32_t)d[4] << 8) | ((uint32_t)d[5] << 16));

    double t_lin;
    double T = compensate_temperature(adc_T, &t_lin);
    double P = compensate_pressure(adc_P, t_lin);

    if (temperature_c) *temperature_c = (float)T;
    if (pressure_pa)   *pressure_pa   = (float)P;
    return ESP_OK;
}
