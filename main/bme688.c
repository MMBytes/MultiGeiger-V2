#include "bme688.h"

#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bme688";

#define I2C_FREQ_HZ    100000

#define BME_ADDR_A     0x76
#define BME_ADDR_B     0x77

// --- Registers ---------------------------------------------------------------
// Same chip family covers BME680 and BME688. The T/P/H register map is
// identical between the two; only the gas-sensor side differs (BME688 adds
// AI/parallel modes). We use neither, so this driver works unchanged on both.
#define REG_CAL_BLK1   0x8A   // 23 bytes: par_T2/T3/P1..P10 (with padding)
#define REG_CAL_BLK2   0xE1   // 14 bytes: par_H1/H2..H7, then par_T1
#define REG_ID         0xD0   // chip ID — should read 0x61
#define REG_RESET      0xE0   // write 0xB6 for soft reset (~10 ms)
#define REG_VARIANT    0xF0   // 0x00 = BME680, 0x01 = BME688 (informational)
#define REG_CTRL_GAS_0 0x70   // bit 3 = heat_off (1 = disable heater)
#define REG_CTRL_GAS_1 0x71   // bit 4 = run_gas (0 = no gas conversion)
#define REG_CTRL_HUM   0x72   // osrs_h[2:0]
#define REG_CTRL_MEAS  0x74   // osrs_t[7:5] | osrs_p[4:2] | mode[1:0]
#define REG_CONFIG     0x75   // filter[4:2]
#define REG_MEAS_STAT0 0x1D   // bit 7 new_data, bit 5 measuring, bit 6 gas_meas
#define REG_DATA_PTH   0x1F   // 8 bytes: P(3) T(3) H(2)

#define CHIP_ID_BME68X 0x61

#define MODE_SLEEP     0x00
#define MODE_FORCED    0x01

// --- Oversampling / filter settings -----------------------------------------
// ctrl_hum:    osrs_h = x1 (001) — humidity noise is sensor-floor-limited;
//              going higher just costs conversion time without measurable gain.
// ctrl_meas:   osrs_t = x2 (010) | osrs_p = x16 (101) | mode bits set per read.
//              T x2 keeps self-heating bias minimal (it propagates to H via
//              the calibration formula); P x16 maximises pressure resolution.
//              Matches BME680 application note BST-BME680-AN014 example
//              configuration.
// config:      filter = OFF (000) at bits[4:2] — at our 150 s read interval
//              the filter time-constant becomes pure latency rather than useful
//              smoothing (oversampling already does multi-sample averaging
//              within each forced conversion).
// ctrl_gas_0:  heat_off = 1 (heater always off — gas channel disabled)
// ctrl_gas_1:  run_gas  = 0 (no gas conversion appended to T/P/H)
#define CTRL_HUM_VAL    0x01
#define CTRL_MEAS_BASE  ((0x02 << 5) | (0x05 << 2))   // mode bits OR'd in per read
#define CONFIG_VAL      0x00
#define CTRL_GAS_0_VAL  0x08      // heat_off = 1
#define CTRL_GAS_1_VAL  0x00      // run_gas  = 0

// --- State -------------------------------------------------------------------
static i2c_master_bus_handle_t  s_bus   = NULL;
static i2c_master_dev_handle_t  s_dev   = NULL;
static bool                     s_ready = false;

// Calibration coefficients — names match the Bosch BME68x reference driver
// so the compensation formulas below read like the published source.
// All values are derived once at init from the on-chip NVM.
static uint16_t par_T1;
static int16_t  par_T2;
static int8_t   par_T3;
static uint16_t par_P1;
static int16_t  par_P2;
static int8_t   par_P3;
static int16_t  par_P4, par_P5;
static int8_t   par_P6, par_P7;
static int16_t  par_P8, par_P9;
static uint8_t  par_P10;
static uint16_t par_H1, par_H2;
static int8_t   par_H3, par_H4, par_H5;
static uint8_t  par_H6;
static int8_t   par_H7;

// --- Low-level I2C helpers ---------------------------------------------------

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t n) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, 100);
}

// --- Probe / attach ----------------------------------------------------------

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

// Walk both candidate addresses, attach, and verify chip ID 0x61. The chip-ID
// check is what distinguishes us from BME280 (0x60) and BMP390 (0x60) at the
// same I2C addresses — without it we would false-positive on either.
static esp_err_t probe_and_attach(uint8_t *found_addr, bool skip_addr_77) {
    const uint8_t candidates[2] = { BME_ADDR_A, BME_ADDR_B };
    for (int i = 0; i < 2; i++) {
        if (skip_addr_77 && candidates[i] == BME_ADDR_B) continue;
        if (i2c_master_probe(s_bus, candidates[i], 100) != ESP_OK) continue;
        esp_err_t err = attach_dev(candidates[i]);
        if (err != ESP_OK) return err;
        uint8_t id = 0;
        if (read_regs(REG_ID, &id, 1) == ESP_OK && id == CHIP_ID_BME68X) {
            *found_addr = candidates[i];
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

// --- Calibration -------------------------------------------------------------

// Calibration is split across two NVM blocks (0x8A..0xA0 and 0xE1..0xEA);
// the layout matches Bosch's official BME68x driver indices exactly so the
// extraction below mirrors the reference. Extra bytes inside each block
// (heater coefficients, padding) are simply not read out — we don't use the
// gas channel.
static esp_err_t read_calibration(void) {
    uint8_t b1[23];
    esp_err_t err = read_regs(REG_CAL_BLK1, b1, sizeof(b1));
    if (err != ESP_OK) return err;

    uint8_t b2[10];   // 0xE1..0xEA — covers H1..H7 plus T1
    err = read_regs(REG_CAL_BLK2, b2, sizeof(b2));
    if (err != ESP_OK) return err;

    // Pressure + temperature coefficients (block 1).
    par_T2  = (int16_t) (b1[0]  | ((uint16_t)b1[1]  << 8));
    par_T3  = (int8_t)   b1[2];
    // b1[3] is padding
    par_P1  = (uint16_t)(b1[4]  | ((uint16_t)b1[5]  << 8));
    par_P2  = (int16_t) (b1[6]  | ((uint16_t)b1[7]  << 8));
    par_P3  = (int8_t)   b1[8];
    // b1[9] is padding
    par_P4  = (int16_t) (b1[10] | ((uint16_t)b1[11] << 8));
    par_P5  = (int16_t) (b1[12] | ((uint16_t)b1[13] << 8));
    par_P7  = (int8_t)   b1[14];
    par_P6  = (int8_t)   b1[15];
    // b1[16..17] is padding
    par_P8  = (int16_t) (b1[18] | ((uint16_t)b1[19] << 8));
    par_P9  = (int16_t) (b1[20] | ((uint16_t)b1[21] << 8));
    par_P10 = (uint8_t)  b1[22];

    // Humidity coefficients (block 2). H1 and H2 are 12-bit values that share
    // the nibble at 0xE2 — H2 takes the high nibble, H1 takes the low nibble.
    par_H2 = (uint16_t)(((uint16_t)b2[0] << 4) | (b2[1] >> 4));
    par_H1 = (uint16_t)(((uint16_t)b2[2] << 4) | (b2[1] & 0x0F));
    par_H3 = (int8_t)   b2[3];
    par_H4 = (int8_t)   b2[4];
    par_H5 = (int8_t)   b2[5];
    par_H6 = (uint8_t)  b2[6];
    par_H7 = (int8_t)   b2[7];

    // Temperature T1 lives at the end of block 2 (0xE9 LSB / 0xEA MSB).
    par_T1 = (uint16_t)(b2[8] | ((uint16_t)b2[9] << 8));

    return ESP_OK;
}

// --- Init --------------------------------------------------------------------

esp_err_t bme688_init(i2c_master_bus_handle_t bus, bool skip_addr_77) {
    if (s_ready) return ESP_OK;

    // Bus is owned by env_sensor — just record the handle for our helpers.
    s_bus = bus;

    uint8_t addr = 0;
    esp_err_t err = probe_and_attach(&addr, skip_addr_77);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BME680/688 not found on I2C (tried %s)",
                 skip_addr_77 ? "0x76 only" : "0x76 and 0x77");
        return err;
    }

    // Variant byte separates BME680 (0x00) from BME688 (0x01). Logged as info
    // only — we drive both identically since the gas channel is disabled.
    uint8_t variant = 0;
    (void)read_regs(REG_VARIANT, &variant, 1);
    ESP_LOGI(TAG, "%s found at 0x%02X (chip ID 0x61 verified)",
             variant == 0x01 ? "BME688" : "BME680", addr);

    // Soft reset, then settle. Datasheet calls for ~5 ms; 10 ms is generous
    // and matches the BME280 driver's pattern.
    err = write_reg(REG_RESET, 0xB6);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    err = read_calibration();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "calibration read failed: %s", esp_err_to_name(err));
        return err;
    }

    // ctrl_hum must be written BEFORE ctrl_meas (datasheet §5.3.4). Gas
    // controls are written first to ensure the heater is off and run_gas=0
    // before any forced-mode conversion can be triggered.
    if ((err = write_reg(REG_CTRL_GAS_0, CTRL_GAS_0_VAL)) != ESP_OK) return err;
    if ((err = write_reg(REG_CTRL_GAS_1, CTRL_GAS_1_VAL)) != ESP_OK) return err;
    if ((err = write_reg(REG_CTRL_HUM,   CTRL_HUM_VAL))   != ESP_OK) return err;
    if ((err = write_reg(REG_CONFIG,     CONFIG_VAL))     != ESP_OK) return err;

    // Leave the chip in sleep — bme688_read() flips it to forced mode per call.
    err = write_reg(REG_CTRL_MEAS, CTRL_MEAS_BASE | MODE_SLEEP);
    if (err != ESP_OK) return err;

    s_ready = true;
    ESP_LOGI(TAG, "BME68x ready (osrs T=x2 P=x16 H=x1, IIR off, gas off, forced mode)");
    return ESP_OK;
}

bool bme688_present(void) {
    return s_ready;
}

// --- Compensation (Bosch BME68x reference, floating-point path, datasheet
//     §5.3.2.4 / §5.3.2.5 / §5.3.2.6) ----------------------------------------
//
// Float over int because the ESP32 has hardware FPU and the formulas are
// considerably easier to read this way. Matches the BMP390 driver's choice.

static float compensate_temperature(uint32_t adc_T, float *t_fine_out) {
    float var1 = ((float)adc_T / 16384.0f - (float)par_T1 / 1024.0f) * (float)par_T2;
    float dt   = (float)adc_T / 131072.0f - (float)par_T1 / 8192.0f;
    float var2 = (dt * dt) * ((float)par_T3 * 16.0f);
    float t_fine = var1 + var2;
    if (t_fine_out) *t_fine_out = t_fine;
    return t_fine / 5120.0f;   // °C
}

static float compensate_pressure(uint32_t adc_P, float t_fine) {
    float var1 = (t_fine / 2.0f) - 64000.0f;
    float var2 = var1 * var1 * ((float)par_P6 / 131072.0f);
    var2 = var2 + (var1 * (float)par_P5 * 2.0f);
    var2 = (var2 / 4.0f) + ((float)par_P4 * 65536.0f);
    var1 = ((((float)par_P3 * var1 * var1) / 16384.0f) + ((float)par_P2 * var1)) / 524288.0f;
    var1 = (1.0f + (var1 / 32768.0f)) * (float)par_P1;
    if (var1 == 0.0f) return 0.0f;     // avoid divide-by-zero on a wild calib

    float p = 1048576.0f - (float)adc_P;
    p = ((p - (var2 / 4096.0f)) * 6250.0f) / var1;
    var1 = ((float)par_P9 * p * p) / 2147483648.0f;
    var2 = p * ((float)par_P8 / 32768.0f);
    float var3 = (p / 256.0f) * (p / 256.0f) * (p / 256.0f) * ((float)par_P10 / 131072.0f);
    return p + (var1 + var2 + var3 + ((float)par_P7 * 128.0f)) / 16.0f;   // Pa
}

static float compensate_humidity(uint16_t adc_H, float t_fine) {
    float temp_comp = t_fine / 5120.0f;
    float var1 = (float)adc_H -
                 (((float)par_H1 * 16.0f) + (((float)par_H3 / 2.0f) * temp_comp));
    float var2 = var1 *
                 (((float)par_H2 / 262144.0f) *
                  (1.0f + (((float)par_H4 / 16384.0f) * temp_comp) +
                          (((float)par_H5 / 1048576.0f) * temp_comp * temp_comp)));
    float var3 = (float)par_H6 / 16384.0f;
    float var4 = (float)par_H7 / 2097152.0f;

    float h = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
    if      (h > 100.0f) h = 100.0f;
    else if (h < 0.0f)   h = 0.0f;
    return h;   // %RH
}

// --- Read --------------------------------------------------------------------

esp_err_t bme688_read(float *t_out, float *h_out, float *p_out) {
    if (!s_ready) return ESP_FAIL;

    // Trigger one forced-mode conversion. Writing mode bits to ctrl_meas
    // wakes the chip, runs T/P/H acquisition with the configured oversampling,
    // then returns to sleep automatically.
    esp_err_t err = write_reg(REG_CTRL_MEAS, CTRL_MEAS_BASE | MODE_FORCED);
    if (err != ESP_OK) return err;

    // T x2 + P x16 + H x1 typical conversion time (datasheet 3.5.2.4):
    //   1.97 + (2+16+1)*1.97 = ~39 ms typ; max ~46 ms. Wait 60 ms for margin.
    // Gas channel is off so the ~150 ms heater time does not apply.
    vTaskDelay(pdMS_TO_TICKS(60));

    // Read 8 bytes: pressure(0..2 msb,lsb,xlsb) + temperature(3..5) + humidity(6..7).
    uint8_t d[8];
    err = read_regs(REG_DATA_PTH, d, sizeof(d));
    if (err != ESP_OK) return err;

    // 20-bit P and T (high nibble of xlsb is the lowest 4 bits); 16-bit H.
    uint32_t adc_P = ((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | ((uint32_t)d[2] >> 4);
    uint32_t adc_T = ((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | ((uint32_t)d[5] >> 4);
    uint16_t adc_H = ((uint16_t)d[6] << 8)  |  (uint16_t)d[7];

    // 0x80000 / 0x8000 are the "channel disabled / settling" sentinels — we
    // configured all three so treat them as a transient I/O hiccup.
    if (adc_T == 0x80000 || adc_P == 0x80000 || adc_H == 0x8000) return ESP_FAIL;

    float t_fine;
    float T = compensate_temperature(adc_T, &t_fine);
    float P = compensate_pressure(adc_P, t_fine);
    float H = compensate_humidity(adc_H, t_fine);

    if (t_out) *t_out = T;
    if (p_out) *p_out = P;
    if (h_out) *h_out = H;
    return ESP_OK;
}
