#include "bmp390.h"

#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "telemetry.h"

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
//   OSR:      osr_p = 101 (x32) at bits[2:0]; osr_t = 000 (x1) at bits[5:3]
//   CONFIG:   iir_filter = 000 (off) at bits[3:1] — at our 150 s read interval
//             the filter time-constant is pure latency; oversampling already
//             does the per-read averaging.
// Oversampling matches Bosch's "drone" / highest-pressure-resolution preset
// (datasheet table 10): pressure is the channel where oversampling pays off
// (sub-Pa noise = sub-cm altitude resolution); temperature is just used to
// linearise the pressure compensation, so x1 is sufficient.
#define PWR_CTRL_FORCED     0x13   // press_en | temp_en | forced
#define OSR_VAL             0x05   // (000 << 3) | 101
#define CONFIG_VAL          0x00   // iir_filter off

static i2c_master_dev_handle_t s_dev   = NULL;
static bool                    s_ready = false;

// V2.6.19: last-read cache consumed by the telemetry read callbacks below.
// Written ONLY by bmp390_read(); read ONLY by sd_logger's row build on the
// same task — no locking needed. Invalidated on read failure so the CSV
// emits an empty cell, never stale data.
static bool  s_tm_valid;
static float s_tm_t_c;
static float s_tm_p_pa;

static bool tm_read_t(char *cell, size_t cap, void *arg) {
    (void)arg;
    if (!s_tm_valid) return false;
    snprintf(cell, cap, "%.2f", (double)s_tm_t_c);
    return true;
}

static bool tm_read_p(char *cell, size_t cap, void *arg) {
    (void)arg;
    if (!s_tm_valid) return false;
    snprintf(cell, cap, "%.2f", (double)s_tm_p_pa / 100.0);
    return true;
}

// Floating-point calibration coefficients — derived from NVM once at init.
// Names follow the Bosch compensation algorithm (datasheet 8.5).
static double par_T1, par_T2, par_T3;
static double par_P1, par_P2, par_P3, par_P4;
static double par_P5, par_P6, par_P7, par_P8, par_P9, par_P10, par_P11;

// --- Calibration -------------------------------------------------------------

static esp_err_t load_calibration(void) {
    uint8_t d[21];
    esp_err_t err = i2c_dev_read_regs(s_dev, REG_CALIB, d, sizeof(d));
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
    // V2.3.16: par_T1 divisor was 2.52e-2 — a typo from when this driver was
    // written for V2.3.6. Should be 2^-8 = 0.00390625, equivalent to multiplying
    // by 256. Symptom: par_T1 came out ~6.5× too small → t_lin wildly off
    // (~120 °C instead of ~18 °C at room temp) → pressure compensation
    // produced ~+25 % offset → reported pressure 1287 hPa when actual was
    // 1024 hPa. Surfaced 2026-05-10 on the dust node FeatherS3 first time
    // BMP390 was actually wired in production. Bosch SDK and Adafruit
    // BMP3xx library both use pow(2, -8) here. Cross-checked every other
    // constant in this block — only par_T1 had the typo.
    par_T1  = (double)nvm_T1  * 256.0;        // / 2^-8 = * 2^8 = * 256
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

    esp_err_t err = i2c_add_device(bus, BMP390_ADDR, 100000, &s_dev);
    if (err != ESP_OK) return err;

    uint8_t chip_id = 0;
    err = i2c_dev_read_regs(s_dev, REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK || chip_id != CHIP_ID_BMP390) {
        ESP_LOGW(TAG, "BMP390 chip ID mismatch: got 0x%02X (want 0x%02X)",
                 chip_id, CHIP_ID_BMP390);
        i2c_dev_teardown(&s_dev);
        return ESP_ERR_NOT_FOUND;
    }

    err = load_calibration();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP390 calibration read failed: %s", esp_err_to_name(err));
        i2c_dev_teardown(&s_dev);
        return err;
    }

    // Configure oversampling and IIR filter. OSR and CONFIG are written once;
    // PWR_CTRL is written per-measurement to trigger forced-mode conversions.
    if ((err = i2c_dev_write_reg(s_dev, REG_OSR,    OSR_VAL))    != ESP_OK) return err;
    if ((err = i2c_dev_write_reg(s_dev, REG_CONFIG, CONFIG_VAL)) != ESP_OK) return err;

    s_ready = true;

    // Prime the oversampling chain with 10 throwaway samples before the first
    // real read. Bosch's BMP3xx KB confirms the first ~N samples after begin()
    // are unsettled even with IIR disabled — the OSR=32x averaging chain needs
    // actual measurement cycles to flush, a wall-clock delay alone won't do
    // it. Without priming the first reading post-init is consistently ~40 hPa
    // low (verified empirically in dusty-code's mh12 fix, 2026-05-08,
    // src/sensors/bmp3xx/bmp3xx.cpp). At OSR_p=32x / OSR_t=1x each forced read
    // is ~30 ms, so this costs ~300 ms once at boot — trivial inside
    // env_sensor_init which runs before WiFi/HTTP/TX.
    //
    // s_ready was set true above (Option A) so bmp390_read() doesn't refuse
    // these calls. Practically safe because nothing else runs during this
    // ~300 ms window — env_sensor_init is sequential, the first user-facing
    // read is 150 s away in cycle #1.
    for (int i = 0; i < 10; i++) {
        float t, p;
        (void)bmp390_read(&t, &p);
    }

    ESP_LOGI(TAG, "BMP390 ready at 0x%02X (P x32, T x1, IIR off, forced mode, "
             "primed 10 samples)", BMP390_ADDR);

    // V2.6.19: register our CSV columns once. Guards against a second,
    // idempotent init call (env_sensor's dual-bus probe) double-registering.
    static bool s_tm_registered = false;
    if (!s_tm_registered) {
        s_tm_registered = true;
        telemetry_register("BMP390 Temperature [C]", tm_read_t, NULL);
        telemetry_register("BMP390 Pressure [hPa]",  tm_read_p, NULL);
    }
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
    if (!s_ready) {
        s_tm_valid = false;
        return ESP_FAIL;
    }

    // Trigger a single forced-mode conversion.
    esp_err_t err = i2c_dev_write_reg(s_dev, REG_PWR_CTRL, PWR_CTRL_FORCED);
    if (err != ESP_OK) {
        s_tm_valid = false;
        return err;
    }

    // P x32 + T x1 worst-case measurement time per datasheet (234 + p*392 +
    // t*313 µs) is ~13 ms; 30 ms gives comfortable margin.
    vTaskDelay(pdMS_TO_TICKS(30));

    // 6 bytes: press[0..2] (xlsb, lsb, msb), temp[3..5] (xlsb, lsb, msb).
    uint8_t d[6];
    err = i2c_dev_read_regs(s_dev, REG_DATA_0, d, sizeof(d));
    if (err != ESP_OK) {
        s_tm_valid = false;
        return err;
    }

    int32_t adc_P = (int32_t)(d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16));
    int32_t adc_T = (int32_t)(d[3] | ((uint32_t)d[4] << 8) | ((uint32_t)d[5] << 16));

    double t_lin;
    double T = compensate_temperature(adc_T, &t_lin);
    double P = compensate_pressure(adc_P, t_lin);

    if (temperature_c) *temperature_c = (float)T;
    if (pressure_pa)   *pressure_pa   = (float)P;

    // V2.6.19: cache from the LOCAL computed values, not the (possibly NULL)
    // out-pointers — the telemetry callbacks above read this cache.
    s_tm_t_c   = (float)T;
    s_tm_p_pa  = (float)P;
    s_tm_valid = true;
    return ESP_OK;
}
