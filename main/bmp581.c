#include "bmp581.h"

#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bmp581";

// Two possible I2C addresses depending on SDO tie. Adafruit #6407 ties SDO
// LOW (0x46), SparkFun ties SDO HIGH (0x47). We probe 0x46 first because
// Adafruit is the deployment target; 0x47 falls in cleanly for cross-board.
#define BMP581_ADDR_PRIMARY    0x46
#define BMP581_ADDR_SECONDARY  0x47

// Chip ID 0x50 = BMP581. ID 0x51 is the BMP585 (different Bosch product, NOT
// a "secondary" BMP581 — confirmed by ESPHome source). We reject 0x51 to catch
// wrong-part populates rather than silently treating it as compatible.
#define CHIP_ID_BMP581         0x50
#define CHIP_ID_BMP585         0x51

// Register map — datasheet §7. We only touch the few registers needed for
// forced-mode reads. DSP_CONFIG (0x30) reset value 0x03 already enables full
// P+T compensation with pre-IIR data routing — no write needed. DSP_IIR
// (0x31) reset value 0x00 already has both filters bypassed — no write
// needed either.
#define REG_CHIP_ID            0x01
#define REG_INT_STATUS         0x27   // bit 4 (por) clear-on-read
#define REG_STATUS             0x28   // bit 1 status_nvm_rdy, bit 2 status_nvm_err
#define REG_TEMP_DATA_XLSB     0x1D   // 6-byte burst: T_XLSB, T_LSB, T_MSB, P_XLSB, P_LSB, P_MSB
#define REG_OSR_CONFIG         0x36   // press_en | osr_p[5:3] | osr_t[2:0]
#define REG_ODR_CONFIG         0x37   // deep_dis | odr[6:2] | pwr_mode[1:0]
#define REG_CMD                0x7E   // soft reset = 0xB6

// Bosch's "high resolution" preset (datasheet Table 9, OSR ladder code 0b100):
// OSR_p = 16x, OSR_t = 1x → 0.21 PaRMS noise, 11.4 ms total conversion.
// Matches ESPHome's default for the same chip. At our 150 s polling interval
// atmospheric noise is far above 0.21 Pa, so higher OSR is wasted power.
//   bit 6   press_en  = 1   (enable pressure measurement)
//   bit 5:3 osr_p     = 100 (16x)
//   bit 2:0 osr_t     = 000 (1x)
#define OSR_CONFIG_VAL         0x60

// pwr_mode = FORCED (0b10), odr field ignored in forced mode, deep_dis = 0
// (allow auto-DEEP_STANDBY between cycles for 0.55 µA idle current).
#define ODR_CONFIG_FORCED      0x02
#define ODR_CONFIG_STANDBY     0x00

// ESPHome's wait formula: ceil(1.05 × (tconv_p + tconv_t)) ms. At 16x/1x
// that's ceil(1.05 × (10.4 + 1.0)) = 12 ms. Bosch datasheet §2 specs the
// individual tconv values with ±5 % tolerance, which the 1.05 factor already
// covers.
#define FORCED_WAIT_MS         12

static i2c_master_dev_handle_t s_dev   = NULL;
static bool                    s_ready = false;
static uint8_t                 s_addr  = 0;     // actual address we bound to (0x46 or 0x47)

// --- Low-level I2C helpers ---------------------------------------------------

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t n) {
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, 100);
}

// --- Init --------------------------------------------------------------------

esp_err_t bmp581_init(i2c_master_bus_handle_t bus) {
    if (s_ready) return ESP_OK;

    // Probe both candidate addresses. Each probe times out in ~50 ms when the
    // address is empty, so worst-case bring-up cost is ~100 ms when no BMP581
    // is fitted. That's well under the env_sensor probe budget.
    uint8_t addr = 0;
    if (i2c_master_probe(bus, BMP581_ADDR_PRIMARY, 50) == ESP_OK) {
        addr = BMP581_ADDR_PRIMARY;
    } else if (i2c_master_probe(bus, BMP581_ADDR_SECONDARY, 50) == ESP_OK) {
        addr = BMP581_ADDR_SECONDARY;
    } else {
        ESP_LOGW(TAG, "BMP581 not found at 0x%02X or 0x%02X",
                 BMP581_ADDR_PRIMARY, BMP581_ADDR_SECONDARY);
        return ESP_ERR_NOT_FOUND;
    }

    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &devcfg, &s_dev);
    if (err != ESP_OK) return err;

    // Verify chip ID. 0x50 = BMP581 (what we want). 0x51 = BMP585 (a different
    // chip with a slightly different application profile — the registers are
    // similar enough that we'd probably "work" but accuracy specs differ).
    // Reject anything else.
    uint8_t chip_id = 0;
    err = read_regs(REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BMP581 chip-ID read failed at 0x%02X: %s",
                 addr, esp_err_to_name(err));
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return err;
    }
    if (chip_id == CHIP_ID_BMP585) {
        ESP_LOGW(TAG, "Found BMP585 (chip ID 0x51) at 0x%02X — wrong part, "
                 "expected BMP581 (0x50). Ignoring.", addr);
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_ERR_NOT_FOUND;
    }
    if (chip_id != CHIP_ID_BMP581) {
        ESP_LOGW(TAG, "BMP581 chip-ID mismatch at 0x%02X: got 0x%02X (want 0x%02X)",
                 addr, chip_id, CHIP_ID_BMP581);
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    // Verify NVM is ready and error-free (datasheet §4.3.9 post-power-up
    // checklist). status_nvm_rdy lives at bit 1, status_nvm_err at bit 2.
    uint8_t status = 0;
    err = read_regs(REG_STATUS, &status, 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BMP581 STATUS read failed: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return err;
    }
    if (!(status & 0x02) || (status & 0x04)) {
        ESP_LOGW(TAG, "BMP581 NVM not ready or error: STATUS=0x%02X", status);
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return ESP_FAIL;
    }

    // Clear the POR bit in INT_STATUS (clear-on-read register, datasheet §7.21).
    // Not strictly required but means a future bmp581-internal reset detection
    // would actually see the bit.
    uint8_t int_status = 0;
    (void)read_regs(REG_INT_STATUS, &int_status, 1);

    // Apply the operating profile. OSR_CONFIG must be written in STANDBY mode
    // or the write is silently lost (datasheet §4.3.8). Reset state IS
    // STANDBY, so this works on a fresh chip; per-cycle FORCED auto-returns
    // to STANDBY too. DSP_CONFIG and DSP_IIR are left at their reset values
    // (0x03 and 0x00 respectively) which already give us P+T compensation
    // and BYPASS — no writes needed.
    err = write_reg(REG_OSR_CONFIG, OSR_CONFIG_VAL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP581 OSR_CONFIG write failed: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return err;
    }

    s_addr  = addr;
    s_ready = true;

    // Prime the oversampling chain with 10 throwaway samples before the first
    // real read. Bosch's BMP5xx KB confirms the first ~N samples after begin()
    // are unsettled even with IIR disabled — the OSR averaging chain needs
    // actual measurement cycles to flush, a wall-clock delay alone won't do
    // it. Mirror of dusty-code's mh12 fix (2026-05-08,
    // src/sensors/bmp5xx/bmp5xx.cpp). At OSR_p=16x / OSR_t=1x each forced
    // read is ~12 ms, so this costs ~120 ms once at boot — trivial inside
    // env_sensor_init which runs before WiFi/HTTP/TX.
    //
    // s_ready was set true above (Option A) so bmp581_read() doesn't refuse
    // these calls. Practically safe because nothing else runs during this
    // ~120 ms window — env_sensor_init is sequential, the first user-facing
    // read is 150 s away in cycle #1.
    for (int i = 0; i < 10; i++) {
        float t, p;
        (void)bmp581_read(&t, &p);
    }

    ESP_LOGI(TAG, "BMP581 ready at 0x%02X (P x16, T x1, IIR off, forced mode, "
             "primed 10 samples)", addr);
    return ESP_OK;
}

bool bmp581_present(void) {
    return s_ready;
}

// --- Read --------------------------------------------------------------------

esp_err_t bmp581_read(float *temperature_c, float *pressure_pa) {
    if (!s_ready) return ESP_FAIL;

    // Trigger a single forced-mode conversion. Datasheet §4.3.7: from STANDBY
    // a write of pwr_mode=0b10 starts the first measurement immediately.
    esp_err_t err = write_reg(REG_ODR_CONFIG, ODR_CONFIG_FORCED);
    if (err != ESP_OK) return err;

    // V2.3.31: precise busy-wait. vTaskDelay(pdMS_TO_TICKS(12)) at the default
    // CONFIG_FREERTOS_HZ=100 (10 ms tick) rounds to 1 tick = 0..10 ms actual,
    // shorter than the chip's 11.4 ms forced-mode conversion → stale data
    // register read. 12 ms busy-wait per 150 s TX cycle = 0.008 % CPU.
    esp_rom_delay_us(FORCED_WAIT_MS * 1000);

    // Burst-read 6 bytes from TEMP_DATA_XLSB. Datasheet §4.5.1 mandates a
    // single-burst read of all 6 data bytes to ride the shadow-register logic
    // that guarantees consistency across a measurement boundary.
    //   d[0] = T_XLSB, d[1] = T_LSB, d[2] = T_MSB
    //   d[3] = P_XLSB, d[4] = P_LSB, d[5] = P_MSB
    uint8_t d[6];
    err = read_regs(REG_TEMP_DATA_XLSB, d, sizeof(d));
    if (err != ESP_OK) return err;

    // Temperature: signed 24-bit, °C = raw / 2^16 (datasheet §4.5).
    int32_t t_raw = ((int32_t)d[2] << 16) | ((int32_t)d[1] << 8) | (int32_t)d[0];
    if (t_raw & 0x00800000) t_raw |= 0xFF000000;   // sign-extend bit 23
    float T = (float)t_raw / 65536.0f;

    // Pressure: unsigned 24-bit, Pa = raw / 2^6 (datasheet §4.5). 1/64 Pa
    // resolution, native — no calibration coefficients to apply, BMP5xx is
    // factory-trimmed.
    uint32_t p_raw = ((uint32_t)d[5] << 16) | ((uint32_t)d[4] << 8) | (uint32_t)d[3];
    float P = (float)p_raw / 64.0f;

    if (temperature_c) *temperature_c = T;
    if (pressure_pa)   *pressure_pa   = P;
    return ESP_OK;
}
