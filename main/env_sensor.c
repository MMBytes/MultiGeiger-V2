#include "env_sensor.h"

#include "esp_log.h"

#include "sht45.h"
#include "bmp581.h"
#include "bmp390.h"
#include "bme688.h"
#include "bme280.h"

static const char *TAG = "env";

// V2.3.29: bus ownership moved to i2c_bus.c. This module is now a pure
// consumer — env_sensor_init(bus) accepts a bus handle and runs the
// cascade probe on it. Vext gating (Heltec) and LDO2 enable (FeatherS3-D
// STEMMA2) are i2c_bus.c's job — by the time we're called, the bus is
// already powered up and ready to ACK.

// --- Init --------------------------------------------------------------------

esp_err_t env_sensor_init(i2c_master_bus_handle_t bus) {
    if (!bus) return ESP_ERR_INVALID_ARG;

    // BMP390, BME688 and BME280 all use I2C addresses 0x76/0x77 — only ONE of
    // the three can physically be on the bus at a time. We probe in priority
    // order (highest accuracy first): BMP581 (6th-gen Bosch, 0x46/0x47 —
    // independent of the 0x77 family, so it can coexist with any of them) →
    // BMP390 (dedicated barometer, fixed 0x77 on the Adafruit breakout) →
    // BME688 (newer Bosch T/P/H, gas off) → BME280 (legacy fallback). Each
    // later probe in the 0x77 family skips its address if a prior driver
    // claimed it, defending against an accidental dual-population on different
    // addresses (e.g. one chip jumpered to 0x76). Chip-ID is also verified
    // inside each driver — but BMP390 and BME280 both report 0x60, so chip-ID
    // alone is not enough; the address-skip ordering is the real defence.
    //
    // V2.3.29: each sub-driver's init is idempotent (returns early if its
    // device is already bound). main.c may re-call env_sensor_init() with a
    // different bus if the first call found nothing — sub-drivers re-probe
    // cleanly on the new bus.

    sht45_init(bus);   // 0x44 — separate I2C address, no conflict possible

    bool bmp581_ok = (bmp581_init(bus) == ESP_OK);  // 0x46/0x47 — independent of 0x77 family
    (void)bmp581_ok;
    bool bmp390_ok = (bmp390_init(bus) == ESP_OK);
    bool bme688_ok = (bme688_init(bus, bmp390_ok) == ESP_OK);
    bool bme_addr_77_busy = bmp390_ok || bme688_ok;
    bme280_init(bus, bme_addr_77_busy);

    ESP_LOGI(TAG, "env sensor: %s", env_sensor_name());
    return ESP_OK;
}

// --- Public API --------------------------------------------------------------

bool env_sensor_present(void) {
    return sht45_present() || bmp581_present() || bmp390_present() ||
           bme688_present() || bme280_present();
}

esp_err_t env_sensor_read(float *temperature_c, float *humidity_pct,
                          float *pressure_pa,
                          char *raw_log, size_t raw_log_cap) {
    float t = 0, h = 0, p = 0;
    bool  have_t = false, have_h = false, have_p = false;

    // V2.3.26: optional per-sensor raw-reading log. Each present sensor that
    // gets called appends one segment (its own T/H/P, or "read failed"). The
    // caller logs this alongside the fused result, so the source of any single
    // field is visible — essential for diagnosing flaky sensors that fail
    // silently and let the cascade fall through to a fallback chip (the H=0.00%
    // problem on esp32-176432, May 2026). NULL buffer disables the log build.
    size_t rl_off = 0;
    if (raw_log && raw_log_cap > 0) raw_log[0] = '\0';
    #define RL(...) do { \
        if (raw_log && raw_log_cap > rl_off + 1) { \
            int _n = snprintf(raw_log + rl_off, raw_log_cap - rl_off, __VA_ARGS__); \
            if (_n > 0) rl_off += (size_t)_n; \
        } \
    } while (0)

    // Temperature + humidity: SHT45 is primary (best accuracy and lowest
    // self-heating of all our T/H options).
    if (sht45_present()) {
        float st, sh;
        if (sht45_read(&st, &sh) == ESP_OK) {
            t = st; h = sh;
            have_t = have_h = true;
            RL("SHT45: T=%.2f°C  H=%.2f%%, ", st, sh);
        } else {
            RL("SHT45: read failed, ");
        }
    }

    // Pressure (and temperature fallback): BMP581 is the highest-priority
    // pressure source — 6th-gen Bosch barometer with factory-trimmed digital
    // output, ±0.4 Pa relative accuracy, 0.21 PaRMS noise at our OSR setting.
    // Lives at 0x46/0x47 so it can coexist with any 0x77-family chip.
    if (bmp581_present()) {
        float bt, bp;
        if (bmp581_read(&bt, &bp) == ESP_OK) {
            p = bp;
            have_p = true;
            if (!have_t) { t = bt; have_t = true; }
            RL("BMP581: T=%.2f°C P=%.2fhPa, ", bt, bp / 100.0f);
        } else {
            RL("BMP581: read failed, ");
        }
    }

    // BMP390 — second-priority pressure source, used when BMP581 isn't
    // populated. Dedicated barometric chip (no humidity), 0x77 on the
    // Adafruit breakout.
    if (bmp390_present() && !have_p) {
        float bt, bp;
        if (bmp390_read(&bt, &bp) == ESP_OK) {
            p = bp;
            have_p = true;
            if (!have_t) { t = bt; have_t = true; }
            RL("BMP390: T=%.2f°C P=%.2fhPa, ", bt, bp / 100.0f);
        } else {
            RL("BMP390: read failed, ");
        }
    }

    // BME688 — third-priority T/P/H source, only present when neither BMP581
    // (different address) nor BMP390 (same address) claimed the slot. Fills
    // any remaining gap.
    if (bme688_present() && (!have_t || !have_h || !have_p)) {
        float bt, bh, bp;
        if (bme688_read(&bt, &bh, &bp) == ESP_OK) {
            if (!have_t) { t = bt; have_t = true; }
            if (!have_h) { h = bh; have_h = true; }
            if (!have_p) { p = bp; have_p = true; }
            RL("BME688: T=%.2f°C  H=%.2f%% P=%.2fhPa, ", bt, bh, bp / 100.0f);
        } else {
            RL("BME688: read failed, ");
        }
    }

    // BME280 — legacy fallback, only present when BMP390 and BME688 aren't
    // (all three share I2C 0x77).
    if (bme280_present() && (!have_t || !have_h || !have_p)) {
        float bt, bh, bp;
        if (bme280_read(&bt, &bh, &bp) == ESP_OK) {
            if (!have_t) { t = bt; have_t = true; }
            if (!have_h) { h = bh; have_h = true; }
            if (!have_p) { p = bp; have_p = true; }
            RL("BME280: T=%.2f°C  H=%.2f%% P=%.2fhPa, ", bt, bh, bp / 100.0f);
        } else {
            RL("BME280: read failed, ");
        }
    }

    // Strip trailing ", " so the caller can append " <fused>" with one space.
    if (raw_log && rl_off >= 2 &&
        raw_log[rl_off - 2] == ',' && raw_log[rl_off - 1] == ' ') {
        raw_log[rl_off - 2] = '\0';
    }
    #undef RL

    if (!have_t && !have_h && !have_p) return ESP_FAIL;

    if (temperature_c) *temperature_c = t;
    if (humidity_pct)  *humidity_pct  = h;
    if (pressure_pa)   *pressure_pa   = p;
    return ESP_OK;
}

// Returns the active-sensor name in priority order. The "+" separator marks
// data fusion across two chips (e.g. "SHT45+BMP581" = SHT45 supplies T/H,
// BMP581 supplies P). BMP581 lives at 0x46/0x47 so it can coexist with any
// of the 0x77-family chips (BMP390/BME688/BME280) — but BMP581 is always
// preferred for pressure when present, so the 0x77 chip is only useful as
// T/H backstop in that case (and we only get to BME688/BME280 if BMP581 is
// absent and they happen to also be populated). Listing every theoretical
// combination would explode the table; instead we cover the realistic
// populates: SHT45+BMP581, SHT45+BMP390, SHT45+BME688, SHT45+BME280, plus
// the single-chip degenerate cases.
const char *env_sensor_name(void) {
    bool has_sht = sht45_present();
    bool has_581 = bmp581_present();
    bool has_390 = bmp390_present();
    bool has_b68 = bme688_present();
    bool has_b28 = bme280_present();

    if (has_sht && has_581) return "SHT45+BMP581";
    if (has_sht && has_390) return "SHT45+BMP390";
    if (has_sht && has_b68) return "SHT45+BME688";
    if (has_sht && has_b28) return "SHT45+BME280";
    if (has_sht)            return "SHT45";
    if (has_581)            return "BMP581";
    if (has_390)            return "BMP390";
    if (has_b68)            return "BME688";
    if (has_b28)            return "BME280";
    return "none";
}

void env_sensor_heat_periodic(uint32_t now_ms, float humidity_pct) {
    sht45_heat_periodic(now_ms, humidity_pct);
}
