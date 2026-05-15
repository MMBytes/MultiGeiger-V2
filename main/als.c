// V2.3.29 — ALS-PT19 ambient light sensor driver. See als.h for design.

#include "als.h"
#include "hal.h"

#if HAL_HAS_ALS

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "als";

// FeatherS3-D ALS-PT19 wiring: GPIO 4 → ADC1_CH3, 12 dB attenuation
// gives the full ~0–3.3 V input range (ALS output never exceeds ~3.0 V
// in saturation given the on-board transimpedance resistor + 3V3 rail).
#define ALS_ADC_UNIT     ADC_UNIT_1
#define ALS_ADC_CHANNEL  ADC_CHANNEL_3
#define ALS_ADC_ATTEN    ADC_ATTEN_DB_12

static adc_oneshot_unit_handle_t s_adc   = NULL;
static adc_cali_handle_t         s_cali  = NULL;
static bool                      s_ready = false;

esp_err_t als_init(void) {
    if (s_ready) return ESP_OK;

    // 1. ADC unit
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ALS_ADC_UNIT,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "adc_oneshot_new_unit: %s", esp_err_to_name(err));
        return ESP_OK;   // non-fatal — sensor just won't be present
    }

    // 2. Channel config
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ALS_ADC_ATTEN,
    };
    err = adc_oneshot_config_channel(s_adc, ALS_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "adc_oneshot_config_channel: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
        return ESP_OK;
    }

    // 3. Curve-fitting calibration (ESP32-S3 supports this scheme).
    //    Falls back to nominal-full-scale linear conversion if it fails
    //    (e.g. eFuse calibration data is missing — would happen on
    //    very early/dev silicon).
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ALS_ADC_UNIT,
        .atten    = ALS_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "adc_cali curve-fitting unavailable (%s) — "
                      "mV reading will use uncalibrated nominal full-scale",
                 esp_err_to_name(err));
        s_cali = NULL;
    }

    s_ready = true;
    ESP_LOGI(TAG, "ALS-PT19 ready (GPIO 4, ADC1_CH3, 12 dB atten%s)",
             s_cali ? "" : ", uncalibrated");
    return ESP_OK;
}

bool als_present(void) {
    return s_ready;
}

esp_err_t als_read(uint32_t *raw, uint32_t *millivolts, float *lux) {
    if (!s_ready) return ESP_FAIL;

    int adc_raw = 0;
    esp_err_t err = adc_oneshot_read(s_adc, ALS_ADC_CHANNEL, &adc_raw);
    if (err != ESP_OK) return err;

    int mv = 0;
    if (s_cali) {
        adc_cali_raw_to_voltage(s_cali, adc_raw, &mv);
    } else {
        // Nominal-full-scale fallback: 12 dB attenuation gives ~3.1 V
        // full scale on ESP32-S3, but the practical ADC ceiling is
        // 3.3 V × (raw/4095). Close enough for the uncalibrated path.
        mv = (adc_raw * 3300) / 4095;
    }

    if (raw)        *raw        = (uint32_t)adc_raw;
    if (millivolts) *millivolts = (uint32_t)mv;
    if (lux) {
        // ALS-PT19: ~16 µA at 100 lux at VCE=5 V. Adafruit's typical
        // 10 kΩ transimpedance resistor + 3V3 rail puts the sensitivity
        // at roughly 1.6 mV/lux in the linear (low-to-mid) range. Above
        // ~3000 mV the photodiode saturates and the slope flattens —
        // we don't model that, so reported lux above ~1875 should be
        // read as "saturated, very bright" rather than absolute.
        // ±50 % accuracy is realistic — calibration with a reference
        // lux meter would tighten this but isn't worth doing for the
        // dark/dim/lit categorisation we use.
        *lux = (float)mv / 1.6f;
    }
    return ESP_OK;
}

const char *als_brightness_label(float lux) {
    if (lux <    1.0f) return "dark";
    if (lux <   30.0f) return "very dim";
    if (lux <  100.0f) return "dim indoor";
    if (lux <  300.0f) return "indoor lit";
    if (lux < 1000.0f) return "bright indoor";
    if (lux < 5000.0f) return "outdoor shade";
    return "direct sun (saturated)";
}

#else   // HAL_HAS_ALS == 0

esp_err_t als_init(void) { return ESP_OK; }
bool      als_present(void) { return false; }
esp_err_t als_read(uint32_t *raw, uint32_t *millivolts, float *lux) {
    (void)raw; (void)millivolts; (void)lux;
    return ESP_FAIL;
}
const char *als_brightness_label(float lux) {
    (void)lux;
    return "n/a";
}

#endif  // HAL_HAS_ALS
