#pragma once

/** @file
 *  @brief V2.3.29: ALS-PT19 ambient light sensor driver (analog, FeatherS3-D only).
 *
 *  Adafruit's UM FeatherS3-D has an onboard ALS-PT19 phototransistor
 *  wired to **GPIO 4** (= ADC1_CH3 on ESP32-S3). Provides a rough
 *  ambient-light reading in millivolts and approximate lux. Compiled
 *  out on boards without `HAL_HAS_ALS` — `als_present()` returns false
 *  and the /status page skips the ambient-light row.
 *
 *  Why this is approximate, not photometric:
 *    The ALS-PT19 is a phototransistor with a non-linear lux↔current
 *    response and a peak sensitivity at ~525 nm. Adafruit's onboard
 *    transimpedance resistor sets the slope to roughly **1.6 mV/lux**
 *    in the linear region (low-to-mid light levels). Above ~3000 mV
 *    the device saturates. Lux conversion is ±50 % typical — fine for
 *    "dark / dim / lit / bright / sun" categorisation, not for
 *    calibrated photometry.
 *
 *  ADC1 (used here) is independent of WiFi — no coexistence conflicts
 *  (only ADC2 is shared with the WiFi front-end).
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/** @brief Initialise the ALS — sets up ADC1 oneshot read + curve-fitting
 *  calibration. Always returns ESP_OK; calibration failure is logged
 *  and reduces accuracy (mV reading falls back to nominal-full-scale
 *  linear conversion) but doesn't disable the sensor.
 *
 *  No-op on boards without HAL_HAS_ALS.
 */
esp_err_t als_init(void);

/** @brief True if the ALS is enabled on this board AND als_init()
 *  succeeded. /status checks this before drawing the ambient-light row.
 */
bool als_present(void);

/** @brief Take one ADC sample. Pointers may be NULL.
 *
 *  @param raw          12-bit ADC count (0..4095). NULL to skip.
 *  @param millivolts   Calibrated voltage in mV at the ADC pin. NULL to skip.
 *  @param lux          Approximate lux (mV / 1.6 + saturation handling).
 *                      ±50 % typical accuracy. NULL to skip.
 *  @return ESP_OK on success, ESP_FAIL if ALS isn't present.
 */
esp_err_t als_read(uint32_t *raw, uint32_t *millivolts, float *lux);

/** @brief Map a lux value to a short qualitative descriptor like
 *  "dark", "indoor lit", "bright indoor", etc. Bands chosen to match
 *  the ALS-PT19's useful linear range. Useful for /status display.
 */
const char *als_brightness_label(float lux);
