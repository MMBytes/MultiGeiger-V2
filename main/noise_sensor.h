#pragma once

/** @file
 *  @brief Noise-level sensor facade — currently DNMS only.
 *
 *  Auto-detects noise sensors at the bus layer (currently just the hbitter
 *  DNMS at 0x55 — pre-flashed Teensy 4.0 + ICS-43434 mic, sold by Nettigo as
 *  the "NAM DNMS Kit"). Exposes a trigger/read API that returns A-weighted
 *  equivalent sound level (LAeq) plus min/max over the integration window.
 *
 *  Designed in the same shape as env_sensor / pm_sensor so a future second
 *  noise sensor (e.g. SoundLevelMeter) can drop in alongside DNMS with no
 *  caller changes.
 *
 *  The DNMS Teensy runs LAeq integration internally — the host triggers one
 *  CALCULATE_LEQ to "finalise" the window, polls READ_DATA_READY, then issues
 *  READ_LEQ to retrieve the result. The integration window spans the time
 *  between adjacent CALCULATE_LEQ commands, so the canonical pattern is:
 *  trigger at the END of each TX cycle, read at the START of the next, giving
 *  a full ~150 s LAeq window per cycle. First cycle uses the trigger from
 *  noise_sensor_init() at boot.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** @brief One DNMS measurement record.
 *
 *  Units: dB(A) — A-weighted SPL. Field naming mirrors the upstream
 *  airrohr-firmware / dusty-code convention used on sensor.community.
 */
typedef struct {
    float laeq;     // A-weighted equivalent sound level over the window
    float la_min;   // minimum dB(A) sample seen during the window
    float la_max;   // maximum dB(A) sample seen during the window
} noise_sample_t;

/** @brief Probe for a noise sensor on the bus, verify, and trigger the
 *         first integration window.
 *
 *  Always returns ESP_OK — sensor absence is non-fatal. Check
 *  noise_sensor_present() to know whether noise data is available.
 *
 *  When DNMS is detected:
 *    - Soft-reset the Teensy state machine
 *    - READ_VERSION and verify response starts with "DNMS"
 *    - CALCULATE_LEQ to start the first integration window. The first
 *      noise_sensor_read() will read this window's LAeq.
 */
esp_err_t noise_sensor_init(i2c_master_bus_handle_t bus);

/** @brief True if a noise sensor was detected and successfully started. */
bool noise_sensor_present(void);

/** @brief Short human-readable label, e.g. "DNMS" or "none". */
const char *noise_sensor_name(void);

/** @brief Cached firmware version string from the DNMS Teensy
 *         (e.g. "DNMS-V5.0.0" — first 4 chars must equal "DNMS").
 *
 *  Returns "none" when no DNMS is present. Read once at init, cached
 *  for subsequent calls (no I²C traffic on this path).
 */
const char *noise_sensor_version(void);

/** @brief Trigger the next LAeq integration window.
 *
 *  Sends DNMS CALCULATE_LEQ. Returns immediately — the Teensy continues
 *  integrating until the next call (or noise_sensor_read, which finalises
 *  the previous window first then reads). Call once per TX cycle, AFTER
 *  noise_sensor_read in the same cycle, so each cycle's read returns the
 *  LAeq accumulated since the previous cycle's read.
 */
esp_err_t noise_sensor_trigger(void);

/** @brief Read the most recent integration window's LAeq + min/max.
 *
 *  Polls DNMS READ_DATA_READY (max ~10 s) before issuing READ_LEQ. CRC8 is
 *  verified on every word — any mismatch returns ESP_FAIL without
 *  populating *out. On success the sample is also cached for
 *  noise_sensor_get_last_sample().
 *
 *  Returns ESP_OK on success, ESP_FAIL on no DNMS / read error /
 *  CRC error / data-ready timeout.
 */
esp_err_t noise_sensor_read(noise_sample_t *out);

/** @brief Return the most recent successful noise_sensor_read() result.
 *
 *  Safe to call from any task. Returns ESP_FAIL if no successful read has
 *  happened yet (typically during the first cycle after boot — the LAeq
 *  window is still integrating).
 */
esp_err_t noise_sensor_get_last_sample(noise_sample_t *out);
