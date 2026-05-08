#pragma once

/** @file
 *  @brief Particulate-matter sensor facade — currently SPS30 only.
 *
 *  Auto-detects PM sensors at the bus layer (currently just the Sensirion
 *  SPS30 at 0x69) and exposes a single read API that returns mass and
 *  number concentrations plus the typical particle size. Designed in the
 *  same shape as env_sensor.[ch] so a future second PM sensor (e.g. PMS5003,
 *  HM3301) can drop in alongside SPS30 with no caller changes.
 *
 *  The PM sensor shares the I²C master bus owned by env_sensor — pm_sensor_init
 *  takes the bus handle from env_sensor_get_i2c_bus(). Pull-ups on that bus are
 *  the same set already wired for the THP sensors.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/** @brief One full SPS30 measurement record.
 *
 *  Field order matches the wire format Sensirion sends in float mode
 *  (4 mass channels, 5 number-concentration channels, typical particle size).
 *  Mass concentrations are µg/m³, number concentrations #/cm³, size µm.
 *
 *  Sensirion field naming convention used in upstream firmwares (matches the
 *  SPS30_* value_type keys consumed by sensor.community / Madavi):
 *    pm1_0 = SPS30_P0   (PM mass <= 1 µm)
 *    pm2_5 = SPS30_P2   (PM mass <= 2.5 µm)
 *    pm4_0 = SPS30_P4   (PM mass <= 4 µm)
 *    pm10  = SPS30_P1   (PM mass <= 10 µm)
 *    nc0_5 = SPS30_N05  (number conc. <= 0.5 µm)
 *    nc1_0 = SPS30_N1   (number conc. <= 1 µm)
 *    nc2_5 = SPS30_N25  (number conc. <= 2.5 µm)
 *    nc4_0 = SPS30_N4   (number conc. <= 4 µm)
 *    nc10  = SPS30_N10  (number conc. <= 10 µm)
 *    typ_size_um = SPS30_TS
 */
typedef struct {
    float pm1_0;
    float pm2_5;
    float pm4_0;
    float pm10;
    float nc0_5;
    float nc1_0;
    float nc2_5;
    float nc4_0;
    float nc10;
    float typ_size_um;
} pm_sample_t;

/** @brief Probe for a PM sensor on the supplied I²C bus and start measurement.
 *
 *  Always returns ESP_OK — sensor absence is non-fatal. Check
 *  pm_sensor_present() to know whether PM data is available.
 *
 *  When SPS30 is detected:
 *    - device is reset (clean state)
 *    - continuous-measurement mode is started (float wire format)
 *    - fan auto-cleaning interval stays at the factory default of 7 days
 */
esp_err_t pm_sensor_init(i2c_master_bus_handle_t bus);

/** @brief True if a PM sensor was detected and successfully started. */
bool pm_sensor_present(void);

/** @brief Short human-readable label, e.g. "SPS30" or "none". */
const char *pm_sensor_name(void);

/** @brief Read one PM measurement record.
 *
 *  Polls the sensor's data-ready flag (max ~1 s) before fetching. In
 *  continuous mode the sensor produces a new sample every second so
 *  data-ready is essentially always true at our 150 s cycle cadence.
 *
 *  Returns ESP_OK on success, ESP_FAIL on no PM sensor / read error.
 *  On success the sample is also cached for pm_sensor_get_last_sample().
 */
esp_err_t pm_sensor_read(pm_sample_t *out);

/** @brief PM sensor self-diagnosis flags.
 *
 *  Sensor-agnostic shape so a future second PM driver can populate the same
 *  struct. SPS30 maps directly: fan_fail = status bit 4, laser_fail = bit 5,
 *  fan_speed_warn = bit 21. raw = the full 32-bit status word for diagnostics.
 */
typedef struct {
    bool     fan_fail;        // fan RPM out of target for >10 s, or blocked
    bool     laser_fail;      // laser current out of regulation
    bool     fan_speed_warn;  // fan drifting from target (not yet failed)
    uint32_t raw;             // raw status register (driver-specific encoding)
} pm_sensor_status_t;

/** @brief Read and refresh the PM sensor's self-diagnosis status.
 *
 *  Issues an I²C transaction — call from the cycle thread, NOT from HTTP
 *  handlers (which run on a different task and would race). On success the
 *  result is also cached for pm_sensor_get_last_status().
 *
 *  Returns ESP_OK on success, ESP_FAIL on no PM sensor / I²C error.
 */
esp_err_t pm_sensor_read_status(pm_sensor_status_t *out);

/** @brief Return the most recent successful pm_sensor_read() result.
 *
 *  Safe to call from any task. Returns ESP_FAIL if no successful read has
 *  happened yet (typically during the first cycle after boot).
 */
esp_err_t pm_sensor_get_last_sample(pm_sample_t *out);

/** @brief Return the most recent pm_sensor_read_status() result.
 *
 *  Safe to call from any task. Returns ESP_FAIL if no status read has
 *  happened yet (typically during the first cycle after boot).
 */
esp_err_t pm_sensor_get_last_status(pm_sensor_status_t *out);
