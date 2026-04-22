#pragma once

/** @file
 *  @brief Upload pipeline for Madavi, sensor.community, and Radmon.
 *
 *  tx_transmit() enqueues a snapshot onto a CPU1-pinned worker task so
 *  mbedTLS handshakes don't starve the CPU0 idle task (which feeds the
 *  task watchdog). Each target is POST (or GET for Radmon) with retry,
 *  response-body checks for Radmon, and a circuit breaker that backs off
 *  when Radmon fails repeatedly — repeated failed TLS handshakes fragment
 *  the heap and eventually stall the whole pipeline.
 */

#include <stdbool.h>
#include <stdint.h>

// Si22G calibration: µSv/h = cps / 12.2792 (empirical vs. odlinfo.bfs.de reference).
#define SI22G_CPS_TO_USVPH (1.0f / 12.2792f)

// Radmon circuit breaker — 3 consecutive all-retry failures → skip 20 cycles.
#define RADMON_FAIL_THRESHOLD 3
#define RADMON_SKIP_CYCLES    20

typedef struct {
    bool        enabled;
    const char *url_http;
    const char *url_https;
    bool        use_https;
    bool        use_insecure;   // skip cert verification (cert bundle used otherwise)
} tx_target_t;

typedef struct {
    // Per-window measurement payload.
    uint32_t dt_ms;
    uint32_t hv_pulses;
    uint32_t gm_counts;
    uint32_t cpm;
    uint32_t min_micro;
    uint32_t max_micro;

    // BME280 environmental readings. bme_valid = false means the sensor is
    // absent or the last read failed — skip the T/H/P fields in the payloads.
    bool  bme_valid;
    float bme_temperature_c;
    float bme_humidity_pct;
    float bme_pressure_pa;

    // Station altitude (m above sea level) and flag for emitting
    // pressure-at-sealevel to sensor.community. When the flag is false the
    // altitude-derived value is omitted regardless of altitude.
    float station_altitude_m;
    bool  send_sealevel_pressure;

    // Identification.
    const char *sw_version;
    const char *chip_id;

    // WiFi signal strength at snapshot time (dBm). -127 = unknown / not associated.
    int8_t rssi;

    // Upload targets.
    tx_target_t madavi;
    tx_target_t sensorc;
    tx_target_t radmon;
    const char *radmon_user;
    const char *radmon_password;
} tx_context_t;

/** @brief Create the worker task and queue. Call once at boot. */
void tx_setup(void);

/** @brief Enqueue one transmission cycle. Non-blocking; drops if the worker
 *         is still busy on the previous cycle.
 */
void tx_transmit(const tx_context_t *ctx);

/** @brief True when the TX queue is empty and no job is in-flight.
 *
 *  Used to defer FTP uploads so they don't share bandwidth and heap with
 *  an active TX cycle.
 */
bool tx_is_idle(void);
