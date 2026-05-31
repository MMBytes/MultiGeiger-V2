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

#include "pm_sensor.h"      // pm_sample_t
#include "noise_sensor.h"   // noise_sample_t

// Si22G calibration: µSv/h = cps / 12.2792 (empirical vs. odlinfo.bfs.de reference).
#define SI22G_CPS_TO_USVPH (1.0f / 12.2792f)

// Circuit breaker applied to all three upload targets (Madavi, sensor.community,
// Radmon): 3 consecutive all-retry failures trip the breaker; the target is then
// skipped for 20 cycles (~50 min at 150 s interval) before being retried.
#define TX_CB_FAIL_THRESHOLD 3
#define TX_CB_SKIP_CYCLES    20

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

    // When false, the entire radiation payload (CPM, dose, HV, pulse stats)
    // is suppressed. Madavi sends only the THP body, sensor.community sends
    // only the X-PIN 11 POST, and Radmon (radiation-only) is skipped.
    // Mirrors config_t.tube_enabled — copied into the context at snapshot
    // time so the worker task sees a stable value across the cycle.
    bool tube_enabled;

    // BME280 environmental readings. bme_valid = false means the sensor is
    // absent or the last read failed — skip the T/H/P fields in the payloads.
    bool  bme_valid;
    float bme_temperature_c;
    float bme_humidity_pct;
    float bme_pressure_pa;

    // Sensirion SPS30 particulate-matter readings. pm_valid = false means
    // either no PM sensor is attached or the last read failed — skip all
    // SPS30_* fields in the payloads. Madavi picks them up in the env body
    // alongside BME280_*; sensor.community gets them as a separate X-PIN 12
    // POST.
    bool        pm_valid;
    pm_sample_t pm;

    // hbitter DNMS noise sensor readings. noise_valid = false means either no
    // DNMS is attached or the integration window failed — skip all DNMS_noise_*
    // fields in the payloads. Madavi picks them up in the env body alongside
    // BME280_* / SPS30_*; sensor.community gets them as a separate X-PIN 15
    // POST (canonical airrohr DNMS_API_PIN).
    bool             noise_valid;
    noise_sample_t   noise;

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

    // openSenseMap — single HTTPS POST per cycle to ingress.opensensemap.org
    // /boxes/<box_id>/data?luftdaten=1. Body is the standard combined
    // Luftdaten payload (Si22G_* + BME280_* + SPS30_* fields).
    tx_target_t osm;            // V2.5.5: url_http/url_https stay NULL (dynamic per-box URL)
    const char *osm_box_id;
    // V2.3.16: optional OSM access token. Empty string = unauthenticated upload
    // (the historical Luftdaten path). When set, send_osm adds an
    // `Authorization: Bearer <token>` header — needed if the box has the
    // server-side authentication toggle enabled on opensensemap.org.
    const char *osm_access_token;

    // aqi.eco — single HTTPS POST per cycle to api.aqi.eco/update/<token>.
    // Body is the standard Luftdaten payload prefixed with an esp8266id
    // field (legacy field name, accepts our esp32-N chip ID format).
    tx_target_t aqi;            // V2.5.5: url_http/url_https stay NULL (dynamic per-token URL)
    const char *aqi_token;

    // GMCMap (gmcmap.com / GQ Electronics) — radiation-only GET upload, HTTP
    // only (gmcmap has no TLS). Skipped when tube_enabled is false. V2.5.1.
    tx_target_t gmc;
    const char *gmc_account_id;
    const char *gmc_geiger_id;

    // ThingSpeak — generic GET channel update keyed by the channel write API
    // key. field1=CPM, field2=µSv/h, field3/4=CPM (current; no rolling
    // windows), +field5/6/7=T/H/P when bme_valid. HTTPS supported. V2.5.1.
    tx_target_t thingspeak;
    const char *thingspeak_api_key;

    // ThingSpeak (Particulate Matter) — a SECOND, independent ThingSpeak
    // channel for the SPS30 dust node. Separate write key. NOT tube-gated;
    // skipped at runtime when pm_valid is false. field1..4 = PM1.0/PM2.5/
    // PM4.0/PM10, field5/6/7 = T/H/P, field8 = typical particle size (µm).
    // HTTPS supported. V2.5.4.
    tx_target_t thingspeak_pm;
    const char *thingspeak_pm_api_key;
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

/** @brief Per-target upload statistics, surfaced on the status page.
 *
 *  Counters cover the lifetime of the boot. `attempted` counts every cycle
 *  the target was actually called (skipped-because-breaker-open cycles do
 *  NOT increment attempted — they're suppressed before send). `last_rc` is
 *  the HTTP status from the last send (or -1 on transport error). `last_at`
 *  is the unix-epoch timestamp of the last send attempt (0 = never sent).
 *  `breaker_open_cycles` is the number of TX cycles remaining until the
 *  circuit breaker re-closes (0 = closed / normal).
 */
typedef struct {
    uint32_t attempted;
    uint32_t succeeded;
    int      last_rc;
    int64_t  last_at;
    int      breaker_open_cycles;
} tx_target_stats_t;

typedef enum {
    TX_TARGET_MADAVI = 0,
    TX_TARGET_SENSORC,
    TX_TARGET_RADMON,
    TX_TARGET_OSM,
    TX_TARGET_AQI,
    TX_TARGET_GMC,          // V2.5.1: gmcmap.com (radiation-only, HTTP)
    TX_TARGET_THINGSPEAK,   // V2.5.1: ThingSpeak channel update (radiation)
    TX_TARGET_THINGSPEAK_PM, // V2.5.4: ThingSpeak channel update (SPS30 PM)
    TX_TARGET_COUNT
} tx_target_id_t;

/** @brief Short human-readable target name (never NULL). */
const char *tx_target_name(tx_target_id_t id);

/** @brief Read the per-target stats snapshot. Safe from any task —
 *         fields are 32-bit/64-bit aligned; reads are torn-tolerant.
 */
void tx_get_stats(tx_target_id_t id, tx_target_stats_t *out);

/** @brief Populate a tx_target_t for a built-in upload target.
 *
 *  V2.4.1 (C9): URL strings for Madavi / sensor.community / Radmon live
 *  inside transmission.c instead of being hardcoded at the main.c call
 *  site. Caller passes the per-cycle config flags; this fills the static
 *  URLs + `use_insecure = false` default. No-op (leaves `*out` zero-init)
 *  if `id` is one of TX_TARGET_OSM / TX_TARGET_AQI — those URLs are built
 *  dynamically from per-box / per-account tokens, see send_osm() /
 *  send_aqi() in transmission.c.
 */
void tx_target_configure(tx_target_t *out, tx_target_id_t id,
                         bool enabled, bool use_https);
