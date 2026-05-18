/** @file
 *  @brief V2.4.3: Home Assistant MQTT Discovery payload builder (Phase 2).
 *
 *  Iterates a const entity table; for each entity whose driver reports
 *  "present", builds a retained QoS-1 config payload and publishes it
 *  to `homeassistant/sensor/<node_id>/<object_id>/config`. Home
 *  Assistant subscribes to `homeassistant/+/+/+/config` by default and
 *  auto-creates the entities the first time it sees a config message.
 *
 *  Why "short form" keys (`uniq_id` instead of `unique_id`, `stat_t`
 *  instead of `state_topic`, etc.): HA's discovery spec accepts both,
 *  but each TLS handshake / TCP write costs airtime in lossy WiFi
 *  conditions. ~50 % payload reduction by using short forms means a
 *  Heltec V2 with full sensor stack squeezes into ~150 bytes per entity
 *  instead of ~300 — half as many MQTT packet writes.
 *
 *  Why not generate from the state-JSON builder in mqtt.c: the natural
 *  shape is different — state publishing iterates by VALUE PRESENCE
 *  (does this cycle have a PM reading?) while discovery iterates by
 *  DRIVER PRESENCE (does this hardware have an SPS30?). Trying to share
 *  one table would bloat both call sites with conditionals.
 */

#include "mqtt_discovery.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "als.h"
#include "env_sensor.h"
#include "hal.h"
#include "noise_sensor.h"
#include "pm_sensor.h"
#include "veml7700.h"
#include "version.h"

static const char *TAG = "mqtt_disc";

// --- Presence predicates ----------------------------------------------------
//
// Wrapped here so the entity table below is a plain const struct
// (function pointers in a static const initialiser are allowed in C99,
// just not function literals). The "always_present" stub also gives us
// one consistent pattern for system-level fields (uptime, cycles).
static bool always_present(void)         { return true; }
// V2.4.12: per-field env predicates so HA only registers entities for
// fields that an actually-present chip can produce. Pre-V2.4.12 used
// one shared env_present_() for env_t/env_h/env_p, which created a
// phantom 0.00 hPa pressure entity on SHT45-only setups.
static bool env_t_present_(void)         { return env_t_present(); }
static bool env_h_present_(void)         { return env_h_present(); }
static bool env_p_present_(void)         { return env_p_present(); }
static bool pm_present_(void)            { return pm_sensor_present();  }
static bool noise_present_(void)         { return noise_sensor_present(); }
static bool any_light_present_(void)     { return veml7700_present() || als_present(); }

// Geiger-tube entities ride on a separate gate — main.c's cfg.tube_enabled
// (passed in via mqtt_discovery_publish_all). Cached at module scope so
// the predicate can be a plain function-pointer matching the others.
static bool s_tube_enabled = false;
static bool tube_enabled_(void)          { return s_tube_enabled; }

// --- Entity catalog ---------------------------------------------------------
//
// Each row defines one HA sensor entity. Fields kept terse so the table
// stays one row per entity. NULL `device_class` means "no standard HA
// class for this measurement" (e.g. radiation has no HA-recognised
// device_class — see [[reference_matter_smart_home]] memory). NULL
// `state_class` means a momentary value HA shouldn't statistic.
//
// `value_template_extra` is appended INSIDE the Jinja expression — used
// for unit conversion (e.g. Pa → hPa for pressure). NULL means just
// emit `{{ value_json.<field> }}`.
typedef struct {
    const char *json_field;          // matches state JSON key in mqtt.c
    const char *object_id;           // suffix in HA discovery topic
    const char *friendly_name;
    const char *device_class;        // HA-standard or NULL
    const char *unit;                // unit_of_measurement
    const char *state_class;         // "measurement" / "total_increasing" / NULL
    const char *icon;                // mdi:* or NULL
    const char *value_template_xform; // optional Jinja transform, e.g. "/100" for Pa→hPa
    bool      (*present_fn)(void);
} ha_entity_t;

static const ha_entity_t ENTITIES[] = {
    // --- System (always present) -------------------------------------------
    { "cycles",     "cycles",     "TX cycles",       NULL,             NULL,    "total_increasing", "mdi:counter",       NULL, always_present },
    { "reconnects", "reconnects", "WiFi reconnects", NULL,             NULL,    "total_increasing", "mdi:wifi-refresh",  NULL, always_present },
    { "uptime_ms",  "uptime",     "Uptime",          "duration",       "ms",    "measurement",      "mdi:timer-outline", NULL, always_present },

    // --- Geiger tube -------------------------------------------------------
    { "cpm",        "cpm",        "CPM",             NULL,             "CPM",   "measurement",      "mdi:radioactive",   NULL, tube_enabled_ },
    { "usvph",      "dose_rate",  "Dose rate",       NULL,             "µSv/h", "measurement",      "mdi:radioactive",   NULL, tube_enabled_ },
    { "hv_pulses",  "hv_pulses",  "HV pulses",       NULL,             NULL,    "total_increasing", "mdi:flash",         NULL, tube_enabled_ },

    // --- Environment (BME280 / SHT45+BMP581 / etc.) -----------------------
    // V2.4.12: each field gated on its own per-field predicate (e.g.
    // SHT45-only setup → env_t + env_h registered, env_p skipped).
    { "env_t",      "temperature","Temperature",     "temperature",    "°C",    "measurement",      NULL, NULL, env_t_present_ },
    { "env_h",      "humidity",   "Humidity",        "humidity",       "%",     "measurement",      NULL, NULL, env_h_present_ },
    // env_p comes in as Pa from mqtt.c (raw BMP/BME register units).
    // Convert to hPa in the value template so HA charts look natural.
    { "env_p",      "pressure",   "Pressure",        "atmospheric_pressure", "hPa", "measurement", NULL, "/100", env_p_present_ },

    // --- Particulate matter (SPS30) ---------------------------------------
    { "pm1",        "pm1",        "PM1",             "pm1",            "µg/m³", "measurement",      NULL, NULL, pm_present_ },
    { "pm25",       "pm25",       "PM2.5",           "pm25",           "µg/m³", "measurement",      NULL, NULL, pm_present_ },
    // PM4 has no HA device_class — use a generic icon.
    { "pm4",        "pm4",        "PM4",             NULL,             "µg/m³", "measurement",      "mdi:blur",          NULL, pm_present_ },
    { "pm10",       "pm10",       "PM10",            "pm10",           "µg/m³", "measurement",      NULL, NULL, pm_present_ },
    // Number concentrations — no HA class. Shown as "particles/cm³".
    { "nc05",       "nc_05",      "NC 0.5 µm",       NULL,             "1/cm³", "measurement",      "mdi:blur",          NULL, pm_present_ },
    { "nc1",        "nc_1",       "NC 1 µm",         NULL,             "1/cm³", "measurement",      "mdi:blur",          NULL, pm_present_ },
    { "nc25",       "nc_25",      "NC 2.5 µm",       NULL,             "1/cm³", "measurement",      "mdi:blur",          NULL, pm_present_ },
    { "nc4",        "nc_4",       "NC 4 µm",         NULL,             "1/cm³", "measurement",      "mdi:blur",          NULL, pm_present_ },
    { "nc10",       "nc_10",      "NC 10 µm",        NULL,             "1/cm³", "measurement",      "mdi:blur",          NULL, pm_present_ },
    { "ps_typ",     "ps_typ",     "Typical size",    NULL,             "µm",    "measurement",      "mdi:ruler",         NULL, pm_present_ },

    // --- Noise (DNMS) -----------------------------------------------------
    { "noise_laeq", "noise_laeq", "Sound LAeq",      "sound_pressure", "dB",    "measurement",      NULL, NULL, noise_present_ },
    { "noise_min",  "noise_min",  "Sound min",       "sound_pressure", "dB",    "measurement",      NULL, NULL, noise_present_ },
    { "noise_max",  "noise_max",  "Sound max",       "sound_pressure", "dB",    "measurement",      NULL, NULL, noise_present_ },

    // --- Ambient light (VEML7700 OR ALS-PT19) -----------------------------
    { "lux",        "lux",        "Illuminance",     "illuminance",    "lx",    "measurement",      NULL, NULL, any_light_present_ },
};

#define N_ENTITIES (sizeof(ENTITIES) / sizeof(ENTITIES[0]))

// --- Payload builder --------------------------------------------------------
//
// Hand-rolled JSON via snprintf to avoid pulling in cJSON for ~300 bytes
// of per-entity output. Same pattern as mqtt.c's state builder.
//
// Buffer sizing: longest payload (env_p with device_class + xform + dev
// block + 64-char prefix + 19-char chip-id) lands around ~400 bytes.
// 512 leaves headroom for the longest realistic config. Truncation
// (impossible in current sizing) would just produce invalid JSON that
// HA would reject — safe-failure mode.
static int build_payload(char *buf, size_t bufsz,
                         const ha_entity_t *e,
                         const char *chip_id,
                         const char *prefix) {
    size_t n = 0;
    int rem;
#define APPEND(...)                                                     \
    do {                                                                \
        rem = (int)bufsz - (int)n;                                      \
        if (rem <= 1) break;                                            \
        int _w = snprintf(buf + n, rem, __VA_ARGS__);                   \
        if (_w < 0) break;                                              \
        n += (size_t)((_w < rem) ? _w : (rem - 1));                     \
    } while (0)

    APPEND("{\"name\":\"%s\"", e->friendly_name);
    APPEND(",\"uniq_id\":\"geiger_%s_%s\"", chip_id, e->object_id);
    APPEND(",\"obj_id\":\"geiger_%s_%s\"",  chip_id, e->object_id);
    APPEND(",\"stat_t\":\"%s/%s/state\"",   prefix, chip_id);
    if (e->value_template_xform) {
        APPEND(",\"val_tpl\":\"{{ (value_json.%s %s) | round(1) }}\"",
               e->json_field, e->value_template_xform);
    } else {
        APPEND(",\"val_tpl\":\"{{ value_json.%s }}\"", e->json_field);
    }
    APPEND(",\"avty_t\":\"%s/%s/availability\"", prefix, chip_id);
    if (e->unit) {
        APPEND(",\"unit_of_meas\":\"%s\"", e->unit);
    }
    if (e->device_class) {
        APPEND(",\"dev_cla\":\"%s\"", e->device_class);
    }
    if (e->state_class) {
        APPEND(",\"stat_cla\":\"%s\"", e->state_class);
    }
    if (e->icon) {
        APPEND(",\"ic\":\"%s\"", e->icon);
    }
    // Device block — groups all entities under one HA device card.
    // `mf` (manufacturer), `mdl` (model), `sw` (sw_version), `ids`
    // (identifiers — uniquely keys this device across HA fabrics).
    APPEND(",\"dev\":{\"ids\":[\"%s\"]", chip_id);
    APPEND(",\"name\":\"Geiger %s\"", chip_id);
    APPEND(",\"mdl\":\"MultiGeiger V2 %s\"", BOARD_NAME);
    APPEND(",\"sw\":\"%s\"", VERSION_STR);
    APPEND(",\"mf\":\"MMBytes\"}");
    APPEND("}");
#undef APPEND
    return (int)n;
}

// --- Public API -------------------------------------------------------------

int mqtt_discovery_publish_all(esp_mqtt_client_handle_t client,
                               const char *chip_id,
                               const char *prefix) {
    if (!client || !chip_id || !prefix) {
        ESP_LOGW(TAG, "publish_all: null args (client=%p chip=%p prefix=%p)",
                 client, chip_id, prefix);
        return 0;
    }

    char topic[128];
    char payload[512];
    int published = 0;

    for (size_t i = 0; i < N_ENTITIES; i++) {
        const ha_entity_t *e = &ENTITIES[i];
        if (!e->present_fn()) {
            continue;  // sensor not fitted on this board / disabled in config
        }
        snprintf(topic, sizeof(topic),
                 "homeassistant/sensor/geiger_%s/%s/config",
                 chip_id, e->object_id);
        int payload_len = build_payload(payload, sizeof(payload),
                                        e, chip_id, prefix);
        // QoS 1 + retain so HA picks them up even if it joins the broker
        // later than us, AND so a fresh HA install discovers our entities
        // without needing to wait for our next reconnect.
        int msg_id = esp_mqtt_client_publish(
            client, topic, payload, payload_len, /*qos*/ 1, /*retain*/ 1);
        if (msg_id < 0) {
            ESP_LOGW(TAG, "publish failed for %s (msg_id=%d)",
                     e->object_id, msg_id);
            continue;
        }
        published++;
    }

    ESP_LOGI(TAG, "discovery published: %d entities (of %u catalogued)",
             published, (unsigned)N_ENTITIES);
    return published;
}

// Internal hook used by mqtt.c::mqtt_init to seed the tube-enabled
// predicate. Declared here (not in header) because no other caller has
// reason to touch it — keeps the public surface narrow.
void mqtt_discovery_set_tube_enabled(bool enabled);
void mqtt_discovery_set_tube_enabled(bool enabled) {
    s_tube_enabled = enabled;
}
