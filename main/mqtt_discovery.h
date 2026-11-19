#pragma once

/** @file
 *  @brief V2.4.3: Home Assistant MQTT Discovery payload publisher.
 *
 *  Phase 2 of the MQTT rollout. Phase 1 (V2.4.2) added the publish path
 *  itself (`mqtt.c`); this module adds the auto-discovery payloads that
 *  let Home Assistant create the sensor entities automatically from the
 *  device's first publish.
 *
 *  Topic format (HA spec):
 *      homeassistant/sensor/<node_id>/<object_id>/config
 *  Payload: a JSON config object describing one sensor entity. Published
 *  with retain=true so HA picks it up even after restart.
 *
 *  When this fires:
 *      - On every MQTT_EVENT_CONNECTED in `mqtt.c`. Cheap retry — broker
 *        already has retained copies after the first publish; subsequent
 *        publishes overwrite an identical payload at zero broker cost.
 *      - Gated by the user's `mqtt_ha_discovery` config flag (default true
 *        in V2.4.3+).
 *
 *  Entity gating: each sensor entity is only published when its
 *  corresponding driver reports the sensor as present. So a Heltec V2 with
 *  just the Geiger tube and no PM/env/noise sensors gets 4 discovery
 *  entities (cpm / usvph / cycles / reconnects); a fully-loaded
 *  FeatherS3-D with SPS30 + SHT45+BMP581 + DNMS + VEML7700 gets the full
 *  ~18 entities. No-op entries for absent sensors avoids HA showing the
 *  user "unknown" entities they have no hardware for.
 */

#include <stdbool.h>
#include "mqtt_client.h"

/** @brief Publish HA Discovery config for every sensor entity currently
 *         present on this device.
 *
 *  @param client     Live esp-mqtt client handle (must be in CONNECTED state).
 *  @param chip_id    Per-device identifier ("esp32-<decimal>") used in topic +
 *                    HA unique_id / device.identifiers. Caller-owned pointer
 *                    must remain valid for the duration of this call.
 *  @param state_topic  The topic mqtt.c publishes the state JSON on
 *                    ("<prefix>/<chip_id>/state"), copied into every config
 *                    payload so HA knows where to subscribe.
 *  @param avail_topic  The LWT / availability topic, same purpose.
 *
 *  V2.8.31: takes mqtt.c's own pre-built topic strings instead of the
 *  prefix. Before, discovery rebuilt them from the LIVE config, so a /config
 *  Save with a new prefix (without the restart the field asks for) pointed
 *  HA at "<new>/…" while the node kept publishing on "<old>/…".
 *
 *  Iterates the static entity table internally; each present entity
 *  produces one retained QoS-1 publish. Returns the count of entities
 *  successfully published (for logging / diagnostics). Failures are
 *  logged but don't abort the loop — partial publish is preferred to
 *  the broker getting nothing.
 *
 *  Total payload bytes ≈ 200-400 per entity × 4-18 entities = 1-7 KB on
 *  the wire per call. Broker storage: same (one retained message per
 *  topic). Single-shot, not a hot path.
 */
int mqtt_discovery_publish_all(esp_mqtt_client_handle_t client,
                               const char *chip_id,
                               const char *state_topic,
                               const char *avail_topic);

/** @brief Seed the tube-enabled predicate that gates the radiation entities.
 *
 *  Internal to the MQTT module: only mqtt.c calls it (at init and on a
 *  /config Save). V2.8.31: declared here instead of a hand-written extern
 *  in mqtt.c, so a signature change fails the build.
 */
void mqtt_discovery_set_tube_enabled(bool enabled);

/** @brief Seed the per-target upload predicates for the upload-stat entities.
 *
 *  Internal to the MQTT module, like mqtt_discovery_set_tube_enabled(). Only
 *  defined on HAL_MQTT_RICH_STATE (PSRAM) boards; the declaration is
 *  unconditional so this header needs no hal.h.
 */
void mqtt_discovery_set_upload_flags(bool madavi, bool sensorc, bool radmon,
                                     bool osm, bool aqi, bool gmc,
                                     bool thingspeak, bool thingspeak_pm,
                                     bool ftp);
