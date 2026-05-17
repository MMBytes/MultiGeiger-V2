#pragma once

/** @file
 *  @brief MQTT 3.1.1 publish-only client (V2.4.2).
 *
 *  Lifecycle: `mqtt_init()` at boot, `mqtt_publish_state()` once per TX
 *  cycle from main.c. Internally wraps Espressif's `esp-mqtt` client.
 *
 *  Topic layout (all under `<prefix>/<chip-id>/`):
 *    - `state`         — single JSON object with every present sensor's
 *                        readings, published QoS 0, retain=false each cycle
 *    - `availability`  — "online" on connect, LWT "offline" on disconnect;
 *                        retained so HA shows the sensor as unavailable
 *                        immediately when the device drops
 *
 *  HA Discovery topics (`homeassistant/sensor/...`) land in V2.4.3 (Phase 2),
 *  gated by the `mqtt_ha_discovery` config flag.
 *
 *  Connection model: synchronous-init, async background-task.
 *    - `mqtt_init()` returns immediately. If the broker / WiFi isn't up yet
 *      the underlying esp-mqtt client retries with exponential backoff.
 *    - `mqtt_publish_state()` is non-blocking — drops the publish silently
 *      (with a debug log) if the broker isn't currently connected. The
 *      next cycle will publish a fresh sample, so missing one cycle is fine.
 *    - LWT is registered on every (re)connect so even an abrupt power loss
 *      flips the availability topic to "offline" within `keepalive`*1.5 s.
 *
 *  Disabled-by-default. `mqtt_init()` is a no-op when:
 *    - `cfg->mqtt_enable` is false, OR
 *    - `cfg->mqtt_broker[0]` is empty (defensive — don't DNS-bomb the LAN
 *      with empty-host lookups if the user enabled before configuring)
 */

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "main_status.h"
#include "pm_sensor.h"
#include "noise_sensor.h"

/** @brief Wire up the MQTT client. Safe to call once at boot.
 *
 *  Reads the broker / port / creds from `cfg`. Caches `chip_id` for use in
 *  topic builds (caller-owned pointer; must outlive the client — passing
 *  main.c's `g_chip_id` static buffer satisfies this).
 *
 *  If MQTT is disabled or no broker is configured, returns immediately with
 *  no allocations. Otherwise creates the esp-mqtt client and starts the
 *  background connect task; logging shows progress.
 */
void mqtt_init(const config_t *cfg, const char *chip_id);

/** @brief Publish one snapshot of sensor state to `<prefix>/<chip>/state`.
 *
 *  Called from `do_tx_cycle()` in main.c after `tx_transmit()`. Builds a
 *  JSON object inline from the supplied sensor samples + main_status. No-op
 *  if MQTT is disabled or the broker isn't currently connected.
 *
 *  Caller passes the same per-cycle samples it already built for tx_transmit
 *  so we don't re-read I²C sensors here.
 */
void mqtt_publish_state(const main_status_t *st,
                        bool pm_valid, const pm_sample_t *pm,
                        bool noise_valid, const noise_sample_t *noise);

/** @brief True if the client is currently connected to the broker.
 *
 *  For the /status page row added in V2.4.4 (Phase 3). Safe from any task.
 *  Returns false when MQTT is disabled.
 */
bool mqtt_is_connected(void);

/** @brief Cumulative count of successful publishes since boot.
 *
 *  For the /status page. Increments only when esp-mqtt confirmed the publish
 *  was queued (publishes dropped because broker disconnected don't count).
 */
uint32_t mqtt_publish_count(void);
