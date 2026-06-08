#pragma once

/** @file
 *  @brief Status snapshot exposed by main.c to the HTTP server + display.
 *
 *  Single getter replaces the 13 hand-extern'd `main_status_*` accessors
 *  that used to live as `extern uint32_t main_status_cycles(void); ...`
 *  prologues at the top of http_server.c and display.c. The reader gets a
 *  consistent snapshot of all fields in one call.
 *
 *  All fields are filled by `do_tx_cycle()` on the main task; this struct
 *  is the lock-respecting projection — `last_cycle_at` is the only i64
 *  and takes a brief spinlock for atomic 64-bit read on the 32-bit Xtensa
 *  cores (see B1 in version.h V2.4.1). Other scalars are word-aligned
 *  and torn-tolerant, so no lock is taken for them — the snapshot is not
 *  inter-field consistent (e.g. cpm could be from cycle N and env_t from
 *  cycle N+1) but each field individually is intact.
 */

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t cycles;          // total completed TX cycles since boot
    uint32_t last_dt_ms;      // wall-clock duration of the last cycle
    uint32_t last_cpm;        // GM counts-per-minute from the last cycle
    float    last_usvph;      // dose rate from the last cycle
    uint32_t last_hv_pulses;        // cumulative HV recharge pulses since boot
                                    //   (HA total_increasing semantic — MQTT publishes this raw)
    uint32_t last_hv_pulses_delta;  // V2.4.27: HV recharge pulses THIS cycle only.
                                    //   Matches the V1.x firmware's per-cycle semantic
                                    //   for the field named `hv_pulses` in
                                    //   sensor.community / Madavi / Radmon CSV
                                    //   archives. Status page and legacy HTTPS
                                    //   uploads use this; MQTT keeps cumulative.
    bool     last_hv_error;   // true if HV cap never reached full in last cycle
    bool     have_env;        // OR of have_env_{t,h,p} — kept for callers
                              //   that only care "is any env field valid"
                              //   (http_server.c / transmission.c). New
                              //   per-field callers (mqtt.c) use the three
                              //   below to avoid e.g. publishing a phantom
                              //   0 hPa pressure on SHT45-only setups.
    bool     have_env_t;      // V2.4.12
    bool     have_env_h;      // V2.4.12
    bool     have_env_p;      // V2.4.12
    float    env_t;
    float    env_h;
    float    env_p;
    int64_t  last_cycle_at;   // unix epoch when the last cycle finished (0 = never)
    uint32_t last_cycle_ms;   // monotonic uptime ms at last cycle (0 = never)
    uint32_t reconnects;      // WiFi STA reconnect count since boot
    uint32_t i2c_errors;      // V2.4.28: cumulative I²C sensor read failures
    bool     pcnt_filtering;  // V2.5.16: last_cpm is PCNT width-filtered
    uint32_t last_cpm_raw;    // V2.5.16: pre-filter CPM (== last_cpm when off)
    uint32_t pcnt_filter_width_ns;  // V2.5.16: active filter width (0 = off)
} main_status_t;

/** @brief Snapshot the cached last-cycle state into `out`.
 *
 *  Always populates every field. Cheap — a memcpy-style copy with one
 *  brief spinlock around the int64_t read. Safe to call from any task.
 */
void main_status_snapshot(main_status_t *out);

/** @brief Request a deferred reboot from any task.
 *
 *  V2.4.1 (A9): replaces the pre-V2.4.1 polled flag exposed by
 *  http_server.c. Sets a persistent flag AND wakes the main loop via an
 *  event-group bit so the restart fires within ms instead of up to 1 s.
 *  The actual `esp_restart()` is still deferred until `tx_is_idle()`
 *  returns true — see the main loop for the policy.
 *
 *  Idempotent: calling twice has the same effect as once. Once set, the
 *  flag cannot be cleared (the only way out is the reboot).
 */
void main_request_restart(void);

/** @brief Sticky "stop re-arming MQTT and syslog clients" flag.
 *
 *  V2.4.17: the V2.4.13 OTA teardown path calls `mqtt_stop()` /
 *  `syslog_stop()` before the OTA receive loop to free heap. Without
 *  this flag the main-loop poll re-inits both within ~1 s — undoing
 *  the teardown during the bulk of the OTA write. Observed 2026-05-19:
 *  esp32-176432 V2.4.16 → V2.4.16 OTA logged `mqtt: CONNECTED` ~6 s
 *  after `mqtt: stop`, while the OTA recv loop was still running.
 *
 *  Setting this flag tells the main-loop poll to skip re-init for both
 *  MQTT and syslog until reboot. On successful OTA the device reboots
 *  immediately, so services come back fresh on the new firmware. On
 *  failed OTA the user must manually `/reboot` to restore them —
 *  matching `log_ftp_pause()`'s already-sticky semantics.
 *
 *  Deliberately NOT called by V2.4.14's FTPS teardown — that path
 *  wants MQTT to auto-restart between FTPS uploads so per-cycle
 *  publishes resume.
 *
 *  Idempotent. Once set, cannot be cleared (only path out is reboot).
 */
void main_suspend_services(void);

/** @brief Read the V2.4.17 services-suspended flag. */
bool main_services_suspended(void);

/** @brief V2.4.24: mark the start of an OTA receive-and-write window.
 *
 *  Called by `/update`'s POST handler immediately after auth/CSRF pass,
 *  cleared by `main_ota_end()` before returning. While the flag is set,
 *  the main loop's scheduled TX cycle is skipped — frees the WiFi link
 *  for the OTA upload, which is otherwise sharing airtime with
 *  Madavi / sensor.community / Radmon HTTPS POSTs every 2 minutes.
 *
 *  Unlike `main_suspend_services` this flag is NOT sticky — it's
 *  explicitly cleared on every return path of update_post (success and
 *  failure). A failed OTA leaves the sensor functional; TX resumes
 *  immediately on the next main-loop tick after the OTA gives up.
 *
 *  Discovered as a cause of slow / failing OTAs on marginal WiFi after
 *  the 2026-05-22 failed-update log review on a V2.4.22 FeatherS3-D —
 *  CYCLE #220 + #221 (and their Madavi/SC/Radmon TLS handshakes) both
 *  fired during the OTA recv-retry window. Not the root cause of that
 *  particular failure (client-side WiFi flake was), but strict
 *  improvement for every OTA on every device.
 */
void main_ota_begin(void);

/** @brief Pair with `main_ota_begin()` — clears the OTA-in-progress flag.
 *  Call before every return path in the OTA POST handler. */
void main_ota_end(void);

/** @brief Read the V2.4.24 OTA-in-progress flag. */
bool main_ota_in_progress(void);
