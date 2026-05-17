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
    uint32_t last_hv_pulses;  // cumulative HV recharge pulses
    bool     last_hv_error;   // true if HV cap never reached full in last cycle
    bool     have_env;        // env-block fields below are valid
    float    env_t;
    float    env_h;
    float    env_p;
    int64_t  last_cycle_at;   // unix epoch when the last cycle finished (0 = never)
    uint32_t last_cycle_ms;   // monotonic uptime ms at last cycle (0 = never)
    uint32_t reconnects;      // WiFi STA reconnect count since boot
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
