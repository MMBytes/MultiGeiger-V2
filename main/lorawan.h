#pragma once

/** @file
 *  @brief Public C API for the LoRaWAN uplink task (OTAA join + port-1/2 TX).
 *
 *  Real implementation is `lorawan.cpp` (C++, RadioLib) on
 *  `BOARD_HELTEC_WIFI_LORA32_V4_R2` only (`HAL_HAS_LORAWAN==1`). On the ten
 *  other boards this header instead defines `static inline` no-op stubs, so
 *  call sites in main.c / http_server.c stay guard-free everywhere — the
 *  same convention other optional-driver HAL feature flags use.
 */

#include <stdint.h>
#include <stdbool.h>

#include "hal.h"    // HAL_HAS_LORAWAN

/** @brief Lifecycle state surfaced on /status and used to gate uplinks. */
typedef enum {
    LORAWAN_ST_DISABLED = 0,   // lorawan_enabled off
    LORAWAN_ST_NO_CONFIG,      // enabled but EUI/key fields missing/malformed
    LORAWAN_ST_HW_FAIL,        // SX1262 init failed — parked until reboot
    LORAWAN_ST_JOINING,        // OTAA attempts running (with backoff)
    LORAWAN_ST_JOINED,         // session active (fresh join or NVS resume)
} lorawan_state_t;

/** @brief Per-cycle measurement snapshot handed to lorawan_transmit(). */
typedef struct {
    uint32_t gm_counts;        // per-window GM pulse count
    uint32_t dt_ms;            // window length (clamped to 24 bit in codec)
    uint8_t  tube_nbr;         // tube_type_t index — same 0-3 numbering V1.9/ttn2luft uses
    bool     env_valid;        // gate for the port-2 frame
    float    temperature_c;
    float    humidity_pct;
    float    pressure_pa;
} lorawan_snapshot_t;

/** @brief Join/uplink statistics, surfaced on the status page. */
typedef struct {
    lorawan_state_t state;
    uint8_t  region;           // config index (region table in lorawan.cpp)
    uint8_t  subband;
    uint32_t dev_addr;         // 0 until joined
    uint32_t join_attempts;
    uint32_t uplinks_sent;     // successful sendReceive calls (port-1 frames)
    uint32_t duty_skipped;     // cycles dropped by duty-cycle/dwell guard
    uint32_t failed;           // sendReceive hard errors
    int16_t  last_error;       // last RadioLib status code (0 = none)
    int64_t  last_uplink_at;   // unix epoch, 0 = never
    float    last_dl_rssi;     // last downlink RSSI dBm (0 = none seen)
    float    last_dl_snr;
} lorawan_status_t;

#if !HAL_HAS_LORAWAN
// No-op stubs: call sites in main.c / http_server.c stay guard-free on the
// ten boards without the radio. Mirrors how HAL feature flags keep other
// optional drivers' call sites clean.
static inline void lorawan_setup(void) {}
static inline void lorawan_transmit(const lorawan_snapshot_t *snap) { (void)snap; }
static inline bool lorawan_is_idle(void) { return true; }
static inline void lorawan_get_status(lorawan_status_t *out) {
    if (out) {
        *out = (lorawan_status_t){0};
        out->state = LORAWAN_ST_DISABLED;
    }
}
static inline const char *lorawan_state_name(lorawan_state_t st) { (void)st; return "disabled"; }
static inline const char *lorawan_region_name(uint8_t region_idx) { (void)region_idx; return "?"; }
#else

/** @brief Create the LoRaWAN task. Radio init happens on the task, not here. */
void lorawan_setup(void);

/** @brief Enqueue one uplink attempt. Non-blocking; drops if the task is
 *         still busy (joining or mid-uplink) on the previous cycle.
 */
void lorawan_transmit(const lorawan_snapshot_t *snap);

/** @brief True when the LoRaWAN task is idle (safe to hand it new work). */
bool lorawan_is_idle(void);

/** @brief Read the current join/uplink status snapshot. Safe from any task. */
void lorawan_get_status(lorawan_status_t *out);

/** @brief Short human-readable name for a lorawan_state_t value (never NULL). */
const char *lorawan_state_name(lorawan_state_t st);

/** @brief Short human-readable region name for a config-table index (never
 *         NULL); used by the /config dropdown and /status.
 */
const char *lorawan_region_name(uint8_t region_idx);

#endif
