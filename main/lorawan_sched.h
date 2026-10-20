#pragma once

/** @file
 *  @brief Pure LoRaWAN scheduling rules (V2.8.14) — no IDF/FreeRTOS/RadioLib.
 *
 *  Header-only, C11-and-C++-compatible, modeled on lorawan_codec.h: the
 *  host-side test runner under `test/` includes it directly. Consumed by
 *  `lorawan.cpp`, which supplies the radio-side inputs (the spreading factor
 *  of the current data rate, uptime seconds).
 *
 *  With LoRaWAN active, the node runs ONE schedule: the LoRaWAN uplink
 *  interval replaces the "Sensor data upload interval" for every TX round
 *  (WiFi uploads, MQTT, display), so each frame carries the counts of its own
 *  window and nothing has to be added up across cycles (user decision
 *  2026-10-19).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** @brief Uplink interval per LoRa spreading factor, in ms.
 *
 *  User-set table (2026-10-19): SF7 4 min, SF8 7, SF9 12, SF10 22, SF11 and
 *  SF12 30 min. SF7-SF10 stay under TTN's Fair Use Policy (30 s uplink air
 *  time per device per day) for the port-1 frame alone: 23 B on air
 *  (10 B payload), ~62/113/206/371 ms, i.e. ~22-25 s per day; the hourly
 *  port-2 frame of a Geiger+env node adds ~1-8 s per day on top. SF11/SF12
 *  are a deliberate exception (~40 s / ~71 s per day for port 1),
 *  still far inside the EU868 1 % duty cycle. NOT stretched by NbTrans
 *  (user decision "R2"): repeats the network orders multiply the air time.
 *  SF below 7 (FSK data rates, which have no SF) maps to the SF7 row; above
 *  12 to the SF12 row.
 */
static inline uint32_t lw_sched_interval_ms(uint8_t sf) {
    if (sf <= 7)  return  4u * 60u * 1000u;
    if (sf == 8)  return  7u * 60u * 1000u;
    if (sf == 9)  return 12u * 60u * 1000u;
    if (sf == 10) return 22u * 60u * 1000u;
    return 30u * 60u * 1000u;                  // SF11, SF12 (and anything above)
}

/** @brief Minimum gap between two port-2 (environment) frames on a node that
 *         also sends port 1, in seconds: once per hour, minus 2 min of slack
 *         so a 30-min cycle whose frames land a few seconds early still sends
 *         every second cycle instead of drifting to every third. */
#define LW_PORT2_MIN_GAP_S (60u * 60u - 120u)

/** @brief Is a port-2 frame due this cycle?
 *
 *  Geiger + environment node: at most once per LW_PORT2_MIN_GAP_S (the
 *  environment changes slowly; port 1 keeps the full cadence). Environment-
 *  only node (no tube): every cycle, it is the node's only frame.
 *  @param tube       port 1 is sent this cycle (tube enabled)
 *  @param sent_once  a port-2 frame went on air since boot
 *  @param now_s      uptime seconds now
 *  @param last_s     uptime seconds of the last port-2 frame (if sent_once)
 *  Unsigned difference, so it stays correct across an uptime counter wrap.
 */
static inline bool lw_sched_port2_due(bool tube, bool sent_once, uint32_t now_s, uint32_t last_s) {
    if (!tube || !sent_once) return true;
    return (uint32_t)(now_s - last_s) >= LW_PORT2_MIN_GAP_S;
}

/** @brief Decode the `lorawan_dr` setting: 0 = ADR (network-controlled, the
 *         default), 1..6 = fixed DR0..DR5.
 *  @return true and *dr set for a fixed data rate; false for ADR (or an out-
 *          of-range NVS value, which falls back to ADR).
 */
static inline bool lw_sched_fixed_dr(uint8_t setting, uint8_t *dr) {
    if (setting == 0 || setting > 6 || dr == NULL) return false;
    *dr = (uint8_t)(setting - 1u);
    return true;
}

/** @brief Should a frame received in a Class A window be handed to RadioLib?
 *
 *  RadioLib (7.8.1) stops a frame's NbTrans repeats, and skips RX2, as soon
 *  as RX1 received ANY frame — before it has looked at it. On a busy channel
 *  that is often someone else's downlink, or our own with bit errors (LoRaWAN
 *  downlinks carry no PHY CRC). Only frames that can be ours count:
 *  @param joined  session active: accept data downlinks (MType 3/5, LoRaWAN
 *                 §4.2.1) addressed to exactly @p own_addr; while joining,
 *                 accept only join accepts (MType 1, encrypted, so the
 *                 addressee can't be checked here).
 *  @param frame   first bytes of the frame (MHDR, then DevAddr little-endian)
 *  @param len     frame length in bytes
 */
static inline bool lw_sched_frame_is_ours(bool joined, uint32_t own_addr, const uint8_t *frame,
                                          size_t len) {
    if (frame == NULL || len < 1) return false;
    const uint8_t mtype = (uint8_t)(frame[0] >> 5);
    if ((frame[0] & 0x03u) != 0) return false;   // Major != LoRaWAN R1
    if (!joined) return mtype == 1 && (len == 17 || len == 33);   // join accept sizes
    if (mtype != 3 && mtype != 5) return false;
    if (len < 12) return false;                   // MHDR + FHDR(7) + MIC(4) minimum
    const uint32_t addr = (uint32_t)frame[1] | ((uint32_t)frame[2] << 8) |
                          ((uint32_t)frame[3] << 16) | ((uint32_t)frame[4] << 24);
    return addr == own_addr;
}
