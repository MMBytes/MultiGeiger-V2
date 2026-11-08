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
#include <stdio.h>

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

/** @brief Would RadioLib 8.0 keep a frame it read out of a Class A window?
 *
 *  Mirrors LoRaWANNode::checkDownlink() (LoRaWAN.cpp, RadioLib 8.0.0): a
 *  frame it rejects there is dropped silently — the window counts as empty,
 *  RX2 still opens and the NbTrans repeats go on. The RX-window observer uses
 *  this to mark such a frame "ignored" in the node log. (checkDownlink()'s
 *  length limits — over 255 B, too short, over the data rate's size — are
 *  checked before the frame is read, so the observer never sees those bytes.)
 *  @param joined  session active: keep data downlinks (MType 3/5) addressed
 *                 to exactly @p own_addr and at least MHDR + FHDR + FOpts +
 *                 MIC long; while joining, keep only join accepts (MType 1)
 *                 of 17 or 33 B (encrypted, so the addressee can't be checked).
 *  @param frame   the frame as read (MHDR, DevAddr little-endian, FCtrl, ...)
 *  @param len     frame length in bytes
 *  @return true if RadioLib passes the frame on to its parser.
 */
static inline bool lw_sched_frame_is_ours(bool joined, uint32_t own_addr, const uint8_t *frame,
                                          size_t len) {
    if (frame == NULL || len < 1) return false;
    const uint8_t mtype = (uint8_t)(frame[0] >> 5);
    if (!joined) return mtype == 1 && (len == 17 || len == 33);   // join accept sizes
    if (mtype != 3 && mtype != 5) return false;
    if (len < 6) return false;                    // no FCtrl byte to read
    // MHDR 1 + DevAddr 4 + FCtrl 1 + FCnt 2 + FOpts + MIC 4
    if (len < 12u + (size_t)(frame[5] & 0x0Fu)) return false;
    const uint32_t addr = (uint32_t)frame[1] | ((uint32_t)frame[2] << 8) |
                          ((uint32_t)frame[3] << 16) | ((uint32_t)frame[4] << 24);
    return addr == own_addr;
}

/** @brief The "next LoRaWAN uplink" line (V2.8.21, user request: the node log
 *         says WHEN the next uplink comes, not only the interval). main.c logs
 *         it after every upload round with @p duty_s 0 (schedule only); the
 *         LoRaWAN worker logs it again after the round's last exchange when
 *         the duty-cycle limiter holds the next uplink past that round
 *         (lorawan.cpp, log_duty_hold()). "about": the uplink can still wait a
 *         few seconds for a WiFi transfer.
 *  @param in_s  seconds until the next round (whole minutes from 60 s on, rounded)
 *  @param hh    local hour of that moment, or < 0 if the clock is not set yet
 *               (a LoRaWAN-only node may still read 1970 — no time is shown then)
 *  @param sf    the spreading factor the interval follows, 0 = not known (no tail)
 *  @param duty_s seconds until RadioLib's duty-cycle limiter allows the next uplink
 *               (0 = no wait). If it outlasts @p in_s, the uplink comes then, not at
 *               the round: the line says so, and hh/mm/ss must be THAT moment.
 *  @return the snprintf() length. */
static inline int lw_next_uplink_line(char *out, size_t n, uint32_t in_s, int hh, int mm, int ss, uint8_t sf,
                                      uint32_t duty_s) {
    if (out == NULL || n == 0) return 0;
    const bool duty = duty_s > in_s;
    const uint32_t eff_s = duty ? duty_s : in_s;
    char in[16];
    if (eff_s < 60) snprintf(in, sizeof(in), "%lu s", (unsigned long)eff_s);
    else            snprintf(in, sizeof(in), "%lu min", (unsigned long)((eff_s + 30u) / 60u));
    char round[16];
    if (in_s < 60) snprintf(round, sizeof(round), "%lu s", (unsigned long)in_s);
    else           snprintf(round, sizeof(round), "%lu min", (unsigned long)((in_s + 30u) / 60u));
    char sched[16] = "";
    if (sf) snprintf(sched, sizeof(sched), "SF%u schedule", (unsigned)sf);
    char why[64] = "";   // "duty-cycle wait; upload round in " (33) + round (<= 15) + ", "
    if (duty) snprintf(why, sizeof(why), "duty-cycle wait; upload round in %s%s", round, sf ? ", " : "");
    if (hh < 0) {
        return snprintf(out, n, "next LoRaWAN uplink in about %s (%s%s%sclock not set yet)", in, why, sched,
                        (sf || duty) ? "; " : "");
    }
    if (duty) {
        return snprintf(out, n, "next LoRaWAN uplink at about %02d:%02d:%02d (in %s: %s%s)", hh, mm, ss, in, why,
                        sched);
    }
    return snprintf(out, n, "next LoRaWAN uplink at about %02d:%02d:%02d (in %s%s%s)", hh, mm, ss, in,
                    sf ? ", " : "", sched);
}
