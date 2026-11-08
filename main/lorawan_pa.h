// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

/** @file lorawan_pa.h
 *  @brief Heltec V4 power-amplifier gain that RadioLib must subtract (V2.8.22) — pure, no IDF/RadioLib.
 *
 *  Header-only and C/C++-compatible, so test/test_main.c includes it directly.
 *  lorawan.cpp hands the result to setOutputPowerOffset() before begin().
 *
 *  Why: the LoRaWAN stack asks for a power that is already the band's EIRP
 *  limit (EU868 16 dBm, AU915 30 dBm). Any amplifier between the SX1262 and
 *  the antenna must be declared to RadioLib, which then programs the radio
 *  that much lower — otherwise the PA's gain lands on top of the limit.
 *  Before V2.8.22 only the V4.3's KCT8103L was declared; a V4.2 with the
 *  GC1109's PA switched on (lorawan_high_power) sent ~11 dB above the limit.
 */

#include <stdbool.h>
#include <stdint.h>

/** Net TX gain of the Heltec V4's PA at low drive, both front ends: Heltec's
 *  868 MHz table for the GC1109 (meshtastic/firmware#8070, at the antenna
 *  port) gives +10.8..+11.25 dB for 1-14 dBm drive, compressing to +7 dB at
 *  22 dBm; Meshtastic's heltec_v4 TX_GAIN_LORA table runs 11 -> 7 alike. The
 *  low-drive value errs on the low side at high drive, never above the limit
 *  by more than the table's 0.25 dB spread. */
#define LW_PA_GAIN_DB 11

/** @brief dB RadioLib programs the radio below the stack's power.
 *  @param kct8103l    front end probed as a KCT8103L (V4.3): its PA is always
 *                     in the TX path, whatever lorawan_high_power says
 *  @param high_power  lorawan_high_power: on a GC1109 (V4.2) GPIO46 switches
 *                     its PA into the TX path; off = PA bypassed, no gain
 *  @return LW_PA_GAIN_DB when a PA is in the TX path, else 0 */
static inline int8_t lw_pa_offset_db(bool kct8103l, bool high_power) {
    return (kct8103l || high_power) ? (int8_t)LW_PA_GAIN_DB : (int8_t)0;
}
