// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

/** @file lorawan_mac.h
 *  @brief Plain-words decoder for LoRaWAN MAC commands (V2.8.23) — pure, no IDF/RadioLib.
 *
 *  Turns the MAC commands of a received downlink (network -> node) and the
 *  answers RadioLib queued for the next uplink (node -> network) into one
 *  readable log line each, e.g.
 *    "DevStatusReq (asks for battery level and link margin)"
 *    "DevStatusAns — battery not measured (0xFF), margin +9 dB".
 *
 *  Why: RadioLib carries the commands out silently and then deletes them, so
 *  until V2.8.22 the node log only said "MAC commands only". Most owners have
 *  no TTN console or gateway records, so the node log has to say what the
 *  network asked for and what the node answered.
 *
 *  Command lengths mirror RadioLib 8.0's MacTable (LoRaWAN.cpp) so this
 *  decoder splits a FOpts field exactly where RadioLib does; an unknown CID
 *  ends the decode, as it ends RadioLib's. Field layouts: LoRaWAN L2 1.0.4
 *  §5 (MAC commands) and RP002-1.0.4 (ChMaskCntl meanings per region type).
 *  Header-only and C/C++-compatible, so test/test_main.c includes it directly.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "lorawan_rxobs.h"   // rxobs_cat(): the shared truncation-safe append

/** Buffer size for one MAC line. Worst case: 15 bytes of FOpts as fifteen
 *  1-byte DevStatusReq, 15 x (52 characters + "; ") = 810. Stays under the
 *  node log's 1024-byte line limit with the "MAC from network: " prefix.
 *  Longer text would be cut, never overrun. */
#define LW_MAC_LINE_MAX 840

/** Offset from the GPS epoch (1980-01-06) to the Unix epoch, and the GPS-UTC
 *  leap-second count in force since 2017-01-01 (no leap second announced
 *  since); used only to print DeviceTimeAns as a UTC date. */
#define LW_MAC_GPS_UNIX_OFFSET_S 315964800UL
#define LW_MAC_GPS_LEAP_S        18UL

/** @brief Payload length (without the CID byte) of MAC command @p cid, as
 *         RadioLib 8.0's MacTable has it.
 *  @param down true = network -> node (request / downlink answer), false = node -> network
 *  @return the length, or -1 for a CID RadioLib does not know */
static inline int lw_mac_len(uint8_t cid, bool down) {
    switch (cid) {
    case 0x01: return 1;              // ResetInd / ResetConf (1.1)
    case 0x02: return down ? 2 : 0;   // LinkCheckAns / LinkCheckReq
    case 0x03: return down ? 4 : 1;   // LinkADRReq / LinkADRAns
    case 0x04: return down ? 1 : 0;   // DutyCycleReq / DutyCycleAns
    case 0x05: return down ? 4 : 1;   // RXParamSetupReq / RXParamSetupAns
    case 0x06: return down ? 0 : 2;   // DevStatusReq / DevStatusAns
    case 0x07: return down ? 5 : 1;   // NewChannelReq / NewChannelAns
    case 0x08: return down ? 1 : 0;   // RXTimingSetupReq / RXTimingSetupAns
    case 0x09: return down ? 1 : 0;   // TxParamSetupReq / TxParamSetupAns
    case 0x0A: return down ? 4 : 1;   // DlChannelReq / DlChannelAns
    case 0x0B: return 1;              // RekeyConf / RekeyInd (1.1)
    case 0x0C: return down ? 1 : 0;   // ADRParamSetupReq / ADRParamSetupAns (1.1)
    case 0x0D: return down ? 5 : 0;   // DeviceTimeAns / DeviceTimeReq
    case 0x0E: return down ? 2 : 0;   // ForceRejoinReq (1.1)
    case 0x0F: return 1;              // RejoinParamSetupReq / Ans (1.1)
    case 0x20: return 1;              // DeviceModeConf / DeviceModeInd (1.1, class C)
    case 0x80: return down ? 5 : 0;   // proprietary (RadioLib knows only 0x80, 5 B down)
    default:   return -1;
    }
}

/** @brief Uplink answers RadioLib repeats in every uplink until the next
 *         downlink arrives (MacTable "persist"; LoRaWAN 1.0.4 §5: the
 *         answers whose loss would desynchronise the RX settings). */
static inline bool lw_mac_answer_sticky(uint8_t cid) {
    return cid == 0x01 || cid == 0x05 || cid == 0x08 || cid == 0x09 || cid == 0x0A || cid == 0x0B ||
           cid == 0x20;
}

/** @brief Little-endian 24-bit frequency field (units of 100 Hz) as MHz text
 *         without trailing zeros: 9233000 -> "923.3", 8695250 -> "869.525". */
static inline void lw_mac_mhz(uint32_t f100, char *out, size_t sz) {
    uint32_t frac = f100 % 10000u;   // 4 digits = 100 Hz resolution
    int digits = 4;
    while (digits > 1 && frac % 10u == 0) { frac /= 10u; digits--; }
    // Literal fixed-width formats, not "%0*lu": gcc's -Wformat-truncation
    // cannot bound a run-time width and fails the -Werror host-test build.
    const unsigned long mhz = (unsigned long)(f100 / 10000u), fr = (unsigned long)frac;
    switch (digits) {
    case 1:  snprintf(out, sz, "%lu.%01lu", mhz, fr); break;
    case 2:  snprintf(out, sz, "%lu.%02lu", mhz, fr); break;
    case 3:  snprintf(out, sz, "%lu.%03lu", mhz, fr); break;
    default: snprintf(out, sz, "%lu.%04lu", mhz, fr); break;
    }
}

/** @brief Set bits of @p mask as channel numbers from @p base, runs folded:
 *         "8-15, 65"; "none" for an empty mask. */
static inline void lw_mac_ch_list(uint16_t mask, int bits, int base, char *buf, size_t sz, int *n) {
    bool any = false;
    for (int i = 0; i < bits;) {
        if (!(mask & (1u << i))) { i++; continue; }
        int j = i;
        while (j + 1 < bits && (mask & (1u << (j + 1)))) j++;
        if (j > i) rxobs_cat(buf, sz, n, "%s%d-%d", any ? ", " : "", base + i, base + j);
        else       rxobs_cat(buf, sz, n, "%s%d", any ? ", " : "", base + i);
        any = true;
        i = j + 1;
    }
    if (!any) rxobs_cat(buf, sz, n, "none");
}

/** @brief ChMaskCntl 5: each of the 10 low mask bits ADDS a bank of eight
 *         125 kHz channels (bit b = channels 8b..8b+7); on fixed plans banks
 *         0-7 also add 500 kHz channel 64+b. Other channels keep their state. */
static inline void lw_mac_banks(uint16_t mask, bool fixed_plan, char *buf, size_t sz, int *n) {
    rxobs_cat(buf, sz, n, "adds channels ");
    bool any = false;
    for (int b = 0; b < 10; b++) {
        if (!(mask & (1u << b))) continue;
        rxobs_cat(buf, sz, n, "%s%d-%d", any ? ", " : "", 8 * b, 8 * b + 7);
        any = true;
    }
    if (!any) rxobs_cat(buf, sz, n, "none");
    if (fixed_plan && (mask & 0xFFu)) {
        rxobs_cat(buf, sz, n, " and 500 kHz channel%s ", (mask & 0xFFu & ((mask & 0xFFu) - 1u)) ? "s" : "");
        lw_mac_ch_list((uint16_t)(mask & 0xFFu), 8, 64, buf, sz, n);
    }
}

/** @brief The channel part of one LinkADRReq block, as RadioLib 8.0 applies it
 *         (LoRaWAN.cpp preprocessMacLinkAdr(), which follows RP002-1.0.4):
 *         fixed-channel plans (US915/AU915) and dynamic ones (EU868 & co.)
 *         differ only for ChMaskCntl 6 and 7. CN470 (a fixed plan with its
 *         own table) is not decoded separately — no node of ours uses it. */
static inline void lw_mac_chmask(uint8_t cntl, uint16_t mask, bool fixed_plan, char *buf, size_t sz, int *n) {
    if (cntl <= 3 || (cntl == 4 && !fixed_plan)) {
        // A 16-channel block replaced as a whole (fixed plans: 125 kHz channels).
        rxobs_cat(buf, sz, n, "%schannels %d-%d set to: ", fixed_plan ? "125 kHz " : "", cntl * 16, cntl * 16 + 15);
        lw_mac_ch_list(mask, 16, cntl * 16, buf, sz, n);
    } else if (cntl == 4) {
        rxobs_cat(buf, sz, n, "500 kHz channels set to: ");
        lw_mac_ch_list(mask, 8, 64, buf, sz, n);
    } else if (cntl == 5) {
        lw_mac_banks(mask, fixed_plan, buf, sz, n);
    } else if (cntl == 6 && fixed_plan) {
        rxobs_cat(buf, sz, n, "all 125 kHz channels on, 500 kHz channels set to: ");
        lw_mac_ch_list(mask, 8, 64, buf, sz, n);
    } else if (cntl == 6) {
        rxobs_cat(buf, sz, n, "all defined channels on");
    } else if (fixed_plan) {
        rxobs_cat(buf, sz, n, "all 125 kHz channels off, 500 kHz channels set to: ");
        lw_mac_ch_list(mask, 8, 64, buf, sz, n);
    } else {
        rxobs_cat(buf, sz, n, "ChMaskCntl 7 (reserved in this region, so the node rejects the channel mask)");
    }
}

/** @brief True if @p b holds only commands that need no answer (LinkCheckAns,
 *         DeviceTimeAns) and decodes completely — the only case where an
 *         empty answer queue is fully explained by the commands themselves. */
static inline bool lw_mac_down_answerless(const uint8_t *b, size_t len) {
    size_t i = 0;
    while (i < len) {
        if (b[i] != 0x02 && b[i] != 0x0D) return false;
        i += 1 + (size_t)lw_mac_len(b[i], true);
    }
    return len > 0 && i == len;
}

/** @brief Unix seconds -> "YYYY-MM-DD HH:MM:SS" (proleptic Gregorian, UTC),
 *         without gmtime_r (missing on some host toolchains). Days-to-civil
 *         after H. Hinnant's chrono algorithm. */
static inline void lw_mac_utc(uint32_t unix_s, char *out, size_t sz) {
    const long days = (long)(unix_s / 86400u);
    const unsigned sec = (unsigned)(unix_s % 86400u);
    const long z = days + 719468;
    const long era = z / 146097;
    const unsigned doe = (unsigned)(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    const unsigned d = doy - (153 * mp + 2) / 5 + 1;
    const unsigned m = mp < 10 ? mp + 3 : mp - 9;
    const long y = (long)yoe + era * 400 + (m <= 2);
    snprintf(out, sz, "%04ld-%02u-%02u %02u:%02u:%02u", y, m, d, sec / 3600, (sec / 60) % 60, sec % 60);
}

/** @brief "accepted" / "REJECTED" for one ACK bit of an answer. */
static inline const char *lw_mac_ok(uint8_t status, unsigned bit) {
    return (status & (1u << bit)) ? "accepted" : "REJECTED";
}

/** @brief Raw bytes as hex, for the parts no decoder covers. */
static inline void lw_mac_hex(const uint8_t *b, size_t len, char *buf, size_t sz, int *n) {
    for (size_t i = 0; i < len; i++) rxobs_cat(buf, sz, n, "%02x", (unsigned)b[i]);
}

/** @brief Decode the MAC commands of one downlink (network -> node) into
 *         @p out, commands separated by "; ".
 *  @param b, len    the FOpts bytes (or a decrypted FPort-0 payload)
 *  @param fixed_plan the band uses fixed channels (US915/AU915/CN470) — only
 *                   changes how a LinkADRReq channel mask reads */
static inline void lw_mac_format_down(const uint8_t *b, size_t len, bool fixed_plan, char *out, size_t sz) {
    int n = 0;
    if (sz == 0) return;
    out[0] = '\0';
    size_t i = 0;
    bool seen_adr = false;   // a LinkADRReq block was already decoded in this frame
    while (i < len) {
        const uint8_t cid = b[i];
        const int pl = lw_mac_len(cid, true);
        if (i > 0) rxobs_cat(out, sz, &n, "; ");
        if (pl < 0) {
            rxobs_cat(out, sz, &n, "unknown MAC command 0x%02x — the rest is not decoded (RadioLib skips it "
                       "too): ", (unsigned)cid);
            lw_mac_hex(&b[i], len - i, out, sz, &n);
            return;
        }
        if (i + 1 + (size_t)pl > len) {
            rxobs_cat(out, sz, &n, "%02x: incomplete (%u of %d bytes) — not applied by RadioLib", (unsigned)cid,
                       (unsigned)(len - i - 1), pl);
            return;
        }
        const uint8_t *p = &b[i + 1];
        char f[48];   // "1677.7215" at most; sized for -Wformat-truncation
        switch (cid) {
        case 0x01:
            rxobs_cat(out, sz, &n, "ResetConf (LoRaWAN 1.1 reset confirmed, version %u)", (unsigned)(p[0] & 0x0F));
            break;
        case 0x02:
            rxobs_cat(out, sz, &n, "LinkCheckAns — our uplink arrived %u dB above the demodulation floor, heard "
                       "by %u gateway%s", (unsigned)p[0], (unsigned)p[1], p[1] == 1 ? "" : "s");
            break;
        case 0x03: {
            // A second LinkADRReq NOT adjacent to the first block: RadioLib
            // answers it with a rejection (all ACK bits 0) without applying it
            // (parseDownlink(): `if(mAdr)` branch), one 5-byte command at a time.
            if (seen_adr) {
                rxobs_cat(out, sz, &n, "LinkADRReq (a second, separate one) — not applied: the node rejects it, "
                           "only one LinkADRReq block per downlink counts");
                break;
            }
            seen_adr = true;
            // RadioLib applies a run of back-to-back LinkADRReq as ONE block
            // (parseDownlink(): the next 5 bytes complete and CID 0x03 again):
            // every channel mask in order, DR / TX power / NbTrans from the
            // LAST one. Mirror that grouping so the line says what is applied.
            int blocks = 1;
            while (i + 5u * (size_t)(blocks + 1) <= len && b[i + 5u * (size_t)blocks] == 0x03) blocks++;
            const uint8_t *last = &b[i + 5u * (size_t)(blocks - 1) + 1];
            // DataRate_TXPower: DR in bits 7..4, power index in 3..0 (15 = keep).
            // Redundancy: ChMaskCntl in bits 6..4, NbTrans in 3..0.
            const unsigned dr = last[0] >> 4, pw = last[0] & 0x0F, nbt = last[3] & 0x0F;
            if (blocks > 1) rxobs_cat(out, sz, &n, "LinkADRReq (%d blocks, applied together) — ", blocks);
            else            rxobs_cat(out, sz, &n, "LinkADRReq — ");
            if (dr == 15) rxobs_cat(out, sz, &n, "data rate unchanged, ");
            else          rxobs_cat(out, sz, &n, "DR%u, ", dr);
            if (pw == 15) rxobs_cat(out, sz, &n, "TX power unchanged, ");
            else if (pw == 0) rxobs_cat(out, sz, &n, "TX power index 0 (max EIRP), ");
            else          rxobs_cat(out, sz, &n, "TX power index %u (max EIRP - %u dB), ", pw, 2 * pw);
            for (int k = 0; k < blocks; k++) {
                const uint8_t *q = &b[i + 5u * (size_t)k + 1];
                if (k > 0) rxobs_cat(out, sz, &n, ", then ");
                lw_mac_chmask((uint8_t)((q[3] >> 4) & 0x07), (uint16_t)(q[1] | (q[2] << 8)), fixed_plan, out, sz,
                              &n);
            }
            // NbTrans 0: LoRaWAN 1.0.x falls back to the default 1 (RadioLib:
            // rev 0); only 1.1 keeps the current value. This decoder only ever
            // sees 1.0.x FOpts (1.1 encrypts them), so the 1.0 rule applies.
            if (nbt == 0) rxobs_cat(out, sz, &n, ", NbTrans 0 = back to the default of 1 transmission per uplink");
            else          rxobs_cat(out, sz, &n, ", NbTrans %u (each uplink sent %u time%s)", nbt, nbt,
                                     nbt == 1 ? "" : "s");
            i += 5u * (size_t)blocks;
            continue;
        }
        case 0x04: {
            const unsigned m = p[0] & 0x0F;
            // MaxDCycle 0: no limit from the network; the band's own rule still applies.
            if (m == 0) rxobs_cat(out, sz, &n, "DutyCycleReq — no extra limit from the network, only the band's "
                                   "own (MaxDCycle 0)");
            else        rxobs_cat(out, sz, &n, "DutyCycleReq — on air at most 1/%u of the time (MaxDCycle %u)",
                                   1u << m, m);
            break;
        }
        case 0x05: {
            const uint32_t fr = (uint32_t)p[1] | ((uint32_t)p[2] << 8) | ((uint32_t)p[3] << 16);
            lw_mac_mhz(fr, f, sizeof(f));
            rxobs_cat(out, sz, &n, "RXParamSetupReq — RX1 DR offset %u, RX2 DR%u at %s MHz",
                       (unsigned)((p[0] >> 4) & 0x07), (unsigned)(p[0] & 0x0F), f);
            break;
        }
        case 0x06:
            rxobs_cat(out, sz, &n, "DevStatusReq (asks for battery level and link margin)");
            break;
        case 0x07: {
            const uint32_t fr = (uint32_t)p[1] | ((uint32_t)p[2] << 8) | ((uint32_t)p[3] << 16);
            if (fr == 0) {
                rxobs_cat(out, sz, &n, "NewChannelReq — channel %u removed", (unsigned)p[0]);
            } else {
                lw_mac_mhz(fr, f, sizeof(f));
                rxobs_cat(out, sz, &n, "NewChannelReq — channel %u at %s MHz, DR%u-DR%u", (unsigned)p[0], f,
                           (unsigned)(p[4] & 0x0F), (unsigned)(p[4] >> 4));
            }
            break;
        }
        case 0x08: {
            const unsigned d = p[0] & 0x0F;
            rxobs_cat(out, sz, &n, "RXTimingSetupReq — RX1 opens %u s after the uplink", d ? d : 1u);
            break;
        }
        case 0x09: {
            // MaxEIRP index -> dBm (LoRaWAN 1.0.4 §5.12 table).
            static const uint8_t eirp[16] = { 8, 10, 12, 13, 14, 16, 18, 20, 21, 24, 26, 27, 29, 30, 33, 36 };
            rxobs_cat(out, sz, &n, "TxParamSetupReq — max EIRP %u dBm, uplink dwell time %s, downlink dwell "
                       "time %s", (unsigned)eirp[p[0] & 0x0F], (p[0] & 0x10) ? "400 ms limit" : "no limit",
                       (p[0] & 0x20) ? "400 ms limit" : "no limit");
            break;
        }
        case 0x0A: {
            const uint32_t fr = (uint32_t)p[1] | ((uint32_t)p[2] << 8) | ((uint32_t)p[3] << 16);
            lw_mac_mhz(fr, f, sizeof(f));
            rxobs_cat(out, sz, &n, "DlChannelReq — channel %u answers (RX1) on %s MHz", (unsigned)p[0], f);
            break;
        }
        case 0x0B:
            rxobs_cat(out, sz, &n, "RekeyConf (LoRaWAN 1.1, version %u)", (unsigned)(p[0] & 0x0F));
            break;
        case 0x0C:
            rxobs_cat(out, sz, &n, "ADRParamSetupReq — ADRACKReq after %u uplinks without a downlink, then a "
                       "backoff step every %u", 1u << (p[0] >> 4), 1u << (p[0] & 0x0F));
            break;
        case 0x0D: {
            const uint32_t gps = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
                                 ((uint32_t)p[3] << 24);
            char t[40];   // 19 used; sized for the compiler's worst case (-Wformat-truncation)
            lw_mac_utc(gps + LW_MAC_GPS_UNIX_OFFSET_S - LW_MAC_GPS_LEAP_S, t, sizeof(t));
            rxobs_cat(out, sz, &n, "DeviceTimeAns — network time %s.%03u UTC at the end of our uplink", t,
                       (unsigned)(p[4] * 1000u / 256u));
            break;
        }
        case 0x0E:
            rxobs_cat(out, sz, &n, "ForceRejoinReq (LoRaWAN 1.1): ");
            lw_mac_hex(p, 2, out, sz, &n);
            break;
        case 0x0F:
            rxobs_cat(out, sz, &n, "RejoinParamSetupReq (LoRaWAN 1.1): %02x", (unsigned)p[0]);
            break;
        case 0x20:
            rxobs_cat(out, sz, &n, "DeviceModeConf — class %s", p[0] == 0 ? "A" : p[0] == 2 ? "C" : "?");
            break;
        default:   // 0x80
            rxobs_cat(out, sz, &n, "proprietary MAC command 0x%02x: ", (unsigned)cid);
            lw_mac_hex(p, (size_t)pl, out, sz, &n);
            break;
        }
        i += 1 + (size_t)pl;
    }
}

/** @brief Decode the MAC answers queued for the next uplink (node -> network)
 *         into @p out, separated by "; ". Answers RadioLib repeats until a
 *         downlink arrives get " (repeated in every uplink until the next
 *         downlink)". */
static inline void lw_mac_format_up(const uint8_t *b, size_t len, char *out, size_t sz) {
    int n = 0;
    if (sz == 0) return;
    out[0] = '\0';
    size_t i = 0;
    while (i < len) {
        const uint8_t cid = b[i];
        const int pl = lw_mac_len(cid, false);
        if (i > 0) rxobs_cat(out, sz, &n, "; ");
        if (pl < 0 || i + 1 + (size_t)pl > len) {
            rxobs_cat(out, sz, &n, "unknown/incomplete answer 0x%02x: ", (unsigned)cid);
            lw_mac_hex(&b[i], len - i, out, sz, &n);
            return;
        }
        const uint8_t *p = &b[i + 1];
        switch (cid) {
        case 0x01: rxobs_cat(out, sz, &n, "ResetInd (LoRaWAN 1.1, version %u)", (unsigned)(p[0] & 0x0F)); break;
        case 0x02: rxobs_cat(out, sz, &n, "LinkCheckReq (our own request)"); break;
        case 0x03:
            rxobs_cat(out, sz, &n, "LinkADRAns — TX power %s, data rate %s, channel mask %s", lw_mac_ok(p[0], 2),
                       lw_mac_ok(p[0], 1), lw_mac_ok(p[0], 0));
            break;
        case 0x04: rxobs_cat(out, sz, &n, "DutyCycleAns (limit applied)"); break;
        case 0x05:
            rxobs_cat(out, sz, &n, "RXParamSetupAns — RX1 DR offset %s, RX2 DR %s, RX2 frequency %s",
                       lw_mac_ok(p[0], 2), lw_mac_ok(p[0], 1), lw_mac_ok(p[0], 0));
            break;
        case 0x06: {
            // Battery: 0 = external power, 1..254 = level, 255 = not measured.
            // Margin: SNR of the received downlink, 6-bit two's complement, dB.
            int m = p[1] & 0x3F;
            if (m & 0x20) m -= 64;
            rxobs_cat(out, sz, &n, "DevStatusAns — battery ");
            if (p[0] == 0)        rxobs_cat(out, sz, &n, "external power (0)");
            else if (p[0] == 255) rxobs_cat(out, sz, &n, "not measured (0xFF)");
            else                  rxobs_cat(out, sz, &n, "%u of 254 (%u %%)", (unsigned)p[0],
                                             (unsigned)((p[0] * 100u + 127u) / 254u));
            rxobs_cat(out, sz, &n, ", margin %+d dB (SNR of the downlink as received)", m);
            break;
        }
        case 0x07:
            rxobs_cat(out, sz, &n, "NewChannelAns — data rate range %s, frequency %s", lw_mac_ok(p[0], 1),
                       lw_mac_ok(p[0], 0));
            break;
        case 0x08: rxobs_cat(out, sz, &n, "RXTimingSetupAns (applied)"); break;
        case 0x09: rxobs_cat(out, sz, &n, "TxParamSetupAns (applied)"); break;
        case 0x0A:
            rxobs_cat(out, sz, &n, "DlChannelAns — uplink channel %s, frequency %s",
                       (p[0] & 0x02) ? "exists" : "NOT defined (REJECTED)", lw_mac_ok(p[0], 0));
            break;
        case 0x0B: rxobs_cat(out, sz, &n, "RekeyInd (LoRaWAN 1.1, version %u)", (unsigned)(p[0] & 0x0F)); break;
        case 0x0C: rxobs_cat(out, sz, &n, "ADRParamSetupAns (applied)"); break;
        case 0x0D: rxobs_cat(out, sz, &n, "DeviceTimeReq (our own request)"); break;
        case 0x0F: rxobs_cat(out, sz, &n, "RejoinParamSetupAns: %02x", (unsigned)p[0]); break;
        case 0x20:
            rxobs_cat(out, sz, &n, "DeviceModeInd — class %s", p[0] == 0 ? "A" : p[0] == 2 ? "C" : "?");
            break;
        default:   // 0x0E / 0x80: no uplink form in RadioLib's table (length 0)
            rxobs_cat(out, sz, &n, "MAC command 0x%02x", (unsigned)cid);
            break;
        }
        if (lw_mac_answer_sticky(cid)) rxobs_cat(out, sz, &n, " (repeated in every uplink until the next downlink)");
        i += 1 + (size_t)pl;
    }
}
