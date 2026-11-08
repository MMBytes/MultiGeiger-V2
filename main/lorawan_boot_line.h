// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

/** @file lorawan_boot_line.h
 *  @brief The "boot: LoRaWAN:" banner line (V2.8.21) — pure formatter, no IDF/RadioLib.
 *
 *  Header-only and C/C++-compatible, so test/test_main.c includes it directly.
 *  syslog.c fills lw_boot_t from lorawan_get_status() and logs the result next
 *  to the other "boot: <Key>:" lines.
 *
 *  Why: the join (or session restore) runs in the first seconds after boot,
 *  but syslog only starts once the STA has an address (~2 min later), so the
 *  per-attempt join lines may never reach the server. One line rebuilt from
 *  state when the socket is up says where LoRaWAN stands — verdict first.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "lorawan_rxobs.h"   // lw_err_text()

/** @brief Outcome of this boot's LoRaWAN start, as far as it has got. */
typedef enum {
    LW_BOOT_OFF = 0,     // lorawan_enabled off
    LW_BOOT_NO_CONFIG,   // enabled, EUI/key fields missing or malformed
    LW_BOOT_HW_FAIL,     // SX1262 init failed
    LW_BOOT_JOINING,     // no session yet, attempts running
    LW_BOOT_RESUMED,     // session restored from NVS, no join request sent
    LW_BOOT_JOINED,      // fresh OTAA join this boot
} lw_boot_kind_t;

/** @brief Inputs of the line; filled from lorawan_status_t. */
typedef struct {
    lw_boot_kind_t kind;
    const char    *region;      // "EU868", "AU915", ... (never NULL)
    uint8_t        subband;     // 0 = none / all channels (left out of the line)
    uint32_t       dev_addr;    // RESUMED / JOINED
    uint32_t       attempts;    // join attempts this boot (JOINING)
    int16_t        last_error;  // last failed attempt's RadioLib code (JOINING)
    // JOINED only: the attempt that got the accept.
    uint8_t        ok_dr;       // 0xFF = not set (fixed plans: DR from the channel)
    uint8_t        ok_sf;
    uint32_t       ok_khz;      // uplink frequency, kHz
    uint8_t        ok_window;   // 1 / 2, 0 = unknown
    // Data rate mode and the radio's TX power range (after any PA offset).
    bool           adr;         // true = ADR, false = fixed DR (fixed_dr)
    uint8_t        fixed_dr;
    int8_t         pwr_min;     // pwr_min == pwr_max = not known (left out)
    int8_t         pwr_max;
} lw_boot_t;

/** @brief Format the text after "LoRaWAN: " into @p out (always terminated).
 *  Examples:
 *    "session resumed — DevAddr 260b1234, region AU915 sub-band 2, ADR, TX power 2..33 dBm"
 *    "joined via RX1 at DR3 (SF9, 867.7 MHz) — DevAddr 260b1234, region EU868, ADR"
 *    "joining — attempt 3, last error -1116 (no join accept received), region EU868, ADR"
 *    "off (disabled in /config)"
 *  @return the snprintf() length (>= n means truncated), 0 for a NULL/0 buffer. */
static inline int lw_boot_line(char *out, size_t n, const lw_boot_t *b) {
    if (out == NULL || n == 0) return 0;
    if (b == NULL) { out[0] = '\0'; return 0; }
    if (b->kind == LW_BOOT_OFF) return snprintf(out, n, "off (disabled in /config)");
    if (b->kind == LW_BOOT_NO_CONFIG) {
        return snprintf(out, n, "not started: DevEUI, JoinEUI or AppKey missing or malformed in /config");
    }
    if (b->kind == LW_BOOT_HW_FAIL) {
        return snprintf(out, n, "not started: radio init failed (see the \"SX1262 begin failed\" line)");
    }

    // ", region AU915 sub-band 2, ADR, TX power 2..33 dBm"
    char tail[96];
    char sb[16] = "";
    if (b->subband) snprintf(sb, sizeof(sb), " sub-band %u", (unsigned)b->subband);
    char mode[16];
    if (b->adr) snprintf(mode, sizeof(mode), "ADR");
    else        snprintf(mode, sizeof(mode), "fixed DR%u", (unsigned)b->fixed_dr);
    char pwr[32] = "";
    if (b->pwr_min != b->pwr_max) snprintf(pwr, sizeof(pwr), ", TX power %d..%d dBm", (int)b->pwr_min, (int)b->pwr_max);
    snprintf(tail, sizeof(tail), "region %s%s, %s%s", b->region ? b->region : "?", sb, mode, pwr);

    if (b->kind == LW_BOOT_RESUMED) {
        return snprintf(out, n, "session resumed — DevAddr %08lx, %s", (unsigned long)b->dev_addr, tail);
    }
    if (b->kind == LW_BOOT_JOINING) {
        if (b->attempts == 0) return snprintf(out, n, "joining — first attempt pending, %s", tail);
        return snprintf(out, n, "joining — attempt %lu, last error %d (%s), %s", (unsigned long)b->attempts,
                        (int)b->last_error, lw_err_text(b->last_error), tail);
    }

    // LW_BOOT_JOINED
    char via[12] = "";   // " via RX255" worst case for a uint8_t
    if (b->ok_window) snprintf(via, sizeof(via), " via RX%u", (unsigned)b->ok_window);
    char dr[12] = "";
    if (b->ok_dr != 0xFF) snprintf(dr, sizeof(dr), " at DR%u", (unsigned)b->ok_dr);
    return snprintf(out, n, "joined%s%s (SF%u, %lu.%lu MHz) — DevAddr %08lx, %s", via, dr, (unsigned)b->ok_sf,
                    (unsigned long)(b->ok_khz / 1000), (unsigned long)((b->ok_khz % 1000) / 100),
                    (unsigned long)b->dev_addr, tail);
}
