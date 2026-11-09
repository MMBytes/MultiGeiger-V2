// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

/** @file lorawan_rxobs.h
 *  @brief Passive RX-window observer and its "rx windows:" log line (V2.8.21).
 *
 *  Records what the SX1262 did around ONE uplink or join request from the
 *  SPI commands RadioLib already sends — nothing is added to, or changed in,
 *  RadioLib's SPI traffic. lorawan.cpp feeds every SPI transaction of an armed
 *  exchange into rxobs_note(); afterwards rxobs_format() and rxobs_uplink_tx()
 *  render the node-log lines. Pure C (no IDF, no RadioLib), so the host tests
 *  in test/test_main.c drive it with simulated SPI sequences.
 *
 *  Why one line per exchange: the node log is all most owners have (no TTN
 *  console or gateway records), so the line must say on its own which
 *  channel each NbTrans repeat used, what each RX window saw, and why fewer
 *  repeats than NbTrans went out.
 *
 *  SX126x opcodes / IRQ bits: SX1261/2 datasheet §13 (RadioLib's
 *  RADIOLIB_SX126X_CMD_* / RADIOLIB_SX126X_IRQ_*). The RadioLib status codes
 *  below are cross-checked against RadioLib's TypeDef.h in lorawan.cpp.
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lorawan_sched.h"   // lw_sched_frame_is_ours()

// RadioLib status codes the observer names (values from RadioLib TypeDef.h).
#define LW_RXOBS_ERR_TX_TIMEOUT          (-5)
#define LW_RXOBS_ERR_CRC_MISMATCH        (-7)
#define LW_RXOBS_ERR_DOWNLINK_MALFORMED  (-1102)
#define LW_RXOBS_ERR_INVALID_PORT        (-1104)
#define LW_RXOBS_ERR_NO_RX_WINDOW        (-1105)
#define LW_RXOBS_ERR_MIC_MISMATCH        (-1112)

#define OBS_CMD_SET_TX        0x83   // SetTx
#define OBS_CMD_SET_RX        0x82   // SetRx — 24-bit timeout, 15.625 µs/LSB
#define OBS_CMD_SET_FREQ      0x86   // SetRfFrequency — 32-bit, F = raw * 32 MHz / 2^25
#define OBS_CMD_SET_MOD       0x8B   // SetModulationParams — byte 1 = LoRa SF
#define OBS_CMD_GET_IRQ       0x12   // GetIrqStatus — reply bytes 2..3 = IRQ flags
#define OBS_CMD_CLEAR_IRQ     0x02   // ClearIrqStatus — first one after SetTx = TX end
#define OBS_CMD_READ_BUFFER   0x1E   // ReadBuffer — reply: 2 cmd + 1 status byte, then the
                                     // FIFO data (Module::SPItransferStream layout)
#define OBS_IRQ_RX_DONE       0x0002
#define OBS_IRQ_HEADER_VALID  0x0010
#define OBS_IRQ_HEADER_ERR    0x0020
#define OBS_IRQ_CRC_ERR       0x0040
#define OBS_IRQ_TIMEOUT       0x0200

/** Repeats recorded per exchange. TTN sets NbTrans <= 3; LoRaWAN allows 15 —
 *  TX past this are still counted (n_tx) and shown as "+N more TX not listed". */
#define OBS_MAX_REP 4

/** Buffer size for the "rx windows:" line (verdict first, so truncation
 *  only ever drops detail). */
#define LW_RXOBS_LINE_MAX 640

/** One RX window over all repeats of an exchange (folded). */
typedef struct {
    int64_t  open_us;        // time of the SetRx that opened the window
    uint32_t timeout_us;     // SX126x RX timeout programmed for it
    uint32_t freq_raw;       // SetRfFrequency word in force when it opened
    uint8_t  sf;             // LoRa SF in force when it opened
    uint16_t irq;            // OR of the GetIrqStatus replies of the CURRENT repeat
    uint16_t best_irq;       // most informative earlier repeat (rxobs_rank()); valid
    bool     has_best;       //   only if has_best — see rxobs_fold()
    // Frame read out of the FIFO in this window (RadioLib's readData() after
    // RX_DONE): MHDR + DevAddr are plaintext in a data downlink, enough to
    // tell "for us" from "someone else's" from "ours, corrupted" (LoRaWAN
    // downlinks carry NO PHY CRC).
    bool     rx_seen;
    uint8_t  rx_len;
    uint8_t  rx_mhdr;
    uint32_t rx_addr;        // little-endian on air (LoRaWAN FHDR)
    uint8_t  n_open;         // how many repeats opened this window
    bool     ignored;        // RadioLib dropped the frame (not ours / unreadable)
} rxobs_win_t;

/** What ONE repeat's window saw. */
typedef struct {
    bool     opened;
    bool     rx_seen;        // a frame was read out of the FIFO (fields below valid)
    bool     ignored;        // RadioLib dropped the frame: the window counts as empty
    uint8_t  rx_len;
    uint8_t  rx_mhdr;
    uint16_t irq;            // OR of this window's GetIrqStatus replies
    uint32_t rx_addr;
    // V2.8.23: the MAC part of a data downlink (rxobs_note_mac()). FOpts are
    // plaintext in LoRaWAN 1.0.x (encrypted in 1.1); an FPort-0 payload is
    // always encrypted, so only its length is kept.
    bool     mac_seen;       // a data down with a consistent FOptsLen (fields below valid)
    uint8_t  fopts_len;      // FCtrl bits 3..0: MAC command bytes in the header (0..15)
    uint8_t  fopts[15];
    int16_t  fport;          // -1 = no FPort (frame without payload)
    uint8_t  frm_len;        // FRMPayload bytes after the FPort
} rxobs_hit_t;

/** One transmission (NbTrans repeat, or a MAC-only uplink RadioLib sent inside
 *  the exchange). */
typedef struct {
    uint32_t    tx_freq_raw;
    uint32_t    air_us;      // SetTx to TX end (first ClearIrqStatus); 0 = end not seen
    rxobs_hit_t w[2];        // [0] = RX1, [1] = RX2
} rxobs_rep_t;

/** Observer state; one instance, written only by the LoRaWAN worker task. */
typedef struct {
    bool     on;             // armed — set/cleared by the worker around one exchange
    uint32_t cur_freq_raw;   // last SetRfFrequency word seen (RX or TX)
    uint8_t  cur_sf;         // last SF seen
    int64_t  tx_us;          // SetTx time; 0 = no TX seen yet (pre-TX SetRx calls are
                             // RadioLib's RNG sampling, not RX windows — ignored)
    int64_t  tx_done_us;     // TX end: first ClearIrqStatus after SetTx; 0 = not seen
                             // (SX126x::finishTransmit() clears the IRQs within ~1 ms of
                             // TX_DONE; RadioLib's checkIrq() read before it is dropped
                             // by the n_win > 0 check in GetIrqStatus)
    uint32_t tx_freq_raw;
    uint8_t  tx_sf;
    uint8_t  n_tx;           // transmissions in this exchange
    int      n_win;          // windows opened after the CURRENT transmission (0..2)
    rxobs_win_t win[2];      // [0] = RX1, [1] = RX2 (Class A), folded over repeats
    uint8_t  nb_trans;       // NbTrans in force when the exchange started
    rxobs_rep_t rep[OBS_MAX_REP];   // per transmission, [n_tx - 1] = current
    uint32_t own_addr;       // this node's DevAddr; 0 = not joined (join accepts expected)
} rxobs_t;

/** How an exchange splits up: the frame's own repeats, then any MAC-only
 *  uplink RadioLib's parseDownlink() sent inside it (its SetTx land in the
 *  same observer run). */
typedef struct {
    int own_tx;    // TX of the frame itself (repeats 1..own_tx)
    int end_rep;   // 0-based repeat whose window ended the exchange, -1 = none
    int end_win;   // 1 = RX1, 2 = RX2 of that repeat, 0 = none
} rxobs_split_t;

/** @brief How informative one window result is, highest first: a frame (clean >
 *         CRC error > header error > header only) beats a bare preamble ("no
 *         timeout"), which beats a plain timeout. "No timeout" is the ABSENCE of
 *         a bit, so OR-ing raw flags over repeats would let one repeat's TIMEOUT
 *         hide another repeat's preamble. */
static inline int rxobs_rank(uint16_t irq) {
    if ((irq & OBS_IRQ_RX_DONE) && !(irq & OBS_IRQ_CRC_ERR)) return 6;
    if (irq & OBS_IRQ_CRC_ERR)      return 5;
    if (irq & OBS_IRQ_HEADER_ERR)   return 4;
    if (irq & OBS_IRQ_HEADER_VALID) return 3;
    if (!(irq & OBS_IRQ_TIMEOUT))   return 2;   // preamble kept the window open
    return 1;                                   // plain timeout
}

/** @brief Fold a window's current-repeat flags into its best-so-far result. */
static inline void rxobs_fold(rxobs_win_t *w) {
    if (!w->has_best || rxobs_rank(w->irq) >= rxobs_rank(w->best_irq)) w->best_irq = w->irq;
    w->has_best = true;
}

/** @brief A window's final result over all repeats (the current repeat's flags
 *         are not folded until the window reopens). */
static inline uint16_t rxobs_final(const rxobs_win_t *w) {
    if (!w->has_best || rxobs_rank(w->irq) >= rxobs_rank(w->best_irq)) return w->irq;
    return w->best_irq;
}

/** @brief Arm the observer for one exchange. Keeps cur_freq/cur_sf (RadioLib
 *         may reuse the modem settings without resending them) and own_addr.
 *  @param nb_trans NbTrans RadioLib will use, read BEFORE the exchange (a
 *         LinkADRReq in its downlink changes it for the NEXT frame); 1 for a join. */
static inline void rxobs_begin(rxobs_t *o, uint8_t nb_trans) {
    o->tx_us = 0;
    o->tx_done_us = 0;
    o->n_tx = 0;
    o->n_win = 0;
    memset(o->win, 0, sizeof(o->win));
    o->nb_trans = nb_trans;
    memset(o->rep, 0, sizeof(o->rep));
    o->on = true;
}

/** @brief The current repeat's record for window @p i (0 = RX1), or NULL past OBS_MAX_REP. */
static inline rxobs_hit_t *rxobs_hit(rxobs_t *o, int i) {
    if (o->n_tx == 0 || o->n_tx > OBS_MAX_REP || i < 0 || i > 1) return NULL;
    return &o->rep[o->n_tx - 1].w[i];
}

/** @brief V2.8.23: keep the MAC part of a data downlink read out of the FIFO.
 *         Frame layout (LoRaWAN 1.0.4 §4): MHDR 1, DevAddr 4, FCtrl 1, FCnt 2,
 *         FOpts 0..15, then optionally FPort 1 + FRMPayload, MIC 4 — so 12
 *         bytes plus FOpts at least. Kept for every frame; the caller only
 *         uses the one RadioLib accepted (rxobs_accepted_hit()).
 *  @param f, flen the frame as read (from MHDR, without the SPI header) */
static inline void rxobs_note_mac(rxobs_hit_t *h, const uint8_t *f, size_t flen) {
    h->mac_seen = false;
    const uint8_t mtype = f[0] >> 5;
    if ((mtype != 3 && mtype != 5) || flen < 12) return;   // not a data down
    const uint8_t fol = f[5] & 0x0F;
    if (flen < 12u + fol) return;
    h->mac_seen = true;
    h->fopts_len = fol;
    memcpy(h->fopts, &f[8], fol);
    if (flen > 12u + fol) {
        h->fport = f[8 + fol];
        h->frm_len = (uint8_t)(flen - 12u - fol - 1u);
    } else {
        h->fport = -1;
        h->frm_len = 0;
    }
}

/** @brief SetRfFrequency word → MHz. */
static inline double rxobs_mhz(uint32_t raw) { return (double)raw * 32.0 / 33554432.0; }

/** @brief Feed one SPI transaction of an armed exchange.
 *  @param now_us time of the transaction (esp_timer µs on the node)
 *  @param out    request bytes; @p in = reply bytes (same length, may be NULL) */
static inline void rxobs_note(rxobs_t *o, int64_t now_us, const uint8_t *out, size_t len, const uint8_t *in) {
    if (out == NULL || len == 0) return;
    switch (out[0]) {
    case OBS_CMD_SET_FREQ:
        if (len >= 5) {
            o->cur_freq_raw = ((uint32_t)out[1] << 24) | ((uint32_t)out[2] << 16) |
                              ((uint32_t)out[3] << 8) | out[4];
        }
        break;
    case OBS_CMD_SET_MOD:
        if (len >= 2) o->cur_sf = out[1];
        break;
    case OBS_CMD_SET_TX:
        // A repeat (or a MAC-only uplink) starts a new window pair. The folded
        // win[] keeps each window's MOST INFORMATIVE result over all repeats
        // (the network answers whichever repeat the gateway heard); rep[]
        // keeps every repeat separately.
        o->tx_us = now_us;
        o->tx_done_us = 0;
        o->tx_freq_raw = o->cur_freq_raw;
        o->tx_sf = o->cur_sf;
        if (o->n_tx < 255) o->n_tx++;
        o->n_win = 0;
        if (o->n_tx <= OBS_MAX_REP) o->rep[o->n_tx - 1].tx_freq_raw = o->cur_freq_raw;
        break;
    case OBS_CMD_SET_RX:
        // Timeout 0xFFFFFF = continuous RX: RadioLib's random() sampling
        // wideband RSSI (once per session, to seed rand()), never a Class A window.
        if (len >= 4 && out[1] == 0xFF && out[2] == 0xFF && out[3] == 0xFF) break;
        if (len >= 4 && o->tx_us != 0 && o->n_win < 2) {
            rxobs_win_t *w = &o->win[o->n_win++];
            w->open_us = now_us;
            // 15.625 µs per LSB = 1/64 ms (datasheet §13.1.4 SetRx).
            w->timeout_us = ((((uint32_t)out[1] << 16) | ((uint32_t)out[2] << 8) | out[3]) * 125u) / 8u;
            w->freq_raw = o->cur_freq_raw;
            w->sf = o->cur_sf;
            if (w->n_open > 0) rxobs_fold(w);   // keep the earlier repeat's result
            w->irq = 0;
            if (w->n_open < 255) w->n_open++;
            rxobs_hit_t *h = rxobs_hit(o, o->n_win - 1);
            if (h) h->opened = true;
        }
        break;
    case OBS_CMD_CLEAR_IRQ:
        // Only the FIRST clear after SetTx marks TX end (finishTransmit()); the
        // next one, still before RX1's SetRx, is startReceive()'s own clear.
        if (o->tx_us != 0 && o->tx_done_us == 0 && o->n_win == 0) {
            o->tx_done_us = now_us;
            if (o->n_tx >= 1 && o->n_tx <= OBS_MAX_REP) {
                o->rep[o->n_tx - 1].air_us = (uint32_t)(now_us - o->tx_us);
            }
        }
        break;
    case OBS_CMD_GET_IRQ:
        if (len >= 4 && in && o->n_win > 0) {
            const uint16_t irq = (uint16_t)(((uint16_t)in[2] << 8) | in[3]);
            o->win[o->n_win - 1].irq |= irq;
            rxobs_hit_t *h = rxobs_hit(o, o->n_win - 1);
            if (h) h->irq |= irq;
        }
        break;
    case OBS_CMD_READ_BUFFER:
        // The frame is read after the window that received it closed, so it
        // belongs to win[n_win - 1]. 3 header bytes (cmd, offset, status),
        // then the frame. RadioLib 8.0's checkDownlink() reads every frame it
        // did not reject on length; one it then rejects on MType/DevAddr/FOpts
        // is dropped silently (the window counts as empty, RX2 and the repeats
        // go on) — lw_sched_frame_is_ours() mirrors that decision.
        if (len >= 3 + 5 && in && o->n_win > 0) {
            rxobs_win_t *w = &o->win[o->n_win - 1];
            w->rx_seen = true;
            w->rx_len  = (uint8_t)((len - 3) > 255 ? 255 : (len - 3));
            w->rx_mhdr = in[3];
            w->rx_addr = (uint32_t)in[4] | ((uint32_t)in[5] << 8) |
                         ((uint32_t)in[6] << 16) | ((uint32_t)in[7] << 24);
            w->ignored = !lw_sched_frame_is_ours(o->own_addr != 0, o->own_addr, &in[3], len - 3);
            rxobs_hit_t *h = rxobs_hit(o, o->n_win - 1);
            if (h) {
                h->rx_seen = true;
                h->rx_len  = w->rx_len;
                h->rx_mhdr = w->rx_mhdr;
                h->rx_addr = w->rx_addr;
                h->ignored = w->ignored;
                rxobs_note_mac(h, &in[3], len - 3);   // V2.8.23
            }
        }
        break;
    default:
        break;
    }
}

/** @brief Append to @p buf at *@p n; once truncated, every later call is a no-op
 *         (*n then stays >= sz, so no caller computes a negative size). */
static inline void rxobs_cat(char *buf, size_t sz, int *n, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));
static inline void rxobs_cat(char *buf, size_t sz, int *n, const char *fmt, ...) {
    if (*n < 0 || (size_t)*n >= sz) return;
    va_list ap;
    va_start(ap, fmt);
    const int r = vsnprintf(buf + *n, sz - (size_t)*n, fmt, ap);
    va_end(ap);
    if (r > 0) *n += r;
}

/** @brief Describe a frame a window received, e.g. " (data down for 260b1234 (not
 *         ours), 33 B)". MType = MHDR bits 7..5: 1 join accept, 3/5 (un)confirmed
 *         data down. A DevAddr within 4 bits of ours is most likely OUR frame
 *         with bit errors (a foreign one differs in ~16 bits on average). */
static inline void rxobs_frame(const rxobs_t *o, uint8_t mhdr, uint8_t len, uint32_t addr, char *out, size_t sz) {
    const uint8_t mtype = mhdr >> 5;
    if (mtype == 1) {
        snprintf(out, sz, " (join accept, %u B)", (unsigned)len);
    } else if (mtype == 3 || mtype == 5) {
        const char *whose;
        if (o->own_addr == 0) {
            whose = "";
        } else if (addr == o->own_addr) {
            whose = " (ours)";
        } else if (__builtin_popcount(addr ^ o->own_addr) <= 4) {
            whose = " (likely ours, corrupted)";
        } else {
            whose = " (not ours)";
        }
        snprintf(out, sz, " (data down for %08lx%s, %u B)", (unsigned long)addr, whose, (unsigned)len);
    } else {
        // Uplink MTypes or Proprietary in a downlink window: garbage or foreign.
        snprintf(out, sz, " (MHDR 0x%02x, %u B — not a downlink, likely corrupted)", (unsigned)mhdr,
                 (unsigned)len);
    }
}

/** @brief What one window saw, from the IRQ flags RadioLib reads back.
 *         RadioLib 8.0's Class A windows latch only RX_DONE, HEADER_ERR and
 *         TIMEOUT (LoRaWAN.cpp receiveClassA(): "no PHY CRC on downlinks"), with
 *         a symbol-count preamble timeout: TIMEOUT = no preamble found;
 *         neither TIMEOUT nor RX_DONE = a detected preamble held the window
 *         open but no frame followed. The CRC_ERR and HEADER_VALID branches
 *         only fire if a future RadioLib latches those flags again. */
static inline const char *rxobs_result(uint16_t irq) {
    if (irq & OBS_IRQ_CRC_ERR)    return "CRC_ERR";
    if (irq & OBS_IRQ_RX_DONE)    return "RX_DONE";
    if (irq & OBS_IRQ_HEADER_ERR) return "HDR_ERR";
    if (irq & OBS_IRQ_TIMEOUT) {
        return (irq & OBS_IRQ_HEADER_VALID) ? "header, then timeout" : "timeout, no preamble";
    }
    if (irq & OBS_IRQ_HEADER_VALID) return "header, no end";
    return "no timeout (preamble?)";
}

/** @brief Plain words for a RadioLib status code, for every LoRaWAN log line
 *         that names one (printed as "<code> (<words>)"). Covers the codes
 *         begin(), activateOTAA(), sendReceive(), setDatarate(), loadBuffers()
 *         and sendMacCommandReq() can return (RadioLib 8.0 TypeDef.h); the
 *         numbers are literals so this header stays RadioLib-free — every one
 *         of them is static_assert-ed against RadioLib's name in lorawan.cpp.
 *  @return a non-empty string without the code number. */
static inline const char *lw_err_text(int16_t st) {
    switch (st) {
    case -1:    return "unknown error";
    case -2:    return "radio chip not found (SPI wiring, reset or power)";
    case -4:    return "payload too long for this data rate, even without MAC answers";
    case LW_RXOBS_ERR_TX_TIMEOUT: return "TX timeout (the radio did not report the end of the transmission)";
    case -6:    return "RX timeout";
    case LW_RXOBS_ERR_CRC_MISMATCH: return "frame corrupted (integrity check failed)";
    case -12:   return "invalid frequency for this radio";
    case -13:   return "invalid TX power for this radio";
    case -16:   return "SPI write to the radio failed";
    case -20:   return "radio in the wrong modem mode";
    case -24:   return "LoRa header damaged";
    case -28:   return "internal error (NULL pointer)";
    case -501:  return "no channel in this region allows this data rate";
    // SX126x chip faults: -703 from begin()'s TCXO setup; -705..-707 from the
    // status byte or BUSY wait of ANY radio command, so also mid-exchange.
    case -703:  return "TCXO voltage not accepted by the radio";
    case -705:  return "radio stayed busy, SPI command timed out";
    case -706:  return "radio rejected an SPI command as invalid";
    case -707:  return "radio could not execute an SPI command (TCXO vs crystal setting?)";
    case -1101: return "not joined to a network";
    case LW_RXOBS_ERR_DOWNLINK_MALFORMED: return "downlink addressed to us but malformed (bad FPort or FOpts), dropped";
    case -1103: return "LoRaWAN revision not supported";
    case LW_RXOBS_ERR_INVALID_PORT: return "invalid FPort, frame corrupted";
    case LW_RXOBS_ERR_NO_RX_WINDOW: return "RX window missed: RX1 held open past RX2, or too late";
    case -1106: return "no channel available for this data rate";
    case -1108: return "uplink not allowed yet (duty-cycle limit)";
    case -1109: return "MAC command queue full";
    case -1111: return "join accept nonce invalid (replayed accept)";
    case LW_RXOBS_ERR_MIC_MISMATCH: return "MIC mismatch, frame corrupted";
    case -1114: return "frame too long for the 400 ms dwell-time limit";
    case -1115: return "saved data corrupt or empty (checksum mismatch)";
    case -1116: return "no join accept received";
    case -1119: return "saved nonces discarded (keys or version changed)";
    case -1120: return "saved session discarded (keys or version changed)";
    case -1123: return "invalid sub-band for this region";
    case -1124: return "payload fits, but not together with the queued MAC answers";
    default:    return "RadioLib error, see RadioLib TypeDef.h";
    }
}

/** @brief True for sendReceive() errors that come from the RECEIVE side only,
 *         after the uplink itself went on air (FCnt advanced): a frame with our
 *         DevAddr that parseDownlink() rejected (-1102 malformed, -1104 FPort,
 *         -1112 MIC — downlinks carry no PHY CRC, so usually bit errors) or RX2
 *         already past (-1105). Counted as "sent, no usable downlink", not as a
 *         failed uplink. (RadioLib drops someone else's downlink by itself.) */
static inline bool lw_downlink_unusable(int16_t st) {
    return st == LW_RXOBS_ERR_DOWNLINK_MALFORMED || st == LW_RXOBS_ERR_INVALID_PORT ||
           st == LW_RXOBS_ERR_NO_RX_WINDOW || st == LW_RXOBS_ERR_MIC_MISMATCH;
}

/** Which restore line fits a failed loadBuffers() (see lw_restore_verdict()). */
typedef enum {
    LW_RESTORE_OK = 0,     // session restored
    LW_RESTORE_NO_PERS,    // no DevNonce history saved (new board / full reset)
    LW_RESTORE_PERS_BAD,   // DevNonce history saved but rejected (keys/EUIs changed, corrupt)
    LW_RESTORE_NO_SESS,    // history fine, no session saved
    LW_RESTORE_SESS_BAD,   // history fine, session saved but rejected
    LW_RESTORE_SESS_UNFINISHED, // session saved right after a join, before its
                                // first uplink (withheld from RadioLib, see lorawan.cpp)
} lw_restore_t;

/** @brief Classify a loadBuffers() result. RadioLib 8.0 restores the
 *         persistence buffer (DevNonce/JoinNonce) first and returns on
 *         failure without calling the session restore callback.
 *  @param st          loadBuffers()'s result
 *  @param pers_found  the persistence restore callback found a blob in NVS
 *  @param sess_called the session restore callback was called at all
 *  @param sess_found  ... and found a blob in NVS
 *  @param sess_unfinished ... which was still "activating" and so handed to
 *                     RadioLib as empty (RadioLib then returns -1115) */
static inline lw_restore_t lw_restore_verdict(int16_t st, bool pers_found, bool sess_called, bool sess_found,
                                              bool sess_unfinished) {
    if (st == 0) return LW_RESTORE_OK;
    if (!pers_found) return LW_RESTORE_NO_PERS;
    if (!sess_called) return LW_RESTORE_PERS_BAD;
    if (!sess_found) return LW_RESTORE_NO_SESS;
    if (sess_unfinished) return LW_RESTORE_SESS_UNFINISHED;
    return LW_RESTORE_SESS_BAD;
}

/** @brief A window RadioLib passed over: a frame it read and rejected, or an
 *         RX_DONE it never read (too long/short, over the data rate's size). */
static inline bool rxobs_hit_dropped(const rxobs_hit_t *h) {
    return h->opened && (h->ignored || ((h->irq & OBS_IRQ_RX_DONE) && !h->rx_seen));
}

/** @brief One window of one repeat: "RX1 timeout, no preamble", "RX2 not opened",
 *         "RX1 RX_DONE (data down for 260b1234 (not ours), 14 B) ignored, repeats go on". */
static inline void rxobs_hit_cat(const rxobs_t *o, char *buf, size_t sz, int *n, int i, const rxobs_hit_t *h) {
    if (!h->opened) { rxobs_cat(buf, sz, n, "RX%d not opened", i + 1); return; }
    char frame[72] = "";
    if (h->rx_seen) rxobs_frame(o, h->rx_mhdr, h->rx_len, h->rx_addr, frame, sizeof(frame));
    else if (h->irq & OBS_IRQ_RX_DONE) snprintf(frame, sizeof(frame), " (not read: RadioLib dropped it, too long/short)");
    rxobs_cat(buf, sz, n, "RX%d %s%s%s", i + 1, rxobs_result(h->irq), frame,
              rxobs_hit_dropped(h) ? " ignored, repeats go on" : "");
}

/** @brief Which repeat's window ended the exchange, and how many TX were the
 *         frame's own. @p st = sendReceive()'s result (1/2 = downlink in RX1/RX2,
 *         0 = none, < 0 = error). */
static inline rxobs_split_t rxobs_split(const rxobs_t *o, int16_t st) {
    rxobs_split_t s;
    s.own_tx = o->n_tx;
    s.end_rep = -1;
    s.end_win = 0;
    if (st == 0) return s;   // nothing ended it; a dropped frame let the repeats go on
    const int recorded = o->n_tx < OBS_MAX_REP ? o->n_tx : OBS_MAX_REP;
    // st < 0: any error breaks RadioLib's repeat loop and a failed parse never
    // nests, so the ending window is in the LAST repeat. st > 0: a nested
    // MAC-only uplink may follow, so the frame's repeat is the first one with a
    // frame RadioLib kept. An RX_DONE nobody read only counts as a fallback.
    const int first = (st < 0 && recorded > 0) ? recorded - 1 : 0;
    for (int pass = 0; pass < 2 && s.end_rep < 0; pass++) {
        for (int r = first; r < recorded && s.end_rep < 0; r++) {
            for (int k = 0; k < 2; k++) {
                // st < 0: the ending window is the LAST one opened, so RX2 first.
                const int i = st < 0 ? 1 - k : k;
                const rxobs_hit_t *h = &o->rep[r].w[i];
                const bool hit = pass == 0 ? h->rx_seen : (h->irq & OBS_IRQ_RX_DONE) != 0;
                if (h->opened && !h->ignored && hit) { s.end_rep = r; s.end_win = i + 1; break; }
            }
        }
    }
    if (s.end_rep >= 0) s.own_tx = s.end_rep + 1;
    return s;
}

/** @brief V2.8.23: the window record of the downlink RadioLib accepted, or
 *         NULL. Only for @p st = 1/2 (valid downlink): the same window
 *         rxobs_split() names as the end of the exchange. */
static inline const rxobs_hit_t *rxobs_accepted_hit(const rxobs_t *o, int16_t st) {
    if (st != 1 && st != 2) return NULL;
    const rxobs_split_t s = rxobs_split(o, st);
    if (s.end_rep < 0 || s.end_rep >= OBS_MAX_REP) return NULL;
    const rxobs_hit_t *h = &o->rep[s.end_rep].w[s.end_win - 1];
    return h->rx_seen ? h : NULL;
}

/** @brief V2.8.23: the first frame RadioLib kept in the windows of a MAC-only
 *         uplink it sent inside the exchange (TX after the frame's own
 *         repeats), or NULL. Whether RadioLib accepted it is not visible here;
 *         the caller only uses it when that downlink's answers are queued. */
static inline const rxobs_hit_t *rxobs_nested_hit(const rxobs_t *o, int16_t st) {
    const rxobs_split_t s = rxobs_split(o, st);
    const int recorded = o->n_tx < OBS_MAX_REP ? o->n_tx : OBS_MAX_REP;
    for (int r = s.own_tx; r < recorded; r++) {
        for (int i = 0; i < 2; i++) {
            const rxobs_hit_t *h = &o->rep[r].w[i];
            if (h->opened && !h->ignored && h->rx_seen) return h;
        }
    }
    return NULL;
}

/** @brief Two repeats read the same in the log (one shared entry) if neither
 *         received a frame and their windows ended the same way. */
static inline bool rxobs_rep_same(const rxobs_rep_t *a, const rxobs_rep_t *b) {
    for (int i = 0; i < 2; i++) {
        if (a->w[i].rx_seen || b->w[i].rx_seen || a->w[i].opened != b->w[i].opened) return false;
        if (a->w[i].opened && strcmp(rxobs_result(a->w[i].irq), rxobs_result(b->w[i].irq)) != 0) return false;
    }
    return true;
}

/** @brief The NbTrans > 1 form: verdict first, then every repeat's channel and
 *         window results, then the window timing of the last TX. Examples:
 *   NbTrans 3 (set by network) | all 3 sent | #1 867.100, #2 868.500, #3 867.300:
 *     RX1 timeout, no preamble; RX2 timeout, no preamble | last TX: RX1 ...
 *   NbTrans 3 (set by network) | stopped after 1 of 3: valid downlink in RX1 of
 *     #1 — a downlink ends the repeats (LoRaWAN) | #1 867.900: RX1 RX_DONE (...) */
static inline void rxobs_format_multi(const rxobs_t *o, char *buf, size_t sz, int16_t st) {
    int n = 0;
    *buf = '\0';
    rxobs_cat(buf, sz, &n, "NbTrans %u%s |", (unsigned)o->nb_trans, o->nb_trans > 1 ? " (set by network)" : "");

    const rxobs_split_t s = rxobs_split(o, st);
    const unsigned own = (unsigned)s.own_tx, want = o->nb_trans;
    const unsigned extra = o->n_tx > own ? o->n_tx - own : 0;
    const bool valid = (st == 1 || st == 2);
    if (s.end_rep >= 0 && valid) {
        if (own < want) {
            rxobs_cat(buf, sz, &n, " stopped after %u of %u: valid downlink in RX%d of #%u — a downlink ends "
                      "the repeats (LoRaWAN)", own, want, s.end_win, own);
        } else {
            rxobs_cat(buf, sz, &n, " all %u sent, valid downlink in RX%d of #%u", own, s.end_win, own);
        }
    } else if (s.end_rep >= 0) {
        if (own < want) {
            rxobs_cat(buf, sz, &n, " stopped after %u of %u: unusable frame in RX%d of #%u, %d (%s), ended them "
                      "early — LoRaWAN says only a valid downlink ends the repeats", own, want, s.end_win, own,
                      (int)st, lw_err_text(st));
        } else {
            rxobs_cat(buf, sz, &n, " all %u sent; #%u ended on an unusable frame in RX%d, %d (%s)", own, own,
                      s.end_win, (int)st, lw_err_text(st));
        }
    } else if (st < 0) {
        // "at TX k": the k-th TX was the last one tried (with -5 the one that failed).
        if (own < want) rxobs_cat(buf, sz, &n, " stopped at TX %u of %u: %d (%s)", own, want, (int)st, lw_err_text(st));
        else            rxobs_cat(buf, sz, &n, " all %u sent, then %d (%s)", own, (int)st, lw_err_text(st));
    } else if (own < want) {
        rxobs_cat(buf, sz, &n, " stopped after %u of %u, no downlink (cause not seen)", own, want);
    } else {
        rxobs_cat(buf, sz, &n, " all %u sent", own);
    }
    if (extra > 0) {
        rxobs_cat(buf, sz, &n, "; then %u TX of a MAC-only uplink (M1..: RadioLib sends MAC answers that do "
                  "not fit the next frame on their own)", extra);
    }
    rxobs_cat(buf, sz, &n, " |");

    const int recorded = o->n_tx < OBS_MAX_REP ? o->n_tx : OBS_MAX_REP;
    for (int i = 0; i < recorded;) {
        // Group only within the frame's own repeats or within the MAC-only TX.
        int j = i + 1;
        while (j < recorded && (j < s.own_tx) == (i < s.own_tx) && rxobs_rep_same(&o->rep[i], &o->rep[j])) j++;
        for (int k = i; k < j; k++) {
            rxobs_cat(buf, sz, &n, "%s%c%d %.3f", k == i ? " " : ", ", k < s.own_tx ? '#' : 'M',
                      k < s.own_tx ? k + 1 : k - s.own_tx + 1, rxobs_mhz(o->rep[k].tx_freq_raw));
        }
        rxobs_cat(buf, sz, &n, ": ");
        rxobs_hit_cat(o, buf, sz, &n, 0, &o->rep[i].w[0]);
        rxobs_cat(buf, sz, &n, "; ");
        rxobs_hit_cat(o, buf, sz, &n, 1, &o->rep[i].w[1]);
        rxobs_cat(buf, sz, &n, " |");
        i = j;
    }
    if (o->n_tx > OBS_MAX_REP) rxobs_cat(buf, sz, &n, " +%u more TX not listed |", (unsigned)(o->n_tx - OBS_MAX_REP));

    // Window timing once, from the last TX (same delays every repeat).
    const int64_t ref = o->tx_done_us ? o->tx_done_us : o->tx_us;
    rxobs_cat(buf, sz, &n, " last TX:");
    for (int i = 0; i < 2; i++) {
        const rxobs_win_t *w = &o->win[i];
        if (i < o->n_win) {
            rxobs_cat(buf, sz, &n, "%s RX%d %.3f SF%u @+%ld ms to=%lu ms", i ? ";" : "", i + 1, rxobs_mhz(w->freq_raw),
                      (unsigned)w->sf, (long)((w->open_us - ref) / 1000), (unsigned long)(w->timeout_us / 1000));
        } else {
            rxobs_cat(buf, sz, &n, "%s RX%d not opened", i ? ";" : "", i + 1);
        }
    }
}

/** @brief Render the "rx windows:" text of one exchange into @p buf.
 *         One TX: "RX1 868.100 SF9 @+4989 ms to=113 ms timeout, no preamble | RX2 ...";
 *         NbTrans > 1 or more than one TX: rxobs_format_multi(). Window offsets are
 *         from TX end of the LAST TX, so they read as the RX1/RX2 delay minus
 *         RadioLib's early-open margin (from SetTx, incl. the ToA, if TX end was
 *         not seen). @p st = the exchange's result (0 for a join). */
static inline void rxobs_format(const rxobs_t *o, char *buf, size_t sz, int16_t st) {
    if (sz == 0) return;
    if (o->tx_us == 0) { snprintf(buf, sz, "no TX seen"); return; }
    if (o->nb_trans > 1 || o->n_tx > 1) { rxobs_format_multi(o, buf, sz, st); return; }
    const int64_t ref = o->tx_done_us ? o->tx_done_us : o->tx_us;
    int n = 0;
    *buf = '\0';
    for (int i = 0; i < 2; i++) {
        const rxobs_win_t *w = &o->win[i];
        if (w->n_open == 0) {
            // RX2 is skipped when RX1 received a frame RadioLib kept — normal.
            rxobs_cat(buf, sz, &n, "%sRX%d not opened", i ? " | " : "", i + 1);
            continue;
        }
        char when[24];
        if (i < o->n_win) snprintf(when, sizeof(when), "@+%ld ms", (long)((w->open_us - ref) / 1000));
        else              snprintf(when, sizeof(when), "@earlier TX");
        char frame[72] = "";
        const uint16_t irq = rxobs_final(w);
        if (w->rx_seen) rxobs_frame(o, w->rx_mhdr, w->rx_len, w->rx_addr, frame, sizeof(frame));
        else if (irq & OBS_IRQ_RX_DONE) snprintf(frame, sizeof(frame), " (not read: RadioLib dropped it, too long/short)");
        const bool dropped = w->ignored || ((irq & OBS_IRQ_RX_DONE) && !w->rx_seen);
        rxobs_cat(buf, sz, &n, "%sRX%d %.3f SF%u %s to=%lu ms %s%s%s", i ? " | " : "", i + 1, rxobs_mhz(w->freq_raw),
                  (unsigned)w->sf, when, (unsigned long)(w->timeout_us / 1000), rxobs_result(irq), frame,
                  dropped ? " ignored (RadioLib dropped it, the window counts as empty)" : "");
    }
}

/** @brief The per-exchange parts of the "uplink OK:" line: @p freqs = the
 *         frame's own repeats' channels ("867.9/868.3/868.1"; "/..." past
 *         OBS_MAX_REP; "?" if no TX was seen), @p tx = "2/3" (sent / NbTrans in
 *         force) or "1", plus " +N MAC-only (from FCnt F)" for TX of a MAC-only
 *         uplink RadioLib sent inside the exchange. @p fcnt_wire = the frame's FCnt. */
static inline void rxobs_uplink_tx(const rxobs_t *o, int16_t st, uint32_t fcnt_wire, char *freqs, size_t fsz,
                                   char *tx, size_t tsz) {
    const rxobs_split_t s = rxobs_split(o, st);
    const int own = s.own_tx < OBS_MAX_REP ? s.own_tx : OBS_MAX_REP;
    int n = 0;
    if (fsz > 0) freqs[0] = '\0';
    for (int i = 0; i < own; i++) {
        rxobs_cat(freqs, fsz, &n, "%s%.1f", i ? "/" : "", rxobs_mhz(o->rep[i].tx_freq_raw));
    }
    if (s.own_tx > OBS_MAX_REP) rxobs_cat(freqs, fsz, &n, "/...");
    if (n == 0 && fsz > 0) snprintf(freqs, fsz, "?");
    n = 0;
    if (tsz > 0) tx[0] = '\0';
    if (o->nb_trans > 1) rxobs_cat(tx, tsz, &n, "%d/%u", s.own_tx, (unsigned)o->nb_trans);
    else                 rxobs_cat(tx, tsz, &n, "%d", s.own_tx);
    if (o->n_tx > s.own_tx) {
        rxobs_cat(tx, tsz, &n, " +%d MAC-only (from FCnt %lu)", (int)o->n_tx - s.own_tx,
                  (unsigned long)fcnt_wire + 1);
    }
}
