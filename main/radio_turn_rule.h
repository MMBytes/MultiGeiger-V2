#pragma once

/** @file
 *  @brief When may a radio transfer start (V2.8.15)? Pure functions, free of
 *  FreeRTOS, so the host tests can cover them.
 *
 *  Nodes that use WiFi uploads AND LoRaWAN keep the two from transmitting at
 *  the same time. The rule reuses the busy flags that already exist:
 *    - tx_is_idle()       (transmission.c) — no WiFi upload round running
 *                         or queued; already gates FTPS, the 24 h PSA
 *                         refresh, the gratuitous ARP and deferred restarts;
 *    - lorawan_on_air()   (lorawan.cpp, s_busy) — a LoRaWAN exchange (join
 *                         attempt, or uplink with its receive windows) is
 *                         running; the flag the OTA suspension waits on;
 *    - log_ftp_is_busy()  (log_ftp.c) — an FTPS log upload is running.
 *  Each side publishes its own flag first and then checks the other's
 *  (seq-cst, the handshake lorawan.cpp already uses against the OTA
 *  suspension), so two transfers can never both decide to go.
 *
 *  Who yields: a LoRaWAN exchange and an FTPS upload back off when the other
 *  side is busy (clear their flag, retry later); a WiFi upload round, already
 *  marked busy, waits for a running exchange to end. A round therefore never
 *  waits more than one poll for an exchange that is itself waiting for the
 *  round (the exchange's brief publish-check-clear window) — no deadlock.
 *  A round with no upload target enabled doesn't wait at all.
 *
 *  Why at all: on the XIAO + Wio bench node (2026-10-21) four LoRaWAN
 *  exchanges failed with RADIOLIB_ERR_TX_TIMEOUT (-5), each in the same
 *  second as a Madavi HTTPS upload started; none in the days before. Both
 *  workers run on CPU1 at the same priority; whether CPU contention or RF /
 *  power coupling is the cause is not proven, so the overlap itself goes.
 *
 *  FTPS and a WiFi upload round never overlap anyway: FTPS runs on the main
 *  task only when tx_is_idle(), and no new round starts while it runs (the
 *  rounds are started by the same main task).
 */

#include <stdbool.h>

/** @brief A LoRaWAN exchange may go: no WiFi upload round running or
 *         queued, and no FTPS upload running. */
static inline bool rt_lorawan_may_start(bool tx_idle, bool ftps_busy) {
    return tx_idle && !ftps_busy;
}

/** @brief An FTPS upload may go: the existing tx_is_idle() rule, plus no
 *         LoRaWAN exchange on air. */
static inline bool rt_ftps_may_start(bool tx_idle, bool on_air) {
    return tx_idle && !on_air;
}

/** @brief A WiFi upload round (already marked busy) may run: no LoRaWAN
 *         exchange on air. */
static inline bool rt_tx_round_may_start(bool on_air) {
    return !on_air;
}
