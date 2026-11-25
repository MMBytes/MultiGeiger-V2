#pragma once

/** @file
 *  @brief Upload send results that are not HTTP status codes, and how the
 *  round dispatcher (tx_dispatch_one(), transmission.c) classifies a result.
 *  Pure, free of FreeRTOS, so the host tests can cover it.
 *
 *  A send function returns the HTTP status, or one of:
 *   -1 = retries used up (no connection, connection lost, or a 5xx on
 *        every attempt),
 *   -2 = WiFi down,
 *   TX_RC_SKIPPED, TX_RC_NO_ANSWER (below).
 *  V2.8.35: the codes got names after TX_RC_NO_ANSWER was first given the
 *  value -3, which the "not configured" returns already used.
 */

#include <stdbool.h>

/** @brief Send result: not sent, the target is not configured (empty key,
 *  ID or box ID) or has nothing to send (ThingSpeak PM without a PM reading). */
#define TX_RC_SKIPPED        (-3)

/** @brief Send result (V2.8.35, openSenseMap only): the request was handed
 *  to the network but no answer came within the timeout
 *  (ESP_ERR_HTTP_EAGAIN, which esp_http_client returns only while waiting
 *  for the response). Not retried and not counted toward the breaker: the
 *  30-day check of 2026-11-25 found ~98 % of such attempts stored by
 *  openSenseMap anyway (a retry only stored them twice). Logged as
 *  "no answer, probably stored". */
#define TX_RC_NO_ANSWER      (-4)

/** @brief Does a send result count toward the target's circuit breaker?
 *  Only -1 does (retries used up: no connection, or a 5xx on every
 *  attempt); a returned HTTP status (4xx), WiFi down, skipped and
 *  no-answer results do not.
 *  @param rc  send result
 *  @param ok  true when rc is one of the target's success codes */
static inline bool tx_rc_counts_to_breaker(int rc, bool ok) {
    return !ok && rc == -1;
}

/** @brief Word for the round log line "<target>: <word> (rc=N)".
 *  @param rc  send result
 *  @param ok  true when rc is one of the target's success codes
 *  @return static string, never NULL */
static inline const char *tx_rc_word(int rc, bool ok) {
    if (ok) return "ok";
    if (rc == TX_RC_NO_ANSWER) return "no answer, probably stored";
    return "error";
}
