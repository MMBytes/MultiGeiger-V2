#pragma once

/** @file
 *  @brief openSenseMap upload timing (V2.8.35): timeout per attempt, time
 *  budget per upload, and when another attempt may start. Pure, free of
 *  FreeRTOS, so the host tests can cover the boundary.
 *
 *  Used by send_osm_to() (transmission.c) for both openSenseMap targets
 *  (production and staging), each with its own budget. The other upload
 *  targets keep HTTP_TIMEOUT_MS x HTTP_MAX_RETRIES (15 s x 4).
 *
 *  Why 25 s: openSenseMap's production server often answers slowly. With a
 *  30 s test timeout on a production node, 25 of 628 first attempts
 *  answered after 15 s (17 after 20 s, only 1 after 25 s); with the old
 *  15 s timeout those were timeouts.
 *  A timeout is not retried (TX_RC_NO_ANSWER, tx_rc.h): comparing 30 days
 *  of that node's rounds with the values the box stored (2026-11-25, via
 *  the public read API) showed that openSenseMap stored ~98 % of the attempts the node had given up on, so every retry
 *  after a timeout only stored the data a second time. 502 and connection
 *  failures store nothing; those are retried within the budget.
 *  timeout_ms is a silence limit per phase (connect, TLS handshake, each
 *  read), so a slow connect failure can itself take ~25 s; the budget then
 *  stops after 3 such attempts (26 + 2 + 26 + 2 + 26 = 82 s, a fourth would
 *  need 82 + 2 + 25 > 90). Fast failures (502, refused connection, a few
 *  seconds each) get all 4 attempts.
 *  90 s is the normal maximum, not a hard cap: an attempt whose connect and
 *  TLS handshake are both slow but succeed can run past it, since each
 *  phase has its own 25 s.
 *  OSM runs last in TX_TABLE, so a long upload only delays the end of the
 *  round; with staging AND production both unreachable a round can take
 *  ~180 s, longer than the 150 s default interval. The next round is then
 *  not dropped: tx_task() takes a round out of the queue before running
 *  it, so the next one waits in the queue and starts late. Until the
 *  breakers trip (3 rounds) the worker is never idle, so FTPS, the PSA
 *  refresh and LoRaWAN exchanges (tx_is_idle()) wait their turn.
 */

#include <stdbool.h>
#include <stdint.h>

#define OSM_TIMEOUT_MS      25000   ///< esp_http_client timeout_ms per attempt
#define OSM_RETRY_PAUSE_MS   2000   ///< pause before the next attempt
#define OSM_BUDGET_MS       90000   ///< time budget for one upload, all attempts

/** @brief May another attempt start after @p used_ms of this upload?
 *         Only if the pause plus a full timeout still fit in the budget
 *         (63 000 ms used -> yes, ends at 90 s; 63 001 -> no). */
static inline bool osm_may_retry(int64_t used_ms) {
    return used_ms + OSM_RETRY_PAUSE_MS + OSM_TIMEOUT_MS <= OSM_BUDGET_MS;
}
