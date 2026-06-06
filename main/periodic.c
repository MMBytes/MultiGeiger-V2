/** @file
 *  @brief Implementation of `periodic_loop()`. See periodic.h for the
 *  full rationale and call-site documentation.
 *
 *  Extracted from `log_ftp.c::log_ftp_loop()` in V2.4.19 — the chores
 *  here are functionally unrelated to FTP and only co-habited that file
 *  for historical convenience. Behaviour is preserved exactly: same
 *  24h interval, same `tx_is_idle()` gate, same retry-on-busy semantics
 *  (no timestamp advance until the chore actually runs).
 */

#include "periodic.h"

#include "esp_log.h"
#include "esp_heap_caps.h"     // heap_caps_get_largest_free_block (V2.5.14 heap guard)

#include "psa/crypto.h"        // mbedtls_psa_crypto_free / psa_crypto_init
#include "mbedtls/ssl.h"       // pulls in supporting types for the PSA call

#include "log_ftp.h"           // log_ftp_note_psa_refreshed (resets FTP-side OOM counter)
#include "net_arp.h"           // net_arp_send_gratuitous
#include "transmission.h"      // tx_is_idle
#include "mqtt.h"              // mqtt_is_initialized / mqtt_stop (V2.4.31)
#include "main_status.h"       // main_request_restart (V2.5.14 heap guard)

static const char *TAG = "periodic";

// V2.5.14: threshold-gated reboot safety-net for unattended, long-uptime
// nodes. INTERNAL-DRAM fragmentation (free heap flat, but the *largest
// contiguous* block shrinking) is the OTA-stall precursor and a long-tail OOM
// risk on month+ uptimes. The 24h PSA refresh below slows but does NOT fully
// prevent it (esp32-5965048 soak: INTERNAL largest 71.7K -> 62.5K over 4.8
// days despite the refresh). A reboot is the only certain reset of
// fragmentation — but this fires ONLY when genuinely starved, never on a
// transient dip, and (critically) never in a boot-loop.
//
// Disabled entirely when floor_kb == 0 (the default). Called every tick from
// periodic_loop(); all gating is internal.
static void periodic_heap_guard(uint32_t now_ms, uint32_t floor_kb) {
    if (floor_kb == 0) { return; }            // feature off (default)

    // Boot-loop hardening tunables. The arm-delay is the load-bearing one:
    // the catastrophic failure of any auto-reboot is a loop on a node you
    // can't reach, not a missed reboot. A reboot is recoverable; a loop
    // bricks the deployment. Defaults from the 5965048 soak (resting largest
    // 62-71K, per-cycle low-water free 95K).
    const uint32_t MIN_UPTIME_MS     = 2UL * 60 * 60 * 1000;   // 2h arm-delay
    const uint32_t MIN_REBOOT_GAP_MS = 6UL * 60 * 60 * 1000;   // <=1 reboot / 6h
    const int      CONFIRM_SAMPLES   = 3;                      // consecutive idle hits

    static int      s_below = 0;
    static uint32_t s_last_reboot_ms = 0;     // 0 = none this boot

    // (1) ARM-DELAY — never act in the first 2h of uptime, so a node that
    //     boots already-fragmented (or with a mis-set floor) physically
    //     cannot reboot itself in a tight loop.
    if (now_ms < MIN_UPTIME_MS) { s_below = 0; return; }

    // (2) RATE-LIMIT — belt-and-suspenders: at most one guard-reboot per 6h
    //     even if (1) were somehow bypassed.
    if (s_last_reboot_ms != 0 &&
        (int32_t)(now_ms - s_last_reboot_ms) < (int32_t)MIN_REBOOT_GAP_MS) {
        return;
    }

    // (3) SAMPLE ONLY AT REST — a mid-flight TLS handshake transiently grabs a
    //     large contiguous block, so largest-free dips then recovers. Gating
    //     on tx_is_idle() measures the true resting floor and auto-filters
    //     those transients (no separate de-glitch timer needed).
    if (!tx_is_idle()) { return; }

    uint32_t largest = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    uint32_t floor_b = floor_kb * 1024u;
    if (largest >= floor_b) { s_below = 0; return; }   // healthy -> reset streak

    // (4) DEBOUNCE — require N consecutive idle samples below the floor before
    //     acting; one unlucky reading never triggers a reboot.
    if (++s_below < CONFIRM_SAMPLES) { return; }

    // Starved + confirmed -> clean reboot via the SAME path /reboot uses
    // (persists state, flushes applog, esp_restart from the main loop). The
    // loud WARN lands in /log + syslog + the post-reboot trail so the reason
    // is never a mystery.
    ESP_LOGW(TAG,
             "HEAP GUARD: INTERNAL largest=%u < floor=%u (%lukB) for %d idle "
             "samples, uptime=%us — rebooting to defragment",
             (unsigned)largest, (unsigned)floor_b, (unsigned long)floor_kb,
             s_below, (unsigned)(now_ms / 1000));
    s_last_reboot_ms = now_ms;
    main_request_restart();
}

void periodic_loop(uint32_t now_ms, uint32_t heap_guard_floor_kb) {
    // V2.5.14: heap-guard auto-reboot safety-net. Runs every tick (self-gated),
    // BEFORE the 24h early-return below — so it isn't starved by that gate.
    periodic_heap_guard(now_ms, heap_guard_floor_kb);

    // V2.3.23: scheduled PSA crypto refresh. Slow heap fragmentation
    // accumulates across hundreds of TLS handshakes (HTTPS + FTPS
    // combined) — observed in production V2.3.22 soak with min_free
    // dropping ~14 KB/h before stabilising. `mbedtls_psa_crypto_free()`
    // + `psa_crypto_init()` returns the entire PSA slot pool to empty,
    // giving the heap allocator a chance to recompact. Cost: ~20-50 ms
    // extra latency on the next TLS handshake (key material re-derived
    // on first use). Sensor / WiFi / HTTP server unaffected.
    //
    // Gating: `tx_is_idle()` ensures no HTTPS handshake is mid-flight
    // on the worker (CPU1) — calling `psa_crypto_free` during one
    // would corrupt its state. FTPS is structurally safe: this code
    // runs on the main task, and FTPS upload (`do_ftp_upload`) also
    // runs on the main task — they can't overlap because the main
    // task is single-threaded. If the worker is busy when we'd fire,
    // we just defer to the next tick (1 s) and eventually hit an idle
    // window between cycles (TX cycle is ~10 s active out of every
    // 150 s).
    //
    // 24h interval is comfortably below any conceivable fragmentation
    // accumulation rate; if min_free ever shows linear-not-stabilising
    // drop on overnight soaks, drop to 12 h or add a heap-threshold
    // trigger alongside this time-based one.
    //
    // V2.4.19: piggy-back a gratuitous ARP on the same 24h schedule —
    // safety-net for sensors that hold a single WiFi association for
    // days without reconnecting. The per-reconnect ARP in `main.c`
    // handles the common case; this one catches the never-reconnects
    // edge. See `[[reference_mqtt_one_way_loss_after_wifi_roam]]`.
    static uint32_t s_last_refresh_ms = 0;
    const uint32_t  REFRESH_INTERVAL_MS = 24UL * 60 * 60 * 1000;   // 24h

    if ((int32_t)(now_ms - s_last_refresh_ms) < (int32_t)REFRESH_INTERVAL_MS) {
        return;
    }
    if (!tx_is_idle()) {
        // TX worker busy — try again on next loop tick (1 s). No
        // timestamp update so we keep retrying until idle.
        return;
    }

    // V2.4.31: stop the persistent MQTT client BEFORE freeing PSA.
    //
    // The tx_is_idle() gate above only covers the *transient* HTTPS/FTPS
    // handshakes on the worker. The MQTT client (V2.4.2+) holds a
    // *persistent* TLS session whose AES-GCM record keys live in PSA key
    // slots — `mbedtls_psa_crypto_free()` invalidates them, so the next
    // MQTT TLS write (a keepalive ping, or the FTP-prep DISCONNECT) fails
    // `-0x0084` and the live connection breaks. Symptom before this fix
    // (diagnosed 2026-05-30 from three serial logs, esp32-5963724 +
    // 5965048): every 24h the refresh was immediately followed by
    // `mqtt_client: Error sending ping` / `DISCONNECTED`, then a ~4.5 s
    // blocking `mqtt_stop()`. See [[reference_mqtt_ha_integration]].
    //
    // Stopping MQTT first sends a clean DISCONNECT over still-valid crypto;
    // main.c's "STA has IP + clock sane → starting MQTT client" poll
    // re-inits it on the next tick (the same path FTP relies on). FTP-prep's
    // own `mqtt_is_initialized()` guard then sees it already down and skips
    // its stop, so it's a single clean bounce. Bonus: this actually releases
    // MQTT's PSA slots, so the free now empties the pool — previously the
    // live session held its slots across the refresh, partly defeating the
    // defrag this whole chore exists for.
    if (mqtt_is_initialized()) {
        ESP_LOGI(TAG, "PSA refresh: stopping MQTT first (its TLS keys live in PSA slots)");
        mqtt_stop();
    }

    ESP_LOGI(TAG, "PSA crypto subsystem refresh (24h scheduled)");
    mbedtls_psa_crypto_free();
    psa_status_t ps = psa_crypto_init();
    if (ps == PSA_SUCCESS) {
        ESP_LOGI(TAG, "psa_crypto_init: ok");
        // Reset the FTP-side consecutive-OOM streak counter — a healthy
        // 24h refresh "blesses" any in-flight stretch of post-OOM
        // recoveries. Implementation lives in log_ftp.c because the
        // counter does too.
        log_ftp_note_psa_refreshed();
    } else {
        ESP_LOGE(TAG, "psa_crypto_init failed: %d (PSA in unknown state)",
                 (int)ps);
    }

    net_arp_send_gratuitous();

    s_last_refresh_ms = now_ms;
}
