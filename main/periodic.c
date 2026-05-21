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

#include "psa/crypto.h"        // mbedtls_psa_crypto_free / psa_crypto_init
#include "mbedtls/ssl.h"       // pulls in supporting types for the PSA call

#include "log_ftp.h"           // log_ftp_note_psa_refreshed (resets FTP-side OOM counter)
#include "net_arp.h"           // net_arp_send_gratuitous
#include "transmission.h"      // tx_is_idle

static const char *TAG = "periodic";

void periodic_loop(uint32_t now_ms) {
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
