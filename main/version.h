#pragma once
// Bump before build; commit after successful flash.
//
// V2.3.19 — bug fix release. The V2.3.15-deferred FTPS+TLS 1.3 "426
// Connection reset by peer" failure on the data channel is fixed by
// properly flushing the TLS close_notify alert before tearing down
// the socket.
//
// Root cause: `mbedtls_ssl_close_notify()` is non-blocking and frequently
// returns `MBEDTLS_ERR_SSL_WANT_WRITE` right after a large body upload
// while the lwIP TCP send buffer drains. Pre-V2.3.19 `io_close()` ignored
// the return value and immediately closed the TCP socket, so the
// close_notify alert never made it onto the wire.
//
// RFC 8446 §6.1: TLS 1.3 servers MUST treat TCP close without close_notify
// as a possible truncation attack. The project's LAN FTPS server reacted
// with "426 Connection reset by peer" on the data channel after the body
// otherwise transferred cleanly. TLS 1.2 servers are lenient about this
// for backwards compatibility with old broken clients, so the bug only
// manifested once we re-enabled TLS 1.3 in V2.3.5.
//
// Diagnosis came from comparing FileZilla (which loops close_notify until
// the alert is actually flushed — standard mature-TLS-client behaviour)
// against our fire-and-forget call. The cipher (AES-128-GCM) and key
// exchange (ECDHE-secp384r1-RSA-PSS-RSAE-SHA256) FileZilla negotiated
// against the same server were both well within mbedTLS's defaults, ruling
// out crypto-suite mismatch.
//
// Fix in `log_ftp.c::io_close`: deadline-bounded retry loop using the
// same WANT_READ / WANT_WRITE continuation pattern we already use in
// `io_send_all` / `io_recv1`. 2 s deadline keeps us from blocking
// forever on a genuinely stalled socket; in normal operation the loop
// fires once or twice and exits in <50 ms. ~15 LOC.
//
// HTTPS targets (Madavi / SC / Radmon / OSM / aqi.eco) own their own
// close path inside esp_http_client; this fix doesn't touch them. They
// have not exhibited analogous issues — the cloud terminators are more
// permissive than the LAN FTPS server about truncation.
//
// Now that the fix is in place, the V2.3.15 `ftp_tls12_only` checkbox
// (default ON) can be unticked to use TLS 1.3 against the LAN FTPS
// server. Default stays ON for one release as a safety net; if V2.3.19
// soaks cleanly we can flip the default to OFF in V2.3.20.
//
// OTA-safe from V2.3.18 (no partition layout changes, no sdkconfig
// changes). 15 release artefacts (5 × 3 boards).
#define VERSION_STR "V2.3.19"
