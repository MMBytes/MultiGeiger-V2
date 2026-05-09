#pragma once
// Bump before build; commit after successful flash.
// V2.3.10 — FTPS TLS 1.3 regression fix + diagnostic logging.
//   - V2.3.5 enabled TLS 1.3 globally for HTTPS speedup. Side effect: FTPS
//     uploads to a local LAN server (vsftpd / proftpd / pure-ftpd or similar
//     hosted on a router or NAS at 192.168.123.1 in our case) started
//     failing with "USER reject: -1" — TLS handshake completed, USER command
//     was sent, but the very first read on the secured stream returned -1.
//     Many embedded FTPS daemons advertise willingness to negotiate TLS 1.3
//     but mishandle the post-handshake protocol (NewSessionTicket arriving
//     unexpectedly, removal of renegotiation), and close the connection
//     immediately after the client sends its first application data.
//   - Fix: cap the FTPS mbedTLS config at TLS 1.2 max via
//     `mbedtls_ssl_conf_max_tls_version(&t->conf, MBEDTLS_SSL_VERSION_TLS1_2)`
//     in `tls_ctx_init`. HTTPS targets are unaffected — they go through
//     esp_tls (separate config path) and their cloud terminators
//     (nginx / Cloudflare / aws-elb) implement TLS 1.3 correctly. FTPS
//     gets no benefit from 1.3 anyway: one local-LAN handshake per hour,
//     1-RTT saving is sub-millisecond.
//   - Diagnostic 1: log the negotiated TLS version + ciphersuite after each
//     successful FTPS handshake (`mbedtls_ssl_get_version` /
//     `mbedtls_ssl_get_ciphersuite`). Confirms what was selected and gives
//     a starting point for future server-side compatibility issues.
//   - Diagnostic 2: surface the precise mbedTLS error code (via
//     `mbedtls_strerror`) inside `io_recv1` and `io_send_all`'s TLS paths.
//     Previously the wrapper collapsed everything to "-1" — the user could
//     see "USER reject: -1" but not WHY. Now they see the actual mbedTLS
//     error name (e.g. MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY,
//     MBEDTLS_ERR_NET_RECV_FAILED, etc.).
//   - No source change to non-FTPS code; HTTPS POST behaviour unchanged from
//     V2.3.9.
#define VERSION_STR "V2.3.10"
