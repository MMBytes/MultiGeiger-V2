#pragma once
// Bump before build; commit after successful flash.
//
// V2.3.22 — bug fix release. Closes out the V2.3.5-introduced FTPS+TLS 1.3
// "426 Connection reset by peer" regression for good. V2.3.19 fixed the
// first half (close_notify wasn't being flushed); V2.3.22 fixes the second
// half (server's NewSessionTickets weren't being drained, causing kernel
// to send TCP RST instead of FIN at close).
//
// Diagnosis path (V2.3.22-pre1 → pre3): added per-iteration logging to
// `io_close()` after pre2 confirmed the V2.3.19 close_notify was sending
// successfully but the server STILL saw "Connection reset". The pre3
// iteration trace showed the smoking gun:
//
//   drain iter=1 rc=WANT_READ
//   drain iter=2 rc=NEW_SESSION_TICKET   ← server sent ticket #1
//   drain iter=3 rc=WANT_READ
//   drain iter=4 rc=NEW_SESSION_TICKET   ← server sent ticket #2
//   drain iter=5 rc=PEER_CLOSE_NOTIFY    ← clean bidirectional close
//   shutdown(SHUT_WR) rc=0
//   close() rc=0
//   FTPS upload OK
//
// Pre-V2.3.22 we'd close the TCP socket immediately after our close_notify
// went out — leaving the two NewSessionTickets unread in the kernel
// receive buffer. Linux/lwIP TCP semantics: close() with unread data sends
// RST instead of FIN. The server saw RST, reported "Connection reset by
// peer" → 426. TLS 1.2 doesn't send post-handshake NewSessionTickets like
// 1.3 does, so 1.2 worked all along.
//
// Three changes in `main/log_ftp.c::io_close`:
//
//   1. **Bidirectional close drain loop** — after our close_notify is
//      flushed, loop on `mbedtls_ssl_read()` until we get either
//      MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY (clean), benign error, or 1 s
//      deadline. Continues on MBEDTLS_ERR_SSL_RECEIVED_NEW_SESSION_TICKET
//      (the V2.3.15 control-flow signal). Same pattern OpenSSL's
//      SSL_shutdown() uses internally — what FileZilla and every mature
//      TLS client does.
//
//   2. **Explicit shutdown(SHUT_WR) before close()** — belt-and-braces.
//      Tells lwIP "send FIN, queue remaining data, then close" — kernel
//      cannot send RST when SHUT_WR is requested. Standard TCP half-close
//      pattern.
//
//   3. **One concise summary log line** per close: `TLS shutdown: drain
//      iters=N tickets=M clean (PEER_CLOSE_NOTIFY)` or `no
//      PEER_CLOSE_NOTIFY` if the drain timed out. Replaces the verbose
//      pre1/pre2/pre3 per-iteration logging — keeps observability without
//      cluttering /log.
//
// Plus: **main task stack 8 KB → 16 KB** (sdkconfig.defaults). FTPS runs
// on the main task; the TLS 1.3 handshake + post-handshake state is
// deeper than TLS 1.2 was. The original 8 KB was sized for the V1-era
// stack profile and proved insufficient under the pre1/pre2 debug-callback
// load. Even with mbedTLS protocol debug removed in V2.3.22, the bigger
// stack is good insurance against future feature growth on the main task.
//
// Plus: **TX worker stack 10 KB → 16 KB** (`transmission.c`). HTTPS uses
// the worker task; same TLS-1.3-deeper rationale. Cost: +6 KB DRAM per
// task, irrelevant against the dozens of free KB on Heltec genuine and
// the megabytes on PSRAM boards.
//
// `ftp_tls12_only` checkbox stays as the safety net (default ON for
// existing devices upgrading from V2.3.21, where it was on). After
// V2.3.22 soaks cleanly with the checkbox unticked, V2.3.23 can flip the
// default to OFF and finally close out the V2.3.5 TLS 1.3 regression arc.
//
// HTTPS targets unaffected — esp_http_client owns its own close path,
// which has presumably been doing bidirectional close all along (no
// reports of HTTPS truncation issues across the same TLS 1.3 timeline).
//
// OTA-safe from V2.3.21 (no partition layout changes). 20 release
// artefacts (5 × 4 boards). Heltec genuine + 4 MB knock-off + FeatherS3-D
// + QT Py all share the FTPS code path, all benefit from the fix.
#define VERSION_STR "V2.3.22"
