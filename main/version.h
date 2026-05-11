#pragma once
// Bump before build; commit after successful flash.
//
// V2.3.23 — closes out the V2.3.5 → V2.3.22 FTPS+TLS 1.3 arc + adds slow-
// fragmentation prevention. Two small changes:
//
//   1. **`ftp_tls12_only` default flipped from true to false.** TLS 1.3 is
//      now the FTPS default after V2.3.22's bidirectional-close fix landed
//      and was production-validated against the project's LAN FTPS server
//      (esp32-176432 → esp32-12276328:48807, ~22 successful uploads
//      observed across ~1 hour with no 426 events).
//
//      The /config "Limit FTPS to TLS 1.2" checkbox stays as a safety net
//      for any future server we haven't yet handled. Existing devices
//      upgrading from V2.3.22 keep their saved value via NVS — only fresh
//      / NVS-erased devices get the new default. Pre-V2.3.22 the default
//      was true because TLS 1.3 against this server got "426 Connection
//      reset" (now fixed by draining server's NewSessionTickets before TCP
//      close). See `reference_ftps_tls13_investigation.md` for the full
//      arc.
//
//   2. **Scheduled PSA crypto refresh every 24h.** Adds a third trigger
//      to the existing PSA-reset machinery (already fires on 5 consecutive
//      OOMs and on TCP write stalls). New trigger is purely time-based —
//      flushes any slow heap fragmentation that accumulates across hundreds
//      of TLS handshakes (HTTPS + FTPS combined).
//
//      Observed in production V2.3.22 1-hour soak: min_free dropped ~14 KB
//      across 22 uploads, in step-shaped chunks of 6-8 KB on specific
//      uploads (free_heap stayed flat — no leak, just fragmentation).
//      Likely stabilises but the 24h scheduled refresh is cheap insurance.
//
//      Implementation in `log_ftp.c::log_ftp_loop` BEFORE the
//      ftp_enabled early return, so it fires on every device regardless
//      of FTP being configured. Gating:
//        - `tx_is_idle()` — no HTTPS handshake mid-flight on TX worker
//          (CPU1). If busy, defer to next loop tick (1 s); eventually hits
//          an idle window between TX cycles (~10 s active out of every
//          150 s).
//        - FTPS not in progress: structurally guaranteed, since both
//          FTPS upload and this timer check run on the main task which is
//          single-threaded.
//
//      Cost: ~20-50 ms extra latency on the FIRST TLS handshake after the
//      reset (PSA key material re-derived on first use). Sensor / WiFi /
//      HTTP server / NVS / log ring all unaffected. Worst-case race
//      window if HTTPS handshake starts between `tx_is_idle()` check and
//      `mbedtls_psa_crypto_free()` call: one HTTPS retry. Same race
//      acceptance the existing reset triggers have used since V2.3.15.
//
//      24h interval is comfortably below any conceivable fragmentation
//      accumulation rate; if min_free ever shows linear-not-stabilising
//      drop on overnight soaks, drop to 12 h or add a heap-threshold
//      trigger alongside this time-based one.
//
// OTA-safe from V2.3.22 (no partition layout changes, no sdkconfig
// changes). 20 release artefacts (5 × 4 boards). All four boards share
// the FTPS code path and benefit from both changes.
#define VERSION_STR "V2.3.23"
