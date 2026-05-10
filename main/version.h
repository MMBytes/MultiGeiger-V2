#pragma once
// Bump before build; commit after successful flash.
// V2.3.15 — FTPS PSA-OOM fix (slot pool exhaustion under HTTPS+FTPS contention).
//
// Symptom (V2.2.0 ran 63 h clean; V2.3.x failed within hours after OTA):
//   - FTPS uploads to LAN target failing with "ssl_setup: -0x008d" or
//     "TLS handshake: -0x008d UNKNOWN ERROR CODE (0080) : UNKNOWN ERROR CODE
//     (000D)" — sometimes immediately, sometimes after 9-11 KB of body
//     transmitted. ~3 successful uploads in 19 h. Heltec sensor only.
//
// Diagnosis:
//   -141 = -0x008D = PSA_ERROR_INSUFFICIENT_MEMORY (psa/crypto_values.h).
//   PSA error codes are a separate namespace from mbedTLS's own — that's why
//   mbedtls_strerror returned "UNKNOWN ERROR CODE" (couldn't decode either
//   half). In mbedtls 4.x (ESP-IDF 6.0), legacy crypto APIs (mbedtls_aes_*,
//   mbedtls_md_*, ECDHE key derivation, AES-GCM init) are wrappers over PSA.
//   PSA maintains a fixed-size global slot table (default 32). Every TLS
//   handshake transiently consumes ~5-10 slots. Our per-cycle load is
//   5 HTTPS targets (Madavi / SC / Radmon / OSM / aqi.eco) + FTPS (ctrl +
//   data) — peaking at ~30+ active slots. Default 32 sat right at the edge,
//   and contention between the HTTPS worker (CPU1) and the FTPS path (main)
//   tipped it over once V2.3.5 enabled TLS 1.3 in the build (more PSA paths
//   compiled in even though we only negotiate 1.2 for FTPS).
//
// Four-part fix:
//   1. sdkconfig.defaults: CONFIG_MBEDTLS_PSA_KEY_SLOT_COUNT 32 → 128.
//      4× headroom; ~+9 KB DRAM. First and most likely sufficient defence.
//   2. log_ftp.c: decode -141 explicitly as PSA OOM in error logs (no more
//      "UNKNOWN ERROR CODE" mystery).
//   3. log_ftp.c: belt-and-braces mbedtls_ssl_session_reset before _free in
//      io_close + handshake-error paths. Forces PSA-backend resource release.
//      Also: free the saved ctrl_session blob immediately after the data-
//      channel handshake completes (previously held until end of upload —
//      ~30 s of unnecessary PSA-slot occupancy per upload).
//   4. log_ftp.c: nuclear PSA reset path. Tracks consecutive uploads that
//      hit -141; after 5 in a row (~15 min of sustained failure), if the
//      HTTPS worker is idle, calls mbedtls_psa_crypto_free + psa_crypto_init
//      to wipe and re-initialise the entire PSA subsystem. Recovers from
//      slot leaks regardless of root cause. Sub-microsecond race vs the
//      worker's tx_busy gate is accepted (worst case: one HTTPS retry).
//
// Also in V2.3.15: TLS 1.2 cap exposed as /config checkbox "Limit FTPS to
// TLS 1.2" (NVS key ftp_t12only). V2.3.15 bench testing — see item 9 — found
// TLS 1.3 still fails on this project's specific LAN FTPS server even after
// fixing the original V2.3.5 regression cause. The cap defaults to TRUE
// (TLS 1.2) so FTPS works out of the box; users with TLS 1.3-capable FTPS
// servers can untick to opt in. HTTPS targets unaffected (always free to
// negotiate 1.3 via esp_tls).
//
// Also in V2.3.15 (second-pass fixes after first bench observation):
//   5. CONFIG_LWIP_TCP_SND_BUF_DEFAULT 5744 → 16384 (sdkconfig.defaults). The
//      first V2.3.15 bench run on the Heltec showed FTPS stalling at exactly
//      5120 bytes (5 × FTP_WRITE_CHUNK at 1024) because the lwIP send buffer
//      defaults to 4 × MSS = 5744 B. Once the buffer fills, the next send()
//      blocks for ACKs; under transient WiFi pressure, our 15 s wall-clock
//      deadline fires and we abandon the upload. Bigger buffer = far lower
//      stall probability. ~+10 KB DRAM per active TCP socket (one at a time).
//   6. FTP_WRITE_CHUNK 1024 → 4096 (log_ftp.c). 4× larger TLS records, better
//      payload-to-overhead ratio, 4× fewer mbedtls_ssl_write calls per
//      upload (each one a PSA pressure point because mbedTLS hands AES-GCM
//      record encryption to PSA). Pairs with the bigger TCP send buffer.
//   7. Preemptive PSA reset on write stall (log_ftp.c). The bench showed that
//      a write stall reliably leaks PSA crypto state — subsequent retries
//      then fail with -141 at the data-channel handshake until the (a)
//      consecutive-OOM threshold (5) eventually fires the reset, wasting
//      4 retries × 3 min = 12 min. The new s_stall_observed flag triggers
//      the same nuclear PSA reset on the SAME upload that stalled, so the
//      next attempt is clean. Both paths still tx_is_idle gated.
//   8. Per-upload heap drift diagnostic (log_ftp.c). Logs free heap, lifetime
//      minimum, and largest contiguous block before AND after each upload.
//      PSA in mbedTLS 4.x stores key material on the heap (hybrid keystore
//      — slot table is static, key material is malloc'd), so a slow PSA
//      leak shows up as monotonically decreasing min_free over hours.
//   9. **The fix for the V2.3.5–V2.3.9 "USER reject: -1" regression.**
//      Item 7's diagnostic exposed -0x7B00 = MBEDTLS_ERR_SSL_RECEIVED_NEW_
//      SESSION_TICKET coming out of mbedtls_ssl_read on the very first
//      TLS 1.3 attempt of the second-pass build. This is NOT an error — it's
//      a control-flow signal mbedTLS 4.x raises when a TLS 1.3 server sends
//      the post-handshake NewSessionTicket message, inviting the application
//      to optionally save the session for resumption. Our io_recv1 didn't
//      recognise the signal and returned -1 — exactly the "USER reject: -1"
//      symptom. Fix: io_recv1 (and defensively io_send_all) treat -0x7B00
//      like WANT_READ — continue the loop. mbedTLS has already processed
//      the ticket; calling ssl_read again returns the actual application
//      data. Future TLS 1.3 session-resumption work would harvest the
//      ticket here via mbedtls_ssl_get_session() instead of discarding.
//
//      THIS WAS THE V2.3.5–V2.3.9 BUG. Three independent bugs surfaced
//      across this investigation, not one:
//        Bug A — Unhandled NewSessionTicket → "USER reject: -1" (FIXED, this
//                item). Was the V2.3.5 regression cause.
//        Bug B — PSA crypto pool exhaustion under load and after write
//                stalls (FIXED via items 1, 5, 7, 11, 13).
//        Bug C — TLS 1.3 against this project's specific LAN FTPS server
//                still fails with "426 Data Connection: Connection reset"
//                on the data channel even with bug A fixed. PARKED. Verified
//                independent of session reuse (bench-tested 2026-05-10).
//                Likely server doesn't fully implement TLS 1.3 on the data
//                channel, or its require_ssl_reuse=YES policy needs the
//                modern TLS 1.3 PSK-extension resumption path that
//                mbedtls_ssl_set_session() doesn't provide. TLS 1.2 +
//                session reuse works flawlessly. See
//                reference_ftps_tls13_investigation.md memory for the full
//                failure matrix and Options A-D for future investigation.
//
//      Because bug C remains, V2.3.15 ships with the TLS 1.2 cap default-on
//      (DEF_FTP_TLS12_ONLY = true). The new /config checkbox lets users
//      with TLS 1.3-capable FTPS servers opt back into 1.3.
//
//  10. **SPS30 → sensor.community PIN 1 field naming corrected.** V2.3.2
//      shipped with `SPS30_`-prefixed fields in the SC PIN 1 body
//      (`SPS30_P0`, `SPS30_P2`, ...). I'd written a confident comment
//      claiming the server distinguished PM sensor types by field-name
//      prefix on PIN 1 — pure fabrication, never verified. Server actually
//      returned HTTP 400 on every PM POST since V2.3.2 (silent because no
//      device had SPS30 wired with `pm_valid=true` until the dust node
//      10.11.12.72 in V2.3.12+ deployment). Surfaced as `pm rc=400` in
//      bench logs 2026-05-10. Fix: build_sensorc_pm_body emits the
//      canonical unprefixed Luftdaten names (`P0`/`P2`/`P4`/`P1`/`N05`/
//      `N1`/`N25`/`N4`/`N10`/`TS`) — same schema every PM sensor uses
//      regardless of physical type. The server uses the X-Sensor header
//      (chip ID) to track which device sent which value. Madavi / OSM /
//      aqi.eco builders unchanged — they DO use the `SPS30_` prefix
//      because their routing semantics are different. Verified by reading
//      airrohr-firmware sendSensorCommunity which strips the
//      `<SENSOR>_` prefix before posting (line 6160 in current master).
//      Same class of mistake as bug 9's PIN 12 (rationalised plausible
//      story without checking the source). Both lessons logged in
//      `feedback_trust_authoritative_source_over_memory.md` (round 1 +
//      round 2).
//
// Deferred to V2.3.16: TLS session resumption for the HTTPS path. Will hold
// long-lived esp_http_client handles per target with save_client_session=true,
// so subsequent cycles resume the saved TLS session ticket (1-RTT) instead of
// fresh ECDHE handshakes (2-RTT, ~5x more PSA slots). Bigger refactor of
// transmission.c; ship after V2.3.15 confirms the FTPS path is stable.
#define VERSION_STR "V2.3.15"
