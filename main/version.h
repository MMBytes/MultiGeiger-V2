#pragma once
// Bump before build; commit after successful flash.
//
// V2.3.16 — five small wins, one experiment reverted.
//
//   1. **Streaming FTPS upload (zero-copy via applog_stream API).** Replaces
//      applog_snapshot's malloc-the-entire-ring pattern with a non-allocating
//      iterator that returns segment pointers directly into ring memory.
//      Body is sent in 4 KB chunks straight from the ring; no body buffer
//      malloc. Saves ~60 KB internal DRAM during the upload window on the
//      Heltec (which had crawled to min_free=276 bytes at full ring under
//      V2.3.15) and ~1 MB PSRAM on FeatherS3-D. The /log HTTP endpoint
//      continues to use applog_snapshot — it needs a Content-Length-friendly
//      contiguous response.
//
//   2. **4 MB Heltec knock-off partition layout fix.** V2.3.15 4 MB layout
//      was factory + ota_0 (1.875 MB each). That worked for the FIRST OTA
//      (factory → ota_0) but failed on the SECOND OTA with
//      ESP_ERR_OTA_PARTITION_CONFLICT — only one OTA slot to write to, and
//      we were already running from it. New layout: ota_0 + ota_1 (no
//      factory), 1.875 MB each. Bootloader handles the no-factory case by
//      defaulting to ota_0 when otadata is blank; subsequent OTAs cycle
//      ota_0 ↔ ota_1 indefinitely. UPGRADE NOTE: changing the partition
//      table requires a CABLE REFLASH (esp_ota only updates the app, not
//      the partition table itself).
//
//   3. **4MB / 8MB Heltec distinction.** New BOARD_HELTEC_V2_4MB compile-
//      time macro (set alongside BOARD_HELTEC_V2 by CMakeLists.txt for the
//      4 MB build). hal.h's BOARD_NAME splits to "heltec_v2_4mb" / "heltec_v2"
//      / "feathers3_d". The /update OTA upload form prompt explicitly reads
//      "Select a firmware .bin for Heltec WiFi Kit 32 (4MB)" or "(8MB)" so
//      the user picks the right binary up front. Pairs with the V2.3.13
//      chip-ID validation: form steers, V2.3.13 catches mistakes.
//
//   4. **"BOARD: <name>" boot-log line** in app_main right after the
//      version-string log. Makes serial-log forensics trivially unambiguous
//      about which build is running.
//
//   5. **BMP390 calibration typo fix.** par_T1 divisor was `2.52e-2` (=0.0252)
//      instead of `2^-8` (=0.00390625, equivalent to *256). Latent typo since
//      V2.3.6 when the driver was written — surfaced 2026-05-10 the first
//      time a BMP390 was actually wired in production (dust node FeatherS3).
//      Symptom: pressure reported ~25 % high (1287 hPa vs actual 1024 hPa).
//      Mechanism: par_T1 ~6.5× too small → BMP390-internal t_lin wildly off
//      (~120 °C instead of ~18 °C) → temperature-correction terms in the
//      pressure compensation polynomial multiply out to a +263 hPa offset.
//      Cross-checked all other 13 calibration constants in load_calibration
//      against the Bosch SDK and Adafruit BMP3xx library — only par_T1 had
//      the typo.
//
//   6. **FTP host:port parsing.** The `ftp_host` config field now accepts an
//      optional `:port` suffix (e.g. `192.168.1.1:2121` for an FTP daemon on
//      a non-default port). When absent or malformed, the connection uses
//      the default FTP control port 21. Parser lives in
//      `do_ftp_upload` (~15 LOC) and only affects the control-channel
//      connect; the data channel still uses the port the server returns
//      from PASV. The /config form label updated to hint at the syntax.
//
//   7. **openSenseMap access token field** (`osm_access_token`, NVS key
//      `osm_tok`, default empty). Mirrors the existing `aqi_token` pattern.
//      OSM recently added an opt-in "require authentication" toggle per box
//      on their dashboard — when enabled, unauthenticated POSTs to the
//      Luftdaten endpoint are rejected. Setting this token (generated under
//      "Edit Box" → "Access Token") and leaving the box-side toggle on lets
//      uploads continue. send_osm passes the token as `Authorization:
//      Bearer <token>` when non-empty; uploads stay unauthenticated when
//      empty (backward compat with existing boxes).
//
// REVERTED experiment — long-lived HTTPS handles with TLS session resumption.
//
// V2.3.16-pre2 introduced file-static cached_client_t per target with
// `save_client_session=true`, hoping to eliminate the per-cycle PSA leak by
// holding handles across cycles and resuming via the saved session ticket.
// Bench-tested 2026-05-10 on the dust node:
//
//   - Cycle 1: all 5 targets fresh handshake → cached → OK
//   - Cycles 2..N: every target's first POST attempt fails with
//     "Connection reset by peer" (server closed keep-alive after ~60-120 s
//     vs our 150 s cycle interval). Retry then triggers a fresh handshake.
//   - The retry log shows "esp-x509-crt-bundle: Certificate validated" —
//     fresh ECDHE handshake, NOT a TLS-session-resumed one. Either
//     esp_http_client invalidates the saved ticket on connection-reset
//     error, or the server rejects the resumption ticket and falls back to
//     fresh. Either way: PSA pressure unchanged from V2.3.15.
//   - Net effect: cycle TX time bloated 10 s → 28 s (2.8× slower) due to
//     wasted attempts + 2 s retry delays per failed target, with zero PSA
//     savings. Empirically worse than per-cycle init+cleanup.
//
// Reverted before V2.3.16 ship. Per-cycle init+cleanup is the stable
// baseline. PSA leak mitigation continues via V2.3.15's defensive
// infrastructure (slot bump 32→128, nuclear PSA reset on consecutive OOMs
// AND on FTPS write stalls). Real fix to the per-handshake PSA leak would
// need to be in mbedTLS itself, or via TLS session resumption that actually
// works (would need investigation into why the saved ticket isn't being
// used on reconnect — out of scope for V2.3.16).
//
// What stays from V2.3.16 attempts: items 1-5 above. What goes: the
// long-lived HTTPS handle cache and the "N cached clients holding ~M KB"
// per-cycle log diagnostic.
#define VERSION_STR "V2.3.16"
