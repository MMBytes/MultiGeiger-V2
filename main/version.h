#pragma once
// Bump before build; commit after successful flash.
//
// V2.3.17 — patch release on top of V2.3.16. Two small but high-value items:
//
//   1. **`/log` HTTP endpoint converted to streaming.** The endpoint was using
//      `applog_snapshot()` which mallocs the entire 60 KB ring buffer to
//      satisfy a Content-Length-based response. Replaced with the same
//      `applog_stream_begin/_end` API V2.3.16 added for FTPS, plus chunked
//      Transfer-Encoding via `httpd_resp_send_chunk()`. Body buffer alloc
//      drops from 60 KB to 0 KB.
//
//      Why this mattered: the V2.3.16 FTP_Investigation overnight showed
//      `min_free` dipping to 316 bytes on the Heltec — extremely close to
//      OOM. Tracing the log revealed the cause: every browser hit on /log
//      caused a 60 KB transient peak, and when that overlapped with
//      concurrent heap-heavy events (Radmon retry storm allocating fresh
//      mbedTLS contexts, /reboot parser errors allocating error buffers,
//      FTPS pre-upload setup), peak demand stacked to ~95 KB. Starting from
//      ~125 KB free, the system got dangerously close to allocation
//      failure. With /log now streaming, the body-alloc contribution to
//      that stack is zero — the user can browse /log freely without
//      pressing on free heap.
//
//      Side benefit: same change pattern applies if any other endpoint ever
//      needs to stream a large in-memory blob — applog_stream is a clean
//      reusable iterator API.
//
//   2. **`tx_run` heap log line now includes `min_free`.** Pre-V2.3.17 the
//      per-cycle log only showed current `free` and `max_alloc`. Adding
//      `min_free` makes the lifetime-watermark visible at every TX cycle,
//      not just inside the FTPS pre/post-upload lines. Lets us correlate
//      transient peaks with whatever was happening across the codebase
//      (not just FTPS-specific moments).
//
// Bench validation needed (V2.3.17 ship validation):
//   - Browse /log multiple times in a row — should NOT cause min_free dips.
//   - Steady-state min_free should stabilise much higher (probably >50 KB
//     instead of the 316-byte cliff observed in V2.3.16).
//   - Per-cycle "free heap before TX" line shows min_free across many
//     cycles, helping spot remaining transient-peak sources (NTP refresh,
//     WiFi reconnect, etc.).
#define VERSION_STR "V2.3.17"
