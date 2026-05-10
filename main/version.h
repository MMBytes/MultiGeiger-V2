#pragma once
// Bump before build; commit after successful flash.
//
// V2.3.18 — feature release. Status page expansion, FeatherS3-D log ring
// growth, FTP log-line polish.
//
//   1. **Status page (`/`) substantially expanded.** Previously: chip ID,
//      MAC, network, AP SSID, heap, uptime + PM block. Now organised into
//      labelled blocks: Device, Network, System, Cycle, Radiation,
//      Environment, Noise, Particulate matter, Uploads.
//
//      - **Device**: chip model + revision + cores + features (WiFi/BLE/
//        EmbFlash/EmbPSRAM), flash + PSRAM size, board name (heltec_v2 /
//        heltec_v2_4mb / feathers3_d), firmware version + build date,
//        antenna routing (PCB / u.FL on FeatherS3-D, N/A elsewhere).
//      - **Network**: SSID, security mode, PHY (b/g/n), bandwidth (BW20/40),
//        channel, BSSID, AID, RSSI, IP/Gateway/Netmask/DNS, reconnects
//        since boot. STA-mode only — AP mode shows just IP.
//      - **System**: uptime, decoded reset reason (POWER_ON / TASK_WDT /
//        BROWNOUT / etc.), free / min free / max alloc heap, NTP synced
//        + current wall-clock, AP SSID.
//      - **Cycle**: cycle counter, last dt_ms, configured interval, last
//        cycle wall-clock + "X seconds ago", time until next cycle.
//      - **Radiation** (only when tube_enabled): CPM, dose rate µSv/h,
//        HV pulses/min + cumulative, HV-error badge if set.
//      - **Environment** (only when env sensor present): sensor name
//        (SHT45+BMP581 / SHT45+BMP390 / BME688 / etc.), T/H/P readings.
//      - **Noise** (only when DNMS present): sensor name + version, LAeq,
//        LA min/max.
//      - **Particulate matter**: existing block, now with consistent
//        <h3>Particulate matter</h3> header.
//      - **Uploads**: HTML table — one row per *enabled* target with
//        succeeded/attempted, last response code (colour-coded), breaker
//        state (closed/open with cycles remaining). FTP/FTPS line below
//        with last upload timestamp + age, bytes, OK/FAIL badge, next-in.
//
//      Implementation: streamed via `httpd_resp_send_chunk` with a single
//      shared 1600-byte stack scratch buffer (same V2.3.17 /log pattern).
//      Zero per-request heap allocation. Stack bounded ~2.7 KB / 8 KB.
//      All sensor data read from caches populated by the worker — no I²C
//      from the HTTP context.
//
//   2. **PSRAM log ring 1 MB → 4 MB on FeatherS3-D.** PSRAM was almost
//      entirely idle (~7 MB free of 8 MB at steady state). Streaming
//      FTPS + /log post-V2.3.16/17 means ring size no longer creates
//      transient internal-DRAM peaks, so growing the ring is essentially
//      free. 4 MB gives ~40 hours of history at typical log volume,
//      while leaving 4 MB PSRAM headroom for any future feature.
//      Single line change in `applog.c` (`#if HAL_HAS_PSRAM` branch).
//      Heltec stays at 60 KB.
//
//   3. **FTP/FTPS log line cleanup.** Three small wording changes in
//      `log_ftp.c` so the heap-snapshot lines are SSL-agnostic and the
//      upload line reads naturally:
//      - `FTPS pre-upload heap:` → `Pre-upload heap:`
//      - `FTPS post-upload heap:` → `Post-upload heap:`
//      - `FTP%s: uploading` → `FTP%s uploading` (no colon between
//        protocol + verb)
//
//   4. **min_free on status page.** New "Min free heap" line in the
//      System block, mirroring the V2.3.17 addition to the per-cycle
//      tx_run log line. Same `esp_get_minimum_free_heap_size()` source.
//
//   5. **TX stats accumulators (per upload target).** New module-level
//      `s_stats[5]` array in `transmission.c` exposed via `tx_get_stats()`
//      / `tx_target_name()`. Tracks per-target attempted, succeeded,
//      last_rc, last_at, breaker_open_cycles. The breaker counter was
//      previously a per-target static local in `tx_run`; promoted into
//      the stats array so the status page reads the same value the
//      orchestrator writes (single-writer, lock-free, torn-tolerant).
//
//   6. **FTP stats accumulator.** New `log_ftp_get_stats()` in
//      `log_ftp.c`. Tracks have_last, last_ok, last_at, last_bytes,
//      next_due_ms. Populated at the end of `do_ftp_upload`. last_bytes
//      only updated on success so a failed retry doesn't blank the
//      previous-good count.
//
//   7. **Cycle/cache state lifted to file scope in `main.c`.** Cached
//      cpm, usvph, dt_ms, hv_pulses, hv_error, env T/H/P, last cycle
//      wall-clock + monotonic-ms. 14 small `main_status_*()` accessors
//      let `http_server.c` render the page without reaching into private
//      module state. Single-writer (main task), multi-reader (HTTP task)
//      lock-free pattern on word-aligned scalars.
//
// OTA-safe from V2.3.17 (no partition layout changes). 15 release
// artefacts (5 × 3 boards). No new sdkconfig flags.
#define VERSION_STR "V2.3.18"
