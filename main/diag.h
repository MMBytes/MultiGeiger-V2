#pragma once

/** @file
 *  @brief Lightweight diagnostic counters (V2.4.28).
 *
 *  Global atomic counter for I²C sensor read failures, surfaced on the
 *  status page, in the CYCLE log line (which lands in /log via applog),
 *  and in MQTT (under MQTT_RICH_STATE on PSRAM boards). Sensor read sites
 *  in main.c::do_tx_cycle() call `diag_i2c_error_inc()` once on each
 *  failed top-level read (env / PM / noise / light) — one increment per
 *  failed read regardless of how many underlying I²C transactions were
 *  attempted.
 *
 *  All sensors in V2.4.x are on the shared I²C bus (env, PM=SPS30, noise=
 *  DNMS, light=VEML7700, etc.) — see i2c_bus.c. The counter therefore
 *  reflects bus-level health.
 *
 *  Why one counter, not per-sensor: a rising aggregate is the high-value
 *  signal for "something is wrong" (supply marginal, bus contention,
 *  flaky cable). Per-sensor breakdown can be added later if the
 *  aggregate shows movement — start simple. Counting at the call site
 *  also keeps drivers untouched.
 *
 *  Scope deliberately tight:
 *    - One counter, all sensor read failures across env / PM / noise / light.
 *    - No reset path — only way to clear is reboot. Matches the rest of our
 *      diagnostic counters (reconnects, tx stats, ftp stats).
 *
 *  Cheap: one `atomic_fetch_add` per failure. Negligible vs the failed read
 *  that just happened (which itself took milliseconds and logged a WARN).
 */

#include <stdint.h>

/** @brief Increment the I²C sensor-read-error counter by one. Safe from any task. */
void diag_i2c_error_inc(void);

/** @brief Read the current I²C sensor-read-error count. Safe from any task. */
uint32_t diag_i2c_errors(void);

/** @brief Log internal/DMA-capable RAM free/largest/min at a labelled point.
 *
 *  V2.4.32 Tier-1 net-stack instrumentation. WiFi + lwIP RX buffers live in
 *  INTERNAL (DMA) RAM, NOT the PSRAM that dominates esp_get_free_heap_size();
 *  a multi-day drain there starves sustained inbound TCP (OTA-upload stalls)
 *  while the total "free heap" stays flat and hides it. Call per-cycle and at
 *  OTA-prep so the /log → FTP time series shows which capability bucket drains
 *  over uptime. `where` is a short context label (e.g. "per-cycle", "OTA prep").
 */
void diag_log_heap(const char *where);

/** @brief Log one combined line: free/min_free/max_alloc PSRAM-dominated
 *  heap summary + the INTERNAL/DMA capability split (diag_log_heap()'s
 *  fields), all in a single ESP_LOGI call.
 *
 *  V2.6.21 standalone SD-logging: tx_run() (transmission.c) has logged the
 *  same information as two separate lines once per networked TX cycle since
 *  V2.3.17/V2.4.32 — standalone mode never reaches tx_run() (do_tx_cycle
 *  returns via sd_logger_cycle() before ever calling tx_transmit()), so a
 *  standalone node had zero per-cycle heap visibility in /log until this was
 *  added to the standalone branch. One line here (not two, unlike tx_run())
 *  by explicit request.
 */
void diag_log_heap_standalone(void);
