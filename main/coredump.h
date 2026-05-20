#pragma once

/** @file
 *  @brief Post-mortem core dump access — read/stream/erase the panic dump
 *         that ESP-IDF wrote to the `coredump` partition.
 *
 *  V2.4.18: pairs with the 64 KB `coredump` partition added to
 *  `partitions.csv` / `partitions_4mb.csv`, and with
 *  `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y` in every per-board sdkconfig.
 *  When the panic handler fires, ESP-IDF writes an ELF blob (register
 *  state + every task's stack snapshot + the panic-reason string) into
 *  the partition before rebooting. This module exposes that blob over
 *  the air via `GET /coredump.elf` so deployed sensors don't need to be
 *  pulled off the wall and connected to USB to get a backtrace.
 *
 *  Lifecycle:
 *    - `coredump_init()` called once at boot. Probes the partition, runs
 *      `esp_core_dump_image_check()`, populates the cached summary
 *      (task name, PC, panic reason) for cheap status-page reads.
 *    - HTTP server calls `coredump_have_dump()` / `coredump_get_size()` /
 *      `coredump_get_summary_html()` per /status render — all cheap.
 *    - HTTP server calls `coredump_stream_to_http()` on GET
 *      /coredump.elf — reads the partition in chunks and writes to the
 *      response. Single 64 KB partition; chunked TCP write keeps RAM
 *      use flat.
 *    - User downloads, decodes off-device with `espcoredump.py`, then
 *      POSTs /coredump_erase to clear the partition.
 *
 *  Why a separate module instead of inlining in http_server.c:
 *  the summary parse runs once at boot before HTTP is up — cleanest
 *  place is its own init step, called from app_main alongside
 *  `applog_init()`. http_server.c stays focused on HTTP.
 */

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Probe the coredump partition at boot. Caches presence + summary.
 *
 *  Safe to call once. If a valid dump is present, parses
 *  `esp_core_dump_get_summary()` and `esp_core_dump_get_panic_reason()`
 *  into static buffers for cheap repeated reads. If `image_check()`
 *  returns ESP_ERR_NOT_FOUND, leaves everything zeroed.
 *
 *  Logs a one-line ESP_LOGW summary if a dump was found, so the boot
 *  log line is preserved in /log and syslog even if the user never
 *  opens the status page.
 */
void coredump_init(void);

/** @brief True if a valid core dump is currently stored in flash. */
bool coredump_have_dump(void);

/** @brief Total dump size in bytes (incl. checksum). 0 if no dump. */
size_t coredump_get_size(void);

/** @brief Render a one-line HTML summary into `out`.
 *
 *  Format when dump is present:
 *    "yes &middot; <SZ> bytes &middot; task=<NAME> PC=0x<HEX> &middot; <REASON>"
 *  When absent:
 *    "none"
 *
 *  Always writes a NUL-terminated string. Caller passes a buffer of
 *  ~256 bytes — the panic-reason string is the longest variable part
 *  and IDF caps it at ~200 bytes.
 */
void coredump_get_summary_html(char *out, size_t sz);

/** @brief Stream the coredump partition contents to an HTTP response.
 *
 *  Caller must have already set Content-Type
 *  (`application/octet-stream`) and Content-Disposition. This function
 *  reads the partition in CHUNK_SIZE chunks and calls
 *  `httpd_resp_send_chunk` for each, then a final zero-length chunk
 *  to terminate the response.
 *
 *  Returns the underlying httpd error code on first failure, or
 *  ESP_OK on full success. Caller should treat any non-OK as a
 *  TCP-side problem (client gone, etc.) and just return ESP_OK to
 *  the framework.
 *
 *  Pre-condition: `coredump_have_dump()` returns true. Calling with
 *  no dump returns ESP_ERR_NOT_FOUND without writing anything.
 */
esp_err_t coredump_stream_to_http(httpd_req_t *req);

/** @brief Erase the coredump partition.
 *
 *  Calls `esp_core_dump_image_erase()` which fills the partition with
 *  0xFF. After successful erase, the cached state is also cleared —
 *  subsequent `coredump_have_dump()` returns false until the next
 *  panic. Returns ESP_OK on success or the underlying error.
 */
esp_err_t coredump_erase(void);

#ifdef __cplusplus
}
#endif
