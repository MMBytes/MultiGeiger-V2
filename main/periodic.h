#pragma once

/** @file
 *  @brief Main-task periodic housekeeping — chores that need a sane
 *  call site on the main task with `tx_is_idle()` gating but aren't
 *  conceptually owned by any single feature module.
 *
 *  Pre-V2.4.19 these were squatted inside `log_ftp.c::log_ftp_loop()`
 *  for convenience (FTP shares the "runs on main, blocks for seconds at
 *  a time" property with this code). The naming was misleading — a
 *  reader searching for "where do we do periodic ARP?" would not look
 *  in the FTP module. V2.4.19 extracted them here so the file name
 *  matches the function.
 *
 *  Currently runs:
 *  1. **24h PSA crypto subsystem refresh** — `mbedtls_psa_crypto_free()`
 *     + `psa_crypto_init()` to return the slot pool to empty.
 *     Compensates for slow heap fragmentation across hundreds of TLS
 *     handshakes (V2.3.23). Independent of FTP being enabled.
 *     V2.4.31: stops the persistent MQTT client first — its TLS session's
 *     keys live in PSA slots, so freeing the pool out from under a live
 *     MQTT connection broke it every 24h. The main loop re-inits MQTT on
 *     the next tick.
 *  2. **24h gratuitous ARP safety-net** — `net_arp_send_gratuitous()`
 *     so that sensors which never reconnect for days still keep
 *     upstream AP/mesh bridge forwarding tables warm. The per-
 *     reconnect ARP in `main.c` handles the common case (V2.4.19).
 *
 *  Both chores share the same 24h cadence and the same `tx_is_idle()`
 *  gate. The TX-worker idle check matters because:
 *    - `mbedtls_psa_crypto_free()` would corrupt an in-flight HTTPS
 *      handshake's state. (The *persistent* MQTT TLS session is a
 *      separate case the idle gate can't see — V2.4.31 handles it by
 *      stopping MQTT before the free; see periodic.c.)
 *    - The ARP itself is safe to fire concurrently but the user's
 *      explicit preference (2026-05-21) is to avoid sharing airtime
 *      with TX/FTP activity.
 *
 *  FTP overlap is structurally impossible — both this and FTPS upload
 *  run on the main task and that task is single-threaded.
 */

#include <stdint.h>

/** @brief Call once per main-loop tick with the current monotonic
 *  uptime in milliseconds. No-op for most ticks (chores fire at most
 *  once per 24h each). Cheap when nothing is due.
 */
void periodic_loop(uint32_t now_ms);
