#pragma once

/** @file
 *  @brief Gratuitous ARP helper — keeps upstream AP / mesh bridge forwarding
 *  tables fresh for this sensor's MAC so broker→sensor unicast frames don't
 *  get blackholed after long idle periods or WiFi roams.
 *
 *  V2.4.19: introduced after the 2026-05-21 incident on esp32-5963724 where
 *  mosquitto→sensor SYN-ACK was 100 % dropped at the mesh-AP bridge while
 *  sensor→mosquitto direction worked perfectly. Mesh AP had aged out + then
 *  re-learned wrong after the WiFi roam. Detailed post-mortem in
 *  `[[reference_mqtt_one_way_loss_after_wifi_roam]]`.
 *
 *  lwIP's default behaviour is to send a single gratuitous ARP on `GOT_IP`
 *  only. On a stable link that's days/weeks apart — long enough for any
 *  bridge aging timer (typically 5 min) to expire. This helper is called
 *  from two places to close the gap:
 *
 *  1. After each successful WiFi reconnect (main.c, on EV_GOT_IP) — fires
 *     a second gratuitous ARP a tick after lwIP's automatic one, giving
 *     the upstream bridges a redundant chance to relearn the path before
 *     it matters for downstream traffic (MQTT broker, OTA, etc.).
 *  2. As part of the 24h PSA crypto refresh (log_ftp.c) — low-overhead
 *     safety-net for the rare case where a sensor never reconnects for
 *     days and the bridge entry expires anyway.
 *
 *  Not gated on TX/FTP internally — callers do that with `tx_is_idle()`
 *  (FTP gating is structural, both call sites run on the main task).
 */

/** @brief Send one gratuitous ARP on the STA netif. Safe from any task —
 *  takes the lwIP TCP/IP core lock internally. No-op if the STA netif
 *  doesn't exist or has no IP. Costs one 60-byte ARP broadcast on-air
 *  and a single µs-level lwIP call.
 */
void net_arp_send_gratuitous(void);
