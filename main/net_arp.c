/** @file
 *  @brief Implementation of net_arp_send_gratuitous(). See net_arp.h for
 *  the rationale and call-site documentation.
 */

#include "net_arp.h"

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_net_stack.h"   // esp_netif_get_netif_impl

#include "lwip/etharp.h"           // etharp_gratuitous
#include "lwip/tcpip.h"            // LOCK_TCPIP_CORE / UNLOCK_TCPIP_CORE

static const char *TAG = "net_arp";

void net_arp_send_gratuitous(void) {
    // WIFI_STA_DEF is the netif key the default station-mode WiFi netif
    // registers under (see esp_netif_create_default_wifi_sta() in main.c).
    // Returns NULL if STA mode was never initialised — silent no-op.
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) return;

    // Convert the esp-netif handle to the underlying lwIP `struct netif *`
    // that etharp_gratuitous() expects. Returns NULL if the netif is not
    // backed by lwIP (shouldn't happen for the standard WiFi STA path).
    struct netif *lwip_netif = esp_netif_get_netif_impl(netif);
    if (!lwip_netif) return;

    // etharp_gratuitous walks the netif's outbound queue and must not
    // race the lwIP TCP/IP task. LOCK_TCPIP_CORE serialises against it.
    // The call itself completes within µs — single ARP frame built and
    // queued for the WiFi driver. No blocking on TX confirmation.
    LOCK_TCPIP_CORE();
    etharp_gratuitous(lwip_netif);
    UNLOCK_TCPIP_CORE();

    ESP_LOGI(TAG, "sent gratuitous ARP (bridge-table refresh)");
}
