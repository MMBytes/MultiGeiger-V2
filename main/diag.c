#include "diag.h"

#include <stdatomic.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "diag";

static atomic_uint_least32_t g_i2c_errors = 0;

void diag_i2c_error_inc(void) {
    atomic_fetch_add(&g_i2c_errors, 1);
}

uint32_t diag_i2c_errors(void) {
    return (uint32_t)atomic_load(&g_i2c_errors);
}

void diag_log_heap(const char *where) {
    // V2.4.32 (Tier-1 net-stack instrumentation): WiFi + lwIP RX buffers are
    // allocated from INTERNAL (DMA-capable) RAM — NOT the PSRAM that dominates
    // esp_get_free_heap_size(). A slow drain of internal/DMA RAM over multi-day
    // uptime starves sustained INBOUND TCP (seen as OTA-upload stalls after one
    // window on esp32-5965048, 2026-05-30) while the PSRAM-based "free heap"
    // stays flat and hides it. Logging the capability split per-cycle (→ /log →
    // FTP) and at OTA-prep gives the time series needed to spot WHICH bucket
    // drains. Read all three counters: stable free + falling largest/min =
    // fragmentation, not a leak. Cheap: ~5 heap_caps reads, no locks held.
    ESP_LOGI(TAG,
             "%s heap: INTERNAL free=%u largest=%u min=%u | DMA free=%u largest=%u",
             where,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
}

void diag_log_heap_standalone(void) {
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_free  = esp_get_minimum_free_heap_size();
    uint32_t max_alloc = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    ESP_LOGI(TAG,
             "free heap: %u / min_free=%u / max_alloc=%u / "
             "INTERNAL free=%u largest=%u min=%u | DMA free=%u largest=%u",
             (unsigned)free_heap, (unsigned)min_free, (unsigned)max_alloc,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
}
