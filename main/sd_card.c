#include "sd_card.h"

#include "hal.h"

#if HAL_HAS_SD_CARD

#include <string.h>

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_http_server.h"
#include "sdmmc_cmd.h"
#if HAL_SD_USE_SDMMC
#include "driver/sdmmc_host.h"
#else
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "http_server.h"

static const char *TAG = "sd_card";

static sdmmc_card_t *s_card;
static bool          s_mounted;

// esp_vfs_select() — used by httpd's blocking accept loop — walks ESP-IDF's
// *global* VFS table on every call, not just httpd's own sockets. Mounting/
// unmounting registers/unregisters our slot in that same table.
//
// Root cause of the two 2026-07-16 card-pull crashes (LoadProhibited,
// excvaddr 0xc, httpd task): NOT a race. esp_vfs_unregister_with_id()
// (vfs.c) NULLs s_vfs[i] but never shrinks s_vfs_count, so after unmount
// the table has a PERSISTENT NULL hole until the next successful mount
// refills it. esp_vfs_select()'s NULL-slot guard (vfs_calls.c:660) then
// executes ESP_LOGD(..., vfs->offset) with vfs==NULL — offset is at +0xc —
// on httpd's very next select(), deterministically, same task, no
// concurrency needed. Real fix: CONFIG_LOG_MAXIMUM_EQUALS_DEFAULT (INFO)
// in sdkconfig, which dead-codes that ESP_LOGD entirely (V2.6.21).
//
// The serialization below stays as defense-in-depth against the SEPARATE,
// genuine race the vfs_calls.c comments warn about: a cross-task
// unregister can free a vfs_entry_t while an in-flight select() on another
// task has already fetched the pointer (fd-conversion + start_select loops
// read entry pointers non-atomically). httpd_queue_work() runs its callback
// ON the httpd task itself, serialized with its own select() loop, so our
// mount/unmount can never overlap httpd's select() mid-body.
// Two escape hatches: before httpd_start() (the very first boot-time mount,
// main.c ~line 1308) there's no httpd task yet to race, and if we're
// somehow already ON the httpd task (no caller does this today, but a
// future admin-triggered remount endpoint might) queuing to ourselves
// would deadlock — both cases just call straight through instead.
typedef struct {
    void              (*fn)(void);
    SemaphoreHandle_t  done;
} vfs_op_ctx_t;

static void vfs_op_trampoline(void *arg) {
    vfs_op_ctx_t *ctx = (vfs_op_ctx_t *)arg;
    ctx->fn();
    xSemaphoreGive(ctx->done);
}

// Returns true if fn() actually ran (via the serialized path or one of the
// two straight-through escape hatches above), false if it was skipped.
static bool run_serialized_with_httpd(void (*fn)(void)) {
    httpd_handle_t hd = http_server_get_handle();
    if (hd == NULL || strcmp(pcTaskGetName(NULL), "httpd") == 0) {
        fn();
        return true;
    }

    static SemaphoreHandle_t s_done;
    if (s_done == NULL) {
        s_done = xSemaphoreCreateBinary();
    }

    vfs_op_ctx_t ctx = { .fn = fn, .done = s_done };
    if (httpd_queue_work(hd, vfs_op_trampoline, &ctx) != ESP_OK) {
        // V2.6.21 (bench-confirmed 2026-07-16, second card-pull crash): this
        // used to fall back to calling fn() directly here. That reintroduces
        // the exact race this whole function exists to prevent — fn() would
        // run on the CURRENT task while httpd's task can simultaneously be
        // inside its own esp_vfs_select(), same as having no serialization at
        // all. A queue-work failure is a rare ctrl-socket hiccup; the safe
        // response is to skip this attempt and let the caller retry next
        // cycle (sd_logger_cycle() already retries mount/unmount every
        // cycle on failure), not take the one path known to crash.
        ESP_LOGW(TAG, "httpd_queue_work failed, deferring to next cycle");
        return false;
    }
    xSemaphoreTake(ctx.done, portMAX_DELAY);
    return true;
}

static esp_err_t s_mount_result;

static void do_mount_work(void) {
    if (s_mounted) { s_mount_result = ESP_OK; return; }

    // format_if_mount_failed deliberately false: a wrong-FS card (exFAT,
    // NTFS) must surface as an error the user can see, never be wiped.
    esp_vfs_fat_sdmmc_mount_config_t mcfg = {
        .format_if_mount_failed = false,
        .max_files              = 2,
        .allocation_unit_size   = 16 * 1024,
    };
    esp_err_t err;

#if HAL_SD_USE_SDMMC
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk = PIN_SD_CLK;
    slot.cmd = PIN_SD_CMD;
    slot.d0  = PIN_SD_D0;
    slot.d1  = PIN_SD_D1;
    slot.d2  = PIN_SD_D2;
    slot.d3  = PIN_SD_D3;
    slot.width = 4;
    // Belt + braces: the board has external pull-ups (SparkFun's own SDIO
    // benchmark runs on it), the internal ~45k ones just back them up.
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    err = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot, &mcfg, &s_card);
#else
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus = {
        .mosi_io_num     = PIN_SD_PICO,
        .miso_io_num     = PIN_SD_POCI,
        .sclk_io_num     = PIN_SD_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };
    // ESP_ERR_INVALID_STATE = bus already initialised from a previous
    // mount/unmount round — fine, reuse it (we never free the bus).
    err = spi_bus_initialize(host.slot, &bus, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
        s_mount_result = err;
        return;
    }
    sdspi_device_config_t dev = SDSPI_DEVICE_CONFIG_DEFAULT();
    dev.gpio_cs = PIN_SD_CS;
    dev.host_id = host.slot;
    err = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &dev, &mcfg, &s_card);
#endif

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mount failed: %s (no card / not FAT32?)", esp_err_to_name(err));
        s_mount_result = err;
        return;
    }
    s_mounted = true;
    ESP_LOGI(TAG, "mounted %s: %s %lluMB",
             SD_MOUNT_POINT, s_card->cid.name,
             ((unsigned long long)s_card->csd.capacity * s_card->csd.sector_size) / (1024 * 1024));
    s_mount_result = ESP_OK;
}

esp_err_t sd_card_mount(void) {
    if (!run_serialized_with_httpd(do_mount_work)) {
        // Deferred, not attempted — don't return a stale s_mount_result from
        // some earlier call. sd_logger_cycle() treats any non-ESP_OK as a
        // failed cycle and retries next time.
        return ESP_ERR_INVALID_STATE;
    }
    return s_mount_result;
}

bool sd_card_mounted(void) {
    return s_mounted;
}

static void do_unmount_work(void) {
    if (!s_mounted) return;
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
    s_card    = NULL;
    s_mounted = false;
    ESP_LOGI(TAG, "unmounted");
}

void sd_card_unmount(void) {
    run_serialized_with_httpd(do_unmount_work);
}

#else  // !HAL_HAS_SD_CARD — inert stubs, same convention as fuel_gauge.c

esp_err_t sd_card_mount(void)   { return ESP_ERR_NOT_SUPPORTED; }
bool      sd_card_mounted(void) { return false; }
void      sd_card_unmount(void) { }

#endif
