#include "sd_card.h"

#include "hal.h"

#if HAL_HAS_SD_CARD

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#if HAL_SD_USE_SDMMC
#include "driver/sdmmc_host.h"
#else
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#endif

static const char *TAG = "sd_card";

static sdmmc_card_t *s_card;
static bool          s_mounted;

esp_err_t sd_card_mount(void) {
    if (s_mounted) return ESP_OK;

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
        return err;
    }
    sdspi_device_config_t dev = SDSPI_DEVICE_CONFIG_DEFAULT();
    dev.gpio_cs = PIN_SD_CS;
    dev.host_id = host.slot;
    err = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &dev, &mcfg, &s_card);
#endif

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mount failed: %s (no card / not FAT32?)", esp_err_to_name(err));
        return err;
    }
    s_mounted = true;
    ESP_LOGI(TAG, "mounted %s: %s %lluMB",
             SD_MOUNT_POINT, s_card->cid.name,
             ((unsigned long long)s_card->csd.capacity * s_card->csd.sector_size) / (1024 * 1024));
    return ESP_OK;
}

bool sd_card_mounted(void) {
    return s_mounted;
}

void sd_card_unmount(void) {
    if (!s_mounted) return;
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
    s_card    = NULL;
    s_mounted = false;
    ESP_LOGI(TAG, "unmounted");
}

#else  // !HAL_HAS_SD_CARD — inert stubs, same convention as fuel_gauge.c

esp_err_t sd_card_mount(void)   { return ESP_ERR_NOT_SUPPORTED; }
bool      sd_card_mounted(void) { return false; }
void      sd_card_unmount(void) { }

#endif
