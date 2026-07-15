#pragma once

/** @file
 *  @brief V2.6.19: microSD FAT32 mount for the standalone SD-logging mode.
 *
 *  Per-board host (spec §3): the S3 Thing Plus slot is fully SDIO-wired, so
 *  it mounts via the SDMMC 4-bit host — native SD protocol with per-transfer
 *  data CRC (SPI mode leaves data CRC off by default; on a solar node that
 *  WILL brown out eventually, the integrity check is the point). The C5 has
 *  no SDMMC peripheral at all, so it mounts via SDSPI. Everything above the
 *  mount — FATFS, VFS, POSIX stdio — is identical for both.
 *
 *  No card-detect pin is used on either board (C5's isn't even connected by
 *  default): card presence == mount attempt succeeding. Mount failure is
 *  non-fatal; sd_logger retries once per TX cycle.
 *
 *  On HAL_HAS_SD_CARD=0 boards all functions are inert stubs — callers need
 *  no #ifdef (same convention as fuel_gauge.h).
 */

#include <stdbool.h>
#include "esp_err.h"

#define SD_MOUNT_POINT "/sd"

/** @brief Mount the card (idempotent — ESP_OK if already mounted).
 *  @return ESP_OK, ESP_ERR_NOT_SUPPORTED on non-SD boards, or the
 *          mount/driver error (no card, bad FS, ...). */
esp_err_t sd_card_mount(void);

bool sd_card_mounted(void);

/** @brief Unmount + release so a later sd_card_mount() can retry cleanly.
 *  Called by sd_logger after write failures (e.g. card pulled mid-run). */
void sd_card_unmount(void);
