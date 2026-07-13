/** @file
 *  @brief Small shared helpers for system-introspection strings.
 *
 *  Extracted from http_server.c at V2.4.26 so mqtt.c can reuse the same
 *  reset-reason mapping when populating the state JSON. Kept header-only
 *  (`static inline`) to avoid the ceremony of a new .c file for one
 *  switch — both callers compile the same ~14-line lookup with no
 *  observable size impact.
 */
#pragma once

#include "esp_system.h"
#include "esp_chip_info.h"

static inline const char *reset_reason_str(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "POWER_ON";
        case ESP_RST_EXT:       return "EXT_PIN";
        case ESP_RST_SW:        return "SOFT (esp_restart)";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "OTHER_WDT";
        case ESP_RST_DEEPSLEEP: return "DEEP_SLEEP_WAKE";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

// Chip silicon model as a short string ("ESP32" / "ESP32-S3" / …). Shared by the
// boot `chip:` log, the /status device block, and the syslog boot banner so the
// model ladder lives in exactly one place. The `default` keeps -Wswitch happy
// (and covers parts this firmware doesn't target, e.g. C6/H2/P4).
static inline const char *chip_model_str(esp_chip_model_t m) {
    switch (m) {
        case CHIP_ESP32:   return "ESP32";
        case CHIP_ESP32S2: return "ESP32-S2";
        case CHIP_ESP32S3: return "ESP32-S3";
        case CHIP_ESP32C3: return "ESP32-C3";
        case CHIP_ESP32C5: return "ESP32-C5";
        default:           return "?";
    }
}
