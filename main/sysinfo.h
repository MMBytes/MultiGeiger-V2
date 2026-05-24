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
