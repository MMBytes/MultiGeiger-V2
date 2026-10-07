/** @file
 *  @brief Host-test shim for <esp_err.h> (V2.8.8).
 *
 *  The error type and the codes the host-compiled headers name. Values match
 *  ESP-IDF's (components/esp_common/include/esp_err.h) so a test comparing
 *  against a literal code means the same thing on host and target.
 *  See test/shim/README.md.
 */
#pragma once

typedef int esp_err_t;

#define ESP_OK                 0
#define ESP_FAIL              -1
#define ESP_ERR_NO_MEM         0x101
#define ESP_ERR_INVALID_ARG    0x102
#define ESP_ERR_INVALID_STATE  0x103
#define ESP_ERR_INVALID_SIZE   0x104
#define ESP_ERR_NOT_FOUND      0x105
#define ESP_ERR_NOT_SUPPORTED  0x106
#define ESP_ERR_TIMEOUT        0x107

static inline const char *esp_err_to_name(esp_err_t code) {
    return code == ESP_OK ? "ESP_OK" : "ESP_ERR";
}
