/** @file
 *  @brief Host-test shim for <esp_log.h> (V2.8.8).
 *
 *  The macros print nothing, but still pass their arguments through a dead
 *  printf() call: the compiler keeps checking the format string against the
 *  arguments (a mismatch fails the -Werror host build, as it would on the
 *  target), and variables used only in a log line do not trip
 *  -Wunused-variable. `##__VA_ARGS__` is the GNU form ESP-IDF itself uses and
 *  zero-argument calls (ESP_LOGE(TAG, "text")) need, so the host build must
 *  not add -Wpedantic. See test/shim/README.md.
 */
#pragma once

#include <stdio.h>

#define SHIM_ESP_LOG_(tag, fmt, ...)                       \
    do {                                                   \
        if (0) printf("%s " fmt "\n", (tag), ##__VA_ARGS__); \
    } while (0)

#define ESP_LOGE(tag, fmt, ...) SHIM_ESP_LOG_(tag, fmt, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) SHIM_ESP_LOG_(tag, fmt, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) SHIM_ESP_LOG_(tag, fmt, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) SHIM_ESP_LOG_(tag, fmt, ##__VA_ARGS__)
#define ESP_LOGV(tag, fmt, ...) SHIM_ESP_LOG_(tag, fmt, ##__VA_ARGS__)
