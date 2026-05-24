cmake_minimum_required(VERSION 3.16)

# ----------------------------------------------------------------------------
# Board selector — one codebase, multiple binaries.
#
# Pick a board by passing -DBOARD=<name> to idf.py:
#     idf.py -DBOARD=heltec_v2             build   (genuine Heltec WiFi Kit 32 V2, 8 MB)
#     idf.py -DBOARD=heltec_v2_4mb         build   (knock-off variant with 4 MB flash)
#     idf.py -DBOARD=feathers3_d           build   (UM FeatherS3-D, ESP32-S3 + 8 MB QSPI PSRAM)
#     idf.py -DBOARD=adafruit_qtpy_esp32_pico  build (ESP32-PICO SiP, 2 MB QSPI PSRAM)
#     idf.py -DBOARD=seeed_xiao_esp32s3    build   (Seeed XIAO ESP32-S3, 8 MB OPI PSRAM — I²C sensor host)
#
# Each board pins its own IDF chip target and pulls a board-specific
# sdkconfig.defaults overlay on top of the shared base.
# heltec_v2 and heltec_v2_4mb share the same hardware definitions in hal.h
# (BOARD_HELTEC_V2 macro) — the only deltas are flash size + partition table.
# ----------------------------------------------------------------------------
if(NOT DEFINED BOARD)
    set(BOARD "heltec_v2" CACHE STRING "Target board (heltec_v2 | heltec_v2_4mb | feathers3_d | adafruit_qtpy_esp32_pico | seeed_xiao_esp32s3)")
endif()

if(BOARD STREQUAL "heltec_v2")
    set(IDF_TARGET "esp32" CACHE STRING "")
    add_compile_definitions(BOARD_HELTEC_V2=1)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.heltec_v2")
elseif(BOARD STREQUAL "heltec_v2_4mb")
    set(IDF_TARGET "esp32" CACHE STRING "")
    # Same hardware macro as heltec_v2 (same pins, same OLED, same Vext) — hal.h
    # treats both identically. The _4MB variant adds an extra macro for code
    # that needs to distinguish flash variants (OTA upload form, boot log).
    add_compile_definitions(BOARD_HELTEC_V2=1 BOARD_HELTEC_V2_4MB=1)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.heltec_v2_4mb")
elseif(BOARD STREQUAL "feathers3_d")
    set(IDF_TARGET "esp32s3" CACHE STRING "")
    # MQTT_RICH_STATE: opt in to the extended state JSON (system + per-target
    # upload stats). PSRAM-equipped board, heap headroom is plenty. See
    # mqtt.c::mqtt_publish_state() and mqtt_discovery.c for the gated sections.
    add_compile_definitions(BOARD_FEATHERS3_D=1 MQTT_RICH_STATE=1)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.feathers3_d")
elseif(BOARD STREQUAL "adafruit_qtpy_esp32_pico")
    # Adafruit QT Py ESP32-PICO (PID 5395). Original ESP32 LX6 SiP with
    # in-package 8 MB flash + 2 MB PSRAM. NeoPixel-equipped, no piezo, drops
    # OLED + speaker via HAL_HAS_OLED=0 / HAL_HAS_SPEAKER=0 in hal.h.
    set(IDF_TARGET "esp32" CACHE STRING "")
    add_compile_definitions(BOARD_ADAFRUIT_QTPY_ESP32_PICO=1 MQTT_RICH_STATE=1)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.adafruit_qtpy_esp32_pico")
elseif(BOARD STREQUAL "seeed_xiao_esp32s3")
    # Seeed Studio XIAO ESP32-S3 (SKU 113991054). ESP32-S3 LX7 dual-core, 8 MB
    # QSPI flash + 8 MB OPI (octal-mode) PSRAM, native USB-C. Intended as an
    # I²C-only sensor host — Geiger tube not wired. See hal.h block under
    # BOARD_SEEED_XIAO_ESP32S3 for pin map and the "intended use" rationale.
    set(IDF_TARGET "esp32s3" CACHE STRING "")
    add_compile_definitions(BOARD_SEEED_XIAO_ESP32S3=1 MQTT_RICH_STATE=1)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.seeed_xiao_esp32s3")
else()
    message(FATAL_ERROR "Unknown BOARD '${BOARD}'. Valid: heltec_v2 | heltec_v2_4mb | feathers3_d | adafruit_qtpy_esp32_pico | seeed_xiao_esp32s3")
endif()

# Per-board sdkconfig cache file. Without this, both builds share a single
# `sdkconfig` cache and the second-board build fails with a target-mismatch
# error from targets.cmake.
set(SDKCONFIG "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.${BOARD}" CACHE FILEPATH "" FORCE)

# ----------------------------------------------------------------------------
# Derive the IDF app-descriptor version from main/version.h so it always
# matches VERSION_STR (the runtime/UI string). Setting PROJECT_VER before
# project() tells ESP-IDF to skip its `git describe` autodetect.
# ----------------------------------------------------------------------------
file(READ "${CMAKE_CURRENT_LIST_DIR}/main/version.h" _version_h)
string(REGEX MATCH "VERSION_STR[ \t]+\"([^\"]+)\"" _ "${_version_h}")
set(PROJECT_VER "${CMAKE_MATCH_1}")

# Tell CMake to re-run configure whenever version.h changes, so PROJECT_VER
# (and therefore the IDF app descriptor visible in the boot log as
# "App version:") always matches the current VERSION_STR. Without this,
# `file(READ)` only fires at configure time and incremental builds keep the
# stale PROJECT_VER from the last full configure — symptom is "App version:"
# showing a previous release while v2_main's runtime log shows the new one.
# Caught 2026-05-09 during V2.3.10 work; fix landed in V2.3.11.
set_property(DIRECTORY APPEND PROPERTY
    CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/main/version.h")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(geiger_v2)

# ----------------------------------------------------------------------------
# V2.4.3: PSA key-slot count override.
#
# `CONFIG_MBEDTLS_PSA_KEY_SLOT_COUNT` was a valid kconfig knob through IDF
# v5.x (set to 128 in sdkconfig.defaults — see comment there for the
# rationale). IDF v6.0 + mbedtls 4.x removed it: PSA configuration moved
# into psa/crypto_config.h. The setting still has to land, just via a
# preprocessor define on the build line. `APPEND` so we don't clobber any
# other -D's that future contributors add via this same property.
# ----------------------------------------------------------------------------
idf_build_set_property(COMPILE_DEFINITIONS "MBEDTLS_PSA_KEY_SLOT_COUNT=128" APPEND)
