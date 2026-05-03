cmake_minimum_required(VERSION 3.16)

# ----------------------------------------------------------------------------
# Board selector — one codebase, multiple binaries.
#
# Pick a board by passing -DBOARD=heltec_v2 (default) or -DBOARD=feathers3_d
# to idf.py:
#     idf.py -DBOARD=heltec_v2   build
#     idf.py -DBOARD=feathers3_d build
#
# Each board pins its own IDF chip target and pulls a board-specific
# sdkconfig.defaults overlay on top of the shared base.
# ----------------------------------------------------------------------------
if(NOT DEFINED BOARD)
    set(BOARD "heltec_v2" CACHE STRING "Target board (heltec_v2 | feathers3_d)")
endif()

if(BOARD STREQUAL "heltec_v2")
    set(IDF_TARGET "esp32" CACHE STRING "")
    add_compile_definitions(BOARD_HELTEC_V2=1)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.heltec_v2")
elseif(BOARD STREQUAL "feathers3_d")
    set(IDF_TARGET "esp32s3" CACHE STRING "")
    add_compile_definitions(BOARD_FEATHERS3_D=1)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.defaults.feathers3_d")
else()
    message(FATAL_ERROR "Unknown BOARD '${BOARD}'. Valid: heltec_v2 | feathers3_d")
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

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(geiger_v2)
