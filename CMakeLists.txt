cmake_minimum_required(VERSION 3.16)

# Derive the IDF app-descriptor version from main/version.h so it always matches
# VERSION_STR (the runtime/UI string). Setting PROJECT_VER before project() tells
# ESP-IDF to skip its `git describe` autodetect.
file(READ "${CMAKE_CURRENT_LIST_DIR}/main/version.h" _version_h)
string(REGEX MATCH "VERSION_STR[ \t]+\"([^\"]+)\"" _ "${_version_h}")
set(PROJECT_VER "${CMAKE_MATCH_1}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(geiger_v2)
