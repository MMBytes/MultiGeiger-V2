#pragma once

// Single source of truth for the firmware version string.
//
// Bump BEFORE building; commit AFTER successful flash (per
// `[[feedback_build_flash_workflow]]`).
//
// V2.4.1 (this release) extracted the per-release WHAT/WHY commentary from
// this header into `../CHANGELOG.md` at the repo root. Pre-V2.4.1 this
// file was 862 LOC of accumulated release archaeology that every build
// had to parse. Open `CHANGELOG.md` for the full version-by-version trail
// (V2.4.1 through V2.3.23 with full bodies; V2.3.22 and earlier as
// headlines with full bodies on GitHub at
// https://github.com/MMBytes/MultiGeiger-V2/releases ).
//
// CMake tracks this file via
// `set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS main/version.h)`
// (see top-level CMakeLists.txt) so the `PROJECT_VER` substituted into
// `esp_app_desc_t` always matches VERSION_STR on the next build.
#define VERSION_STR "V2.7.5"
