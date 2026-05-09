#pragma once
// Bump before build; commit after successful flash.
// V2.3.11 — SHT45 init diagnostics + CMakeLists.txt PROJECT_VER auto-rebuild.
//
//   1. CMakeLists.txt: add CMAKE_CONFIGURE_DEPENDS on main/version.h so the
//      app descriptor (visible in boot log as "App version:") always tracks
//      VERSION_STR. V2.1.22's auto-sync was incomplete — file(READ) only fires
//      at configure time, and CMake didn't know to re-run configure when
//      version.h changed. V2.3.7 → V2.3.10 boot logs all showed App version
//      V2.3.7 because that was the last full reconfigure. Now self-correcting.
//      See feedback_app_descriptor_version_sync.md memory for full analysis.
//
//   2. main/sht45.c: surface per-step error codes during init so future SHT45
//      bring-up failures don't collapse to a bare "probe read failed". Also
//      retry the soft-reset + verification read once with a longer wait if
//      the first attempt fails — distinguishes timing-margin issues from
//      hard hardware faults. Triggered by the FeatherS3-D bench's Adafruit
//      6174 SHT45 board not responding past the i2c_master_probe ACK on its
//      first ever connection (boot log: "SHT45 probe read failed" with no
//      detail of which step in the soft-reset → measure → read sequence
//      actually failed). One boot of V2.3.11 will tell us whether it's
//      defective hardware or a driver-side timing issue.
//
//   No behaviour change for working SHT45 sensors — diagnostics only fire on
//   the failure path. No behaviour change for any other sensor or upload code.
#define VERSION_STR "V2.3.11"
