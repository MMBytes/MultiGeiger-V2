#pragma once
// Bump before build; commit after successful flash.
// V2.3.14 — OTA upload form: board-specific prompt with bold-red board name.
//   - Was: "Select a firmware .bin (from firmware_releases/ or build/geiger_v2.bin)."
//   - Now (heltec_v2 build): "Select a firmware .bin for **Heltec WiFi Kit 32**."
//     (board name in bold + red via inline <b style="color:red">…</b>)
//   - Now (feathers3_d build): "Select a firmware .bin for **FeatherS3**."
//   - Compile-time selection via the existing BOARD_HELTEC_V2 / BOARD_FEATHERS3_D
//     macros from CMakeLists.txt — each .bin is built for exactly one board, so
//     no runtime detection needed. String literal concatenation resolves the
//     board name at compile time, zero runtime cost.
//   - Pairs nicely with V2.3.13's OTA chip-ID validation: the upload form now
//     tells you which binary to pick BEFORE you upload, and V2.3.13 rejects
//     wrong-family uploads if you ignore the prompt.
#define VERSION_STR "V2.3.14"
