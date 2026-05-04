#pragma once
// Bump before build; commit after successful flash.
// V2.2.1 — FeatherS3-D bring-up prep:
//   - Pin remap to Feather A0..A5 (left-edge) for cross-Feather portability
//   - External-antenna toggle in /config (FeatherS3-D u.FL on IO41 = HIGH)
// Heltec V2 build is byte-for-byte equivalent to V2.2.0 in observed behaviour.
#define VERSION_STR "V2.2.1"
