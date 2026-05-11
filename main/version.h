#pragma once
// Bump before build; commit after successful flash.
//
// V2.3.21 — V1 parity release: boot melody + OLED TX-status wiring.
// Two small features that bring the V2 firmware in line with what the
// original V1 firmware (`Git_Repository_Geiger`) shipped from day one but
// that V2 had quietly dropped during the native-IDF rewrite.
//
//   1. **V1 boot melody when `play_sound=true`.** Replaces V2's "two quick
//      clicks" boot chirp with the original ~3.5 s tune
//      (D-E-F#-G | D-E-D | B-C-B). Replicated note-for-note from
//      `Git_Repository_Geiger/multigeiger/speaker.cpp` — same frequencies
//      (× 0.75 scaling baked in to put the notes in the piezo's sweet
//      spot), same durations (× 85 ms unit per V1's TONE() macro). Final
//      "B" uses single-ended drive (PIN_N held LOW, "volume=0" in V1) for
//      a quieter fade.
//
//      New API in `speaker.h`: `melody_step_t` struct (freq_mhz, volume,
//      duration_ms) and `speaker_play_melody(seq)`. The melody-playback
//      path takes priority over ticks while playing; tick requests during
//      a melody are silently dropped (audio interleave sounds bad and the
//      ear can't distinguish them from the music anyway).
//
//      ~80 LOC in speaker.c. No-op stub for HAL_HAS_SPEAKER=0 (QT Py).
//      Audible on Heltec genuine, Heltec 4 MB knock-off, and FeatherS3-D
//      builds (the latter only if a piezo is wired to A3/A4).
//
//   2. **OLED TX status slots wired (V1 parity).** V2 had scaffolded the
//      sensor.community / Madavi / Radmon status slots in `display.h`
//      since V2.0 but `transmission.c::tx_run` never actually called
//      `display_set_status()` for them — they sat at `.` (off) for the
//      lifetime of the firmware. V1 mirrors what we now do here: set
//      SENDING before each `send_xxx()` call, IDLE/ERROR after based on
//      rc, ERROR on breaker-open, IDLE when the target is enabled but
//      skipped due to no-payload / tube-disabled, OFF when disabled in
//      config.
//
//      The status line (page 7 of the OLED) now flickers through the
//      send sequence on every TX cycle — visible feedback that uploads
//      are happening, and immediate indication when one starts failing.
//      Decode chart for the lowercase/uppercase/digit convention: see
//      `display.c::STATUS_CHARS[]`.
//
//      OSM and aqi.eco intentionally don't get OLED slots — only 5 slots
//      total (WiFi/SC/Madavi/Radmon/HV) and they're all taken. Status of
//      OSM/aqi.eco is fully visible on `/` (V2.3.18 status page) which
//      is the better debugging surface anyway.
//
//      ~25 LOC in transmission.c, plus a one-line `#include "display.h"`.
//      Affects only Heltec genuine + Heltec 4 MB knock-off (the only
//      boards with HAL_HAS_OLED=1). FeatherS3-D and QT Py: display.c
//      stubs out, so calls compile to no-ops.
//
// OTA-safe from V2.3.20 (no partition layout changes, no sdkconfig
// changes). 20 release artefacts (5 × 4 boards).
#define VERSION_STR "V2.3.21"
