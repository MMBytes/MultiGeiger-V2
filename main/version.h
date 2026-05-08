#pragma once
// Bump before build; commit after successful flash.
// V2.3.0 — Tube optional:
//   - New tube_enabled config flag (default true). When false the Geiger
//     subsystem is fully disabled — no HV/ISR/gptimer setup, tube_read
//     returns zeros, and Madavi/sensor.community/Radmon skip the radiation
//     payload paths (Madavi sends THP only; sensor.community sends X-PIN 11
//     only; Radmon is skipped entirely). Lets the codebase run as a non-
//     Geiger air-quality node when paired with PM/THP sensors.
#define VERSION_STR "V2.3.0"
