#pragma once
// Bump before build; commit after successful flash.
// V2.3.1 — Sensirion SPS30 driver:
//   - Hand-rolled pure-IDF I²C driver (no Sensirion library dependency).
//   - Auto-detect probe at 0x69 on the env_sensor-owned I²C bus.
//   - Continuous-measurement mode (fan on, 7-day auto-clean automatic).
//   - All 10 channels read (PM1.0 / PM2.5 / PM4.0 / PM10 + 5×NC + typ size)
//     with CRC8 verification on every word.
//   - Logged but NOT yet transmitted (Madavi/SC integration in V2.3.2,
//     openSenseMap + aqi.eco in V2.3.3).
#define VERSION_STR "V2.3.1"
