#pragma once
// Bump before build; commit after successful flash.
// V2.3.7 — DNMS noise sensor support added.
//   - New `main/dnms.[ch]` (~200 LOC) — pure-IDF I²C driver for hbitter's
//     Digital Noise Measuring Sensor. Auto-detects at 0x55. Sensirion-style
//     framing (16-bit BE commands, word + CRC8 reads, poly 0x31 init 0xFF —
//     same CRC routine pattern as SPS30 / SHT45). Five commands:
//     RESET (0x0001), READ_VERSION (0x0002), CALCULATE_LEQ (0x0003),
//     READ_DATA_READY (0x0004), READ_LEQ (0x0005). READ_LEQ returns 3 floats
//     (LAeq, LAmin, LAmax in dB(A)) over 18 wire bytes. Verifies the version
//     string starts with "DNMS" to catch wrong-part populates at 0x55
//     (some Adafruit STEMMA breakouts share that address).
//   - New `main/noise_sensor.[ch]` — facade parallel to pm_sensor / env_sensor.
//     Mutex-protected last-sample cache so HTTP handlers can read without
//     racing the worker on the I²C bus. Trigger / poll-then-read split lets
//     each TX cycle's read return the LAeq for the FULL ~150 s cycle interval
//     (trigger at end of cycle N, read at start of cycle N+1) — same window-
//     spanning pattern as canonical airrohr-firmware.
//   - Hardware: Nettigo NAM DNMS Kit ships with a pre-flashed Teensy 4.0 + I²S
//     MEMS microphone (ICS-43434 or IM72D128). We don't touch the Teensy
//     firmware — same arrangement as SPS30. The Nettigo Teensy is NOT 5 V
//     tolerant, so the DNMS only goes on a 3.3 V I²C bus (our Heltec J_I2C
//     and FeatherS3-D Qwiic both qualify).
//   - Upload paths: 4th sensor.community POST on X-PIN 15 (canonical
//     `DNMS_API_PIN` from airrohr-firmware/defines.h), DNMS_noise_LAeq /
//     _LA_min / _LA_max field naming. Same prefixed names appended to the
//     Madavi env body and to the openSenseMap + aqi.eco Luftdaten body.
//     OSM/aqi body buffers grew 1400→1600 / 1500→1700 to fit the extra fields.
//   - Reference: `reference_dnms.md` memory captures the full protocol notes,
//     X-PIN derivation, and ESPHome cross-check (no DNMS component exists in
//     ESPHome — airrohr-firmware/dnms_i2c.h is the canonical host implementation).
#define VERSION_STR "V2.3.7"
