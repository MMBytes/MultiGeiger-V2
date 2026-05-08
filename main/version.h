#pragma once
// Bump before build; commit after successful flash.
// V2.3.6 — BMP581 driver added to the env_sensor probe chain.
//   - Hand-rolled pure-IDF driver (~150 LOC) parallel to bmp390.c. Probes
//     I2C 0x46 (Adafruit #6407 default) then 0x47 (SparkFun default) and
//     binds to whichever responds. Verifies chip ID 0x50; rejects 0x51
//     (BMP585 — different Bosch product).
//   - Operating profile: pressure x16 oversampling, temperature x1, IIR
//     BYPASS, FORCED mode. Matches Bosch's Table 9 "high resolution" preset
//     and ESPHome's documented default. Wait time after FORCED trigger is
//     12 ms (ESPHome's `ceil(1.05 × (tconv_p + tconv_t))` formula at 16x/1x).
//   - BMP581 is highest-priority pressure source in env_sensor_read().
//     Lives at 0x46/0x47 so it can coexist with any 0x77-family chip on the
//     same bus — env_sensor_name() now returns "SHT45+BMP581" when both are
//     fitted. No NVM coefficient fetch needed (BMP5xx silicon ships with
//     factory-trimmed digital output, headline simplification of this
//     generation): pressure = raw / 64.0 Pa, temperature = signed24 / 65536 °C.
//   - DSP_CONFIG (0x30) and DSP_IIR (0x31) reset values are already correct
//     (full P+T compensation, IIR off both channels), so the driver doesn't
//     touch them — saves two register writes per init.
#define VERSION_STR "V2.3.6"
