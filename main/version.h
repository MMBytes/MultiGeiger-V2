#pragma once
// Bump before build; commit after successful flash.
// V2.3.12 — BMP3xx + BMP5xx 10-sample priming after init.
//   - Mirror of dusty-code's mh12 fix (2026-05-08, src/sensors/bmp3xx/bmp3xx.cpp
//     and src/sensors/bmp5xx/bmp5xx.cpp). Bosch's BMP3xx and BMP5xx pressure
//     sensors need their oversampling chain primed with throwaway samples
//     after begin() — the first reading post-init is consistently wrong
//     (~40 hPa low for BMP3xx, similar settle behaviour for BMP5xx) until
//     ~10 measurement cycles flush the OSR averaging chain. Bosch's own KB
//     recommends discarding the first data point. A wall-clock delay() does
//     NOT fix it; only actual measurement cycles flush the chain.
//   - Fix in main/bmp390.c::bmp390_init and main/bmp581.c::bmp581_init: after
//     OSR/IIR/CONFIG writes, mark s_ready = true, then run 10 throwaway
//     bmpXXX_read() calls in a tight loop, then log "ready". Total cost at
//     boot: ~300 ms BMP390 (10 × ~30 ms at OSR_p=32x), ~120 ms BMP581
//     (10 × ~12 ms at OSR_p=16x). Trivial.
//   - Set s_ready=true BEFORE priming (Option A approach): bmpXXX_present()
//     briefly returns true during the ~120-300 ms priming window, but
//     nothing in the codebase asks during that window — env_sensor_init runs
//     sequentially before WiFi/HTTP/TX, and the first real read happens
//     150 s later in cycle #1. Simpler than the s_priming-flag refactor.
//   - BME68x driver intentionally NOT touched per user direction. (Same
//     Bosch family, but dusty didn't patch it and this release scope is
//     "mirror dusty's mh12 fix" exactly.)
//   - Cannot field-verify on the current FeatherS3-D bench — only BME688
//     (not BMP581 or BMP390) is wired there. Build cleanliness verified;
//     priming-loop correctness depends on the dusty empirical fix translating
//     1:1 to our hand-rolled drivers (same OSR/IIR config, same Bosch chips).
#define VERSION_STR "V2.3.12"
