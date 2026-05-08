#pragma once
// Bump before build; commit after successful flash.
// V2.3.4 — SPS30 device status surface:
//   - sps30_read_device_status() decodes the 32-bit status register
//     (CMD 0xD206): fan_fail (bit 4), laser_fail (bit 5), fan_speed_warn
//     (bit 21).
//   - Read once per cycle from the worker; ESP_LOGE on hard fault
//     (fan/laser), ESP_LOGW on soft warning (speed drift).
//   - pm_sensor facade caches last sample + last status under a mutex
//     so HTTP handlers can read without touching the I²C bus.
//   - / status page exposes a PM sensor block with colour-coded fan/
//     laser/speed badges and the latest PM readings.
#define VERSION_STR "V2.3.4"
