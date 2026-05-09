#pragma once
// Bump before build; commit after successful flash.
// V2.3.9 — sensor.community SPS30 X-PIN fix: 12 → 1.
//   - V2.3.2 introduced the SPS30 sensor.community POST on X-PIN 12. That was
//     wrong. The authoritative SENSOR_TYPES dict in
//     devices.sensor.community/webapp/default_settings.py lists SPS30 (sensor
//     type ID 37) as PIN 1, NOT PIN 12. Cross-checked against
//     airrohr-firmware/ext_def.h which also defines `SPS30_API_PIN 1`. The
//     PIN 12 value never appeared in any authoritative source — it was a
//     research mistake from the V2.3.2 work that I propagated into the
//     reference_pm_aq_upload_targets memory note as "canonical".
//   - PIN 1 is the SHARED "particulate matter" PIN used by every PM sensor
//     in the sensor.community registry: SDS011 (ID 14), PMS3003/1003/7003/
//     5003/6003 (ID 16, 21-24), HPM (25), SPS30 (37), HM3301 (38),
//     IPS-7100 (41), NextPM (42). The receiving server disambiguates between
//     them by the field-name prefix in the JSON body — that's why we send
//     `SPS30_*` prefixed field names: it tells SC which PM sensor type the
//     PIN 1 POST is from. Sending on PIN 12 with the same prefix would have
//     been silently ignored or 400'd by the API.
//   - No actual data was misrouted: the only live device (Heltec at
//     10.11.12.52) is still on V2.3.5 which predates the SPS30 TX path, and
//     no SPS30 hardware has been wired anyway. So the bug was caught before
//     any cycle posted to a wrong PIN.
//   - The other three SC X-PIN values are confirmed correct: 19 for Si22G,
//     11 for BME280-relabeled SHT45+BMP581 fusion, 15 for DNMS noise.
//   - 1-character fix in transmission.c::send_sensorc, plus comment refresh
//     citing the authoritative source. No size change.
#define VERSION_STR "V2.3.9"
