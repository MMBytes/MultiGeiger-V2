#pragma once
// Bump before build; commit after successful flash.
// V2.3.8 — Madavi SPS30 visibility hack (SDS_P1 / SDS_P2 alias).
//   - Madavi's data.php is a 2017-vintage backend that only recognises
//     PPD42NS / SDS011 / PMS / HPM / DHT / BMP / BMP280 / BME280 / HTU21D /
//     DS18B20 / GPS / signal field-name prefixes. SPS30_*, Si22G_*,
//     DNMS_noise_* are silently dropped (no matching $has_* checks → no RRD
//     update → no Grafana graph). Confirmed by reading data.php directly
//     (https://github.com/opendata-stuttgart/madavi-api/blob/master/data.php).
//   - Mirror of the existing BME280_* relabel hack for SHT45+BMP581: the
//     Madavi env body now ALSO emits SDS_P1 = SPS30 PM10 and SDS_P2 = SPS30
//     PM2.5. Madavi's $has_sds011 fires, an SDS011-typed RRD is created per
//     device, and our SPS30 PM10/PM2.5 values become visible on Madavi's
//     Grafana dashboards. The full SPS30_* set is still sent alongside —
//     it's harmless on Madavi (silently dropped) and useful diagnostic in
//     the raw POST log.
//   - Strictly Madavi-only: sensor.community continues to receive the proper
//     SPS30 X-PIN 12 POST with SPS30_* fields; openSenseMap and aqi.eco
//     continue to receive the Luftdaten body with SPS30_* only (their
//     parsers handle the prefix natively, and box-config / account routing
//     decides which channels are visible). The Luftdaten body is unchanged.
//   - Field naming follows dusty-code's SDS011 driver convention
//     (src/sensors/sds011/sds011.cpp:524-526) and matches Madavi's
//     data.php:49 detection: `isset($values["SDS_P1"]) && isset($values["SDS_P2"])`.
//   - PM10 maps to SDS_P1 and PM2.5 maps to SDS_P2 — that's the airrohr
//     SDS011 convention, NOT the SPS30 convention (where SPS30_P1 is PM10
//     and SPS30_P2 is PM2.5). Same final values, different field naming.
//   - +~120 bytes per Madavi env POST. Body buffer (1280 B) has plenty of
//     headroom (~300 B free at worst case).
#define VERSION_STR "V2.3.8"
