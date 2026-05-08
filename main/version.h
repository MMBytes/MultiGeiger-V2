#pragma once
// Bump before build; commit after successful flash.
// V2.3.3 — openSenseMap + aqi.eco upload targets:
//   - openSenseMap: HTTPS POST to ingress.opensensemap.org/boxes/<BOX>/data
//     ?luftdaten=1, single combined Luftdaten body covering Si22G_* +
//     BME280_* + SPS30_* fields. Box ID configured per-device on
//     opensensemap.org and entered in /config.
//   - aqi.eco: HTTPS POST to api.aqi.eco/update/<TOKEN>, same Luftdaten
//     body wrapped with an esp8266id field at the top. Token comes from
//     the user's aqi.eco account.
//   - Both targets share the new build_luftdaten_body builder; each gets
//     its own circuit breaker + retry counters in tx_run.
//   - /config form gets two new rows under "Transmission targets" with
//     the box ID / token text fields.
#define VERSION_STR "V2.3.3"
