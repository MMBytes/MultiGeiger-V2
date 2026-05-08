#pragma once
// Bump before build; commit after successful flash.
// V2.3.2 — SPS30 -> Madavi + sensor.community:
//   - Madavi: SPS30_* fields land in the combined env body alongside
//     BME280_* (renamed from "thp" body for clarity). Single env POST
//     handles every non-radiation source.
//   - sensor.community: new X-PIN 12 POST carrying SPS30_* fields,
//     using the canonical airrohr-firmware SPS30 PIN. Three POSTs total
//     when all sources are live (X-PIN 19 / 11 / 12) sharing one TLS
//     session via keep-alive.
//   - tx_context_t carries pm_valid + pm_sample_t; tx_run gating expands
//     to (tube_enabled || bme_valid || pm_valid).
#define VERSION_STR "V2.3.2"
