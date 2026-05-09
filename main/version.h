#pragma once
// Bump before build; commit after successful flash.
// V2.3.13 — OTA target-board validation.
//   - Before V2.3.13: nothing in our /update handler stopped you from
//     uploading a heltec_v2 .bin to a feathers3_d device (or vice versa).
//     The ESP-IDF bootloader would catch the chip-ID mismatch on next boot
//     and roll back automatically — but that's an ugly recovery: one wasted
//     OTA cycle, a confusing reboot, possible "did the OTA work?" doubt.
//   - V2.3.13 catches it BEFORE commit, returns HTTP 400 with a clear text
//     message in the user's browser, and aborts the OTA cleanly. Bootloader
//     remains as the second line of defence in case our check misses something.
//   - http_server.c::update_post: between esp_ota_end() and esp_ota_set_boot_partition(),
//     read the just-written partition's app descriptor via
//     esp_ota_get_partition_description() and compare its chip_id field to
//     the running chip's actual chip model (ESP_CHIP_ID_ESP32 vs ESP_CHIP_ID_ESP32S3).
//     Also verify project_name == "geiger_v2" — catches "user uploaded random
//     ESP-IDF firmware by mistake".
//   - On mismatch: esp_ota_abort, HTTP 400 with explanation, no commit, no
//     reboot, current firmware keeps running. On match: log the validated
//     project_name + version + build date + chip_id before commit, then proceed.
//   - Doesn't catch: two different ESP32-class boards both running our
//     heltec_v2 build (e.g. WiFi LoRa 32 V2 vs WiFi Kit 32 V2 — both ESP32
//     chips). Out of scope; a board-specific marker string would catch it
//     but the chip_id check is the 80% solution.
#define VERSION_STR "V2.3.13"
