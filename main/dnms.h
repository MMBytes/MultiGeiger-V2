#pragma once

/** @file
 *  @brief hbitter DNMS (Digital Noise Measuring Sensor) driver — pure ESP-IDF I²C.
 *
 *  Hand-rolled implementation of the DNMS host-side wire protocol. The DNMS
 *  hardware is a Teensy 4.0 + I²S MEMS microphone (ICS-43434 or IM72D128)
 *  pre-flashed with hbitter's open-source firmware (https://github.com/hbitter/DNMS).
 *  We only speak its I²C slave protocol — the Teensy firmware is NOT in our
 *  repo, same arrangement as SPS30.
 *
 *  Wire framing matches Sensirion convention (16-bit BE commands, word + CRC8
 *  reads, polynomial 0x31, init 0xFF) — same CRC routine used for SHT45 and
 *  SPS30 in our codebase. Five commands: RESET, READ_VERSION, CALCULATE_LEQ,
 *  READ_DATA_READY, READ_LEQ. Caller-facing struct lives in noise_sensor.h.
 *
 *  Reference: airrohr-firmware/dnms_i2c.h (canonical host implementation;
 *  X-PIN 15 for sensor.community via airrohr-firmware/defines.h DNMS_API_PIN).
 *  See reference_dnms.md memory for full protocol notes.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#include "noise_sensor.h"   // noise_sample_t

/** @brief Probe for DNMS at 0x55, soft-reset, and read+verify version string.
 *
 *  Sequence:
 *    1. i2c_master_probe at 0x55 — bail with ESP_ERR_NOT_FOUND if absent.
 *    2. CMD_RESET (0x0001), wait 1 s for the Teensy to come back up.
 *    3. CMD_READ_VERSION (0x0002), 18-byte response with per-word CRC8.
 *    4. Verify response starts with "DNMS" (4 chars). Mismatch returns
 *       ESP_FAIL — wrong device populated at 0x55.
 *
 *  Returns ESP_OK on success, ESP_ERR_NOT_FOUND if no device responds at
 *  0x55, ESP_FAIL on CRC / version mismatch / I²C error.
 *
 *  Caller is expected to issue dnms_trigger() after init to start the first
 *  LAeq integration window.
 */
esp_err_t dnms_init(i2c_master_bus_handle_t bus);

/** @brief True if init succeeded and the DNMS Teensy responded with a
 *         valid "DNMS..." version string.
 */
bool dnms_present(void);

/** @brief Cached version string from init (NUL-terminated, max 18 chars).
 *
 *  Returns "" when no DNMS is present.
 */
const char *dnms_get_version(void);

/** @brief Send CMD_CALCULATE_LEQ (0x0003) — start a new integration window.
 *
 *  No payload, no response. The Teensy finalises the previous window's
 *  LAeq result internally and starts accumulating a new window. The
 *  finalised result becomes available shortly after — poll dnms_data_ready
 *  to wait for it, then dnms_read_leq to fetch.
 */
esp_err_t dnms_trigger(void);

/** @brief Send CMD_READ_DATA_READY (0x0004), one polled check.
 *
 *  Reads a single 16-bit word + CRC8. Sets *ready to true if the low byte
 *  is non-zero (canonical "result available" flag). Returns ESP_OK on a
 *  good read regardless of ready state; ESP_FAIL on I²C / CRC error.
 *
 *  Caller poll loop: typically 30 ms intervals up to ~10 s timeout.
 */
esp_err_t dnms_data_ready(bool *ready);

/** @brief Send CMD_READ_LEQ (0x0005) and parse 3 IEEE-754 floats.
 *
 *  Returns LAeq, LAmin, LAmax in dB(A) for the most-recently-finalised
 *  window. CRC8 verified on every word — any mismatch returns ESP_FAIL
 *  without populating the output struct.
 *
 *  Wire format: 18 bytes total = 3 floats × 2 words × (2 data + 1 CRC).
 *  Each float is big-endian IEEE 754 split across 2 words.
 */
esp_err_t dnms_read_leq(noise_sample_t *out);
