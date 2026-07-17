#pragma once

/** @file
 *  @brief Pure LoRaWAN payload/hex helpers — no IDF/FreeRTOS/HW.
 *
 *  Header-only, C11-and-C++-compatible (only `<stdint.h>`/`<stdbool.h>`/
 *  `<stddef.h>`/`<string.h>`), modeled on tube_logic.h: the host-side test
 *  runner under `test/` includes it directly and exercises the hex/EUI/
 *  version parsing and the two uplink frame builders on a regular desktop
 *  compiler, with no ESP-IDF toolchain involved.
 *
 *  V2.6.23-dev (T3, LoRaWAN port): consumed by `lorawan.cpp` (C++, RadioLib
 *  EUI/session setup) and `http_server.c` (C, hex parsing for the
 *  LoRaWAN `/config` fields). `lw_build_port1`/`lw_build_port2` must stay
 *  byte-identical to the V1.9 Arduino firmware's TTN uplinks
 *  (Multigeiger_V1.9/.../transmission.cpp `send_ttn_geiger`/`send_ttn_thp`)
 *  so the existing TTN Console payload decoders and the ttn2luft.pdf byte
 *  layout keep working unchanged for V2 nodes.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/** @brief One hex nibble's value, or -1 if @p c is not `0-9`/`a-f`/`A-F`. */
static inline int lw_hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/** @brief Strict hex-string -> byte-buffer decode.
 *
 *  Requires exactly `2*out_len` hex characters — not "at least", not
 *  "up to". LoRaWAN keys/EUIs entered via `/config` are all-or-nothing:
 *  a short/long/garbled string must fail the whole decode rather than
 *  silently truncating or padding, since a partially-decoded 8-byte
 *  session key is far more dangerous than a rejected one.
 *
 *  @param hex      NUL-terminated hex string (case-insensitive).
 *  @param out      Destination buffer, must hold @p out_len bytes.
 *  @param out_len  Expected decoded length in bytes.
 *  @return true iff @p hex is exactly `2*out_len` valid hex chars and
 *          @p out was fully written; false (leaving @p out untouched)
 *          on any length mismatch or non-hex character.
 */
static inline bool lw_hex_decode(const char *hex, uint8_t *out, size_t out_len) {
    if (hex == NULL || out == NULL) return false;
    if (strlen(hex) != out_len * 2) return false;
    for (size_t i = 0; i < out_len; i++) {
        int hi = lw_hex_nibble(hex[2 * i]);
        int lo = lw_hex_nibble(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return true;
}

/** @brief 16 hex chars -> big-endian uint64 EUI (TTN console display order).
 *
 *  The TTN Console shows Dev/Join/App EUIs MSB-first (e.g.
 *  "70B3D57ED0001234"), and RadioLib's `beginOTAA()`/session setters take
 *  EUIs as a plain `uint64_t` in that same MSB-first numeric order — so
 *  this is a straight big-endian byte pack, no reversal needed.
 *
 *  @return false (leaving *out untouched) if @p hex16 isn't exactly 16
 *          valid hex chars.
 */
static inline bool lw_eui_from_hex(const char *hex16, uint64_t *out) {
    if (out == NULL) return false;
    uint8_t b[8];
    if (!lw_hex_decode(hex16, b, sizeof(b))) return false;
    uint64_t v = 0;
    for (size_t i = 0; i < sizeof(b); i++) v = (v << 8) | b[i];
    *out = v;
    return true;
}

/** @brief Parse `"V<major>.<minor>.<patch>"` into the V1.9 packed u16.
 *
 *  Manual digit-by-digit parse (no sscanf) so this stays allocation-free
 *  and host-portable. Layout matches V1.9 transmission.cpp:55-56 exactly
 *  (`sscanf(version, "V%d.%d.%d", ...)` then
 *  `(major<<12)+(minor<<4)+patch`): major in bits 15-12, minor in bits
 *  11-4, patch in bits 3-0.
 *
 *  The patch field is only 4 bits wide (max 15), which V1.9 never hit in
 *  practice — its patch counter reset on every minor bump. V2's patch
 *  numbers run higher (e.g. "V2.6.22") and would silently wrap/alias a
 *  low patch number if truncated with a bare mask; this clamps to 15
 *  instead so an over-15 patch degrades to "as new as this field can
 *  say" rather than aliasing an unrelated earlier patch.
 *
 *  @return 0 for anything that doesn't match the required shape (missing
 *          leading 'V', missing '.', or an empty numeric component) — 0
 *          is never a valid packed version (major 0 has never shipped),
 *          so it doubles as an unambiguous parse-failure sentinel.
 *          Trailing characters after the patch digits (e.g. a future
 *          "-dev" suffix) are ignored, matching sscanf's behaviour.
 */
static inline uint16_t lw_pack_version(const char *version_str) {
    if (version_str == NULL || version_str[0] != 'V') return 0;
    const char *p = version_str + 1;
    unsigned major = 0, minor = 0, patch = 0;
    size_t n;

    n = 0;
    while (*p >= '0' && *p <= '9') { major = major * 10 + (unsigned)(*p - '0'); p++; n++; }
    if (n == 0 || *p != '.') return 0;
    p++;

    n = 0;
    while (*p >= '0' && *p <= '9') { minor = minor * 10 + (unsigned)(*p - '0'); p++; n++; }
    if (n == 0 || *p != '.') return 0;
    p++;

    n = 0;
    while (*p >= '0' && *p <= '9') { patch = patch * 10 + (unsigned)(*p - '0'); p++; n++; }
    if (n == 0) return 0;

    if (patch > 15) patch = 15;   // saturate the 4-bit field, don't wrap
    return (uint16_t)((major << 12) | (minor << 4) | patch);
}

/** @brief Build the 10-byte GM-count uplink (LoRaWAN FPort 1).
 *
 *  Byte-identical to V1.9 `send_ttn_geiger()` (transmission.cpp:218-235):
 *  `gm_counts` as u32 BE, `dt_ms` as u24 BE, `version_packed` as u16 BE,
 *  `tube_nbr` as the last byte.
 *
 *  V1.9's comment on the interval byte says "max ca. 4 hours" but never
 *  enforced it — a longer gap just silently wrapped in the truncating
 *  `& 0xFF` chain. V2 saturates `dt_ms` to the 24-bit field's max
 *  (0xFFFFFF ms, ~4.66h) instead, so an unexpectedly long interval reads
 *  as "as long as this field can say" rather than wrapping to a small,
 *  misleadingly-short one.
 */
static inline void lw_build_port1(uint8_t out[10], uint32_t gm_counts, uint32_t dt_ms,
                                   uint16_t version_packed, uint8_t tube_nbr) {
    out[0] = (uint8_t)((gm_counts >> 24) & 0xFFu);
    out[1] = (uint8_t)((gm_counts >> 16) & 0xFFu);
    out[2] = (uint8_t)((gm_counts >> 8) & 0xFFu);
    out[3] = (uint8_t)(gm_counts & 0xFFu);

    uint32_t dt_clamped = (dt_ms > 0xFFFFFFu) ? 0xFFFFFFu : dt_ms;   // saturate to 24 bits
    out[4] = (uint8_t)((dt_clamped >> 16) & 0xFFu);
    out[5] = (uint8_t)((dt_clamped >> 8) & 0xFFu);
    out[6] = (uint8_t)(dt_clamped & 0xFFu);

    out[7] = (uint8_t)((version_packed >> 8) & 0xFFu);
    out[8] = (uint8_t)(version_packed & 0xFFu);

    out[9] = tube_nbr;
}

/** @brief Build the 5-byte temperature/humidity/pressure uplink (FPort 2).
 *
 *  Byte-identical to V1.9 `send_ttn_thp()` (transmission.cpp:237-244):
 *  `temp_c*10` as s16 BE, `hum_pct*2` truncated to a single byte, and
 *  a pressure field packed as `press_pa/10` as u16 BE.
 *
 *  Pressure-unit finding (V2.6.23-dev T3 verification step): V1.9's
 *  `pressure` argument to `send_ttn_thp()` is in **Pa, not hPa**. It
 *  flows unmodified from `Adafruit_BME280::readPressure()` /
 *  `Adafruit_BME680.pressure` (thp_sensor.cpp:56,64) — both Adafruit
 *  driver APIs return Pascals (sea-level ~101325), never hPa — into
 *  every other transmission path too: the sensor.community/Madavi JSON
 *  bodies send this exact same raw value under `"pressure"` /
 *  `"BME280_pressure"` (transmission.cpp:142-211), and sensor.community's
 *  own `BME280_pressure` field is documented in Pa. So V1.9's
 *  `(int)(pressure / 10)` is `Pa / 10`, e.g. 101325 Pa -> 10132 (not
 *  101325 hPa -> 10132, which would be a physically absurd ~100 bar).
 *  This encoder reproduces that as `press_pa / 10.0f`, unchanged from
 *  the brief's original test vectors — no divisor adjustment was needed.
 *
 *  Unlike V1.9's bare `(int)` cast (which has no overflow guard), this
 *  saturates the packed pressure to the u16 range [0, 65535] since the
 *  field genuinely cannot represent more.
 *
 *  @return false (leaving *out untouched) only if @p out is NULL.
 */
static inline bool lw_build_port2(uint8_t out[5], float temp_c, float hum_pct, float press_pa) {
    if (out == NULL) return false;

    // temp*10 as s16 BE — same formula as V1.9 (`(int)(temperature*10)`),
    // truncated to 16 bits like V1.9's implicit int->int16 byte pack.
    int32_t traw = (int32_t)(temp_c * 10.0f);
    uint16_t traw_u16 = (uint16_t)(int16_t)traw;
    out[0] = (uint8_t)((traw_u16 >> 8) & 0xFFu);
    out[1] = (uint8_t)(traw_u16 & 0xFFu);

    // hum*2 truncated to one byte — V1.9 assigns `int` to `unsigned char`
    // (implicit mod-256, not saturated); reproduced identically here.
    int32_t hraw = (int32_t)(hum_pct * 2.0f);
    out[2] = (uint8_t)(hraw & 0xFF);

    // press_pa/10 as u16 BE, saturating (V1.9 had no such guard).
    float praw_f = press_pa / 10.0f;
    uint16_t praw;
    if (praw_f <= 0.0f)          praw = 0u;
    else if (praw_f >= 65535.0f) praw = 65535u;
    else                         praw = (uint16_t)praw_f;
    out[3] = (uint8_t)((praw >> 8) & 0xFFu);
    out[4] = (uint8_t)(praw & 0xFFu);

    return true;
}
