#pragma once

/** @file
 *  @brief Wire contract for `GET /api/env` — the peer-environment JSON API.
 *
 *  One module owns BOTH directions of this payload. env_api_format() renders
 *  it; env_api_parse() will read it back (added next, alongside the round-trip
 *  tests), and from build phase 3 `env_peer.c` will consume it. They are
 *  co-located deliberately: this endpoint exists because scraping the
 *  hand-rendered `/` page fails SILENTLY with a plausible float (design §3.1),
 *  and a producer and a consumer that drift apart in separate files
 *  reintroduce exactly that failure mode. Keeping both here will let
 *  `test/test_main.c` assert parse(format(x)) == x.
 *
 *  V2.7.3 lands this in slices: the contract and the formatter first, then the
 *  parser, then the `GET /api/env` handler in `http_server.c`. Until that last
 *  slice nothing calls env_api_format() from firmware — the host tests are its
 *  only caller.
 *
 *  Header-only and pure — no ESP-IDF, no FreeRTOS, no clock, no allocation —
 *  so the host test runner under `test/` includes it directly with no
 *  CMakeLists entry. Same arrangement as `tube_logic.h` and `lorawan_codec.h`.
 *
 *  Payload shape (design §3.2), fixed key order, 123 bytes for a typical
 *  fully-valid sample:
 *
 *      {"id":"esp32-1234567","t":18.42,"h":63.10,"p":101325.00,
 *       "t_ok":true,"h_ok":true,"p_ok":true,
 *       "age_ms":41230,"ts":1755500000}
 *
 *  An invalid or non-finite reading is emitted as JSON `null` with its `*_ok`
 *  flag false — `null` rather than `0.0` because a consumer that forgets to
 *  check the flag then gets a parse failure instead of a plausible zero, which
 *  is the silent-wrongness failure this endpoint exists to avoid.
 */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/** @brief `age_ms` value meaning "this node has never completed a cycle".
 *
 *  Design §3.2's never-sampled sentinel. A freshly booted node's status
 *  snapshot is zero-initialised, so a consumer testing age alone would read
 *  0.0 degC / 0 %RH as a zero-age, perfectly valid sample — both silent and
 *  maximally wrong, the worst misread available here. When this value is
 *  present every `*_ok` is false as well; a consumer must require BOTH.
 */
#define ENV_API_AGE_NEVER 0xFFFFFFFFu

/** @brief Capacity of env_api_t::id, including the NUL.
 *
 *  main.c's `g_chip_id` is `char[20]` holding "esp32-<decimal>". 24 leaves
 *  headroom without making the struct awkward.
 */
#define ENV_API_ID_MAX 24

/** @brief Buffer size that always holds a formatted payload plus its NUL.
 *
 *  Worst case is 159 bytes, from the widest value each field can carry:
 *
 *      {"id":" + 23 id chars + ",     32   (ENV_API_ID_MAX - 1 chars)
 *      "t":-999999.94,                15   (width cap holds floats to 10 chars)
 *      "h":-999999.94,                15
 *      "p":-999999.94,                15
 *      "t_ok":true,                   12
 *      "h_ok":true,                   12
 *      "p_ok":true,                   12
 *      "age_ms":4294967295,           20
 *      "ts":-9223372036854775808      25
 *      }                               1
 *                                    ---
 *                                    159   + 1 for the NUL
 *
 *  `null` (4 chars) is never longer than a float — the narrowest float renders
 *  as `0.00`, also 4 — and `false` (+1) never coincides with a wide float,
 *  because one predicate per field drives BOTH the flag and the value: `false`
 *  always pairs with `null`, never with a number. So an all-valid maximum-width
 *  sample is the true ceiling.
 *  Rounded up to 192. Asserted by test_env_api_format_worst_case_fits().
 */
#define ENV_API_BUF_MIN 192

/** @brief One decoded — or to-be-encoded — `/api/env` payload. */
typedef struct {
    char     id[ENV_API_ID_MAX];   ///< Node identity, e.g. "esp32-1234567"
    float    t;                    ///< Temperature degC       (0 when !t_ok)
    float    h;                    ///< Relative humidity %    (0 when !h_ok)
    float    p;                    ///< Pressure Pa            (0 when !p_ok)
                                   ///<   PASCALS, unconverted — the unit
                                   ///<   main_status_t holds (env_sensor.h:70)
                                   ///<   and the unit mqtt.c:505 and
                                   ///<   lw_build_port2 already publish. Only
                                   ///<   the HTML status page converts to hPa
                                   ///<   (http_server.c:691), because that is a
                                   ///<   human interface. Do NOT add a /100
                                   ///<   here to match it.
    bool     t_ok;                 ///< t carries a real reading
    bool     h_ok;                 ///< h carries a real reading
    bool     p_ok;                 ///< p carries a real reading
    uint32_t age_ms;               ///< ms since the sample, or ENV_API_AGE_NEVER
    int64_t  ts;                   ///< Unix epoch of the sample; 0 = no cycle
                                   ///<   has completed yet. NOT a clock-unset
                                   ///<   flag: main.c stamps this with
                                   ///<   time(NULL) unconditionally, so a node
                                   ///<   that has cycled before NTP sync
                                   ///<   publishes a small 1970-era epoch, not
                                   ///<   0. Never test ts to decide whether the
                                   ///<   clock is set — small values are simply
                                   ///<   meaningless. Humans and logs only: the
                                   ///<   control law uses age_ms, which is
                                   ///<   monotonic and so survives NTP steps
                                   ///<   (design §3.2).
} env_api_t;

// --- Shared predicates -------------------------------------------------------

/** @brief True if @p v can be written as a bounded-width JSON number.
 *
 *  Exactly two jobs, and deliberately not a third. printf("%.2f", NAN) yields
 *  "nan", which is not JSON at all — rejecting non-finite values here makes
 *  that structurally impossible rather than something review has to catch. The
 *  magnitude cap is what holds a formatted float to 10 characters, which is the
 *  figure ENV_API_BUF_MIN's derivation rests on.
 *
 *  It is NOT a plausibility check. Deciding whether a reading is believable
 *  belongs to the consumer that acts on it (vent_logic, from build phase 3);
 *  this layer's job is to carry faithfully whatever the sensor reported. An
 *  earlier draft used a +/-10000 bound justified as "no real environmental
 *  sensor leaves this range" — which was false the moment pressure arrived in
 *  Pascals, and would have nulled every real pressure reading.
 */
static inline bool env_api_sane(float v) {
    return isfinite(v) && fabsf(v) < 1.0e6f;
}

/** @brief True if @p id is NUL-terminated within @p cap, non-empty, and safe
 *         to interpolate into a JSON string without escaping.
 *
 *  The id is hardware-derived ("esp32-<decimal>"), so this cannot fire in
 *  practice. It exists so ENV_API_BUF_MIN's arithmetic is a proof rather than
 *  an assumption, and so env_api_parse()'s "no quote inside a string value"
 *  precondition is enforced at the only place able to violate it.
 */
static inline bool env_api_id_ok(const char *id, size_t cap) {
    for (size_t i = 0; i < cap; i++) {
        char c = id[i];
        if (c == 0) return i > 0;                  // terminated and non-empty
        if (c < 0x20 || c > 0x7E) return false;    // control or non-ASCII
        if (c == '"' || c == '\\') return false;   // would break the JSON string
    }
    return false;   // ran off the end: unterminated
}

// --- Formatter ---------------------------------------------------------------

/** @brief Render one reading as a 2-dp number, or as JSON `null`. */
static inline void env_api_num(char *buf, size_t sz, float v, bool ok) {
    if (ok) snprintf(buf, sz, "%.2f", (double)v);
    else snprintf(buf, sz, "null");
}

/** @brief Render @p in as the `/api/env` JSON payload into @p out.
 *
 *  Never emits `nan`, `inf` or `-nan`: a value failing env_api_sane() is
 *  emitted as `null` with its `*_ok` forced false. The wire flag and the wire
 *  value are derived from the SAME predicate, so a later edit cannot leave
 *  `"t":null` sitting next to `"t_ok":true`.
 *
 *  @return bytes written excluding the NUL, or -1 if @p out or @p in is NULL,
 *          @p outsz is below ENV_API_BUF_MIN, or the id fails env_api_id_ok().
 *          Never truncates — the size floor is what makes that guarantee cheap
 *          to hold, and the caller has a fixed-size buffer anyway.
 */
static inline int env_api_format(char *out, size_t outsz, const env_api_t *in) {
    if (!out || !in || outsz < ENV_API_BUF_MIN) return -1;
    if (!env_api_id_ok(in->id, ENV_API_ID_MAX)) return -1;

    // One predicate per field, feeding BOTH the value and the flag.
    const bool t_ok = in->t_ok && env_api_sane(in->t);
    const bool h_ok = in->h_ok && env_api_sane(in->h);
    const bool p_ok = in->p_ok && env_api_sane(in->p);

    char tb[16], hb[16], pb[16];
    env_api_num(tb, sizeof(tb), in->t, t_ok);
    env_api_num(hb, sizeof(hb), in->h, h_ok);
    env_api_num(pb, sizeof(pb), in->p, p_ok);

    // A single snprintf for the whole payload: no `n += snprintf` accumulation
    // to get wrong, and the ENV_API_BUF_MIN floor above means the result
    // cannot be truncated.
    int n = snprintf(out, outsz,
                     "{\"id\":\"%s\",\"t\":%s,\"h\":%s,\"p\":%s,"
                     "\"t_ok\":%s,\"h_ok\":%s,\"p_ok\":%s,"
                     "\"age_ms\":%lu,\"ts\":%lld}",
                     in->id, tb, hb, pb,
                     t_ok ? "true" : "false",
                     h_ok ? "true" : "false",
                     p_ok ? "true" : "false",
                     (unsigned long)in->age_ms,
                     (long long)in->ts);
    if (n < 0 || (size_t)n >= outsz) return -1;
    return n;
}
