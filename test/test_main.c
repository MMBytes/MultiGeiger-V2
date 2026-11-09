// Host-side unit tests for the pure helpers under main/, plus (V2.8.8+)
// firmware .c files compiled unchanged through test/shim/.
//
// Run on a regular host C compiler (gcc on Linux/CI, MSYS2/MinGW or
// WSL on Windows) — no ESP-IDF, no FreeRTOS, no hardware needed.
//
// Build + run locally: `_test.cmd [filter]` from the repo root (Windows) or
// `gcc -I test/shim -I main -DBOARD_HELTEC_V2=1 -Wall -Wextra -Werror -std=c11
// -o test/run test/test_main.c main/history.c && ./test/run [filter]`
// (Linux/macOS) — the flags and sources are HOST_TEST_CFLAGS / HOST_TEST_SRCS
// in .github/workflows/_host-test.yml, which CI runs four ways (native,
// valgrind, ASan+UBSan, -m32) for both build.yml and release.yml.
//
// Scope: the pure, header-only modules — main/util.h, main/tube_logic.h,
// main/lorawan_codec.h, (V2.7.3+) main/env_api.h and (V2.8.0+) main/tls_logic.h —
// and (V2.8.8+) main/history.c, whose FreeRTOS / esp_log includes resolve to
// test/shim/ and whose tube counters are faked in this file. The settings
// schema main/config_fields.def is checked as data (review 2.1 C3a): NVS key
// rules, bounds, defaults, and its match with the /config form in
// main/http_server.c, read as text (so run from the repo root).
// Anything else that depends on IDF / FreeRTOS / hardware (NVS, HTTP handlers,
// sensor drivers, transmission orchestrator) is out of scope — would need
// on-target Unity or QEMU, or a shim of its own. Note the boundary this draws for env_api.h: it covers the wire
// format on both sides (format, parse, round-trip), but NEVER the handler
// that fills the struct from main_status_t, so a unit mix-up there is
// bench-caught, not test-caught.
//
// Convention: each test_xxx() returns 1 on pass, 0 on fail. The runner
// prints PASS/FAIL with the test name and tallies the failures. Non-
// zero exit code if any test failed (so CI marks the job red).

#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util.h"
#include "tube_logic.h"   // clamp_u32, gmc_classify, guard_effective_us
#include "lorawan_codec.h"   // lw_hex_decode, lw_eui_from_hex, lw_pack_version, lw_build_port1/2
#include "lorawan_sched.h"   // V2.8.14: lw_sched_interval_ms / _port2_due / _fixed_dr / _frame_is_ours
#include "lorawan_rxobs.h"   // V2.8.21: RX-window observer + rx windows / uplink OK text
#include "lorawan_boot_line.h" // V2.8.21: "boot: LoRaWAN:" summary line
#include "lorawan_pa.h"      // V2.8.22: Heltec V4 PA gain RadioLib subtracts
#include "lorawan_mac.h"     // V2.8.23: MAC commands + answers in plain words
#include "radio_turn_rule.h" // V2.8.15: rt_lorawan/ftps/tx_round_may_start
#include "env_api.h"         // env_api_format (/api/env wire contract; parse follows)
#include "speaker_logic.h"   // tick_defer_request/poll/clear (V2.7.4 tick defer)
#include "tls_logic.h"       // fingerprint_hex, https_redirect_location, civil_to_epoch (V2.8.0 HTTPS)
#include "history.h"         // V2.8.8: history.c compiled in unchanged (test/shim + fakes below)
#include "tube.h"            // V2.8.8: real prototypes, so the fakes below must match them
#include "tube_pcnt.h"
#include "ota_guard_logic.h"  // V2.8.20: OTA rollback probation rule

static int g_failures = 0;

// V2.8.8 (review 2.1 A): every failure message names the source line and the
// expression that failed, so a FAIL in a long test points at the exact check
// instead of only the test function. EXPECT_INT compares as long long (was
// long): on the ILP32 -m32 leg and on 64-bit Windows `long` is 32 bits, and a
// uint32_t value above INT32_MAX must not wrap negative before the compare.
#define EXPECT_STREQ(actual, expected)                                       \
    do {                                                                     \
        if (strcmp((actual), (expected)) != 0) {                             \
            printf("    line %d: EXPECT_STREQ(%s, %s)\n"                     \
                   "      expected '%s', got '%s'\n",                        \
                   __LINE__, #actual, #expected, (expected), (actual));      \
            return 0;                                                        \
        }                                                                    \
    } while (0)

#define EXPECT_INT(actual, expected)                                         \
    do {                                                                     \
        long long _a = (long long)(actual);                                  \
        long long _e = (long long)(expected);                                \
        if (_a != _e) {                                                      \
            printf("    line %d: EXPECT_INT(%s, %s)\n"                       \
                   "      expected %lld, got %lld\n",                        \
                   __LINE__, #actual, #expected, _e, _a);                    \
            return 0;                                                        \
        }                                                                    \
    } while (0)

// V2.8.8: boolean check with the same line/expression report, for conditions
// that are not a value compare (e.g. "no raw '<' in the output").
#define EXPECT_TRUE(cond)                                                    \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("    line %d: EXPECT_TRUE(%s) failed\n", __LINE__, #cond);\
            return 0;                                                        \
        }                                                                    \
    } while (0)

// V2.7.3: float compare for the env_api wire values. Absolute tolerance, not
// relative — the payload carries 2-dp fixed-point, so the error budget is a
// constant half-centi-unit regardless of magnitude.
#define EXPECT_FLOAT_NEAR(actual, expected, tol)                          \
    do {                                                                  \
        double _a = (double)(actual);                                     \
        double _e = (double)(expected);                                   \
        double _d = _a - _e;                                              \
        if (_d < 0) _d = -_d;                                             \
        if (_d > (double)(tol)) {                                         \
            printf("    line %d: EXPECT_FLOAT_NEAR(%s, %s, %s)\n"         \
                   "      expected %.4f +/- %.4f, got %.4f\n",            \
                   __LINE__, #actual, #expected, #tol,                    \
                   _e, (double)(tol), _a);                                \
            return 0;                                                     \
        }                                                                 \
    } while (0)

// V2.7.3: format into `buf` and fail the calling test if the formatter
// rejected the input. The cases below probe the rendered payload with
// strstr(), and env_api_format()'s reject paths return before writing a single
// byte — so a regression that made one of these inputs invalid would leave the
// test reading an uninitialised stack buffer. That surfaces in CI as a
// valgrind uninitialised-read (the host-test job runs
// --errors-for-leak-kinds=all) plus a garbage-dependent strstr result, exactly
// when a clean named FAIL is most useful. Check the return instead.
#define EXPECT_FORMAT_OK(buf, e)                                             \
    do {                                                                     \
        int _n = env_api_format((buf), sizeof(buf), &(e));                   \
        if (_n < 0) {                                                        \
            printf("    line %d: env_api_format rejected the input "         \
                   "(returned -1)\n", __LINE__);                             \
            return 0;                                                        \
        }                                                                    \
    } while (0)

// ----------------------------------------------------------------------------
// safe_strcpy
// ----------------------------------------------------------------------------

static int test_safe_strcpy_basic(void) {
    char dst[16] = "XXXXXXXXXXXXXXX";
    safe_strcpy(dst, "hello", sizeof(dst));
    EXPECT_STREQ(dst, "hello");
    return 1;
}

static int test_safe_strcpy_truncates(void) {
    char dst[6] = {0};
    safe_strcpy(dst, "hello world", sizeof(dst));
    EXPECT_STREQ(dst, "hello");   // 5 chars + NUL
    EXPECT_INT(dst[5], 0);
    return 1;
}

static int test_safe_strcpy_empty_src(void) {
    char dst[8] = "XXX";
    safe_strcpy(dst, "", sizeof(dst));
    EXPECT_STREQ(dst, "");
    return 1;
}

static int test_safe_strcpy_zero_dstsz(void) {
    // Must not write anything (no buffer to terminate).
    char dst[4] = "abc";
    safe_strcpy(dst, "X", 0);
    EXPECT_STREQ(dst, "abc");   // unmodified
    return 1;
}

static int test_safe_strcpy_dstsz_1(void) {
    // Only room for the NUL.
    char dst[4] = "abc";
    safe_strcpy(dst, "hello", 1);
    EXPECT_INT(dst[0], 0);
    return 1;
}

// V2.7.3: the implementation stopped using strncpy (GCC 16 raises
// -Wstringop-truncation on it). strncpy zero-PADS the whole remainder of the
// destination, and callers such as main.c's wifi_config_t copies hand the
// buffer straight to an IDF driver, so that padding is behaviour, not an
// accident. Pin it rather than trusting the rewrite to have preserved it.
static int test_safe_strcpy_zero_fills_tail(void) {
    char dst[16];
    memset(dst, 'Z', sizeof(dst));   // poison every byte
    safe_strcpy(dst, "abc", sizeof(dst));
    EXPECT_STREQ(dst, "abc");
    for (size_t i = 3; i < sizeof(dst); i++) {
        if (dst[i] != 0) {
            printf("    dst[%u] = 0x%02X, expected 0x00\n",
                   (unsigned)i, (unsigned char)dst[i]);
            return 0;
        }
    }
    // Exact-fit truncation: 15 chars + the forced NUL, no byte left poisoned.
    memset(dst, 'Z', sizeof(dst));
    safe_strcpy(dst, "0123456789abcdefghij", sizeof(dst));
    EXPECT_STREQ(dst, "0123456789abcde");
    EXPECT_INT(dst[15], 0);
    return 1;
}

// ----------------------------------------------------------------------------
// ct_memcmp
// ----------------------------------------------------------------------------

static int test_ct_memcmp_equal(void) {
    const uint8_t a[] = { 0x12, 0x34, 0x56, 0x78 };
    const uint8_t b[] = { 0x12, 0x34, 0x56, 0x78 };
    EXPECT_INT(ct_memcmp(a, b, sizeof(a)), 0);
    return 1;
}

static int test_ct_memcmp_diff_first_byte(void) {
    const uint8_t a[] = { 0x00, 0x34, 0x56, 0x78 };
    const uint8_t b[] = { 0x12, 0x34, 0x56, 0x78 };
    int r = ct_memcmp(a, b, sizeof(a));
    if (r == 0) { printf("    expected non-zero\n"); return 0; }
    return 1;
}

static int test_ct_memcmp_diff_last_byte(void) {
    const uint8_t a[] = { 0x12, 0x34, 0x56, 0x00 };
    const uint8_t b[] = { 0x12, 0x34, 0x56, 0x78 };
    int r = ct_memcmp(a, b, sizeof(a));
    if (r == 0) { printf("    expected non-zero\n"); return 0; }
    return 1;
}

static int test_ct_memcmp_zero_length(void) {
    // Zero-length compare must return 0 (equal) regardless of buffers.
    const uint8_t a[] = { 0xFF };
    const uint8_t b[] = { 0x00 };
    EXPECT_INT(ct_memcmp(a, b, 0), 0);
    return 1;
}

static int test_ct_memcmp_realistic_base64(void) {
    // The actual auth-compare use case from http_server.c: two base64
    // strings of the same length, differing in one byte.
    const char *a = "YWRtaW46RVNQMzJHZWlnZXI=";
    const char *b = "YWRtaW46RVNQMzJHZWlnZVI=";   // differs at index 21 (e/E)
    int r = ct_memcmp(a, b, strlen(a));
    if (r == 0) { printf("    expected non-zero for differing creds\n"); return 0; }
    EXPECT_INT(ct_memcmp(a, a, strlen(a)), 0);   // same buffer = 0
    return 1;
}

// ----------------------------------------------------------------------------
// hex_nibble
// ----------------------------------------------------------------------------

static int test_hex_nibble_digits(void) {
    for (int i = 0; i < 10; i++) {
        char c = (char)('0' + i);
        EXPECT_INT(hex_nibble(c), i);
    }
    return 1;
}

static int test_hex_nibble_lower(void) {
    EXPECT_INT(hex_nibble('a'), 10);
    EXPECT_INT(hex_nibble('f'), 15);
    return 1;
}

static int test_hex_nibble_upper(void) {
    EXPECT_INT(hex_nibble('A'), 10);
    EXPECT_INT(hex_nibble('F'), 15);
    return 1;
}

static int test_hex_nibble_invalid(void) {
    EXPECT_INT(hex_nibble('g'), -1);
    EXPECT_INT(hex_nibble('G'), -1);
    EXPECT_INT(hex_nibble('/'), -1);
    EXPECT_INT(hex_nibble(':'), -1);
    EXPECT_INT(hex_nibble(0), -1);
    EXPECT_INT(hex_nibble(' '), -1);
    return 1;
}

// ----------------------------------------------------------------------------
// url_decode
// ----------------------------------------------------------------------------

static int test_url_decode_plain(void) {
    char s[] = "hello";
    url_decode(s);
    EXPECT_STREQ(s, "hello");
    return 1;
}

static int test_url_decode_plus_to_space(void) {
    char s[] = "hello+world+again";
    url_decode(s);
    EXPECT_STREQ(s, "hello world again");
    return 1;
}

static int test_url_decode_percent_hex(void) {
    char s[] = "a%20b%2Fc%3D";   // " ", "/", "="
    url_decode(s);
    EXPECT_STREQ(s, "a b/c=");
    return 1;
}

static int test_url_decode_lowercase_hex(void) {
    char s[] = "a%2fb%3dc";
    url_decode(s);
    EXPECT_STREQ(s, "a/b=c");
    return 1;
}

static int test_url_decode_mixed(void) {
    char s[] = "name=John+Doe&val=100%25";
    url_decode(s);
    EXPECT_STREQ(s, "name=John Doe&val=100%");
    return 1;
}

static int test_url_decode_malformed_drops_percent(void) {
    // V2.4.1 (B6) behaviour: malformed %XY drops the lone %, keeps XY.
    char s[] = "hello%G5world";
    url_decode(s);
    EXPECT_STREQ(s, "helloG5world");
    return 1;
}

static int test_url_decode_truncated_percent_2(void) {
    // V2.4.1 (B6): %2 with no following hex char → drop %, keep "2".
    char s[] = "abc%2";
    url_decode(s);
    EXPECT_STREQ(s, "abc2");
    return 1;
}

static int test_url_decode_lone_percent_at_end(void) {
    // V2.4.1 (B6): % at very end → drop entirely.
    char s[] = "abc%";
    url_decode(s);
    EXPECT_STREQ(s, "abc");
    return 1;
}

static int test_url_decode_empty(void) {
    char s[] = "";
    url_decode(s);
    EXPECT_STREQ(s, "");
    return 1;
}

static int test_url_decode_only_plus(void) {
    char s[] = "+++";
    url_decode(s);
    EXPECT_STREQ(s, "   ");
    return 1;
}

static int test_url_decode_zero_byte(void) {
    // %00 should decode to a NUL — and that NUL becomes the new string
    // terminator. Anything after it is ignored by strcmp but the bytes
    // are still written.
    char s[] = "abc%00def";
    url_decode(s);
    EXPECT_STREQ(s, "abc");   // strcmp stops at the NUL we wrote
    EXPECT_INT(s[3], 0);
    EXPECT_INT(s[4], 'd');    // d was still written past the NUL
    return 1;
}

// V2.4.1+ (T4-lite): property-based fuzz. Generates ~10k random byte
// strings with a bias toward `%` and `+` (the special-handling chars)
// and asserts three invariants of url_decode:
//   (1) output length ≤ input length (decoding never expands)
//   (2) guard bytes immediately before/after the working buffer are
//       untouched (no out-of-bounds write)
//   (3) the decoder terminates (no infinite loop) for any input
// Seeded with a fixed PRNG state so a failure is reproducible.
// Worth the ~50 ms per CI run as an automated catch for the class of
// bug where some new edge-case input breaks the parser.
static int test_url_decode_fuzz_invariants(void) {
    srand(0xDEADBEEF);
    // enum constants (not `const size_t`) so they're integer constant
    // expressions usable in array dimensions without VLA semantics.
    enum {
        FUZZ_N       = 10000,
        FUZZ_MAX_LEN = 256,
        FUZZ_GUARD   = 8,
        FUZZ_CANARY  = 0xAB,
    };

    // Layout: [GUARD][work, MAX_LEN bytes][GUARD]
    uint8_t buf[FUZZ_GUARD + FUZZ_MAX_LEN + FUZZ_GUARD];
    char   *work = (char *)(buf + FUZZ_GUARD);

    for (size_t i = 0; i < FUZZ_N; i++) {
        memset(buf, FUZZ_CANARY, sizeof(buf));

        // Random length 0..MAX_LEN-1 (leave room for the NUL we'll add).
        size_t len = (size_t)rand() % (FUZZ_MAX_LEN - 1);

        for (size_t j = 0; j < len; j++) {
            int r = rand() & 0xFF;
            char c;
            // Heavy bias toward special chars + ambiguous hex-like input
            // to maximise the chance of hitting parser corner cases.
            if      (r < 32)  c = '%';
            else if (r < 64)  c = '+';
            else if (r < 96)  c = (char)('0' + (r & 0x0F));   // includes :;<=>? near top
            else if (r < 128) c = (char)('a' + (r % 6));      // a..f hex
            else if (r < 160) c = (char)('A' + (r % 6));      // A..F hex
            else if (r < 200) c = (char)('g' + (r % 8));      // non-hex letters
            else              c = (char)(0x20 + (r & 0x3F));  // printable mix
            if (c == 0) c = 'x';   // never plant a mid-string NUL
            work[j] = c;
        }
        work[len] = 0;

        size_t input_len = strlen(work);
        url_decode(work);
        size_t output_len = strlen(work);

        // Invariant 1: never expand.
        if (output_len > input_len) {
            printf("    fuzz iter %zu: output_len=%zu > input_len=%zu\n",
                   i, output_len, input_len);
            return 0;
        }

        // Invariant 2: guard bytes untouched.
        for (size_t k = 0; k < FUZZ_GUARD; k++) {
            if (buf[k] != FUZZ_CANARY) {
                printf("    fuzz iter %zu: pre-guard corrupted at offset %zu\n",
                       i, k);
                return 0;
            }
            if (buf[FUZZ_GUARD + FUZZ_MAX_LEN + k] != FUZZ_CANARY) {
                printf("    fuzz iter %zu: post-guard corrupted at offset %zu\n",
                       i, k);
                return 0;
            }
        }
        // Invariant 3 (termination) is implicit — if we got here, we didn't
        // hang or crash. A real infinite-loop bug would time out the CI job.
    }
    printf("    %d fuzz iterations OK\n", FUZZ_N);
    return 1;
}

// ----------------------------------------------------------------------------
// html_esc
// ----------------------------------------------------------------------------

static int test_html_esc_plain(void) {
    char out[32] = {0};
    html_esc("hello", out, sizeof(out));
    EXPECT_STREQ(out, "hello");
    return 1;
}

static int test_html_esc_ampersand(void) {
    char out[32] = {0};
    html_esc("A&B", out, sizeof(out));
    EXPECT_STREQ(out, "A&amp;B");
    return 1;
}

static int test_html_esc_quote(void) {
    char out[32] = {0};
    html_esc("A\"B", out, sizeof(out));
    EXPECT_STREQ(out, "A&quot;B");
    return 1;
}

static int test_html_esc_angles(void) {
    char out[32] = {0};
    html_esc("<b>x</b>", out, sizeof(out));
    EXPECT_STREQ(out, "&lt;b&gt;x&lt;/b&gt;");
    return 1;
}

static int test_html_esc_all_metas(void) {
    char out[64] = {0};
    html_esc("&<>\"", out, sizeof(out));
    EXPECT_STREQ(out, "&amp;&lt;&gt;&quot;");
    return 1;
}

static int test_html_esc_empty(void) {
    char out[8] = "XXXXXXX";
    html_esc("", out, sizeof(out));
    EXPECT_STREQ(out, "");
    return 1;
}

static int test_html_esc_truncates_safely(void) {
    // bufsz=11 with "AAAA&BBBB". Plain "AAAA" fits (4 bytes + NUL),
    // then the loop's conservative `o + 7 < bufsz` guard (sized for
    // the worst-case 6-char escape `&quot;` + NUL) refuses to start
    // the `&amp;` expansion at o=4 because 4+7 = 11 = bufsz, not <.
    // Result: out[0..3] = "AAAA", out[4] = '\0'. Function never
    // writes past out[bufsz-1] for any non-zero bufsz.
    char out[11];
    memset(out, 'X', sizeof(out));
    html_esc("AAAA&BBBB", out, sizeof(out));
    EXPECT_STREQ(out, "AAAA");
    EXPECT_INT(out[4], 0);
    return 1;
}

static int test_html_esc_tiny_buffer(void) {
    // bufsz=1: loop condition `0 + 7 < 1` is FALSE immediately, so the
    // loop body never runs. out[0] = 0. Result: empty string.
    char out[1];
    out[0] = 'X';
    html_esc("hello", out, sizeof(out));
    EXPECT_INT(out[0], 0);
    return 1;
}

// ----------------------------------------------------------------------------
// url_encode_query_value  (V2.5.20 R2 — was shipped with NO regression test)
// ----------------------------------------------------------------------------

static int test_urlenc_unreserved_passthrough(void) {
    // RFC 3986 §2.3 unreserved set passes through verbatim.
    char out[80] = {0};
    url_encode_query_value(out, sizeof(out),
                           "AZaz09-._~");
    EXPECT_STREQ(out, "AZaz09-._~");
    return 1;
}

static int test_urlenc_reserved_chars(void) {
    // The exact chars that broke the raw Radmon submit URL (& = + %) plus space.
    char out[80] = {0};
    url_encode_query_value(out, sizeof(out), "a&b=c+d%e f");
    EXPECT_STREQ(out, "a%26b%3Dc%2Bd%25e%20f");
    return 1;
}

static int test_urlenc_empty(void) {
    char out[8] = "XXXXXXX";
    url_encode_query_value(out, sizeof(out), "");
    EXPECT_STREQ(out, "");
    return 1;
}

static int test_urlenc_high_byte(void) {
    // Bytes >= 0x80 must encode as upper-case hex %XX (cast-to-unsigned path).
    char out[16] = {0};
    char src[2] = { (char)0xC3, 0 };
    url_encode_query_value(out, sizeof(out), src);
    EXPECT_STREQ(out, "%C3");
    return 1;
}

static int test_urlenc_truncates_on_full_buffer(void) {
    // dstsz too small to hold a full %XX escape → stop early, stay terminated,
    // never write past dst[dstsz-1]. "&" needs 3 bytes; with dstsz=3 the
    // `o + 3 >= dstsz` guard refuses to start it, so output is empty.
    char out[3] = { 'Z', 'Z', 'Z' };
    url_encode_query_value(out, sizeof(out), "&");
    EXPECT_STREQ(out, "");
    EXPECT_INT(out[0], 0);
    return 1;
}

static int test_urlenc_partial_then_truncate(void) {
    // "ab&" into dstsz=4: 'a','b' fit (o=2), then '&' needs o+3=5 >= 4 → stop.
    char out[4];
    memset(out, 'X', sizeof(out));
    url_encode_query_value(out, sizeof(out), "ab&");
    EXPECT_STREQ(out, "ab");
    EXPECT_INT(out[2], 0);
    return 1;
}

static int test_urlenc_zero_dstsz(void) {
    // dstsz == 0 must be a no-op (no buffer to terminate).
    char out[2] = { 'Q', 'Q' };
    url_encode_query_value(out, 0, "hello");
    EXPECT_INT(out[0], 'Q');   // untouched
    return 1;
}

// ----------------------------------------------------------------------------
// clamp_u32  (V2.5.31 — saturating uint64->uint32 µs-delta cast, tube_logic.h)
// ----------------------------------------------------------------------------

// V2.8.8: expected values are plain UINT32_MAX, no longer (long)UINT32_MAX —
// where `long` is 32 bits (64-bit Windows, the ILP32 -m32 leg, the target)
// that cast wrapped to -1, and the old long-typed EXPECT_INT wrapped the
// actual value identically, so the compare only matched by accident.
static int test_clamp_u32_below_max(void) {
    EXPECT_INT(clamp_u32(0), 0);
    EXPECT_INT(clamp_u32(190), 190);
    EXPECT_INT(clamp_u32(UINT32_MAX - 1), UINT32_MAX - 1);
    return 1;
}

static int test_clamp_u32_at_max(void) {
    EXPECT_INT(clamp_u32((uint64_t)UINT32_MAX), UINT32_MAX);
    return 1;
}

static int test_clamp_u32_above_max_saturates(void) {
    // The >71.6-min wrap case: a bare (uint32_t) cast would wrap to a small
    // value; clamp_u32 must saturate to UINT32_MAX so the edge reads as
    // "maximally separated" and never spuriously trips the dead-time guard.
    EXPECT_INT(clamp_u32((uint64_t)UINT32_MAX + 1), UINT32_MAX);
    EXPECT_INT(clamp_u32((uint64_t)UINT32_MAX + 1000000), UINT32_MAX);
    EXPECT_INT(clamp_u32(0xFFFFFFFFFFFFFFFFull), UINT32_MAX);
    return 1;
}

// ----------------------------------------------------------------------------
// gmc_classify  (V2.5.30 dead-time-guard decision, extracted V2.5.31)
// ----------------------------------------------------------------------------

static int test_gmc_guard_off_counts_when_past_gate(void) {
    // guard_us==0 disables the guard: past_gate decides count vs reject.
    EXPECT_INT(gmc_classify(100, true,  0), GMC_COUNT);
    EXPECT_INT(gmc_classify(100, false, 0), GMC_REJECT);
    return 1;
}

static int test_gmc_first_edge_never_guarded(void) {
    // First-ever edge: edt == UINT32_MAX → never <= any guard window → counts.
    EXPECT_INT(gmc_classify(UINT32_MAX, true, 3000), GMC_COUNT);
    return 1;
}

static int test_gmc_guard_blocks_inside_window(void) {
    // Edge inside the guard window AND past the 190µs gate = a real count the
    // guard cost us → GUARD_REMOVED (the guard's true marginal effect).
    EXPECT_INT(gmc_classify(2000, true, 3000), GMC_GUARD_REMOVED);
    // Boundary: edt == guard_us is INSIDE the window (<=), so still blocked.
    EXPECT_INT(gmc_classify(3000, true, 3000), GMC_GUARD_REMOVED);
    return 1;
}

static int test_gmc_guard_block_subgate_is_reject_not_removed(void) {
    // Inside the guard window but NOT past the gate (sub-190µs afterpulse): the
    // fixed gate already owns it, so it's a plain REJECT, not GUARD_REMOVED —
    // this is exactly the "don't double-credit" fix from review #1.
    EXPECT_INT(gmc_classify(120, false, 3000), GMC_REJECT);
    return 1;
}

static int test_gmc_outside_window_counts(void) {
    // edt beyond the guard window and past the gate → normal count.
    EXPECT_INT(gmc_classify(3001, true, 3000), GMC_COUNT);
    return 1;
}

// ----------------------------------------------------------------------------
// guard_effective_us  (V2.5.30 mutual-exclusion policy; pcnt_filter wins)
// ----------------------------------------------------------------------------

static int test_guard_eff_disabled_is_zero(void) {
    EXPECT_INT(guard_effective_us(false, false, 3000), 0);
    return 1;
}

static int test_guard_eff_pcnt_wins(void) {
    // Enabled but pcnt_filter on → guard suppressed (pcnt is authoritative).
    EXPECT_INT(guard_effective_us(true, true, 3000), 0);
    return 1;
}

static int test_guard_eff_enabled_returns_window(void) {
    EXPECT_INT(guard_effective_us(true, false, 3000), 3000);
    EXPECT_INT(guard_effective_us(true, false, 200), 200);
    return 1;
}

// ----------------------------------------------------------------------------
// hv_blank_hit  (V2.6.29 HV blanking window — phantom-pulse suppression)
// ----------------------------------------------------------------------------

static int test_blank_off_never_hits(void) {
    // blank_us==0 disables the window — even a gap of 0 (edge exactly at the
    // FET turn-off instant) must not be blanked.
    EXPECT_INT(hv_blank_hit(0, 0), 0);
    EXPECT_INT(hv_blank_hit(10, 0), 0);
    return 1;
}

static int test_blank_hits_inside_window(void) {
    // The measured phantom delay band (3-16µs after FET turn-off) falls inside
    // the default 30µs window. Boundary: gap == blank_us is INSIDE (<=), same
    // convention as the dead-time guard's edt <= guard_us.
    EXPECT_INT(hv_blank_hit(10, 30), 1);
    EXPECT_INT(hv_blank_hit(3, 30), 1);
    EXPECT_INT(hv_blank_hit(30, 30), 1);
    return 1;
}

static int test_blank_misses_outside_window(void) {
    // A genuine tube pulse arriving after the window must count normally.
    EXPECT_INT(hv_blank_hit(31, 30), 0);
    EXPECT_INT(hv_blank_hit(1000000, 30), 0);
    return 1;
}

static int test_blank_no_hv_pulse_yet_never_hits(void) {
    // Before the first HV pulse the ISR passes UINT32_MAX (same first-edge
    // convention as edt), which can never be <= any window in range.
    EXPECT_INT(hv_blank_hit(UINT32_MAX, 1000), 0);
    return 1;
}

// ----------------------------------------------------------------------------
// tube_count_pin_for  (V2.8.10 Rev C V2/V3 carrier select)
// ----------------------------------------------------------------------------

static int test_count_pin_for_carrier(void) {
    // 3 selects the V3 pad; every other value, including garbage that could
    // only come from a corrupted NVS byte, keeps the V2 pin every deployed
    // carrier uses. A board without a V3 pin (-1) never moves.
    EXPECT_INT(tube_count_pin_for(2, 14, 7), 14);
    EXPECT_INT(tube_count_pin_for(3, 14, 7), 7);
    EXPECT_INT(tube_count_pin_for(3, 14, -1), 14);
    EXPECT_INT(tube_count_pin_for(0, 14, 7), 14);
    EXPECT_INT(tube_count_pin_for(255, 14, 7), 14);
    EXPECT_INT(tube_count_pin_for(3, 7, 44), 44);   // XIAO ESP32-S3 pair
    return 1;
}

static int test_blank_eff_disabled_is_zero(void) {
    EXPECT_INT(blank_effective_us(false, false, 30), 0);
    EXPECT_INT(blank_effective_us(false, true, 30), 0);
    return 1;
}

static int test_blank_eff_pcnt_composes(void) {
    // THE policy pin of the V2.6.29 subtract-mode rework (Fable review
    // MINOR-2): unlike the guard, blanking is NOT suppressed by pcnt_filter —
    // it composes via the subtract mode. The first V2.6.29 cut routed
    // config_effective_blank_us through guard_effective_us (pcnt-superseded);
    // a regression back to that policy fails this case.
    EXPECT_INT(blank_effective_us(true, true, 30), 30);
    EXPECT_INT(blank_effective_us(true, false, 30), 30);
    return 1;
}

// ----------------------------------------------------------------------------
// pcnt_blank_wide  (V2.6.31 width-aware subtract — phantom width is
// temperature-dependent, see tube_logic.h; numbers below are real field-node cycles)
// ----------------------------------------------------------------------------

static int test_bwide_cool_phantoms_wide(void) {
    // Cool tube: width filter removed nothing, all 18 phantoms passed it and
    // sit inside the PCNT count → subtract all 18 (V2.6.29 behavior preserved).
    // (field-node night cycle: pcnt=230, isr=212, blanked=18 → removed 0 → wide 18.)
    EXPECT_INT(pcnt_blank_wide(230, 212, 18), 18);
    return 1;
}

static int test_bwide_warm_phantoms_narrow(void) {
    // Warm tube: phantoms shrank under the 4 µs tooth, the width filter
    // already dropped all 18 → nothing left in pcnt to subtract. This is the
    // double-subtract case the V2.6.29 code got wrong (−6 CPM warm afternoons).
    // (field-node afternoon cycle: pcnt=212, isr=212, blanked=18 → removed 18 → wide 0.)
    EXPECT_INT(pcnt_blank_wide(212, 212, 18), 0);
    return 1;
}

static int test_bwide_mixed_proportional(void) {
    // Shoulder-hours cycle: 8 of 18 phantoms narrow (removed), 10 still wide.
    EXPECT_INT(pcnt_blank_wide(222, 212, 18), 10);
    return 1;
}

static int test_bwide_removed_exceeds_blanked_clamps(void) {
    // Width filter removed MORE than the phantom tally (real narrow pulses /
    // glitch floor on top, the XIAO-class population): wide clamps to 0, never
    // wraps — the subtraction must not manufacture negative counts.
    EXPECT_INT(pcnt_blank_wide(200, 212, 18), 0);
    return 1;
}

static int test_bwide_pcnt_exceeds_isr_pileup(void) {
    // PCNT sees pulses the ISR's 190 µs gate absorbed (pileup): pcnt > isr+B,
    // removed clamps to 0 → subtract the full blanked tally, exactly the
    // V2.6.29 behavior (and no uint32 underflow in the removed term).
    EXPECT_INT(pcnt_blank_wide(235, 212, 18), 18);
    return 1;
}

static int test_bwide_blanking_off_is_noop(void) {
    // blanked==0 (blanking disabled or clean board) → nothing to subtract,
    // whatever the width filter did.
    EXPECT_INT(pcnt_blank_wide(230, 212, 0), 0);
    EXPECT_INT(pcnt_blank_wide(212, 230, 0), 0);
    return 1;
}

static int test_bwide_zero_cycle(void) {
    // Dead-quiet cycle (tube fault / startup): all zeros must yield zero.
    EXPECT_INT(pcnt_blank_wide(0, 0, 0), 0);
    return 1;
}

// ----------------------------------------------------------------------------
// lorawan_codec  (V2.6.23-dev T3 — pure payload/hex helpers, V1.9 byte-compat)
// ----------------------------------------------------------------------------

static int test_lw_hex_decode(void) {
    uint8_t b[4];
    if (!lw_hex_decode("DeadBeef", b, 4)) return 0;
    if (b[0] != 0xDE || b[3] != 0xEF) return 0;
    if (lw_hex_decode("DEADBEE", b, 4))  return 0;  // too short
    if (lw_hex_decode("DEADBEEFAA", b, 4)) return 0; // too long
    if (lw_hex_decode("DEADBEEG", b, 4)) return 0;  // bad char
    return 1;
}

static int test_lw_eui_from_hex(void) {
    uint64_t v = 0;
    if (!lw_eui_from_hex("70B3D57ED0001234", &v)) return 0;
    if (v != 0x70B3D57ED0001234ULL) return 0;
    if (lw_eui_from_hex("70B3", &v)) return 0;      // wrong length
    return 1;
}

static int test_lw_pack_version(void) {
    if (lw_pack_version("V2.6.23") != ((2u<<12)|(6u<<4)|15u)) return 0; // patch clamped
    if (lw_pack_version("V2.6.7")  != ((2u<<12)|(6u<<4)|7u))  return 0;
    if (lw_pack_version("garbage") != 0) return 0;
    return 1;
}

static int test_lw_build_port1(void) {
    uint8_t p[10];
    lw_build_port1(p, 0x01020304u, 0x00A0B0C0u & 0xFFFFFFu, 0x2617u, 3);
    if (p[0]!=0x01||p[1]!=0x02||p[2]!=0x03||p[3]!=0x04) return 0; // counts BE
    if (p[4]!=0xA0||p[5]!=0xB0||p[6]!=0xC0) return 0;             // dt u24 BE
    if (p[7]!=0x26||p[8]!=0x17) return 0;                          // version BE
    if (p[9]!=3) return 0;                                         // tube nbr
    // Saturation: dt_ms >= 2^24 must clamp to 0xFFFFFF (V2 clamps where
    // V1.9 silently wrapped — see lorawan_codec.h).
    lw_build_port1(p, 1, 0x2000000u, 0x2617u, 3);
    if (p[4] != 0xFF || p[5] != 0xFF || p[6] != 0xFF) return 0;
    return 1;
}

static int test_lw_build_port2(void) {
    uint8_t p[5];
    // 21.7 C, 48.5 %, 101325 Pa -> temp*10=217, hum*2=97, 101325/10=10132
    if (!lw_build_port2(p, 21.7f, 48.5f, 101325.0f)) return 0;
    if (p[0]!=(217>>8) || p[1]!=(217&0xFF)) return 0;
    if (p[2]!=97) return 0;
    if (p[3]!=(10132>>8) || p[4]!=(10132&0xFF)) return 0;
    // negative temperature: -5.5 C -> -55 as s16 BE
    if (!lw_build_port2(p, -5.5f, 10.0f, 90000.0f)) return 0;
    if (p[0]!=0xFF || p[1]!=(uint8_t)(-55)) return 0;
    // Saturation: pressure/10 >= 65536 must clamp to 0xFFFF (700000 Pa
    // -> 70000 > 65535).
    if (!lw_build_port2(p, 0.0f, 0.0f, 700000.0f)) return 0;
    if (p[3] != 0xFF || p[4] != 0xFF) return 0;
    return 1;
}

// ----------------------------------------------------------------------------
// lorawan_sched  (V2.8.14 — SF interval table, port-2 cadence, fixed DR,
// Class A frame filter)
// ----------------------------------------------------------------------------

static int test_lw_sched_interval(void) {
    // The user's table: SF7 4, SF8 7, SF9 12, SF10 22, SF11/SF12 30 min.
    EXPECT_INT(lw_sched_interval_ms(7),  240000);
    EXPECT_INT(lw_sched_interval_ms(8),  420000);
    EXPECT_INT(lw_sched_interval_ms(9),  720000);
    EXPECT_INT(lw_sched_interval_ms(10), 1320000);
    EXPECT_INT(lw_sched_interval_ms(11), 1800000);
    EXPECT_INT(lw_sched_interval_ms(12), 1800000);
    // Edges: no SF (FSK, 0) -> SF7 row; above 12 -> SF12 row.
    EXPECT_INT(lw_sched_interval_ms(0),  240000);
    EXPECT_INT(lw_sched_interval_ms(5),  240000);
    EXPECT_INT(lw_sched_interval_ms(13), 1800000);
    // Every row stays inside the 24-bit dt field of the port-1 frame.
    for (unsigned sf = 0; sf < 16; sf++) {
        EXPECT_TRUE(lw_sched_interval_ms((uint8_t)sf) < 0xFFFFFFu);
    }
    return 1;
}

static int test_lw_sched_port2_due(void) {
    // Environment-only node: every cycle, whatever the gap.
    EXPECT_TRUE(lw_sched_port2_due(false, true, 100, 99));
    EXPECT_TRUE(lw_sched_port2_due(false, false, 0, 0));
    // Geiger + environment: the first one is due ...
    EXPECT_TRUE(lw_sched_port2_due(true, false, 50, 0));
    // ... then at most once per hour (with 2 min of slack).
    EXPECT_TRUE(!lw_sched_port2_due(true, true, 1000 + 1800, 1000));   // 30 min: no
    EXPECT_TRUE(!lw_sched_port2_due(true, true, 1000 + 3479, 1000));   // 58 min - 1 s: no
    EXPECT_TRUE(lw_sched_port2_due(true, true, 1000 + 3480, 1000));    // 58 min: yes
    EXPECT_TRUE(lw_sched_port2_due(true, true, 1000 + 3599, 1000));    // a 30-min cycle a few s early
    // Across the 32-bit uptime wrap (unsigned difference).
    EXPECT_TRUE(lw_sched_port2_due(true, true, 3000u, 0xFFFFFFFFu - 1000u));
    EXPECT_TRUE(!lw_sched_port2_due(true, true, 100u, 0xFFFFFFFFu - 100u));
    return 1;
}

static int test_lw_sched_fixed_dr(void) {
    uint8_t dr = 99;
    EXPECT_TRUE(!lw_sched_fixed_dr(0, &dr));   // ADR
    EXPECT_INT(dr, 99);                        // untouched
    EXPECT_TRUE(lw_sched_fixed_dr(1, &dr));
    EXPECT_INT(dr, 0);                         // setting 1 = DR0
    EXPECT_TRUE(lw_sched_fixed_dr(6, &dr));
    EXPECT_INT(dr, 5);                         // setting 6 = DR5
    EXPECT_TRUE(!lw_sched_fixed_dr(7, &dr));   // out of range (corrupt NVS) -> ADR
    EXPECT_TRUE(!lw_sched_fixed_dr(255, &dr));
    EXPECT_TRUE(!lw_sched_fixed_dr(3, NULL));
    return 1;
}

// V2.8.15 radio-turn rule: no WiFi upload (round or FTPS) while a LoRaWAN
// exchange is on air, and no exchange while one runs.
static int test_radio_turn_rule(void) {
    // LoRaWAN exchange: needs no round running/queued and no FTPS
    EXPECT_TRUE(rt_lorawan_may_start(true, false));
    EXPECT_TRUE(!rt_lorawan_may_start(false, false));   // round running or queued
    EXPECT_TRUE(!rt_lorawan_may_start(true, true));     // FTPS running
    EXPECT_TRUE(!rt_lorawan_may_start(false, true));
    // FTPS: the old tx_is_idle() rule plus no exchange on air
    EXPECT_TRUE(rt_ftps_may_start(true, false));
    EXPECT_TRUE(!rt_ftps_may_start(false, false));      // unchanged pre-V2.8.15 rule
    EXPECT_TRUE(!rt_ftps_may_start(true, true));        // exchange on air
    // WiFi upload round: only an exchange on air holds it back
    EXPECT_TRUE(rt_tx_round_may_start(false));
    EXPECT_TRUE(!rt_tx_round_may_start(true));
    return 1;
}

static int test_lw_sched_frame_is_ours(void) {
    // Unconfirmed data down (MHDR 0x60) for DevAddr 0x0A0B0C0D (made up), 12 B minimum frame.
    uint8_t f[33] = { 0x60, 0x0D, 0x0C, 0x0B, 0x0A, 0x00, 0x01, 0x00, 0, 0, 0, 0 };
    EXPECT_TRUE(lw_sched_frame_is_ours(true, 0x0A0B0C0Du, f, 12));
    f[0] = 0xA0;                                                     // confirmed data down
    EXPECT_TRUE(lw_sched_frame_is_ours(true, 0x0A0B0C0Du, f, 12));
    EXPECT_TRUE(!lw_sched_frame_is_ours(true, 0x0A0B0C0Cu, f, 12));  // one bit off: not ours
    EXPECT_TRUE(!lw_sched_frame_is_ours(true, 0x0A0B0C0Du, f, 11));  // shorter than any data frame
    f[0] = 0x40;                                                     // unconfirmed data UP
    EXPECT_TRUE(!lw_sched_frame_is_ours(true, 0x0A0B0C0Du, f, 12));
    f[0] = 0x61;                                                     // Major != 0: RadioLib 8.0 has no Major check
    EXPECT_TRUE(lw_sched_frame_is_ours(true, 0x0A0B0C0Du, f, 12));
    f[0] = 0x60;
    f[5] = 0x03;                                                     // FCtrl: FOptsLen 3
    EXPECT_TRUE(!lw_sched_frame_is_ours(true, 0x0A0B0C0Du, f, 14));  // 12 + 3 needed
    EXPECT_TRUE(lw_sched_frame_is_ours(true, 0x0A0B0C0Du, f, 15));
    EXPECT_TRUE(!lw_sched_frame_is_ours(true, 0x0A0B0C0Du, f, 5));   // no FCtrl byte
    f[5] = 0x00;
    // While joining: only a join accept (MHDR 0x20) of 17 or 33 B.
    f[0] = 0x20;
    EXPECT_TRUE(lw_sched_frame_is_ours(false, 0, f, 17));
    EXPECT_TRUE(lw_sched_frame_is_ours(false, 0, f, 33));
    EXPECT_TRUE(!lw_sched_frame_is_ours(false, 0, f, 18));
    EXPECT_TRUE(!lw_sched_frame_is_ours(true, 0x0A0B0C0Du, f, 17));  // joined: no join accepts
    f[0] = 0x60;
    EXPECT_TRUE(!lw_sched_frame_is_ours(false, 0x0A0B0C0Du, f, 12)); // joining: no data frames
    EXPECT_TRUE(!lw_sched_frame_is_ours(true, 0x0A0B0C0Du, NULL, 12));
    return 1;
}

// lorawan_rxobs.h (V2.8.21) — the RX-window observer and its "rx windows:" /
// "uplink OK:" text, driven with the SPI sequences RadioLib 8.0 sends (ported
// from the rxobs harness; expectations follow RadioLib 8.0: a foreign or
// unread frame never ends an exchange). DevAddrs made up.
#define RXT_OWN     0x260B1234u
#define RXT_FOREIGN 0x66492766u
static int64_t g_rxt_now = 1000000;   // 0 would read as "no TX seen yet"
static rxobs_t g_rxt;
static char    g_rxt_buf[LW_RXOBS_LINE_MAX];

static void rxt_spi(const uint8_t *o, size_t n, const uint8_t *i) { rxobs_note(&g_rxt, g_rxt_now, o, n, i); }
static uint32_t rxt_raw(double mhz) { return (uint32_t)(mhz * 33554432.0 / 32.0 + 0.5); }
static void rxt_freq(double mhz) {
    const uint32_t r = rxt_raw(mhz);
    const uint8_t o[5] = { OBS_CMD_SET_FREQ, (uint8_t)(r >> 24), (uint8_t)(r >> 16), (uint8_t)(r >> 8), (uint8_t)r };
    rxt_spi(o, 5, NULL);
}
static void rxt_sf(uint8_t sf) { const uint8_t o[2] = { OBS_CMD_SET_MOD, sf }; rxt_spi(o, 2, NULL); }
static void rxt_rx(uint32_t ms) {
    const uint32_t t = ms * 64u;
    const uint8_t o[4] = { OBS_CMD_SET_RX, (uint8_t)(t >> 16), (uint8_t)(t >> 8), (uint8_t)t };
    rxt_spi(o, 4, NULL);
}
static void rxt_irq(uint16_t f) {
    const uint8_t o[4] = { OBS_CMD_GET_IRQ, 0, 0, 0 };
    const uint8_t i[4] = { 0, 0, (uint8_t)(f >> 8), (uint8_t)f };
    rxt_spi(o, 4, i);
}
static void rxt_frame(uint8_t mhdr, uint32_t addr, size_t len) {
    uint8_t o[64] = { OBS_CMD_READ_BUFFER }, i[64] = { 0 };
    i[3] = mhdr;
    i[4] = (uint8_t)addr; i[5] = (uint8_t)(addr >> 8); i[6] = (uint8_t)(addr >> 16); i[7] = (uint8_t)(addr >> 24);
    rxt_spi(o, 3 + len, i);   // i[8] = FCtrl 0 (no FOpts)
}
// One TX as RadioLib 8.0 drives it. rx1/rx2: 0 = window not opened, 1 = timeout,
// 2 = our frame, 3 = foreign frame (read, then dropped), 4 = RX_DONE not read.
static void rxt_tx(double mhz, int rx1, int rx2) {
    rxt_sf(12); rxt_freq(mhz);
    const uint8_t tx[4] = { OBS_CMD_SET_TX, 0, 0, 0 };
    rxt_spi(tx, 4, NULL);
    g_rxt_now += 1482000;
    rxt_irq(0x0001);   // checkIrq(TX_DONE)
    const uint8_t c[3] = { OBS_CMD_CLEAR_IRQ, 0, 0 };
    rxt_spi(c, 3, NULL);
    rxt_spi(c, 3, NULL);
    const int rx[2] = { rx1, rx2 };
    for (int w = 0; w < 2; w++) {
        if (!rx[w]) continue;
        if (w == 0) { g_rxt_now += 4991000; rxt_freq(mhz); rxt_rx(2996); }
        else        { g_rxt_now += 1000000; rxt_freq(869.525); rxt_sf(9); rxt_rx(708); }
        g_rxt_now += 50000;
        if (rx[w] == 1) { rxt_irq(OBS_IRQ_TIMEOUT); continue; }
        rxt_irq(OBS_IRQ_RX_DONE | OBS_IRQ_HEADER_VALID);
        if (rx[w] != 4) rxt_frame(0x60, rx[w] == 2 ? RXT_OWN : RXT_FOREIGN, w ? 18 : 14);
    }
    g_rxt_now += 3000000;
}
static void rxt_begin(uint8_t nb_trans) {
    g_rxt.own_addr = RXT_OWN;
    rxobs_begin(&g_rxt, nb_trans);
}
// Assert the "rx windows:" text and the "uplink OK:" freqs / TX parts.
static int rxt_check(int16_t st, const char *windows, const char *freqs, const char *tx) {
    char f[OBS_MAX_REP * 7 + 8], t[48];
    rxobs_format(&g_rxt, g_rxt_buf, sizeof(g_rxt_buf), st);
    EXPECT_STREQ(g_rxt_buf, windows);
    rxobs_uplink_tx(&g_rxt, st, 1127, f, sizeof(f), t, sizeof(t));
    EXPECT_STREQ(f, freqs);
    EXPECT_STREQ(t, tx);
    return 1;
}
#define RXT_LAST_BOTH(f) " | last TX: RX1 " f " SF12 @+4991 ms to=2996 ms; RX2 869.525 SF9 @+6041 ms to=708 ms"
#define RXT_LAST_RX1(f)  " | last TX: RX1 " f " SF12 @+4991 ms to=2996 ms; RX2 not opened"
#define RXT_TT "RX1 timeout, no preamble; RX2 timeout, no preamble"
#define RXT_FOREIGN14 "RX_DONE (data down for 66492766 (not ours), 14 B) ignored, repeats go on"
#define RXT_UNREAD "RX_DONE (not read: RadioLib dropped it, too long/short) ignored, repeats go on"

static int test_rxobs_all_timeouts(void) {   // A
    rxt_begin(3); rxt_tx(867.1, 1, 1); rxt_tx(868.5, 1, 1); rxt_tx(867.3, 1, 1);
    return rxt_check(0, "NbTrans 3 (set by network) | all 3 sent | #1 867.100, #2 868.500, #3 867.300: " RXT_TT
                     RXT_LAST_BOTH("867.300"), "867.1/868.5/867.3", "3/3");
}
static int test_rxobs_own_dl_rx1_first_repeat(void) {   // B
    rxt_begin(3); rxt_tx(867.9, 2, 0);
    return rxt_check(1, "NbTrans 3 (set by network) | stopped after 1 of 3: valid downlink in RX1 of #1 — a downlink "
                     "ends the repeats (LoRaWAN) | #1 867.900: RX1 RX_DONE (data down for 260b1234 (ours), 14 B); "
                     "RX2 not opened" RXT_LAST_RX1("867.900"), "867.9", "1/3");
}
static int test_rxobs_foreign_frame_repeats_go_on(void) {   // C (RadioLib 8.0: dropped, st 0)
    rxt_begin(3); rxt_tx(867.9, 1, 1); rxt_tx(868.3, 3, 1); rxt_tx(868.1, 1, 1);
    return rxt_check(0, "NbTrans 3 (set by network) | all 3 sent | #1 867.900: " RXT_TT " | #2 868.300: RX1 "
                     RXT_FOREIGN14 "; RX2 timeout, no preamble | #3 868.100: " RXT_TT RXT_LAST_BOTH("868.100"),
                     "867.9/868.3/868.1", "3/3");
}
static int test_rxobs_foreign_then_own_dl(void) {   // C2: the later repeat is named
    rxt_begin(3); rxt_tx(867.9, 3, 1); rxt_tx(868.3, 2, 0);
    return rxt_check(1, "NbTrans 3 (set by network) | stopped after 2 of 3: valid downlink in RX1 of #2 — a downlink "
                     "ends the repeats (LoRaWAN) | #1 867.900: RX1 " RXT_FOREIGN14 "; RX2 timeout, no preamble | "
                     "#2 868.300: RX1 RX_DONE (data down for 260b1234 (ours), 14 B); RX2 not opened"
                     RXT_LAST_RX1("868.300"), "867.9/868.3", "2/3");
}
static int test_rxobs_own_malformed_ends_exchange(void) {   // C3: the -1102 left on RadioLib 8.0
    rxt_begin(3); rxt_tx(867.9, 1, 1); rxt_tx(868.3, 2, 0);
    return rxt_check(-1102, "NbTrans 3 (set by network) | stopped after 2 of 3: unusable frame in RX1 of #2, -1102 "
                     "(downlink addressed to us but malformed (bad FPort or FOpts), dropped), ended them early — "
                     "LoRaWAN says only a valid downlink ends the repeats | #1 867.900: " RXT_TT " | #2 868.300: RX1 "
                     "RX_DONE (data down for 260b1234 (ours), 14 B); RX2 not opened" RXT_LAST_RX1("868.300"),
                     "867.9/868.3", "2/3");
}
static int test_rxobs_own_dl_rx2_last_repeat(void) {   // D
    rxt_begin(3); rxt_tx(867.9, 1, 1); rxt_tx(868.3, 1, 1); rxt_tx(868.1, 1, 2);
    return rxt_check(2, "NbTrans 3 (set by network) | all 3 sent, valid downlink in RX2 of #3 | #1 867.900, "
                     "#2 868.300: " RXT_TT " | #3 868.100: RX1 timeout, no preamble; RX2 RX_DONE (data down for "
                     "260b1234 (ours), 18 B)" RXT_LAST_BOTH("868.100"), "867.9/868.3/868.1", "3/3");
}
static int test_rxobs_single_tx(void) {   // E
    rxt_begin(1); rxt_tx(867.5, 1, 1);
    return rxt_check(0, "RX1 867.500 SF12 @+4991 ms to=2996 ms timeout, no preamble | RX2 869.525 SF9 @+6041 ms "
                     "to=708 ms timeout, no preamble", "867.5", "1");
}
static int test_rxobs_single_tx_foreign(void) {   // F (RadioLib 8.0: dropped, RX2 opens, st 0)
    rxt_begin(1); rxt_tx(867.5, 3, 1);
    return rxt_check(0, "RX1 867.500 SF12 @+4991 ms to=2996 ms RX_DONE (data down for 66492766 (not ours), 14 B) "
                     "ignored (RadioLib dropped it, the window counts as empty) | RX2 869.525 SF9 @+6041 ms "
                     "to=708 ms timeout, no preamble", "867.5", "1");
}
static int test_rxobs_rx_window_missed(void) {   // G
    rxt_begin(3); rxt_tx(867.9, 1, 1); rxt_tx(868.3, 1, 0);
    return rxt_check(-1105, "NbTrans 3 (set by network) | stopped at TX 2 of 3: -1105 (RX window missed: RX1 held "
                     "open past RX2, or too late) | #1 867.900: " RXT_TT " | #2 868.300: RX1 timeout, no preamble; "
                     "RX2 not opened" RXT_LAST_RX1("868.300"), "867.9/868.3", "2/3");
}
static int test_rxobs_nested_mac_only(void) {   // K
    rxt_begin(3); rxt_tx(867.9, 2, 0); rxt_tx(868.3, 1, 1); rxt_tx(868.5, 1, 1); rxt_tx(867.1, 1, 1);
    return rxt_check(1, "NbTrans 3 (set by network) | stopped after 1 of 3: valid downlink in RX1 of #1 — a downlink "
                     "ends the repeats (LoRaWAN); then 3 TX of a MAC-only uplink (M1..: RadioLib sends MAC answers "
                     "that do not fit the next frame on their own) | #1 867.900: RX1 RX_DONE (data down for 260b1234 "
                     "(ours), 14 B); RX2 not opened | M1 868.300, M2 868.500, M3 867.100: " RXT_TT
                     RXT_LAST_BOTH("867.100"), "867.9", "1/3 +3 MAC-only (from FCnt 1128)");
}
static int test_rxobs_nested_mac_only_nbtrans1(void) {   // K2
    rxt_begin(1); rxt_tx(867.9, 2, 0); rxt_tx(868.3, 1, 1);
    return rxt_check(1, "NbTrans 1 | all 1 sent, valid downlink in RX1 of #1; then 1 TX of a MAC-only uplink (M1..: "
                     "RadioLib sends MAC answers that do not fit the next frame on their own) | #1 867.900: RX1 "
                     "RX_DONE (data down for 260b1234 (ours), 14 B); RX2 not opened | M1 868.300: " RXT_TT
                     RXT_LAST_BOTH("868.300"), "867.9", "1 +1 MAC-only (from FCnt 1128)");
}
static int test_rxobs_foreign_on_last_repeat(void) {   // L (RadioLib 8.0: st 0)
    rxt_begin(3); rxt_tx(867.9, 1, 1); rxt_tx(868.3, 1, 1); rxt_tx(868.1, 3, 1);
    return rxt_check(0, "NbTrans 3 (set by network) | all 3 sent | #1 867.900, #2 868.300: " RXT_TT " | #3 868.100: "
                     "RX1 " RXT_FOREIGN14 "; RX2 timeout, no preamble" RXT_LAST_BOTH("868.100"),
                     "867.9/868.3/868.1", "3/3");
}
static int test_rxobs_unread_frame_repeats_go_on(void) {   // M2
    rxt_begin(3); rxt_tx(867.9, 4, 1); rxt_tx(868.3, 1, 1); rxt_tx(868.1, 1, 1);
    return rxt_check(0, "NbTrans 3 (set by network) | all 3 sent | #1 867.900: RX1 " RXT_UNREAD "; RX2 timeout, "
                     "no preamble | #2 868.300, #3 868.100: " RXT_TT RXT_LAST_BOTH("868.100"), "867.9/868.3/868.1",
                     "3/3");
}
static int test_rxobs_unread_frames_two_repeats(void) {   // P (formatter robustness; st 0 on RadioLib 8.0)
    rxt_begin(3); rxt_tx(867.9, 4, 1); rxt_tx(868.3, 1, 4);
    return rxt_check(0, "NbTrans 3 (set by network) | stopped after 2 of 3, no downlink (cause not seen) | #1 867.900: "
                     "RX1 " RXT_UNREAD "; RX2 timeout, no preamble | #2 868.300: RX1 timeout, no preamble; RX2 "
                     RXT_UNREAD RXT_LAST_BOTH("868.300"), "867.9/868.3", "2/3");
}
static int test_rxobs_tx_timeout(void) {   // N
    rxt_begin(3); rxt_tx(867.9, 1, 1);
    const uint8_t tx[4] = { OBS_CMD_SET_TX, 0, 0, 0 };
    rxt_spi(tx, 4, NULL);   // second TX, no TX end, no windows
    return rxt_check(-5, "NbTrans 3 (set by network) | stopped at TX 2 of 3: -5 (TX timeout (the radio did not report the end of the transmission)) | #1 867.900: " RXT_TT
                     " | #2 869.525: RX1 not opened; RX2 not opened | last TX: RX1 not opened; RX2 not opened",
                     "867.9/869.5", "2/3");
}
static int test_rxobs_unread_both_windows(void) {   // Q (formatter robustness; st 0 on RadioLib 8.0)
    rxt_begin(3); rxt_tx(867.9, 4, 4);
    return rxt_check(0, "NbTrans 3 (set by network) | stopped after 1 of 3, no downlink (cause not seen) | #1 867.900: "
                     "RX1 " RXT_UNREAD "; RX2 " RXT_UNREAD RXT_LAST_BOTH("867.900"), "867.9", "1/3");
}
#define RXT_I_LINE "NbTrans 15 (set by network) | all 15 sent | #1 867.100: RX1 timeout, no preamble; RX2 RX_DONE " \
    "(data down for 66492766 (not ours), 18 B) ignored, repeats go on | #2 867.300: " RXT_TT " | #3 867.500: RX1 "      \
    "timeout, no preamble; RX2 RX_DONE (data down for 66492766 (not ours), 18 B) ignored, repeats go on | #4 867.700: " \
    RXT_TT " | +11 more TX not listed" RXT_LAST_BOTH("868.300")
static void rxt_fifteen(void) {
    rxt_begin(15);
    for (int k = 0; k < 15; k++) rxt_tx(867.1 + 0.2 * (k % 8), 1, (k % 2) ? 1 : 3);
}
static int test_rxobs_nbtrans_above_max_rep(void) {   // I: OBS_MAX_REP 4, no overflow
    rxt_fifteen();
    return rxt_check(0, RXT_I_LINE, "867.1/867.3/867.5/867.7/...", "15/15");
}
static int test_rxobs_truncation_keeps_verdict(void) {   // J
    char small[100];
    rxt_fifteen();
    rxobs_format(&g_rxt, small, sizeof(small), 0);
    EXPECT_INT(strlen(small), 99);
    EXPECT_TRUE(strncmp(small, RXT_I_LINE, 99) == 0);
    return 1;
}
// lw_err_text (V2.8.21): every RadioLib status code the LoRaWAN code paths can
// return has plain words; unknown codes get the fallback. Values from RadioLib
// 8.0 TypeDef.h (cross-checked against RadioLib in lorawan.cpp).
static int test_lw_err_text(void) {
    static const int16_t known[] = {
        -1, -2, -4, -5, -6, -7, -12, -13, -16, -20, -24, -28, -501,
        -703, -705, -706, -707,
        -1101, -1102, -1103, -1104, -1105, -1106, -1108, -1109, -1111, -1112,
        -1114, -1115, -1116, -1119, -1120, -1123, -1124,
    };
    const char *fallback = lw_err_text(-32000);
    EXPECT_STREQ(fallback, "RadioLib error, see RadioLib TypeDef.h");
    for (size_t i = 0; i < sizeof(known) / sizeof(known[0]); i++) {
        const char *s = lw_err_text(known[i]);
        if (s == NULL || s[0] == '\0' || strcmp(s, fallback) == 0) {
            printf("    code %d has no words\n", (int)known[i]);
            return 0;
        }
    }
    // The -1102 wording must not claim "not for us": on RadioLib 8.0 a foreign
    // frame is dropped silently; -1102 means our DevAddr, malformed.
    EXPECT_TRUE(strstr(lw_err_text(-1102), "not for us") == NULL);
    EXPECT_STREQ(lw_err_text(-4), "payload too long for this data rate, even without MAC answers");
    EXPECT_STREQ(lw_err_text(-1124), "payload fits, but not together with the queued MAC answers");
    // Re-review 2: setDatarate()'s only error; -1115 is a corrupt/empty blob, not
    // a key mismatch (-1119); -7 on a LoRaWAN downlink is a bit error (no PHY CRC).
    EXPECT_STREQ(lw_err_text(-501), "no channel in this region allows this data rate");
    EXPECT_STREQ(lw_err_text(-1115), "saved data corrupt or empty (checksum mismatch)");
    EXPECT_STREQ(lw_err_text(-7), "frame corrupted (integrity check failed)");
    return 1;
}

// V2.8.21 (user): every upload round says when the next LoRaWAN uplink comes.
static int test_lw_next_uplink_line(void) {
    char out[128];   // as main.c's line buffer
    lw_next_uplink_line(out, sizeof(out), 1800, 12, 41, 7, 12, 0);
    EXPECT_STREQ(out, "next LoRaWAN uplink at about 12:41:07 (in 30 min, SF12 schedule)");
    lw_next_uplink_line(out, sizeof(out), 1800, -1, 0, 0, 12, 0);   // clock not set yet
    EXPECT_STREQ(out, "next LoRaWAN uplink in about 30 min (SF12 schedule; clock not set yet)");
    lw_next_uplink_line(out, sizeof(out), 239, 9, 5, 0, 7, 0);      // rounds to whole minutes
    EXPECT_STREQ(out, "next LoRaWAN uplink at about 09:05:00 (in 4 min, SF7 schedule)");
    lw_next_uplink_line(out, sizeof(out), 45, 9, 5, 0, 7, 0);       // under a minute: seconds
    EXPECT_STREQ(out, "next LoRaWAN uplink at about 09:05:00 (in 45 s, SF7 schedule)");
    lw_next_uplink_line(out, sizeof(out), 600, 23, 59, 59, 0, 0);   // SF not known: no schedule tail
    EXPECT_STREQ(out, "next LoRaWAN uplink at about 23:59:59 (in 10 min)");
    // rc3: RadioLib's duty-cycle limiter holds the uplink past the next round —
    // the time is the later moment (caller's job), the line names why.
    lw_next_uplink_line(out, sizeof(out), 180, 15, 8, 10, 12, 500);
    EXPECT_STREQ(out, "next LoRaWAN uplink at about 15:08:10 (in 8 min: duty-cycle wait; upload round in 3 min, "
                      "SF12 schedule)");
    lw_next_uplink_line(out, sizeof(out), 180, -1, 0, 0, 12, 500);
    EXPECT_STREQ(out, "next LoRaWAN uplink in about 8 min (duty-cycle wait; upload round in 3 min, SF12 schedule; "
                      "clock not set yet)");
    lw_next_uplink_line(out, sizeof(out), 1800, 12, 41, 7, 12, 500);   // wait shorter than the round: no mention
    EXPECT_STREQ(out, "next LoRaWAN uplink at about 12:41:07 (in 30 min, SF12 schedule)");
    return 1;
}

// Review fix 1: receive-side-only errors after a real TX are "sent, no usable
// downlink" — incl. -1104 (our DevAddr, FPort rejected by parseDownlink()).
static int test_lw_downlink_unusable(void) {
    EXPECT_TRUE(lw_downlink_unusable(-1102));
    EXPECT_TRUE(lw_downlink_unusable(-1104));
    EXPECT_TRUE(lw_downlink_unusable(-1105));
    EXPECT_TRUE(lw_downlink_unusable(-1112));
    EXPECT_TRUE(!lw_downlink_unusable(-4));
    EXPECT_TRUE(!lw_downlink_unusable(-5));
    EXPECT_TRUE(!lw_downlink_unusable(-1124));
    EXPECT_TRUE(!lw_downlink_unusable(0));
    return 1;
}

// Review fix 2: which restore line fits. loadBuffers() restores the
// persistence buffer first and stops there on failure (the session callback
// is then never called).
static int test_lw_restore_verdict(void) {
    EXPECT_INT(lw_restore_verdict(0, true, true, true, false), LW_RESTORE_OK);
    EXPECT_INT(lw_restore_verdict(-1101, false, false, false, false), LW_RESTORE_NO_PERS);   // new board
    EXPECT_INT(lw_restore_verdict(-1119, true, false, false, false), LW_RESTORE_PERS_BAD);   // keys changed
    EXPECT_INT(lw_restore_verdict(-1101, true, true, false, false), LW_RESTORE_NO_SESS);
    EXPECT_INT(lw_restore_verdict(-1120, true, true, true, false), LW_RESTORE_SESS_BAD);
    // Session saved right after a join, before its first uplink: withheld from
    // RadioLib (handed over all-zero -> -1115), own line instead of "corrupt".
    EXPECT_INT(lw_restore_verdict(-1115, true, true, true, true), LW_RESTORE_SESS_UNFINISHED);
    EXPECT_INT(lw_restore_verdict(-1115, true, true, true, false), LW_RESTORE_SESS_BAD);
    return 1;
}

// lw_boot_line (V2.8.21): the "boot: LoRaWAN:" summary, verdict first.
// DevAddrs made up.
static lw_boot_t lwb(lw_boot_kind_t kind) {
    lw_boot_t b;
    memset(&b, 0, sizeof(b));
    b.kind = kind;
    b.region = "AU915";
    b.subband = 2;
    b.dev_addr = 0x260B1234u;
    b.adr = true;
    b.ok_dr = 0xFF;
    b.pwr_min = 2;
    b.pwr_max = 33;
    return b;
}
static int test_lw_boot_line_states(void) {
    char out[200];
    lw_boot_t b = lwb(LW_BOOT_OFF);
    lw_boot_line(out, sizeof(out), &b);
    EXPECT_STREQ(out, "off (disabled in /config)");
    b = lwb(LW_BOOT_NO_CONFIG);
    lw_boot_line(out, sizeof(out), &b);
    EXPECT_STREQ(out, "not started: DevEUI, JoinEUI or AppKey missing or malformed in /config");
    b = lwb(LW_BOOT_HW_FAIL);
    lw_boot_line(out, sizeof(out), &b);
    EXPECT_STREQ(out, "not started: radio init failed (see the \"SX1262 begin failed\" line)");
    b = lwb(LW_BOOT_RESUMED);
    lw_boot_line(out, sizeof(out), &b);
    EXPECT_STREQ(out, "session resumed — DevAddr 260b1234, region AU915 sub-band 2, ADR, TX power 2..33 dBm");
    b = lwb(LW_BOOT_JOINED);
    b.ok_window = 1; b.ok_dr = 3; b.ok_sf = 9; b.ok_khz = 867700;
    lw_boot_line(out, sizeof(out), &b);
    EXPECT_STREQ(out, "joined via RX1 at DR3 (SF9, 867.7 MHz) — DevAddr 260b1234, region AU915 sub-band 2, ADR, "
                      "TX power 2..33 dBm");
    b = lwb(LW_BOOT_JOINING);
    b.attempts = 3; b.last_error = -1116;
    lw_boot_line(out, sizeof(out), &b);
    EXPECT_STREQ(out, "joining — attempt 3, last error -1116 (no join accept received), region AU915 sub-band 2, ADR, "
                      "TX power 2..33 dBm");
    return 1;
}
static int test_lw_boot_line_variants(void) {
    char out[200];
    lw_boot_t b = lwb(LW_BOOT_JOINING);   // before the first attempt finished
    lw_boot_line(out, sizeof(out), &b);
    EXPECT_STREQ(out, "joining — first attempt pending, region AU915 sub-band 2, ADR, TX power 2..33 dBm");
    b = lwb(LW_BOOT_RESUMED);             // fixed DR, no PA (no power range), EU868 without sub-band
    b.adr = false; b.fixed_dr = 3; b.pwr_min = 0; b.pwr_max = 0; b.region = "EU868"; b.subband = 0;
    lw_boot_line(out, sizeof(out), &b);
    EXPECT_STREQ(out, "session resumed — DevAddr 260b1234, region EU868, fixed DR3");
    b = lwb(LW_BOOT_JOINED);              // fixed plan: DR from the channel, window unknown
    b.ok_sf = 10; b.ok_khz = 917800;
    lw_boot_line(out, sizeof(out), &b);
    EXPECT_STREQ(out, "joined (SF10, 917.8 MHz) — DevAddr 260b1234, region AU915 sub-band 2, ADR, TX power 2..33 dBm");
    char small[40];                       // truncated, terminated, verdict kept
    b = lwb(LW_BOOT_RESUMED);
    lw_boot_line(small, sizeof(small), &b);
    EXPECT_INT(strlen(small), 39);
    EXPECT_TRUE(strncmp(small, "session resumed — DevAddr", strlen("session resumed — DevAddr")) == 0);
    return 1;
}

static int test_rxobs_join_accept_not_ignored(void) {   // joining: own_addr 0
    g_rxt.own_addr = 0;
    rxobs_begin(&g_rxt, 1);
    rxt_sf(12); rxt_freq(868.1);
    const uint8_t tx[4] = { OBS_CMD_SET_TX, 0, 0, 0 };
    rxt_spi(tx, 4, NULL);
    g_rxt_now += 1482000;
    const uint8_t c[3] = { OBS_CMD_CLEAR_IRQ, 0, 0 };
    rxt_spi(c, 3, NULL);
    g_rxt_now += 4991000; rxt_rx(2996);
    g_rxt_now += 50000; rxt_irq(OBS_IRQ_RX_DONE | OBS_IRQ_HEADER_VALID);
    rxt_frame(0x20, 0, 17);
    rxobs_format(&g_rxt, g_rxt_buf, sizeof(g_rxt_buf), 0);
    EXPECT_STREQ(g_rxt_buf, "RX1 868.100 SF12 @+4991 ms to=2996 ms RX_DONE (join accept, 17 B) | RX2 not opened");
    EXPECT_TRUE(!g_rxt.win[0].ignored);
    return 1;
}

// ----------------------------------------------------------------------------
// env_api_format
// ----------------------------------------------------------------------------

// A fully valid sample, reused by several cases below.
static env_api_t sample_typical(void) {
    env_api_t e;
    memset(&e, 0, sizeof(e));
    strcpy(e.id, "esp32-1234567");
    e.t      = 18.42f;
    e.t_ok   = true;
    e.h      = 63.1f;
    e.h_ok   = true;
    e.p      = 101325.0f;
    e.p_ok   = true;   // Pa, as main_status_t holds it
    e.age_ms = 41230u;
    e.ts     = 1755500000LL;
    return e;
}

static int test_env_api_format_typical(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t e = sample_typical();
    EXPECT_FORMAT_OK(buf, e);
    EXPECT_STREQ(buf,
                 "{\"id\":\"esp32-1234567\",\"t\":18.42,\"h\":63.10,\"p\":101325.00,"
                 "\"t_ok\":true,\"h_ok\":true,\"p_ok\":true,"
                 "\"age_ms\":41230,\"ts\":1755500000}");
    // The formatter is pure, so a second call is free: assert its return
    // value is the rendered length (the contract the handler relies on).
    EXPECT_INT(env_api_format(buf, sizeof(buf), &e), (int)strlen(buf));
    return 1;
}

static int test_env_api_format_invalid_field_is_null(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t e = sample_typical();
    e.h_ok      = false;   // no humidity sensor fitted
    EXPECT_FORMAT_OK(buf, e);
    // null, not 0.0: a consumer that forgets h_ok gets a parse error, not a
    // plausible reading. Design section 3.1's whole argument.
    if (strstr(buf, "\"h\":null,") == NULL) return 0;
    if (strstr(buf, "\"h_ok\":false") == NULL) return 0;
    if (strstr(buf, "\"t\":18.42,") == NULL) return 0;   // neighbours intact
    return 1;
}

static int test_env_api_format_nan_is_null_and_clears_flag(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t e = sample_typical();
    e.t         = NAN;   // flag still says true -- the formatter must not believe it
    EXPECT_FORMAT_OK(buf, e);
    if (strstr(buf, "nan") != NULL) return 0;   // never valid JSON
    if (strstr(buf, "\"t\":null,") == NULL) return 0;
    if (strstr(buf, "\"t_ok\":false") == NULL) return 0;
    return 1;
}

static int test_env_api_format_inf_is_null(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t e = sample_typical();
    e.p         = INFINITY;
    EXPECT_FORMAT_OK(buf, e);
    if (strstr(buf, "inf") != NULL) return 0;
    if (strstr(buf, "\"p\":null,") == NULL) return 0;
    if (strstr(buf, "\"p_ok\":false") == NULL) return 0;
    return 1;
}

// The magnitude cap is a WIDTH guard, not a plausibility guard -- it exists so
// ENV_API_BUF_MIN's arithmetic holds, and deciding whether a reading is
// believable is the consumer's job, not the wire format's.
static int test_env_api_format_over_width_cap_is_null(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t e = sample_typical();
    e.t         = 2.0e6f;   // finite, but wider than the derivation allows
    EXPECT_FORMAT_OK(buf, e);
    if (strstr(buf, "\"t\":null,") == NULL) return 0;
    if (strstr(buf, "\"t_ok\":false") == NULL) return 0;
    return 1;
}

// Pressure is carried in PASCALS, unconverted, exactly as main_status_t holds
// it (env_sensor.h:70) and exactly as mqtt.c:505 and lw_build_port2 publish it.
// Only the HTML status page converts to hPa (http_server.c:691), because that
// is a human interface. Pinning this here is what stops someone "helpfully"
// adding a /100 to match the status page and silently changing the contract.
static int test_env_api_format_pressure_is_pascals(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t e = sample_typical();
    e.p         = 101325.0f;   // standard sea level
    EXPECT_FORMAT_OK(buf, e);
    if (strstr(buf, "\"p\":101325.00,") == NULL) return 0;
    if (strstr(buf, "\"p_ok\":true") == NULL) return 0;
    // The realistic atmospheric extremes must both survive the width cap.
    e.p = 87000.0f;
    EXPECT_FORMAT_OK(buf, e);
    if (strstr(buf, "\"p\":87000.00,") == NULL) return 0;
    e.p = 108500.0f;
    EXPECT_FORMAT_OK(buf, e);
    if (strstr(buf, "\"p\":108500.00,") == NULL) return 0;
    return 1;
}

static int test_env_api_format_never_sampled_sentinel(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t e;
    memset(&e, 0, sizeof(e));
    strcpy(e.id, "esp32-1");
    e.age_ms = ENV_API_AGE_NEVER;   // all *_ok false, all values zero
    EXPECT_FORMAT_OK(buf, e);
    if (strstr(buf, "\"age_ms\":4294967295") == NULL) return 0;
    if (strstr(buf, "\"t\":null,") == NULL) return 0;
    if (strstr(buf, "\"h\":null,") == NULL) return 0;
    if (strstr(buf, "\"p\":null,") == NULL) return 0;
    return 1;
}

static int test_env_api_format_rejects_small_buffer(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t e = sample_typical();
    EXPECT_INT(env_api_format(buf, ENV_API_BUF_MIN - 1, &e), -1);
    EXPECT_INT(env_api_format(NULL, sizeof(buf), &e), -1);
    EXPECT_INT(env_api_format(buf, sizeof(buf), NULL), -1);
    return 1;
}

// The buffer floor is a derivation, not a guess. Prove it with the widest
// payload the type system permits.
static int test_env_api_format_worst_case_fits(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t e;
    memset(&e, 0, sizeof(e));
    memset(e.id, 'X', ENV_API_ID_MAX - 1);   // longest id the field can hold
    // Widest float the magnitude cap admits. NOT -999999.99f: in this binade a
    // float's ULP is 0.0625, so that literal snaps UP to exactly -1000000.0f,
    // which env_api_sane() rejects (the bound is strict) -- every field would
    // come out "null" and the case would measure 144, not the ceiling.
    // -999999.9375 is 15999999/16, exactly representable, and prints as
    // "-999999.94" -- the 10 characters the derivation assumes.
    e.t      = -999999.9375f;
    e.t_ok   = true;
    e.h      = -999999.9375f;
    e.h_ok   = true;
    e.p      = -999999.9375f;
    e.p_ok   = true;
    e.age_ms = 0xFFFFFFFFu;   // widest u32: 10 digits
    e.ts     = INT64_MIN;     // widest i64: 20 chars
    int n    = env_api_format(buf, sizeof(buf), &e);
    if (n < 0) return 0;
    EXPECT_INT(n, 159);
    if ((size_t)n + 1 > ENV_API_BUF_MIN) return 0;
    return 1;
}

static int test_env_api_format_rejects_bad_id(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t e = sample_typical();

    strcpy(e.id, "bad\"id");   // a quote would break the JSON string
    EXPECT_INT(env_api_format(buf, sizeof(buf), &e), -1);

    strcpy(e.id, "bad\\id");   // so would a backslash
    EXPECT_INT(env_api_format(buf, sizeof(buf), &e), -1);

    strcpy(e.id, "bad\nid");   // a raw control character is not legal in JSON
    EXPECT_INT(env_api_format(buf, sizeof(buf), &e), -1);

    // Non-ASCII. Written as an explicit byte rather than a UTF-8 literal so the
    // case does not depend on this file's encoding. This is also the one place
    // `char` signedness could matter: on the host char is signed, so 0xC3 reads
    // as negative and trips the `< 0x20` bound, while on Xtensa char is
    // unsigned and it trips `> 0x7E`. Rejected either way -- pin that, because
    // a rewrite using an int without a cast could reject only on one of them.
    strcpy(e.id, "bad?id");
    e.id[3] = (char)0xC3;
    EXPECT_INT(env_api_format(buf, sizeof(buf), &e), -1);

    e.id[0] = 0;   // empty id is not a node identity
    EXPECT_INT(env_api_format(buf, sizeof(buf), &e), -1);

    memset(e.id, 'X', ENV_API_ID_MAX);   // unterminated
    EXPECT_INT(env_api_format(buf, sizeof(buf), &e), -1);
    return 1;
}

// ----------------------------------------------------------------------------
// env_api_parse
// ----------------------------------------------------------------------------

#define TYPICAL_JSON                                                     \
    "{\"id\":\"esp32-1234567\",\"t\":18.42,\"h\":63.10,\"p\":101325.00," \
    "\"t_ok\":true,\"h_ok\":true,\"p_ok\":true,"                         \
    "\"age_ms\":41230,\"ts\":1755500000}"

static int test_env_api_parse_typical(void) {
    env_api_t e;
    if (!env_api_parse(TYPICAL_JSON, &e)) return 0;
    EXPECT_STREQ(e.id, "esp32-1234567");
    EXPECT_FLOAT_NEAR(e.t, 18.42, 0.001);
    EXPECT_FLOAT_NEAR(e.h, 63.10, 0.001);
    EXPECT_FLOAT_NEAR(e.p, 101325.00, 0.001);
    EXPECT_INT(e.t_ok, 1);
    EXPECT_INT(e.h_ok, 1);
    EXPECT_INT(e.p_ok, 1);
    EXPECT_INT(e.age_ms, 41230);
    EXPECT_INT(e.ts, 1755500000);
    return 1;
}

static int test_env_api_parse_null_clears_flag(void) {
    // A hostile or buggy peer sends null alongside a true flag. The value is
    // absent, so the flag must lose -- never the other way round.
    env_api_t e;
    if (!env_api_parse(
            "{\"id\":\"esp32-1\",\"t\":18.42,\"h\":null,\"p\":101325.00,"
            "\"t_ok\":true,\"h_ok\":true,\"p_ok\":true,"
            "\"age_ms\":100,\"ts\":0}",
            &e)) return 0;
    EXPECT_INT(e.h_ok, 0);
    EXPECT_FLOAT_NEAR(e.h, 0.0, 0.0001);
    EXPECT_INT(e.t_ok, 1);
    return 1;
}

static int test_env_api_parse_false_flag_zeroes_value(void) {
    // The mirror image: a real number alongside a false flag. Same rule.
    env_api_t e;
    if (!env_api_parse(
            "{\"id\":\"esp32-1\",\"t\":18.42,\"h\":63.10,\"p\":101325.00,"
            "\"t_ok\":true,\"h_ok\":false,\"p_ok\":true,"
            "\"age_ms\":100,\"ts\":0}",
            &e)) return 0;
    EXPECT_INT(e.h_ok, 0);
    EXPECT_FLOAT_NEAR(e.h, 0.0, 0.0001);
    return 1;
}

static int test_env_api_parse_missing_key_fails(void) {
    env_api_t e;
    // "p" absent entirely -- must be detectably missing, never a misread
    // neighbour (design section 3.5).
    if (env_api_parse(
            "{\"id\":\"esp32-1\",\"t\":18.42,\"h\":63.10,"
            "\"t_ok\":true,\"h_ok\":true,\"p_ok\":true,"
            "\"age_ms\":100,\"ts\":0}",
            &e)) return 0;
    // "age_ms" absent
    if (env_api_parse(
            "{\"id\":\"esp32-1\",\"t\":18.42,\"h\":63.10,\"p\":101325.00,"
            "\"t_ok\":true,\"h_ok\":true,\"p_ok\":true,\"ts\":0}",
            &e)) return 0;
    return 1;
}

static int test_env_api_parse_truncated_fails(void) {
    env_api_t e;
    char      trunc[sizeof(TYPICAL_JSON)];
    // Every truncation point must be rejected, not partially accepted.
    for (size_t cut = 1; cut < strlen(TYPICAL_JSON); cut++) {
        memcpy(trunc, TYPICAL_JSON, cut);
        trunc[cut] = 0;
        if (env_api_parse(trunc, &e)) return 0;
    }
    return 1;
}

static int test_env_api_parse_garbage_fails(void) {
    env_api_t e;
    if (env_api_parse("", &e)) return 0;
    if (env_api_parse("not json at all", &e)) return 0;
    if (env_api_parse("<html><body>404</body></html>", &e)) return 0;
    if (env_api_parse("{}", &e)) return 0;
    return 1;
}

// The whole reason the parser is keyed by name: key order is not a contract,
// and "t" must not be confused with its neighbours "t_ok" or "ts".
static int test_env_api_parse_reordered_keys(void) {
    env_api_t e;
    if (!env_api_parse(
            "{\"ts\":1755500000,\"t_ok\":true,\"age_ms\":41230,\"t\":18.42,"
            "\"p_ok\":true,\"h\":63.10,\"h_ok\":true,\"p\":101325.00,"
            "\"id\":\"esp32-1234567\"}",
            &e)) return 0;
    EXPECT_FLOAT_NEAR(e.t, 18.42, 0.001);
    EXPECT_INT(e.ts, 1755500000);
    EXPECT_INT(e.age_ms, 41230);
    EXPECT_STREQ(e.id, "esp32-1234567");
    return 1;
}

// Adding fields to the endpoint later must not break an older consumer
// (design section 3.5). "ts" is documented optional; unknown keys are ignored.
static int test_env_api_parse_tolerates_extra_and_missing_ts(void) {
    env_api_t e;
    if (!env_api_parse(
            "{\"id\":\"esp32-1\",\"t\":18.42,\"h\":63.10,\"p\":101325.00,"
            "\"t_ok\":true,\"h_ok\":true,\"p_ok\":true,"
            "\"age_ms\":100,\"lux\":123.4,\"cpm\":17}",
            &e)) return 0;
    EXPECT_INT(e.ts, 0);   // absent ts defaults, does not fail
    EXPECT_FLOAT_NEAR(e.t, 18.42, 0.001);
    return 1;
}

static int test_env_api_parse_never_sampled_sentinel(void) {
    env_api_t e;
    if (!env_api_parse(
            "{\"id\":\"esp32-1\",\"t\":null,\"h\":null,\"p\":null,"
            "\"t_ok\":false,\"h_ok\":false,\"p_ok\":false,"
            "\"age_ms\":4294967295,\"ts\":0}",
            &e)) return 0;
    EXPECT_INT(e.age_ms == ENV_API_AGE_NEVER, 1);
    EXPECT_INT(e.t_ok, 0);
    EXPECT_INT(e.h_ok, 0);
    EXPECT_INT(e.p_ok, 0);
    return 1;
}

static int test_env_api_parse_rejects_oversize_id(void) {
    env_api_t e;
    // 40 chars -- longer than ENV_API_ID_MAX. Reject rather than silently
    // truncate: a truncated identity is a different node.
    if (env_api_parse(
            "{\"id\":\"esp32-0000000000000000000000000000000000\","
            "\"t\":18.42,\"h\":63.10,\"p\":101325.00,"
            "\"t_ok\":true,\"h_ok\":true,\"p_ok\":true,"
            "\"age_ms\":100,\"ts\":0}",
            &e)) return 0;
    return 1;
}

static int test_env_api_parse_rejects_over_width_cap_number(void) {
    env_api_t e;
    // env_api_sane() is the SAME predicate on both sides: it rejects NaN/inf
    // and anything wider than the buffer derivation allows. 99999.0 is legal
    // here -- the cap is |v| < 1.0e6, and pressure in Pa lives at ~101325, so
    // a tight "plausible" bound would reject real readings. Judging whether a
    // number is believable belongs to the consumer (vent_logic, Phase 3).
    if (env_api_parse(
            "{\"id\":\"esp32-1\",\"t\":2000000.0,\"h\":63.10,\"p\":101325.00,"
            "\"t_ok\":true,\"h_ok\":true,\"p_ok\":true,"
            "\"age_ms\":100,\"ts\":0}",
            &e)) return 0;
    // ... and the far side of the same boundary must still be accepted.
    if (!env_api_parse(
            "{\"id\":\"esp32-1\",\"t\":18.42,\"h\":63.10,\"p\":108500.00,"
            "\"t_ok\":true,\"h_ok\":true,\"p_ok\":true,"
            "\"age_ms\":100,\"ts\":0}",
            &e)) return 0;
    return 1;
}

static int test_env_api_parse_rejects_negative_age(void) {
    env_api_t e;
    // strtoul would happily wrap "-1" to 4294967295 -- the never-sampled
    // sentinel. Reject the sign explicitly.
    if (env_api_parse(
            "{\"id\":\"esp32-1\",\"t\":18.42,\"h\":63.10,\"p\":101325.00,"
            "\"t_ok\":true,\"h_ok\":true,\"p_ok\":true,"
            "\"age_ms\":-1,\"ts\":0}",
            &e)) return 0;
    return 1;
}

// ----------------------------------------------------------------------------
// env_api round-trip -- the reason format and parse share a file
// ----------------------------------------------------------------------------

static int env_api_same(const env_api_t *a, const env_api_t *b) {
    if (strcmp(a->id, b->id) != 0) return 0;
    if (a->t_ok != b->t_ok) return 0;
    if (a->h_ok != b->h_ok) return 0;
    if (a->p_ok != b->p_ok) return 0;
    if (a->age_ms != b->age_ms) return 0;
    if (a->ts != b->ts) return 0;
    // 2-dp on the wire, so a float survives to within half a centi-unit.
    if (fabs((double)a->t - (double)b->t) > 0.005) return 0;
    if (fabs((double)a->h - (double)b->h) > 0.005) return 0;
    if (fabs((double)a->p - (double)b->p) > 0.005) return 0;
    return 1;
}

static int test_env_api_roundtrip_typical(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t in = sample_typical(), out;
    if (env_api_format(buf, sizeof(buf), &in) < 0) return 0;
    if (!env_api_parse(buf, &out)) return 0;
    EXPECT_INT(env_api_same(&in, &out), 1);
    return 1;
}

static int test_env_api_roundtrip_mixed_validity(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t in = sample_typical(), out;
    in.h_ok      = false;
    in.h         = 0.0f;   // formatter zeroes it via null; match that
    in.p_ok      = false;
    in.p         = 0.0f;
    if (env_api_format(buf, sizeof(buf), &in) < 0) return 0;
    if (!env_api_parse(buf, &out)) return 0;
    EXPECT_INT(env_api_same(&in, &out), 1);
    return 1;
}

static int test_env_api_roundtrip_never_sampled(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t in, out;
    memset(&in, 0, sizeof(in));
    strcpy(in.id, "esp32-1");
    in.age_ms = ENV_API_AGE_NEVER;
    if (env_api_format(buf, sizeof(buf), &in) < 0) return 0;
    if (!env_api_parse(buf, &out)) return 0;
    EXPECT_INT(env_api_same(&in, &out), 1);
    return 1;
}

// A NaN reading must survive the round trip as "no reading", not as a NaN.
static int test_env_api_roundtrip_nan_becomes_invalid(void) {
    char      buf[ENV_API_BUF_MIN];
    env_api_t in = sample_typical(), out;
    in.t         = NAN;
    if (env_api_format(buf, sizeof(buf), &in) < 0) return 0;
    if (!env_api_parse(buf, &out)) return 0;
    EXPECT_INT(out.t_ok, 0);
    EXPECT_FLOAT_NEAR(out.t, 0.0, 0.0001);
    EXPECT_INT(out.h_ok, 1);
    return 1;
}

// ----------------------------------------------------------------------------
// speaker_logic  (V2.7.4 tick defer)
// ----------------------------------------------------------------------------

static int test_tick_defer_zero_periods_fires_next_poll(void) {
    // periods == 0 must reproduce the pre-V2.7.4 shipped behaviour exactly:
    // the tick fires on the very next audio period after the request.
    tick_defer_state_t s = {0};
    tick_defer_request(&s, 0);
    EXPECT_INT(tick_defer_poll(&s), 1);
    return 1;
}

static int test_tick_defer_one_period_holds_then_fires(void) {
    // periods == 1 (the shipped setting) holds for exactly one poll, then fires
    // on the second -- so the tick lands 1-2 ms after the count, not 0-1 ms.
    tick_defer_state_t s = {0};
    tick_defer_request(&s, 1);
    EXPECT_INT(tick_defer_poll(&s), 0);
    EXPECT_INT(tick_defer_poll(&s), 1);
    return 1;
}

static int test_tick_defer_does_not_refire_without_request(void) {
    // The seed-in-poll regression. The bench prototype used "left == 0" as its
    // needs-seeding test INSIDE the poll, so the poll that zeroed the counter
    // re-seeded it and the tick NEVER fired -- a failure invisible downstream,
    // because a silent tick and a working defer produce the SAME phantom count.
    // (test_..._one_period_holds_then_fires also fails against that design; the
    // UNIQUE value here is the three trailing quiet polls, which additionally
    // pin the opposite failure -- a fire that does not clear pending and so
    // auto-refires every period. No other test covers that.)
    tick_defer_state_t s = {0};
    tick_defer_request(&s, 1);
    EXPECT_INT(tick_defer_poll(&s), 0);
    EXPECT_INT(tick_defer_poll(&s), 1);   // fires once
    EXPECT_INT(tick_defer_poll(&s), 0);   // and then stays quiet
    EXPECT_INT(tick_defer_poll(&s), 0);
    EXPECT_INT(tick_defer_poll(&s), 0);
    return 1;
}

static int test_tick_defer_idle_state_never_fires(void) {
    // A poll with no request outstanding must be a no-op, not a fire.
    tick_defer_state_t s = {0};
    EXPECT_INT(tick_defer_poll(&s), 0);
    EXPECT_INT(tick_defer_poll(&s), 0);
    return 1;
}

static int test_tick_defer_n_periods_fires_on_n_plus_one(void) {
    // General case: N periods held, fires on poll N+1. Guards an off-by-one if
    // TICK_DEFER_PERIODS is ever raised.
    tick_defer_state_t s = {0};
    tick_defer_request(&s, 3);
    EXPECT_INT(tick_defer_poll(&s), 0);
    EXPECT_INT(tick_defer_poll(&s), 0);
    EXPECT_INT(tick_defer_poll(&s), 0);
    EXPECT_INT(tick_defer_poll(&s), 1);
    return 1;
}

static int test_tick_defer_request_restarts_countdown(void) {
    // Each request RESTARTS the countdown -- deliberate, because the invariant
    // being protected is "no tick within the recovery window of the most recent
    // count", not "of the first count". Consequence, derived in speaker_logic.h:
    // closely spaced counts MERGE to one tick at the burst's first quiet audio
    // period. Nothing is dropped -- pending persists until it fires. Harmless at
    // ambient rates (~1.5 counts/s -> 99.85% fire at once); see the rate table.
    tick_defer_state_t s = {0};
    tick_defer_request(&s, 1);
    EXPECT_INT(tick_defer_poll(&s), 0);
    tick_defer_request(&s, 1);            // second count arrives mid-defer
    EXPECT_INT(tick_defer_poll(&s), 0);
    EXPECT_INT(tick_defer_poll(&s), 1);
    // ONE tick for the burst, not one per count -- pin the "not a queue" half.
    EXPECT_INT(tick_defer_poll(&s), 0);
    EXPECT_INT(tick_defer_poll(&s), 0);
    return 1;
}

static int test_tick_defer_reusable_after_clear(void) {
    // tick_defer_clear() must leave the block USABLE, not just quiet: the melody
    // path clears mid-playback and real ticks must work again afterwards. Pins
    // the "a stale counter would delay the NEXT tick" claim in the clear doc.
    tick_defer_state_t s = {0};
    tick_defer_request(&s, 1);
    tick_defer_clear(&s);
    tick_defer_request(&s, 1);            // the first count after the melody
    EXPECT_INT(tick_defer_poll(&s), 0);
    EXPECT_INT(tick_defer_poll(&s), 1);   // normal timing, no stale carry-over
    return 1;
}

static int test_tick_defer_sustained_requests_starve_the_tick(void) {
    // Pins the worst case explicitly so it is a known property rather than a
    // surprise: a request before EVERY poll means the tick never fires. Needs
    // deterministic sub-period spacing, which Poisson arrivals never sustain.
    tick_defer_state_t s = {0};
    for (int i = 0; i < 20; i++) {
        tick_defer_request(&s, 1);
        EXPECT_INT(tick_defer_poll(&s), 0);
    }
    // One quiet period is all it takes to recover.
    EXPECT_INT(tick_defer_poll(&s), 1);
    return 1;
}

static int test_tick_defer_clear_cancels_pending(void) {
    // speaker_set_modes() and the melody end-of-sequence path both drop any
    // queued tick. The defer countdown must go with it, or a stale counter
    // survives into the next request.
    tick_defer_state_t s = {0};
    tick_defer_request(&s, 1);
    tick_defer_clear(&s);
    EXPECT_INT(tick_defer_poll(&s), 0);
    EXPECT_INT(tick_defer_poll(&s), 0);
    EXPECT_INT(s.defer_left, 0);
    EXPECT_INT(s.pending, 0);
    return 1;
}

// --- tls_logic.h: fingerprint_hex ------------------------------------------
// V2.8.0: the /status "Certificate" line and the boot log both print the
// SHA-256 fingerprint of the self-signed cert. 32 bytes -> 95 chars + NUL.

static int test_fingerprint_hex_32_bytes(void) {
    uint8_t d[32];
    for (int i = 0; i < 32; i++) d[i] = (uint8_t)(i * 8 + 1);   // 01 09 11 ... F9
    char out[96];
    EXPECT_INT(fingerprint_hex(d, sizeof(d), out, sizeof(out)), 1);
    EXPECT_INT((int)strlen(out), 95);
    EXPECT_INT(strncmp(out, "01:09:11:19:", 12), 0);
    EXPECT_STREQ(out + 93, "F9");
    return 1;
}

static int test_fingerprint_hex_uppercase_and_colons(void) {
    uint8_t d[3] = { 0xAB, 0x00, 0xFF };
    char out[16];
    EXPECT_INT(fingerprint_hex(d, 3, out, sizeof(out)), 1);
    EXPECT_STREQ(out, "AB:00:FF");
    return 1;
}

static int test_fingerprint_hex_exact_fit(void) {
    uint8_t d[2] = { 0x12, 0x34 };
    char out[6];   // "12:34" + NUL = 6 = n*3 exactly
    EXPECT_INT(fingerprint_hex(d, 2, out, sizeof(out)), 1);
    EXPECT_STREQ(out, "12:34");
    return 1;
}

static int test_fingerprint_hex_too_small_fails_and_empties(void) {
    uint8_t d[2] = { 0x12, 0x34 };
    char out[5] = "junk";
    EXPECT_INT(fingerprint_hex(d, 2, out, sizeof(out)), 0);
    EXPECT_STREQ(out, "");
    return 1;
}

static int test_fingerprint_hex_zero_len_fails(void) {
    uint8_t d[1] = { 0 };
    char out[4] = "x";
    EXPECT_INT(fingerprint_hex(d, 0, out, sizeof(out)), 0);
    EXPECT_STREQ(out, "");
    return 1;
}

// --- tls_logic.h: https_redirect_location ----------------------------------
// V2.8.0: the :80 instance answers every non-plain request with
// "301 Location: https://<Host without :port><uri>". Host is attacker-
// controlled text from the request, so the builder must fail closed on
// anything odd rather than emit a half-formed URL.

static int test_redirect_basic(void) {
    char out[128];
    EXPECT_INT(https_redirect_location("10.0.0.7", "/config", out, sizeof(out)), 1);
    EXPECT_STREQ(out, "https://10.0.0.7/config");
    return 1;
}

static int test_redirect_strips_port(void) {
    char out[128];
    EXPECT_INT(https_redirect_location("10.0.0.7:80", "/", out, sizeof(out)), 1);
    EXPECT_STREQ(out, "https://10.0.0.7/");
    return 1;
}

static int test_redirect_keeps_query(void) {
    char out[128];
    EXPECT_INT(https_redirect_location("geiger.lan", "/log?tail=1", out, sizeof(out)), 1);
    EXPECT_STREQ(out, "https://geiger.lan/log?tail=1");
    return 1;
}

static int test_redirect_rejects_empty_host(void) {
    char out[128] = "junk";
    EXPECT_INT(https_redirect_location("", "/", out, sizeof(out)), 0);
    EXPECT_STREQ(out, "");
    return 1;
}

static int test_redirect_rejects_port_only_host(void) {
    char out[128] = "junk";
    EXPECT_INT(https_redirect_location(":80", "/", out, sizeof(out)), 0);
    EXPECT_STREQ(out, "");
    return 1;
}

static int test_redirect_rejects_ipv6_literal(void) {
    char out[128] = "junk";
    EXPECT_INT(https_redirect_location("[fe80::1]:80", "/", out, sizeof(out)), 0);
    EXPECT_STREQ(out, "");
    return 1;
}

static int test_redirect_rejects_uri_without_slash(void) {
    char out[128] = "junk";
    EXPECT_INT(https_redirect_location("10.0.0.7", "config", out, sizeof(out)), 0);
    EXPECT_STREQ(out, "");
    return 1;
}

static int test_redirect_truncation_fails_and_empties(void) {
    char out[20] = "junk";   // "https://10.0.0.7/config" is 23 chars
    EXPECT_INT(https_redirect_location("10.0.0.7", "/config", out, sizeof(out)), 0);
    EXPECT_STREQ(out, "");
    return 1;
}

static int test_redirect_exact_fit(void) {
    char out[24];   // 23 chars + NUL
    EXPECT_INT(https_redirect_location("10.0.0.7", "/config", out, sizeof(out)), 1);
    EXPECT_STREQ(out, "https://10.0.0.7/config");
    return 1;
}

// The three below document what the builder deliberately does NOT do: it
// strips the port and rejects an empty/IPv6-literal host, but it does not
// validate the remaining characters. A trailing dot is a legal FQDN; a space
// or an "@" makes a URL the browser refuses to follow, which is an acceptable
// outcome for a Host header that was already nonsense.

static int test_redirect_host_trailing_dot_passes_through(void) {
    char out[128];
    EXPECT_INT(https_redirect_location("node.lan.", "/", out, sizeof(out)), 1);
    EXPECT_STREQ(out, "https://node.lan./");
    return 1;
}

static int test_redirect_host_with_space_passes_through(void) {
    char out[128];
    EXPECT_INT(https_redirect_location("bad host", "/", out, sizeof(out)), 1);
    EXPECT_STREQ(out, "https://bad host/");
    return 1;
}

static int test_redirect_host_with_at_passes_through(void) {
    char out[128];
    EXPECT_INT(https_redirect_location("user@evil", "/", out, sizeof(out)), 1);
    EXPECT_STREQ(out, "https://user@evil/");
    return 1;
}

static int test_redirect_null_args_fail(void) {
    char out[8] = "junk";
    EXPECT_INT(https_redirect_location(NULL, "/", out, sizeof(out)), 0);
    EXPECT_INT(https_redirect_location("h", NULL, out, sizeof(out)), 0);
    EXPECT_INT(https_redirect_location("h", "/", NULL, 8), 0);
    EXPECT_INT(https_redirect_location("h", "/", out, 0), 0);
    return 1;
}

// --- tls_logic.h: civil_to_epoch / epoch_to_generalized_time -----------------
// V2.8.0: certificate validity is written as X.509 GeneralizedTime in UTC and
// read back from mbedtls_x509_time fields. The device's TZ is user-set (ntp.c
// calls setenv("TZ")), so mktime()/localtime() are unusable here; these two
// are TZ-free by construction. Reference values from `date -u -d @N`.

static int test_civil_epoch_origin(void) {
    EXPECT_INT(civil_to_epoch(1970, 1, 1, 0, 0, 0), 0);
    return 1;
}

static int test_civil_epoch_2026_01_01(void) {
    EXPECT_INT((long)civil_to_epoch(2026, 1, 1, 0, 0, 0), 1767225600L);
    return 1;
}

static int test_civil_epoch_leap_day(void) {
    // 2000-03-01 00:00Z = 951868800; the day before is 2000-02-29.
    EXPECT_INT((long)civil_to_epoch(2000, 2, 29, 0, 0, 0), 951868800L - 86400L);
    return 1;
}

static int test_civil_epoch_time_of_day(void) {
    EXPECT_INT((long)civil_to_epoch(2026, 1, 1, 12, 34, 56), 1767225600L + 12L * 3600 + 34L * 60 + 56);
    return 1;
}

static int test_generalized_time_format(void) {
    char out[15];
    EXPECT_INT(epoch_to_generalized_time(1767225600LL, out, sizeof(out)), 1);
    EXPECT_STREQ(out, "20260101000000");
    return 1;
}

static int test_generalized_time_800_days_later(void) {
    // 2026-01-01 + 800 d = 2028-03-11 (2026: 365, 2027: 365, 2028 is leap:
    // Jan 31 + Feb 29 + Mar 10 = 70 -> day 800 lands on Mar 11).
    char out[15];
    EXPECT_INT(epoch_to_generalized_time(1767225600LL + 800LL * 86400, out, sizeof(out)), 1);
    EXPECT_STREQ(out, "20280311000000");
    return 1;
}

static int test_generalized_time_roundtrip(void) {
    const int64_t t = civil_to_epoch(2031, 12, 31, 23, 59, 59);
    char out[15];
    EXPECT_INT(epoch_to_generalized_time(t, out, sizeof(out)), 1);
    EXPECT_STREQ(out, "20311231235959");
    return 1;
}

static int test_generalized_time_buffer_too_small(void) {
    char out[14] = "junk";
    EXPECT_INT(epoch_to_generalized_time(0, out, sizeof(out)), 0);
    EXPECT_STREQ(out, "");
    return 1;
}

static int test_generalized_time_negative_fails(void) {
    // Pre-1970 has no GeneralizedTime here (the era maths would still produce
    // a year, but a certificate that old is a bug, not a date).
    char out[15] = "junk";
    EXPECT_INT(epoch_to_generalized_time(-1, out, sizeof(out)), 0);
    EXPECT_STREQ(out, "");
    return 1;
}

// --- util.h: V2.8.5 helpers -------------------------------------------------
// parse_long_strict / parse_float_strict replace strtol(val, NULL, 10) in the
// /config schema dispatch, where "abc" used to store 0.

static int test_parse_long_strict_accepts_numbers(void) {
    long v = -1;
    EXPECT_INT(parse_long_strict("150000", &v), 1);  EXPECT_INT(v, 150000);
    EXPECT_INT(parse_long_strict("-7", &v), 1);      EXPECT_INT(v, -7);
    EXPECT_INT(parse_long_strict("+3", &v), 1);      EXPECT_INT(v, 3);
    EXPECT_INT(parse_long_strict(" 42", &v), 1);     EXPECT_INT(v, 42);
    EXPECT_INT(parse_long_strict("0", &v), 1);       EXPECT_INT(v, 0);
    return 1;
}

static int test_parse_long_strict_rejects_garbage(void) {
    long v = 99;
    EXPECT_INT(parse_long_strict("abc", &v), 0);
    EXPECT_INT(parse_long_strict("12abc", &v), 0);
    EXPECT_INT(parse_long_strict("", &v), 0);
    EXPECT_INT(parse_long_strict("0x10", &v), 0);
    EXPECT_INT(parse_long_strict("1.5", &v), 0);
    EXPECT_INT(parse_long_strict("4 ", &v), 0);
    EXPECT_INT(parse_long_strict(NULL, &v), 0);
    EXPECT_INT(parse_long_strict("5", NULL), 0);
    EXPECT_INT(v, 99);   // never written on failure
    return 1;
}

static int test_parse_float_strict(void) {
    float f = -1.0f;
    EXPECT_INT(parse_float_strict("2.47", &f), 1);
    if (fabsf(f - 2.47f) > 1e-6f) return 0;
    EXPECT_INT(parse_float_strict("-0.5", &f), 1);
    if (fabsf(f + 0.5f) > 1e-6f) return 0;
    f = 7.0f;
    EXPECT_INT(parse_float_strict("abc", &f), 0);
    EXPECT_INT(parse_float_strict("1.5x", &f), 0);
    EXPECT_INT(parse_float_strict("", &f), 0);
    if (f != 7.0f) return 0;   // untouched on failure
    // nan / inf parse, but no range check can accept them.
    EXPECT_INT(parse_float_strict("nan", &f), 1);
    if (f >= 0.0f && f <= 100.0f) return 0;
    EXPECT_INT(parse_float_strict("inf", &f), 1);
    if (f >= 0.0f && f <= 100.0f) return 0;
    return 1;
}

static int test_tick_interval_plain(void) {
    EXPECT_INT(tick_interval_elapsed(1000, 0, 1000), 1);
    EXPECT_INT(tick_interval_elapsed(999, 0, 1000), 0);
    EXPECT_INT(tick_interval_remaining(400, 0, 1000), 600);
    EXPECT_INT(tick_interval_remaining(1500, 0, 1000), 0);
    return 1;
}

static int test_tick_interval_across_wrap(void) {
    // The cycle ran 100 ticks before the counter wraps; interval 15000
    // (150 s at 100 Hz). From `last` to 0 is exactly 100 ticks.
    const uint32_t last = UINT32_MAX - 99u;
    EXPECT_INT(tick_interval_elapsed(5u, last, 15000u), 0);        // 105 elapsed
    EXPECT_INT(tick_interval_remaining(5u, last, 15000u), 15000 - 105);
    EXPECT_INT(tick_interval_elapsed(14899u, last, 15000u), 0);    // 14999 elapsed
    EXPECT_INT(tick_interval_elapsed(14900u, last, 15000u), 1);    // 15000 elapsed
    // The old `now >= next` form, with next = last + interval computed
    // before the wrap: next wraps to 14900, so every tick still BEFORE the
    // wrap compares true and the cycle fires back to back. The new form
    // sees only 49 ticks elapsed there.
    const uint32_t old_next   = last + 15000u;         // wrapped to 14900
    const uint32_t pre_wrap   = UINT32_MAX - 50u;
    EXPECT_INT(pre_wrap >= old_next, 1);               // the bug
    EXPECT_INT(tick_interval_elapsed(pre_wrap, last, 15000u), 0);
    return 1;
}

static int test_fmt_kilo4_bands(void) {
    char b[8];
    fmt_kilo4(b, sizeof(b), 0.0f);      EXPECT_STREQ(b, "   0");
    fmt_kilo4(b, sizeof(b), 234.4f);    EXPECT_STREQ(b, " 234");
    fmt_kilo4(b, sizeof(b), 999.6f);    EXPECT_STREQ(b, "1000");
    fmt_kilo4(b, sizeof(b), 3420.0f);   EXPECT_STREQ(b, "3.4k");
    fmt_kilo4(b, sizeof(b), 9949.0f);   EXPECT_STREQ(b, "9.9k");
    fmt_kilo4(b, sizeof(b), 150000.0f); EXPECT_STREQ(b, "99k ");
    return 1;
}

static int test_fmt_kilo4_9950_boundary_stays_4_chars(void) {
    // 9950..9999 used to print "10.0k" (5 chars) and overflow the OLED row.
    char b[8];
    const float vals[] = { 9950.0f, 9975.0f, 9999.9f, 10000.0f, 10499.0f };
    for (size_t i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
        fmt_kilo4(b, sizeof(b), vals[i]);
        EXPECT_INT(strlen(b), 4);
        EXPECT_STREQ(b, "10k ");
    }
    return 1;
}

static int test_tz_offset_add_colon(void) {
    char b[40] = "2026-06-13T20:45:50+1000";
    EXPECT_INT(tz_offset_add_colon(b, strlen(b), sizeof(b)), 25);
    EXPECT_STREQ(b, "2026-06-13T20:45:50+10:00");
    char m[40] = "2026-01-01T00:00:00-0330";
    tz_offset_add_colon(m, strlen(m), sizeof(m));
    EXPECT_STREQ(m, "2026-01-01T00:00:00-03:30");
    return 1;
}

static int test_tz_offset_add_colon_refuses_unsafe(void) {
    // No room for the extra byte: unchanged.
    char tight[25] = "2026-06-13T20:45:50+1000";   // 24 chars + NUL, exactly full
    EXPECT_INT(tz_offset_add_colon(tight, 24, sizeof(tight)), 24);
    EXPECT_STREQ(tight, "2026-06-13T20:45:50+1000");
    // Tail is not an offset: unchanged.
    char z[40] = "2026-06-13T20:45:50Z";
    EXPECT_INT(tz_offset_add_colon(z, strlen(z), sizeof(z)), strlen("2026-06-13T20:45:50Z"));
    EXPECT_STREQ(z, "2026-06-13T20:45:50Z");
    char shortb[8] = "+10";
    EXPECT_INT(tz_offset_add_colon(shortb, 3, sizeof(shortb)), 3);
    return 1;
}

static int test_html_esc_zero_bufsz_writes_nothing(void) {
    char guard[2] = { 'X', 'Y' };
    html_esc("a&b", guard, 0);
    EXPECT_INT(guard[0], 'X');
    EXPECT_INT(guard[1], 'Y');
    return 1;
}

// ----------------------------------------------------------------------------
// Runner
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// html_esc contract  (V2.8.8, review 2.1 C1)
// ----------------------------------------------------------------------------
// ESC_WORST(n) (util.h) is how every /status and /config escape buffer is
// sized. These pin the two halves of its contract — n all-quote chars fit in
// ESC_WORST(n) bytes, one byte less drops exactly the last char — and fuzz the
// general promise: never write past bufsz, always terminate, never emit a raw
// metacharacter, and only ever stop BETWEEN input chars (no half entity).

#define ESC_CANARY 0x5A

// Reverse html_esc for the checks below. Returns the decoded length, or -1 if
// `s` holds a raw metacharacter or an '&' that starts none of the 4 entities.
static int html_unesc(const char *s, char *out, size_t outsz) {
    static const struct { const char *ent; char c; } map[] = {
        { "&amp;", '&' }, { "&quot;", '"' }, { "&lt;", '<' }, { "&gt;", '>' },
    };
    size_t o = 0;
    while (*s) {
        if (*s == '<' || *s == '>' || *s == '"') return -1;
        if (*s == '&') {
            size_t k;
            for (k = 0; k < sizeof(map) / sizeof(map[0]); k++) {
                size_t len = strlen(map[k].ent);
                if (strncmp(s, map[k].ent, len) == 0) { s += len; break; }
            }
            if (k == sizeof(map) / sizeof(map[0])) return -1;
            if (o + 1 >= outsz) return -1;
            out[o++] = map[k].c;
            continue;
        }
        if (o + 1 >= outsz) return -1;
        out[o++] = *s++;
    }
    out[o] = 0;
    return (int)o;
}

static int test_html_esc_worst_case_fits(void) {
    for (size_t n = 1; n <= 64; n++) {
        char in[65];
        char out[ESC_WORST(64) + 8];
        memset(in, '"', n);
        in[n] = 0;
        memset(out, ESC_CANARY, sizeof(out));
        html_esc(in, out, ESC_WORST(n));
        EXPECT_INT(strlen(out), 6 * n);                       // every quote kept
        for (size_t i = 0; i < n; i++) EXPECT_TRUE(strncmp(out + 6 * i, "&quot;", 6) == 0);
        for (size_t i = ESC_WORST(n); i < sizeof(out); i++)   // nothing past bufsz
            EXPECT_INT((unsigned char)out[i], ESC_CANARY);
    }
    return 1;
}

static int test_html_esc_one_byte_short_drops_last_char(void) {
    // ESC_WORST(n) - 1 = 6n+1: the documented reason for the "+2". The last
    // quote no longer fits; the output is the first n-1, cleanly terminated.
    for (size_t n = 1; n <= 64; n++) {
        char in[65];
        char out[ESC_WORST(64) + 8];
        memset(in, '"', n);
        in[n] = 0;
        memset(out, ESC_CANARY, sizeof(out));
        html_esc(in, out, ESC_WORST(n) - 1);
        EXPECT_INT(strlen(out), 6 * (n - 1));
        for (size_t i = ESC_WORST(n) - 1; i < sizeof(out); i++)
            EXPECT_INT((unsigned char)out[i], ESC_CANARY);
    }
    return 1;
}

static int test_html_esc_fuzz_contract(void) {
    // Deterministic LCG so a failure reproduces. Alphabet: the 4 metas, plain
    // ASCII, control chars and high bytes (UTF-8 lead/continuation bytes).
    static const char alpha[] = "&\"<>aZ9 =;'/\t\x01\x7f\x80\xc3\xa9\xff";
    uint32_t seed = 0x2B992DDFu;
    for (int iter = 0; iter < 20000; iter++) {
        char in[81], out[600], dec[90];
        seed = seed * 1664525u + 1013904223u;
        size_t len = (seed >> 8) % 81;
        for (size_t i = 0; i < len; i++) {
            seed = seed * 1664525u + 1013904223u;
            in[i] = alpha[(seed >> 16) % (sizeof(alpha) - 1)];
        }
        in[len] = 0;
        seed = seed * 1664525u + 1013904223u;
        size_t bufsz = 1 + (seed >> 8) % (sizeof(out) - 16);
        memset(out, ESC_CANARY, sizeof(out));
        html_esc(in, out, bufsz);
        for (size_t i = bufsz; i < sizeof(out); i++)          // no write past bufsz
            EXPECT_INT((unsigned char)out[i], ESC_CANARY);
        EXPECT_TRUE(memchr(out, 0, bufsz) != NULL);           // terminated inside
        int d = html_unesc(out, dec, sizeof(dec));
        EXPECT_TRUE(d >= 0);                                  // no raw meta, no half entity
        EXPECT_TRUE(strncmp(dec, in, (size_t)d) == 0);        // a prefix of the input
        if (bufsz >= ESC_WORST(len)) EXPECT_INT(d, len);      // ...and all of it when sized
    }
    return 1;
}

// ----------------------------------------------------------------------------
// config schema  (review 2.1 C3a)
// ----------------------------------------------------------------------------
// main/config_fields.def is the single list of runtime settings: config.h and
// config.c expand it with their own X_* macros into the struct, the defaults,
// the NVS load/save loops and the POST pre-clear + per-field dispatch that
// http_server.c's config_post() calls. Nothing
// checked the list itself, so a mistake in a new line (a 16-char key, a
// duplicate, a default outside its own bounds) compiled cleanly and surfaced
// on a node: NVS rejects the key, or the setting can never be saved. The rows
// below are expanded from the same file, so these tests follow every edit.
// The /config form side is checked against the source text of http_server.c:
// the form is one big format string there, not something the host can render.

#include "config.h"   // config_t + CFG_*_MAX; its esp_err.h resolves to test/shim/

/** @brief One X_* line of config_fields.def, as data. */
typedef struct {
    const char *key;      ///< NVS key, which is also the HTML form field name
    char        type;     ///< 'S' string, 'B' bool, 'U' u32, 'F' f32, '8' u8
    size_t      size;     ///< declared buffer size (strings only)
    size_t      member;   ///< sizeof the generated config_t member (strings only)
    const char *sdef;     ///< string default (strings only)
    double      def;      ///< numeric / bool default
    double      lo, hi;   ///< validation bounds the POST dispatch applies
} schema_row_t;

static const schema_row_t k_schema[] = {
#define X_STR(name, size, key, def) \
    { key, 'S', (size), sizeof(((config_t *)0)->name), def, 0, 0, 0 },
#define X_BOOL(name, key, def)        { key, 'B', 0, 0, NULL, (def), 0, 1 },
#define X_U32(name, key, def, lo, hi) { key, 'U', 0, 0, NULL, (def), (lo), (hi) },
#define X_F32(name, key, def, lo, hi) { key, 'F', 0, 0, NULL, (def), (lo), (hi) },
#define X_U8(name, key, def, lo, hi)  { key, '8', 0, 0, NULL, (def), (lo), (hi) },
#include "config_fields.def"
#undef X_STR
#undef X_BOOL
#undef X_U32
#undef X_F32
#undef X_U8
};
#define SCHEMA_N (sizeof(k_schema) / sizeof(k_schema[0]))

// Form keys POSTed by the /config page (and its LoRa re-join form) that are
// deliberately NOT settings: the two submit buttons config_post() reads, and
// the DevNonce wipe checkbox of the separate re-join POST.
static const char *const k_form_control_keys[] = { "save", "save_restart", "wipe_nonces" };

static bool schema_has_key(const char *k) {
    for (size_t i = 0; i < SCHEMA_N; i++)
        if (strcmp(k_schema[i].key, k) == 0) return true;
    return false;
}

/** @brief Read a whole text file into a malloc'd NUL-terminated buffer, or NULL. */
static char *read_text_file(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    char *buf = NULL;
    if (fseek(f, 0, SEEK_END) == 0) {
        long len = ftell(f);
        if (len > 0 && fseek(f, 0, SEEK_SET) == 0) {
            buf = malloc((size_t)len + 1);
            if (buf && fread(buf, 1, (size_t)len, f) != (size_t)len) {
                free(buf);
                buf = NULL;
            }
            if (buf) buf[len] = '\0';
        }
    }
    fclose(f);
    return buf;
}

// The tests run from the repo root (CI checkout, _test.cmd cd's there).
#define HTTP_SERVER_SRC "main/http_server.c"

static int test_schema_expands(void) {
    // A broken expansion (an X_* macro swallowing lines) would make every
    // per-row test below pass vacuously. 91 fields today; the bound is loose
    // so removing a setting does not need a test edit.
    EXPECT_TRUE(SCHEMA_N > 50);
    return 1;
}

static int test_schema_keys_fit_nvs_and_form(void) {
    // NVS_KEY_NAME_MAX_SIZE is 16 including the NUL, so 15 characters at most;
    // a longer key fails every nvs_set_* at runtime and the setting never
    // persists. The key is also the form field name, and config_post() does
    // not URL-decode keys, so only characters a browser sends unencoded work;
    // the house convention is lowercase, digits and '_'.
    for (size_t i = 0; i < SCHEMA_N; i++) {
        const char *k = k_schema[i].key;
        size_t len = strlen(k);
        if (len == 0 || len > 15) {
            printf("    key '%s': %u chars, NVS allows 1..15\n", k, (unsigned)len);
            return 0;
        }
        for (const char *c = k; *c; c++) {
            if (!((*c >= 'a' && *c <= 'z') || (*c >= '0' && *c <= '9') || *c == '_')) {
                printf("    key '%s': character '%c' outside [a-z0-9_]\n", k, *c);
                return 0;
            }
        }
    }
    return 1;
}

static int test_schema_keys_unique(void) {
    // Two fields on one key would share one NVS slot (last save wins on both)
    // and the POST dispatch would only ever reach the first.
    for (size_t i = 0; i < SCHEMA_N; i++) {
        for (size_t j = i + 1; j < SCHEMA_N; j++) {
            if (strcmp(k_schema[i].key, k_schema[j].key) == 0) {
                printf("    key '%s' used by rows %u and %u\n", k_schema[i].key,
                       (unsigned)i, (unsigned)j);
                return 0;
            }
        }
    }
    return 1;
}

static int test_schema_numeric_defaults_within_bounds(void) {
    // The POST dispatch rejects values outside [lo, hi]. A default outside
    // them is a setting the user cannot save back unchanged: the untouched
    // form re-POSTs it and /config reports it as rejected.
    for (size_t i = 0; i < SCHEMA_N; i++) {
        const schema_row_t *r = &k_schema[i];
        if (r->type == 'S' || r->type == 'B') continue;
        if (!(isfinite(r->def) && isfinite(r->lo) && isfinite(r->hi)) ||
            r->lo > r->hi || r->def < r->lo || r->def > r->hi) {
            printf("    key '%s': default %g outside [%g, %g]\n", r->key, r->def, r->lo, r->hi);
            return 0;
        }
    }
    return 1;
}

static int test_schema_integer_bounds_fit_target_types(void) {
    // The integer dispatch parses into a `long` and compares with (long)lo /
    // (long)hi. On the target `long` is 32 bits, so a bound above INT32_MAX
    // wraps negative and rejects every value; a negative lo would let a
    // negative number through to a uint32_t / uint8_t store, where it wraps
    // huge. U8 fields must also fit their uint8_t.
    for (size_t i = 0; i < SCHEMA_N; i++) {
        const schema_row_t *r = &k_schema[i];
        if (r->type != 'U' && r->type != '8') continue;
        double top = (r->type == '8') ? 255.0 : 2147483647.0;
        if (r->lo < 0 || r->hi > top) {
            printf("    key '%s': bounds [%g, %g] do not fit %s on a 32-bit target\n",
                   r->key, r->lo, r->hi, r->type == '8' ? "uint8_t" : "uint32_t via long");
            return 0;
        }
    }
    return 1;
}

static int test_schema_string_defaults_fit(void) {
    // config_defaults() copies the default with safe_strcpy(.., size), which
    // truncates silently, so an over-long default would ship cut short.
    for (size_t i = 0; i < SCHEMA_N; i++) {
        const schema_row_t *r = &k_schema[i];
        if (r->type != 'S') continue;
        EXPECT_INT(r->member, r->size);   // struct member generated at the declared size
        if (strlen(r->sdef) >= r->size) {
            printf("    key '%s': default is %u chars, buffer holds %u\n", r->key,
                   (unsigned)strlen(r->sdef), (unsigned)(r->size - 1));
            return 0;
        }
    }
    return 1;
}

static int test_schema_every_key_on_config_form(void) {
    // A setting with no form field cannot be changed from /config, and the
    // pre-clear in config_post() turns every bool absent from the POST off on
    // each save, so a missing checkbox silently resets its setting.
    char *src = read_text_file(HTTP_SERVER_SRC);
    if (!src) {
        printf("    cannot read %s (run the tests from the repo root)\n", HTTP_SERVER_SRC);
        return 0;
    }
    int ok = 1;
    for (size_t i = 0; i < SCHEMA_N; i++) {   // report every missing key, not just the first
        char needle[64];
        // In the C source the attribute is written name=\"key\".
        snprintf(needle, sizeof(needle), "name=\\\"%s\\\"", k_schema[i].key);
        if (!strstr(src, needle)) {
            printf("    key '%s' has no form field (%s) in %s\n", k_schema[i].key, needle,
                   HTTP_SERVER_SRC);
            ok = 0;
        }
    }
    free(src);
    return ok;
}

static int test_config_form_fields_are_schema_keys(void) {
    // The reverse direction: a form field whose name is not a schema key is
    // ignored by config_post(), so the user's input is dropped without a
    // message. Only the control keys listed above are allowed to be extra.
    char *src = read_text_file(HTTP_SERVER_SRC);
    if (!src) {
        printf("    cannot read %s (run the tests from the repo root)\n", HTTP_SERVER_SRC);
        return 0;
    }
    int ok = 1, seen = 0;
    const char *open = "name=\\\"";
    for (const char *p = strstr(src, open); p; p = strstr(p, open)) {   // report all
        // Whole attribute only: skip the tail of e.g. filename=\"..\" (the
        // coredump download's Content-Disposition header). Any non-identifier
        // character before it counts, so a field at the start of a string
        // literal ("name=\"..) is still scanned.
        char before = (p == src) ? ' ' : p[-1];
        bool attr = !(isalnum((unsigned char)before) || before == '_' || before == '-');
        p += strlen(open);
        if (!attr) continue;
        const char *end = strstr(p, "\\\"");
        if (!end || end - p == 0 || end - p > 31) continue;   // not a field name
        char name[32];
        memcpy(name, p, (size_t)(end - p));
        name[end - p] = '\0';
        seen++;
        bool control = false;
        for (size_t c = 0; c < sizeof(k_form_control_keys) / sizeof(k_form_control_keys[0]); c++)
            if (strcmp(name, k_form_control_keys[c]) == 0) control = true;
        if (!control && !schema_has_key(name)) {
            printf("    form field '%s' in %s is not in config_fields.def\n", name,
                   HTTP_SERVER_SRC);
            ok = 0;
        }
    }
    free(src);
    EXPECT_TRUE(seen > 50);   // the scan really found the form, not nothing
    return ok;
}

// ----------------------------------------------------------------------------
// config_form.c  (V2.8.10, review 2.1 C3b)
// ----------------------------------------------------------------------------
// main/config_form.c is the /config POST parser and its cross-field rules,
// moved out of http_server.c::config_post() so it runs here unchanged. Each
// test starts from the compiled-in defaults as the "saved" config (prev), hands
// the parser a copy of it (next, exactly what config_post() does) plus a form
// body, and checks what would be stored. Board capabilities come in as a
// struct, so both sides of every board-dependent rule run on the host even
// though this binary is built as a Heltec V2.

#include "config_form.h"

static config_t g_form_prev, g_form_next;
static config_form_result_t g_form_res;

/** @brief Defaults into prev, next = prev — the state config_post() starts from. */
static void form_reset(void) {
    // memset first so struct padding is identical in both copies and the
    // round-trip memcmp below compares only fields.
    memset(&g_form_prev, 0, sizeof g_form_prev);
    config_defaults(&g_form_prev);
    memcpy(&g_form_next, &g_form_prev, sizeof g_form_next);
    memset(&g_form_res, 0xA5, sizeof g_form_res);   // the parser must set every field
}

/** @brief Capabilities with every board-dependent feature off (a bare board). */
static config_form_caps_t form_caps_none(void) {
    config_form_caps_t c;
    memset(&c, 0, sizeof c);
    return c;
}

/** @brief Run the parser on a copy of `body` (it is decoded in place). */
static void form_apply(const char *body, const config_form_caps_t *caps) {
    char buf[1024];
    safe_strcpy(buf, body, sizeof buf);
    config_apply_form(&g_form_next, &g_form_prev, buf, caps, &g_form_res);
}

/** @brief True if `key` appears in the result's rejected list. */
static bool form_rejected_has(const char *key) {
    return strstr(g_form_res.rejected, key) != NULL;
}

static int test_form_roundtrip_unchanged(void) {
    // The page POSTs every ticked box and every value. Re-submitting the
    // defaults (default-true bools ticked, nothing else) must store exactly
    // what was saved: no rule may move a field that the user did not touch.
    form_reset();
    char body[1024];
    int n = 0;
    for (size_t i = 0; i < SCHEMA_N; i++) {
        if (k_schema[i].type == 'B' && k_schema[i].def != 0)
            n += snprintf(body + n, sizeof body - (size_t)n, "%s=on&", k_schema[i].key);
    }
    snprintf(body + n, sizeof body - (size_t)n, "save=1");
    config_form_caps_t caps = form_caps_none();
    form_apply(body, &caps);
    EXPECT_TRUE(memcmp(&g_form_next, &g_form_prev, sizeof g_form_next) == 0);
    EXPECT_STREQ(g_form_res.rejected, "");
    EXPECT_TRUE(!g_form_res.restart_requested);
    return 1;
}

static int test_form_preclears_bools(void) {
    // An unticked checkbox is simply absent from the body, so absent = false.
    form_reset();
    g_form_prev.send_madavi = true;
    g_form_next.send_madavi = true;
    config_form_caps_t caps = form_caps_none();
    form_apply("save=1", &caps);
    EXPECT_TRUE(!g_form_next.send_madavi);
    return 1;
}

static int test_form_url_decodes(void) {
    form_reset();
    config_form_caps_t caps = form_caps_none();
    form_apply("wifi_ssid=a%20b", &caps);
    EXPECT_STREQ(g_form_next.wifi_ssid, "a b");
    return 1;
}

static int test_form_oled_step(void) {
    // oled_bright accepts 0 (off) or 10..100 in steps of 10, tighter than the
    // schema's generic [0,100]; an off-step value keeps the saved one.
    config_form_caps_t caps = form_caps_none();
    form_reset();
    form_apply("oled_bright=55", &caps);
    EXPECT_INT(g_form_next.oled_brightness_pct, g_form_prev.oled_brightness_pct);
    EXPECT_TRUE(form_rejected_has("oled_bright"));
    form_reset();
    form_apply("oled_bright=50", &caps);
    EXPECT_INT(g_form_next.oled_brightness_pct, 50);
    EXPECT_STREQ(g_form_res.rejected, "");
    form_reset();
    form_apply("oled_bright=0", &caps);
    EXPECT_INT(g_form_next.oled_brightness_pct, 0);
    return 1;
}

static int test_form_strict_numeric(void) {
    // "abc" must not be read as 0 (V2.8.5 strict parse): rejected, value kept.
    form_reset();
    config_form_caps_t caps = form_caps_none();
    form_apply("ftp_int=abc", &caps);
    EXPECT_INT(g_form_next.ftp_interval_min, g_form_prev.ftp_interval_min);
    EXPECT_TRUE(form_rejected_has("ftp_int"));
    return 1;
}

static int test_form_save_restart(void) {
    config_form_caps_t caps = form_caps_none();
    form_reset();
    form_apply("save_restart=1", &caps);
    EXPECT_TRUE(g_form_res.restart_requested);
    form_reset();
    form_apply("save=1", &caps);
    EXPECT_TRUE(!g_form_res.restart_requested);
    return 1;
}

static int test_form_11bg_forces_ht20(void) {
    // 802.11b/g has no HT40, so 11bg-only implies 20 MHz even when the
    // (disabled, therefore not POSTed) ht20 box is absent.
    form_reset();
    config_form_caps_t caps = form_caps_none();
    form_apply("wifi_11bg=on", &caps);
    EXPECT_TRUE(g_form_next.wifi_ht20_only);
    return 1;
}

static int test_form_caps_force_off(void) {
    // Each board-dependent checkbox is forced off where the hardware is
    // missing and kept where it exists: both sides of each rule.
    const char *body = "wifi_ext_a=on&i2c_pinout=on&batt_present=on";
    config_form_caps_t caps = form_caps_none();
    form_reset();
    form_apply(body, &caps);
    EXPECT_TRUE(!g_form_next.use_external_antenna);
    EXPECT_TRUE(!g_form_next.i2c_pinout);
    EXPECT_TRUE(!g_form_next.batt_present);
    caps.has_antenna_switch = true;
    caps.has_i2c_pinout_switch = true;
    caps.has_fuel_gauge = true;
    form_reset();
    form_apply(body, &caps);
    EXPECT_TRUE(g_form_next.use_external_antenna);
    EXPECT_TRUE(g_form_next.i2c_pinout);
    EXPECT_TRUE(g_form_next.batt_present);
    return 1;
}

static int test_form_ftp_ps_preserved(void) {
    // With global power save off, the greyed FTP box is not POSTed; the
    // saved choice must survive instead of being cleared.
    form_reset();
    g_form_prev.ftp_ps_disabled = true;
    g_form_next.ftp_ps_disabled = true;
    config_form_caps_t caps = form_caps_none();
    form_apply("wifi_ps_dis=on", &caps);
    EXPECT_TRUE(g_form_next.ftp_ps_disabled);
    return 1;
}

static int test_form_tube_off_clears(void) {
    // No tube: the radiation-only targets and the count-path filters go off.
    form_reset();
    config_form_caps_t caps = form_caps_none();
    form_apply("send_rad=on&send_gmc=on&send_ts=on&pcnt_filt=on&dt_guard=on&hv_blank=on",
               &caps);
    EXPECT_TRUE(!g_form_next.tube_enabled);
    EXPECT_TRUE(!g_form_next.send_radmon);
    EXPECT_TRUE(!g_form_next.send_gmc);
    EXPECT_TRUE(!g_form_next.send_thingspeak);
    EXPECT_TRUE(!g_form_next.pcnt_filter);
    EXPECT_TRUE(!g_form_next.deadtime_guard);
    EXPECT_TRUE(!g_form_next.hv_blank);
    return 1;
}

static int test_form_pcnt_beats_guard(void) {
    form_reset();
    config_form_caps_t caps = form_caps_none();
    form_apply("tube_en=on&pcnt_filt=on&dt_guard=on", &caps);
    EXPECT_TRUE(g_form_next.pcnt_filter);
    EXPECT_TRUE(!g_form_next.deadtime_guard);
    return 1;
}

static int test_form_mqtt_mode_b_needs_ca(void) {
    // Mode B (custom CA) with an empty CA is refused: previous mode and CA
    // kept, and the reason reported.
    form_reset();
    // Saved mode 2 (not the default 0), so restoring "the saved mode" is told
    // apart from merely resetting to 0.
    g_form_prev.mqtt_tls_mode = 2;
    safe_strcpy(g_form_prev.mqtt_tls_ca, "OLD-CA", sizeof g_form_prev.mqtt_tls_ca);
    memcpy(&g_form_next, &g_form_prev, sizeof g_form_next);
    config_form_caps_t caps = form_caps_none();
    form_apply("mqtt_tls=on&mqtt_tls_m=1&mqtt_tls_ca=", &caps);
    EXPECT_INT(g_form_next.mqtt_tls_mode, 2);
    EXPECT_STREQ(g_form_next.mqtt_tls_ca, "OLD-CA");
    EXPECT_TRUE(form_rejected_has("mqtt_tls_m (Mode B needs a CA certificate)"));
    return 1;
}

static int test_form_rejected_list_truncates(void) {
    // Thirty bad values: the report is cut at its buffer, never past it
    // (the ASan leg turns an overrun into a failure).
    form_reset();
    char body[1024];
    int n = 0;
    for (int i = 0; i < 30; i++)
        n += snprintf(body + n, sizeof body - (size_t)n, "ftp_int=x&");
    snprintf(body + n, sizeof body - (size_t)n, "save=1");
    config_form_caps_t caps = form_caps_none();
    form_apply(body, &caps);
    // Thirty keys joined with ", " would be 268 characters: the list must stop at exactly 159
    // characters + NUL, starting with the first keys in order.
    EXPECT_INT((int)strlen(g_form_res.rejected), (int)sizeof g_form_res.rejected - 1);
    EXPECT_TRUE(strncmp(g_form_res.rejected, "ftp_int, ftp_int", 16) == 0);
    return 1;
}

static int test_form_lora_hex(void) {
    // LoRaWAN boards validate the EUIs/key as exact-length hex; other boards
    // take the generic string copy (the fields exist in every build).
    config_form_caps_t caps = form_caps_none();
    caps.has_lorawan = true;
    form_reset();
    form_apply("lora_deveui=0011223344556677", &caps);
    EXPECT_STREQ(g_form_next.lorawan_deveui, "0011223344556677");
    EXPECT_STREQ(g_form_res.rejected, "");
    form_reset();
    safe_strcpy(g_form_prev.lorawan_deveui, "AAAAAAAAAAAAAAAA", sizeof g_form_prev.lorawan_deveui);
    memcpy(&g_form_next, &g_form_prev, sizeof g_form_next);
    form_apply("lora_deveui=xyz", &caps);
    EXPECT_STREQ(g_form_next.lorawan_deveui, "AAAAAAAAAAAAAAAA");
    EXPECT_TRUE(form_rejected_has("lora_deveui"));
    form_apply("lora_deveui=", &caps);
    EXPECT_STREQ(g_form_next.lorawan_deveui, "");
    form_reset();
    form_apply("lora_appkey=0123456789abcdef0123456789abcde", &caps);   // 31 hex chars
    EXPECT_TRUE(form_rejected_has("lora_appkey"));
    caps.has_lorawan = false;
    form_reset();
    form_apply("lora_deveui=xyz", &caps);
    EXPECT_STREQ(g_form_next.lorawan_deveui, "xyz");
    EXPECT_STREQ(g_form_res.rejected, "");
    return 1;
}

static int test_form_revc_forced_v2_without_switch(void) {
    // Only the Rev C socket boards may move the pulse input; a hand-crafted
    // POST on any other board must still store V2.
    form_reset();
    config_form_caps_t caps = form_caps_none();
    form_apply("revc_carrier=3", &caps);
    EXPECT_INT(g_form_next.revc_carrier, 2);
    return 1;
}

static int test_form_revc_v3_with_switch(void) {
    config_form_caps_t caps = form_caps_none();
    caps.has_carrier_rev_switch = true;
    form_reset();
    EXPECT_INT(g_form_prev.revc_carrier, 2);   // default = V2, every deployed carrier
    form_apply("revc_carrier=3", &caps);
    EXPECT_INT(g_form_next.revc_carrier, 3);
    EXPECT_STREQ(g_form_res.rejected, "");
    form_apply("revc_carrier=2", &caps);
    EXPECT_INT(g_form_next.revc_carrier, 2);
    return 1;
}

static int test_form_revc_out_of_range_rejected(void) {
    // Only 2 and 3 exist; anything else keeps the saved value and is reported.
    static const char *const bad[] = { "revc_carrier=0", "revc_carrier=1",
                                       "revc_carrier=4", "revc_carrier=abc" };
    config_form_caps_t caps = form_caps_none();
    caps.has_carrier_rev_switch = true;
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        form_reset();
        g_form_prev.revc_carrier = 3;
        g_form_next.revc_carrier = 3;
        form_apply(bad[i], &caps);
        EXPECT_INT(g_form_next.revc_carrier, 3);
        EXPECT_TRUE(form_rejected_has("revc_carrier"));
    }
    return 1;
}

static int test_form_revc_fixed_board_stores_3(void) {
    // V2.8.11: a board built for one carrier (XIAO + Wio, V3 only) has no
    // switch, but must store the carrier it really uses, not V2.
    config_form_caps_t caps = form_caps_none();
    caps.carrier_rev_fixed = 3;
    form_reset();
    form_apply("revc_carrier=2", &caps);
    EXPECT_INT(g_form_next.revc_carrier, 3);
    form_reset();
    form_apply("save=1", &caps);
    EXPECT_INT(g_form_next.revc_carrier, 3);
    return 1;
}

static int test_form_lora_hp_forced_off_without_fem(void) {
    // V2.8.11: high-power mode drives the Heltec's GC1109 amplifier. A LoRa board
    // without one (XIAO + Wio) must never store it, even from a ticked box or
    // a config saved on a Heltec.
    config_form_caps_t caps = form_caps_none();
    caps.has_lorawan = true;
    form_reset();
    g_form_prev.lorawan_high_power = true;
    g_form_next.lorawan_high_power = true;
    form_apply("lora_en=on&lora_hp=on", &caps);
    EXPECT_TRUE(!g_form_next.lorawan_high_power);
    EXPECT_TRUE(g_form_next.lorawan_enabled);
    return 1;
}

// V2.8.22: whenever a PA is in the TX path its gain goes to RadioLib, so the
// stack's power (= the band's EIRP limit) is what reaches the antenna. Before,
// a V4.2 with high power on got no offset and sent ~11 dB above the limit.
static int test_lw_pa_offset(void) {
    EXPECT_INT(lw_pa_offset_db(false, false), 0);              // V4.2, PA bypassed
    EXPECT_INT(lw_pa_offset_db(false, true), LW_PA_GAIN_DB);   // V4.2, PA on (the fix)
    EXPECT_INT(lw_pa_offset_db(true, false), LW_PA_GAIN_DB);   // V4.3: PA always in TX
    EXPECT_INT(lw_pa_offset_db(true, true), LW_PA_GAIN_DB);    //   setting has no effect
    // EU868: the stack asks 16 dBm EIRP; the radio gets 16 - 11 = 5 dBm, which
    // Heltec's table (868 MHz, PA on) puts at 15.8 dBm at the antenna port.
    EXPECT_INT(16 - lw_pa_offset_db(false, true), 5);
    return 1;
}

// lorawan_mac.h (V2.8.23) — MAC commands and answers in plain words. Vectors:
// a DevStatusReq/Ans pair seen on air (gateway record: downlink FOpts "06",
// then uplink FOpts "06 ff 09"), the rest built from LoRaWAN 1.0.4 §5.
static char g_mac_buf[LW_MAC_LINE_MAX];

static int test_lw_mac_dev_status_pair(void) {
    const uint8_t down[] = { 0x06 };
    lw_mac_format_down(down, sizeof(down), true, g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "DevStatusReq (asks for battery level and link margin)");
    const uint8_t up[] = { 0x06, 0xFF, 0x09 };
    lw_mac_format_up(up, sizeof(up), g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "DevStatusAns — battery not measured (0xFF), margin +9 dB (SNR of the downlink as received)");
    const uint8_t up2[] = { 0x06, 0x00, 0x3B };   // external power, margin 6-bit -5
    lw_mac_format_up(up2, sizeof(up2), g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "DevStatusAns — battery external power (0), margin -5 dB (SNR of the downlink as received)");
    const uint8_t up3[] = { 0x06, 127, 0x00 };
    lw_mac_format_up(up3, sizeof(up3), g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "DevStatusAns — battery 127 of 254 (50 %), margin +0 dB (SNR of the downlink as received)");
    return 1;
}

static int test_lw_mac_link_adr_fixed_plan(void) {
    // AU915 two-block form: all 125 kHz off + 500 kHz ch 65, then ch 8-15 on;
    // DR5, power index 2, NbTrans 1 in the last block.
    const uint8_t down[] = { 0x03, 0x5F, 0x02, 0x00, 0x70, 0x03, 0x52, 0x00, 0xFF, 0x01 };
    lw_mac_format_down(down, sizeof(down), true, g_mac_buf, sizeof(g_mac_buf));
    // RadioLib applies the run as one block: masks in order, DR/power/NbTrans of the last.
    EXPECT_STREQ(g_mac_buf, "LinkADRReq (2 blocks, applied together) — DR5, TX power index 2 (max EIRP - 4 dB), all "
                 "125 kHz channels off, 500 kHz channels set to: 65, then 125 kHz channels 0-15 set to: 8-15, "
                 "NbTrans 1 (each uplink sent 1 time)");
    const uint8_t sub[] = { 0x03, 0xF0, 0x02, 0x00, 0x53 };   // ChMaskCntl 5: sub-band 2, NbTrans 3
    lw_mac_format_down(sub, sizeof(sub), true, g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "LinkADRReq — data rate unchanged, TX power index 0 (max EIRP), adds channels 8-15 and "
                 "500 kHz channel 65, NbTrans 3 (each uplink sent 3 times)");
    const uint8_t ans[] = { 0x03, 0x07, 0x03, 0x06 };
    lw_mac_format_up(ans, sizeof(ans), g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "LinkADRAns — TX power accepted, data rate accepted, channel mask accepted; LinkADRAns — "
                 "TX power accepted, data rate accepted, channel mask REJECTED");
    return 1;
}

static int test_lw_mac_dynamic_plan_settings(void) {
    // EU868: channels 0-2 + 4, RX2 back to 869.525 MHz DR3, RX1 delay 1 s.
    const uint8_t down[] = { 0x03, 0x00, 0x17, 0x00, 0x00, 0x05, 0x03, 0xD2, 0xAD, 0x84, 0x08, 0x00 };
    lw_mac_format_down(down, sizeof(down), false, g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "LinkADRReq — DR0, TX power index 0 (max EIRP), channels 0-15 set to: 0-2, 4, NbTrans 0 = "
                 "back to the default of 1 transmission per uplink; "
                 "RXParamSetupReq — RX1 DR offset 0, RX2 DR3 at 869.525 MHz; RXTimingSetupReq — RX1 opens 1 s after "
                 "the uplink");
    const uint8_t ch[] = { 0x07, 0x03, 0x18, 0x4F, 0x84, 0x50, 0x07, 0x04, 0x00, 0x00, 0x00, 0x00 };
    lw_mac_format_down(ch, sizeof(ch), false, g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "NewChannelReq — channel 3 at 867.1 MHz, DR0-DR5; NewChannelReq — channel 4 removed");
    const uint8_t up[] = { 0x05, 0x07, 0x08 };
    lw_mac_format_up(up, sizeof(up), g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "RXParamSetupAns — RX1 DR offset accepted, RX2 DR accepted, RX2 frequency accepted "
                 "(repeated in every uplink until the next downlink); RXTimingSetupAns (applied) (repeated in every "
                 "uplink until the next downlink)");
    return 1;
}

static int test_lw_mac_time_duty_tx_param(void) {
    // DeviceTimeAns for 2026-11-09 00:08:56.500 UTC: GPS 1478218154 s + 128/256 s.
    const uint8_t down[] = { 0x0D, 0xAA, 0xD1, 0x1B, 0x58, 0x80, 0x04, 0x07, 0x09, 0x3D, 0x02, 0x0C, 0x01 };
    lw_mac_format_down(down, sizeof(down), true, g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "DeviceTimeAns — network time 2026-11-09 00:08:56.500 UTC at the end of our uplink; "
                 "DutyCycleReq — on air at most 1/128 of the time (MaxDCycle 7); TxParamSetupReq — max EIRP 30 dBm, "
                 "uplink dwell time 400 ms limit, downlink dwell time 400 ms limit; LinkCheckAns — our uplink arrived "
                 "12 dB above the demodulation floor, heard by 1 gateway");
    const uint8_t dc0[] = { 0x04, 0x00 };
    lw_mac_format_down(dc0, sizeof(dc0), false, g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "DutyCycleReq — no extra limit from the network, only the band's own (MaxDCycle 0)");
    return 1;
}

static int test_lw_mac_unknown_incomplete_truncated(void) {
    const uint8_t unk[] = { 0x06, 0x55, 0x01 };   // RadioLib stops at an unknown CID
    lw_mac_format_down(unk, sizeof(unk), true, g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "DevStatusReq (asks for battery level and link margin); unknown MAC command 0x55 — the "
                 "rest is not decoded (RadioLib skips it too): 5501");
    const uint8_t inc[] = { 0x03, 0x52 };
    lw_mac_format_down(inc, sizeof(inc), true, g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "03: incomplete (1 of 4 bytes) — not applied by RadioLib");
    lw_mac_format_down(inc, 0, true, g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "");
    // Fifteen 1-byte commands into a small buffer: cut, terminated, no overrun.
    uint8_t many[15];
    memset(many, 0x06, sizeof(many));
    char small[40];
    memset(small, 'x', sizeof(small));
    lw_mac_format_down(many, sizeof(many), true, small, 32);
    EXPECT_INT(strlen(small), 31);
    for (size_t k = 32; k < sizeof(small); k++) EXPECT_TRUE(small[k] == 'x');   // nothing past the limit
    lw_mac_utc(0, small, sizeof(small));   // the epoch and a leap day render
    EXPECT_STREQ(small, "1970-01-01 00:00:00");
    lw_mac_utc(1709208000u, small, sizeof(small));
    EXPECT_STREQ(small, "2024-02-29 12:00:00");
    return 1;
}

static int test_lw_mac_dynamic_cntl_answerless_mhz(void) {
    // EU868 ChMaskCntl 6 (all on), 5 (banks, no 500 kHz on a dynamic plan),
    // 7 (reserved: RadioLib rejects the mask), and an empty block.
    const uint8_t down[] = { 0x03, 0x5F, 0x00, 0x00, 0x60, 0x03, 0x5F, 0x01, 0x00, 0x50,
                             0x03, 0x5F, 0x00, 0x00, 0x70 };
    lw_mac_format_down(down, sizeof(down), false, g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "LinkADRReq (3 blocks, applied together) — DR5, TX power unchanged, all defined channels "
                 "on, then adds channels 0-7, then ChMaskCntl 7 (reserved in this region, so the node rejects the "
                 "channel mask), NbTrans 0 = back to the default of 1 transmission per uplink");
    const uint8_t empty[] = { 0x03, 0x5F, 0x00, 0x00, 0x01 };
    lw_mac_format_down(empty, sizeof(empty), true, g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "LinkADRReq — DR5, TX power unchanged, 125 kHz channels 0-15 set to: none, NbTrans 1 "
                 "(each uplink sent 1 time)");
    // A LinkADRReq cut short after one full block: the full block alone.
    const uint8_t cut[] = { 0x03, 0x5F, 0x00, 0x00, 0x01, 0x03, 0x5F };
    lw_mac_format_down(cut, sizeof(cut), true, g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "LinkADRReq — DR5, TX power unchanged, 125 kHz channels 0-15 set to: none, NbTrans 1 "
                 "(each uplink sent 1 time); 03: incomplete (1 of 4 bytes) — not applied by RadioLib");
    // A second LinkADRReq after another command: rejected by RadioLib, not applied.
    const uint8_t two[] = { 0x03, 0x5F, 0x00, 0x00, 0x01, 0x06, 0x03, 0x2F, 0x00, 0x00, 0x01 };
    lw_mac_format_down(two, sizeof(two), true, g_mac_buf, sizeof(g_mac_buf));
    EXPECT_STREQ(g_mac_buf, "LinkADRReq — DR5, TX power unchanged, 125 kHz channels 0-15 set to: none, NbTrans 1 "
                 "(each uplink sent 1 time); DevStatusReq (asks for battery level and link margin); LinkADRReq (a "
                 "second, separate one) — not applied: the node rejects it, only one LinkADRReq block per downlink "
                 "counts");
    // Empty answer queue: explained only by answerless commands.
    const uint8_t lc[] = { 0x02, 0x0C, 0x01, 0x0D, 1, 2, 3, 4, 5 };
    EXPECT_TRUE(lw_mac_down_answerless(lc, sizeof(lc)));
    const uint8_t mixed[] = { 0x02, 0x0C, 0x01, 0x06 };
    EXPECT_TRUE(!lw_mac_down_answerless(mixed, sizeof(mixed)));
    EXPECT_TRUE(!lw_mac_down_answerless(lc, 2));   // incomplete
    EXPECT_TRUE(!lw_mac_down_answerless(lc, 0));
    char f[48];
    lw_mac_mhz(8680000u, f, sizeof(f));
    EXPECT_STREQ(f, "868.0");
    lw_mac_mhz(9233000u, f, sizeof(f));
    EXPECT_STREQ(f, "923.3");
    return 1;
}

static int test_rxobs_note_mac_frames(void) {
    rxobs_hit_t h;
    memset(&h, 0, sizeof(h));
    // A MAC-only downlink: MHDR 0x60, DevAddr 260b1234 (made up), FCtrl FOptsLen 1, FCnt 128, FOpts 06, MIC.
    const uint8_t f1[] = { 0x60, 0x34, 0x12, 0x0B, 0x26, 0x01, 0x80, 0x00, 0x06, 1, 2, 3, 4 };
    rxobs_note_mac(&h, f1, sizeof(f1));
    EXPECT_TRUE(h.mac_seen);
    EXPECT_INT(h.fopts_len, 1);
    EXPECT_INT(h.fopts[0], 0x06);
    EXPECT_INT(h.fport, -1);
    // Confirmed down, no FOpts, FPort 1 + 1 byte payload (a test downlink).
    const uint8_t f2[] = { 0xA0, 0x34, 0x12, 0x0B, 0x26, 0x20, 0x81, 0x00, 0x01, 0x01, 1, 2, 3, 4 };
    rxobs_note_mac(&h, f2, sizeof(f2));
    EXPECT_TRUE(h.mac_seen);
    EXPECT_INT(h.fopts_len, 0);
    EXPECT_INT(h.fport, 1);
    EXPECT_INT(h.frm_len, 1);
    // FPort 0: MAC commands in the encrypted payload (3 bytes).
    const uint8_t f3[] = { 0x60, 0x34, 0x12, 0x0B, 0x26, 0x00, 0x82, 0x00, 0x00, 9, 9, 9, 1, 2, 3, 4 };
    rxobs_note_mac(&h, f3, sizeof(f3));
    EXPECT_INT(h.fport, 0);
    EXPECT_INT(h.frm_len, 3);
    // Join accept, and a FOptsLen longer than the frame: nothing kept.
    const uint8_t f4[] = { 0x20, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
    rxobs_note_mac(&h, f4, sizeof(f4));
    EXPECT_TRUE(!h.mac_seen);
    const uint8_t f5[] = { 0x60, 0x34, 0x12, 0x0B, 0x26, 0x0F, 0x80, 0x00, 0x06, 1, 2, 3, 4 };
    rxobs_note_mac(&h, f5, sizeof(f5));
    EXPECT_TRUE(!h.mac_seen);
    return 1;
}

static int test_rxobs_accepted_hit(void) {
    // Foreign frame in #1 RX1 (dropped), ours in #2 RX1: the accepted one is #2's.
    rxt_begin(3); rxt_tx(867.9, 3, 1); rxt_tx(868.3, 2, 0);
    const rxobs_hit_t *h = rxobs_accepted_hit(&g_rxt, 1);
    EXPECT_TRUE(h == &g_rxt.rep[1].w[0]);
    EXPECT_INT(h->rx_addr, RXT_OWN);
    EXPECT_TRUE(rxobs_accepted_hit(&g_rxt, 0) == NULL);         // no downlink
    EXPECT_TRUE(rxobs_accepted_hit(&g_rxt, -1112) == NULL);     // unusable frame
    // Ours in RX2 of the only TX: st 2 picks RX2.
    rxt_begin(1); rxt_tx(867.1, 1, 2);
    EXPECT_TRUE(rxobs_accepted_hit(&g_rxt, 2) == &g_rxt.rep[0].w[1]);
    // Ours in #1 RX1, then a MAC-only TX whose RX1 gets another frame of ours:
    // the accepted frame is the FIRST kept one, the nested one is found after it.
    rxt_begin(1); rxt_tx(867.1, 2, 0); rxt_tx(868.1, 2, 0);
    EXPECT_TRUE(rxobs_accepted_hit(&g_rxt, 1) == &g_rxt.rep[0].w[0]);
    EXPECT_INT(rxobs_split(&g_rxt, 1).own_tx, 1);
    EXPECT_TRUE(rxobs_nested_hit(&g_rxt, 1) == &g_rxt.rep[1].w[0]);
    rxt_begin(1); rxt_tx(867.1, 2, 0); rxt_tx(868.1, 1, 1);   // MAC-only TX heard nothing
    EXPECT_TRUE(rxobs_nested_hit(&g_rxt, 1) == NULL);
    // st 1 but no frame recorded (RX_DONE never read out): nothing to decode.
    rxt_begin(1); rxt_tx(867.1, 4, 0);
    EXPECT_TRUE(rxobs_accepted_hit(&g_rxt, 1) == NULL);
    return 1;
}

static int test_form_lora_hp_kept_with_fem(void) {
    config_form_caps_t caps = form_caps_none();
    caps.has_lorawan  = true;
    caps.has_lora_fem = true;
    form_reset();
    form_apply("lora_hp=on", &caps);
    EXPECT_TRUE(g_form_next.lorawan_high_power);
    form_reset();
    g_form_prev.lorawan_high_power = true;
    g_form_next.lorawan_high_power = true;
    form_apply("save=1", &caps);   // unticked box = off, as for every bool
    EXPECT_TRUE(!g_form_next.lorawan_high_power);
    return 1;
}

// ----------------------------------------------------------------------------
// history.c  (V2.8.8, review 2.1 C2)
// ----------------------------------------------------------------------------
// main/history.c is compiled into this binary unchanged. Its FreeRTOS/esp_log
// includes resolve to test/shim/; the four tube.h / tube_pcnt.h functions it
// reads are faked here, so each test drives the monotonic counters and the ms
// clock directly. Filtered source = pcnt total - wide-blanked total, exactly
// as source_total() composes it.

#include "freertos/semphr.h"   // the shim's declaration, so the definition below is type-checked
int shim_mutex_create_fails = 0;

static bool     g_fake_tube_on = true;
static uint32_t g_fake_raw     = 0;   // tube_get_total_counts()
static uint32_t g_fake_pcnt    = 0;   // tube_pcnt_filtered_total()
static uint32_t g_fake_bw      = 0;   // tube_get_blanked_wide_total()

bool     tube_is_enabled(void)             { return g_fake_tube_on; }
uint32_t tube_get_total_counts(void)       { return g_fake_raw; }
uint32_t tube_pcnt_filtered_total(void)    { return g_fake_pcnt; }
uint32_t tube_get_blanked_wide_total(void) { return g_fake_bw; }

static void hist_fresh(void) {
    g_fake_tube_on = true;
    g_fake_raw = g_fake_pcnt = g_fake_bw = 0;
    shim_mutex_create_fails = 0;
    history_init();
}

// One 60 s sample on the raw source: `cpm` counts arrive, the clock advances.
static void hist_minute(uint32_t *t, uint32_t cpm) {
    g_fake_raw += cpm;
    *t += 60000u;
    history_tick(*t, false);
}

static int test_history_first_tick_only_primes(void) {
    history_snapshot_t s;
    uint32_t t = 1000;
    hist_fresh();
    g_fake_raw = 5000;                    // counts since boot: not a minute's worth
    history_tick(t, false);
    history_get(&s);
    EXPECT_INT(s.min_count, 0);
    hist_minute(&t, 42);
    history_get(&s);
    EXPECT_INT(s.min_count, 1);
    EXPECT_INT(s.cpm_now, 42);            // the delta, not the 5042 total
    EXPECT_INT(s.cpm_min[1], HIST_EMPTY); // unfilled slots carry the sentinel
    return 1;
}

static int test_history_samples_every_60s_not_sooner(void) {
    history_snapshot_t s;
    hist_fresh();
    history_tick(0, false);
    g_fake_raw = 30;
    history_tick(59999, false);
    history_get(&s);
    EXPECT_INT(s.min_count, 0);
    history_tick(60000, false);
    history_get(&s);
    EXPECT_INT(s.min_count, 1);
    EXPECT_INT(s.cpm_now, 30);
    return 1;
}

static int test_history_cpm5_cpm15_ramp(void) {
    history_snapshot_t s;
    uint32_t t = 0;
    hist_fresh();
    history_tick(t, false);
    for (uint32_t m = 1; m <= 15; m++) hist_minute(&t, m * 10);   // 10, 20 … 150
    history_get(&s);
    EXPECT_INT(s.cpm_now, 150);
    EXPECT_INT(s.cpm5, (110 + 120 + 130 + 140 + 150) / 5);       // newest 5
    EXPECT_INT(s.cpm15, (10 + 150) * 15 / 2 / 15);               // all 15
    hist_fresh();                                                // fewer than 5 held:
    t = 0;                                                       // mean of what exists
    history_tick(t, false);
    hist_minute(&t, 10);
    hist_minute(&t, 21);
    history_get(&s);
    EXPECT_INT(s.cpm5, 15);                                      // (10+21)/2, floor
    EXPECT_INT(s.cpm15, 15);
    return 1;
}

static int test_history_clamps_huge_minute(void) {
    history_snapshot_t s;
    uint32_t t = 0;
    hist_fresh();
    history_tick(t, false);
    hist_minute(&t, 70000);
    history_get(&s);
    EXPECT_INT(s.cpm_now, 0xFFFE);        // clamped below the 0xFFFF sentinel
    return 1;
}

static int test_history_top_bit_delta_reads_zero(void) {
    // V2.6.29/V2.6.31: the composed filtered total can fall within a minute
    // (the wide-phantom subtrahend lands as a lump). The unsigned delta then
    // wraps to a top-bit value, which must read as 0, never as 65534.
    history_snapshot_t s;
    hist_fresh();
    g_fake_pcnt = 1000;
    history_tick(0, true);
    g_fake_pcnt = 1010;                   // +10 counts...
    g_fake_bw   = 18;                     // ...but an 18-count lump subtracted
    history_tick(60000, true);
    history_get(&s);
    EXPECT_INT(s.min_count, 1);
    EXPECT_INT(s.cpm_now, 0);
    return 1;
}

static int test_history_source_switch_reprimes(void) {
    history_snapshot_t s;
    uint32_t t = 0;
    hist_fresh();
    history_tick(t, false);
    hist_minute(&t, 50);
    g_fake_pcnt = 9;                      // filtered total: a different magnitude
    t += 60000u;
    history_tick(t, true);                // switch: re-prime, no garbage minute
    history_get(&s);
    EXPECT_INT(s.min_count, 1);
    g_fake_pcnt += 33;
    t += 60000u;
    history_tick(t, true);
    history_get(&s);
    EXPECT_INT(s.min_count, 2);
    EXPECT_INT(s.cpm_now, 33);
    return 1;
}

static int test_history_hourly_rollup(void) {
    history_snapshot_t s;
    uint32_t t = 0;
    hist_fresh();
    history_tick(t, false);
    for (int m = 0; m < 59; m++) hist_minute(&t, (m % 2) ? 40 : 20);
    history_get(&s);
    EXPECT_INT(s.hour_count, 0);          // 59 samples: no hour yet
    hist_minute(&t, 40);
    history_get(&s);
    EXPECT_INT(s.hour_count, 1);
    EXPECT_INT(s.cpm_hour[0], 30);        // mean of 30 × 20 and 30 × 40
    for (int m = 0; m < 60; m++) hist_minute(&t, 7);
    history_get(&s);
    EXPECT_INT(s.hour_count, 2);
    EXPECT_INT(s.cpm_hour[1], 7);         // oldest..newest order
    return 1;
}

static int test_history_minute_ring_wraps_oldest_first(void) {
    history_snapshot_t s;
    uint32_t t = 0;
    hist_fresh();
    history_tick(t, false);
    for (uint32_t m = 1; m <= HIST_MIN_DEPTH + 5; m++) hist_minute(&t, m);
    history_get(&s);
    EXPECT_INT(s.min_count, HIST_MIN_DEPTH);
    EXPECT_INT(s.cpm_min[0], 6);                       // 1..5 overwritten
    EXPECT_INT(s.cpm_min[HIST_MIN_DEPTH - 1], HIST_MIN_DEPTH + 5);
    return 1;
}

static int test_history_ms_clock_wrap(void) {
    // 49.7-day uint32 ms rollover: the 60 s gate is an unsigned difference.
    history_snapshot_t s;
    uint32_t t = 0xFFFFFFFFu - 20000u;
    hist_fresh();
    history_tick(t, false);
    hist_minute(&t, 12);                  // t wraps past 0 here
    history_get(&s);
    EXPECT_INT(s.min_count, 1);
    EXPECT_INT(s.cpm_now, 12);
    return 1;
}

static int test_history_inert_when_disabled(void) {
    history_snapshot_t s;
    uint32_t t = 0, cpm = 777;
    hist_fresh();
    g_fake_tube_on = false;
    history_tick(t, false);
    hist_minute(&t, 99);
    history_get(&s);
    EXPECT_INT(s.min_count, 0);
    EXPECT_TRUE(!history_live_cpm(t, false, &cpm));
    EXPECT_INT(cpm, 777);                 // untouched on false
    hist_fresh();
    shim_mutex_create_fails = 1;          // "mutex alloc failed — history disabled"
    history_init();
    shim_mutex_create_fails = 0;
    t = 0;
    history_tick(t, false);
    hist_minute(&t, 99);
    history_get(&s);
    EXPECT_INT(s.min_count, 0);
    EXPECT_INT(s.cpm_min[0], HIST_EMPTY);
    return 1;
}

// --- live 60 s window (history_live_cpm, V2.8.3) ---

// Call the live window once per second from `*t` for `secs` seconds, adding
// `per_s` counts to the given source each second. Returns the last result.
static bool live_run(uint32_t *t, uint32_t secs, uint32_t per_s, bool filt, uint32_t *cpm) {
    bool ok = false;
    for (uint32_t i = 0; i < secs; i++) {
        *t += 1000u;
        if (filt) g_fake_pcnt += per_s; else g_fake_raw += per_s;
        ok = history_live_cpm(*t, filt, cpm);
    }
    return ok;
}

static int test_live_needs_10s_span(void) {
    uint32_t t = 0, cpm = 777;
    hist_fresh();
    EXPECT_TRUE(!history_live_cpm(t, false, &cpm));          // first sample
    EXPECT_TRUE(!live_run(&t, 9, 2, false, &cpm));           // 9 s span
    EXPECT_INT(cpm, 777);
    EXPECT_TRUE(live_run(&t, 1, 2, false, &cpm));            // 10 s span
    EXPECT_INT(cpm, 120);                                     // 2 counts/s
    return 1;
}

static int test_live_window_is_60s(void) {
    uint32_t t = 0, cpm = 0;
    hist_fresh();
    history_live_cpm(t, false, &cpm);
    live_run(&t, 60, 1, false, &cpm);                         // 60 CPM for a minute
    EXPECT_TRUE(live_run(&t, 60, 3, false, &cpm));            // then 180 CPM
    EXPECT_INT(cpm, 180);                                     // only the last 60 s
    return 1;
}

static int test_live_stall_keeps_prestall_base(void) {
    // Main-task stall > 60 s (daily FTPS upload): the window must stretch back
    // to the last pre-stall sample, not blank or shrink to a seconds-old base.
    uint32_t t = 0, cpm = 0;
    hist_fresh();
    history_live_cpm(t, false, &cpm);
    live_run(&t, 100, 1, false, &cpm);
    t += 100000u;                                             // 100 s, no calls
    g_fake_raw += 100;                                        // counts kept coming
    EXPECT_TRUE(history_live_cpm(t, false, &cpm));
    EXPECT_INT(cpm, 60);                                      // 100 counts / 100 s
    EXPECT_TRUE(live_run(&t, 1, 1, false, &cpm));             // still the old base
    EXPECT_INT(cpm, 60);
    return 1;
}

static int test_live_high_rate_no_overflow(void) {
    // The rate is d * 60000 / span in 64 bits. At 2000 counts/s the 60 s
    // window holds d = 120000, and d * 60000 = 7.2e9 > 2^32: a 32-bit
    // intermediate (e.g. `unsigned long` on the ILP32 target / -m32 leg)
    // wraps to 2905032704 and reads 48417 CPM instead of 120000. Pins the
    // 64-bit maths; on an LP64 host only the -m32 leg can see the difference.
    uint32_t t = 0, cpm = 0;
    hist_fresh();
    history_live_cpm(t, false, &cpm);
    EXPECT_TRUE(live_run(&t, 60, 2000, false, &cpm));
    EXPECT_INT(cpm, 120000);
    return 1;
}

static int test_live_ring_holds_60s_at_900ms_cadence(void) {
    // LIVE_SLOTS (72) must exceed 60 s / LIVE_SAMPLE_MS (67 samples) so the
    // sample ~60 s back is still in the ring at the fastest legal cadence.
    // 80 calls of +10 counts every 900 ms, then 66 silent calls (59.4 s):
    // the 72-slot ring still holds the sample 60.3 s back (the second-last
    // counting call), so the window spans only the last counting call's +10
    // and reads 9 CPM. A ring of 67 or fewer samples spans < 60 s here and
    // would read 0.
    uint32_t t = 0, cpm = 0;
    hist_fresh();
    history_live_cpm(t, false, &cpm);
    for (int i = 0; i < 80; i++) { t += 900u; g_fake_raw += 10; history_live_cpm(t, false, &cpm); }
    bool ok = false;
    for (int i = 0; i < 66; i++) { t += 900u; ok = history_live_cpm(t, false, &cpm); }
    EXPECT_TRUE(ok);
    EXPECT_INT(cpm, 9);
    return 1;
}

static int test_live_sample_gate_900ms(void) {
    // Calls every 500 ms append at most one sample per >= 900 ms (one per
    // 1000 ms here), so the 72 slots still reach 60 s back. Silent for
    // 100 s, then 1 count per call (120 CPM) for 20 s: the true 60 s
    // window holds 40 counts -> 40 CPM. Without the gate every call would
    // append, 72 slots would span only 36 s, and the rate would read ~67.
    uint32_t t = 0, cpm = 0;
    hist_fresh();
    history_live_cpm(t, false, &cpm);
    for (int i = 0; i < 200; i++) { t += 500u; history_live_cpm(t, false, &cpm); }
    bool ok = false;
    for (int i = 0; i < 40; i++) { t += 500u; g_fake_raw += 1; ok = history_live_cpm(t, false, &cpm); }
    EXPECT_TRUE(ok);
    EXPECT_INT(cpm, 40);
    return 1;
}

static int test_live_ms_clock_wrap(void) {
    uint32_t t = 0xFFFFFFFFu - 4500u, cpm = 0;
    hist_fresh();
    history_live_cpm(t, false, &cpm);
    EXPECT_TRUE(live_run(&t, 20, 1, false, &cpm));            // spans the wrap
    EXPECT_INT(cpm, 60);
    return 1;
}

static int test_live_source_switch_and_reset_restart(void) {
    uint32_t t = 0, cpm = 0;
    hist_fresh();
    history_live_cpm(t, false, &cpm);
    EXPECT_TRUE(live_run(&t, 20, 1, false, &cpm));
    EXPECT_TRUE(!live_run(&t, 1, 1, true, &cpm));             // switch: window emptied,
    EXPECT_TRUE(!live_run(&t, 9, 1, true, &cpm));             // restarted at that call
    EXPECT_TRUE(live_run(&t, 1, 1, true, &cpm));              // 10 s on the new source
    history_live_reset();                                     // display off/on: the next
    EXPECT_TRUE(!live_run(&t, 10, 1, true, &cpm));            // call is the first sample
    EXPECT_TRUE(live_run(&t, 1, 1, true, &cpm));
    EXPECT_INT(cpm, 60);
    return 1;
}

static int test_live_subtrahend_step_restarts(void) {
    // PCNT subtract mode: the per-cycle wide-phantom lump changes bw in one
    // step; a window spanning it would read low, so it restarts instead.
    uint32_t t = 0, cpm = 0;
    hist_fresh();
    history_live_cpm(t, true, &cpm);
    EXPECT_TRUE(live_run(&t, 30, 2, true, &cpm));
    EXPECT_INT(cpm, 120);
    g_fake_bw += 18;
    EXPECT_TRUE(!live_run(&t, 1, 2, true, &cpm));             // restarted at this call
    EXPECT_TRUE(!live_run(&t, 9, 2, true, &cpm));
    EXPECT_TRUE(live_run(&t, 1, 2, true, &cpm));
    EXPECT_INT(cpm, 120);                                     // no low reading
    return 1;
}

static int test_live_falling_total_restarts(void) {
    // Backstop: a total below the window base (top-bit delta) restarts the
    // window rather than painting 0 CPM for a live tube.
    uint32_t t = 0, cpm = 0;
    hist_fresh();
    g_fake_pcnt = 500;
    history_live_cpm(t, true, &cpm);
    EXPECT_TRUE(live_run(&t, 15, 1, true, &cpm));
    g_fake_pcnt = 100;                                        // fell, bw unchanged
    EXPECT_TRUE(!live_run(&t, 1, 0, true, &cpm));
    EXPECT_TRUE(live_run(&t, 10, 1, true, &cpm));
    EXPECT_INT(cpm, 60);
    return 1;
}

// --- V2.8.20: OTA rollback probation rule (ota_guard_logic) ---

// Only an image on an OTA slot that is still unconfirmed (NEW written by a
// V2.8.20+ updater on an old bootloader, PENDING_VERIFY on a rollback
// bootloader) needs confirming. UNDEFINED (installed by a firmware without
// rollback tracking), VALID and factory never do.
static int test_ota_guard_needs_confirm_states(void) {
    EXPECT_TRUE(ota_guard_needs_confirm(true, OTA_GUARD_ST_NEW));
    EXPECT_TRUE(ota_guard_needs_confirm(true, OTA_GUARD_ST_PENDING));
    EXPECT_TRUE(!ota_guard_needs_confirm(true, OTA_GUARD_ST_VALID));
    EXPECT_TRUE(!ota_guard_needs_confirm(true, OTA_GUARD_ST_UNDEFINED));
    EXPECT_TRUE(!ota_guard_needs_confirm(true, OTA_GUARD_ST_OTHER));
    // factory (not an OTA slot): never, whatever the state reads
    EXPECT_TRUE(!ota_guard_needs_confirm(false, OTA_GUARD_ST_NEW));
    EXPECT_TRUE(!ota_guard_needs_confirm(false, OTA_GUARD_ST_PENDING));
    EXPECT_TRUE(!ota_guard_needs_confirm(false, OTA_GUARD_ST_VALID));
    return 1;
}

// Probation ends 120 s after the boot AP window closed (or would have).
static int test_ota_guard_probation_window_plus_120s(void) {
    const int64_t over = 200LL * 1000000LL;
    EXPECT_TRUE(!ota_guard_probation_over(over + 119999999LL, over));
    EXPECT_TRUE(ota_guard_probation_over(over + 120000000LL, over));
    EXPECT_TRUE(ota_guard_probation_over(over + 999000000LL, over));
    EXPECT_TRUE(!ota_guard_probation_over(over - 1000000LL, over));  // clock before the latch
    // window not over yet (0 = not latched): never, however late
    EXPECT_TRUE(!ota_guard_probation_over(999000000LL, 0));
    return 1;
}

// A failed probation confirm is retried once a minute, not on every main-loop tick.
static int test_ota_guard_retry_spacing_60s(void) {
    EXPECT_TRUE(ota_guard_retry_due(0, 0));   // nothing failed yet: due at once
    EXPECT_TRUE(ota_guard_retry_due(320LL * 1000000LL, 0));
    const int64_t failed = 320LL * 1000000LL;
    const int64_t next = ota_guard_next_retry_us(failed);
    EXPECT_TRUE(next == failed + 60LL * 1000000LL);
    EXPECT_TRUE(!ota_guard_retry_due(failed, next));
    EXPECT_TRUE(!ota_guard_retry_due(next - 1, next));
    EXPECT_TRUE(ota_guard_retry_due(next, next));
    return 1;
}

// V2.8.8 (review 2.1 A): optional name filter. `run [substring]` runs only the
// tests whose function name contains the substring (e.g. `run history_`),
// for iterating on one area. A filter that matches nothing exits 2, so a typo
// can never look like a green run. g_ran counts what actually executed.
static const char *g_filter = NULL;
static int g_ran = 0;

#define RUN(test_fn)                                                         \
    do {                                                                     \
        if (g_filter && !strstr(#test_fn, g_filter)) break;                  \
        g_ran++;                                                             \
        if (test_fn()) {                                                     \
            printf("  PASS  %s\n", #test_fn);                                \
        } else {                                                             \
            printf("  FAIL  %s\n", #test_fn);                                \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

int main(int argc, char **argv) {
    if (argc > 2) {
        printf("usage: %s [test-name-substring]\n", argv[0]);
        return 2;
    }
    if (argc == 2) g_filter = argv[1];

    printf("== safe_strcpy ==\n");
    RUN(test_safe_strcpy_basic);
    RUN(test_safe_strcpy_truncates);
    RUN(test_safe_strcpy_empty_src);
    RUN(test_safe_strcpy_zero_dstsz);
    RUN(test_safe_strcpy_dstsz_1);
    RUN(test_safe_strcpy_zero_fills_tail);

    printf("== ct_memcmp ==\n");
    RUN(test_ct_memcmp_equal);
    RUN(test_ct_memcmp_diff_first_byte);
    RUN(test_ct_memcmp_diff_last_byte);
    RUN(test_ct_memcmp_zero_length);
    RUN(test_ct_memcmp_realistic_base64);

    printf("== hex_nibble ==\n");
    RUN(test_hex_nibble_digits);
    RUN(test_hex_nibble_lower);
    RUN(test_hex_nibble_upper);
    RUN(test_hex_nibble_invalid);

    printf("== url_decode ==\n");
    RUN(test_url_decode_plain);
    RUN(test_url_decode_plus_to_space);
    RUN(test_url_decode_percent_hex);
    RUN(test_url_decode_lowercase_hex);
    RUN(test_url_decode_mixed);
    RUN(test_url_decode_malformed_drops_percent);
    RUN(test_url_decode_truncated_percent_2);
    RUN(test_url_decode_lone_percent_at_end);
    RUN(test_url_decode_empty);
    RUN(test_url_decode_only_plus);
    RUN(test_url_decode_zero_byte);
    RUN(test_url_decode_fuzz_invariants);

    printf("== html_esc ==\n");
    RUN(test_html_esc_plain);
    RUN(test_html_esc_ampersand);
    RUN(test_html_esc_quote);
    RUN(test_html_esc_angles);
    RUN(test_html_esc_all_metas);
    RUN(test_html_esc_empty);
    RUN(test_html_esc_truncates_safely);
    RUN(test_html_esc_tiny_buffer);

    printf("== url_encode_query_value ==\n");
    RUN(test_urlenc_unreserved_passthrough);
    RUN(test_urlenc_reserved_chars);
    RUN(test_urlenc_empty);
    RUN(test_urlenc_high_byte);
    RUN(test_urlenc_truncates_on_full_buffer);
    RUN(test_urlenc_partial_then_truncate);
    RUN(test_urlenc_zero_dstsz);

    printf("== clamp_u32 ==\n");
    RUN(test_clamp_u32_below_max);
    RUN(test_clamp_u32_at_max);
    RUN(test_clamp_u32_above_max_saturates);

    printf("== gmc_classify ==\n");
    RUN(test_gmc_guard_off_counts_when_past_gate);
    RUN(test_gmc_first_edge_never_guarded);
    RUN(test_gmc_guard_blocks_inside_window);
    RUN(test_gmc_guard_block_subgate_is_reject_not_removed);
    RUN(test_gmc_outside_window_counts);

    printf("== guard_effective_us ==\n");
    RUN(test_guard_eff_disabled_is_zero);
    RUN(test_guard_eff_pcnt_wins);
    RUN(test_guard_eff_enabled_returns_window);

    printf("== hv_blank_hit ==\n");
    RUN(test_blank_off_never_hits);
    RUN(test_blank_hits_inside_window);
    RUN(test_blank_misses_outside_window);
    RUN(test_blank_no_hv_pulse_yet_never_hits);

    printf("== tube_count_pin_for ==\n");
    RUN(test_count_pin_for_carrier);

    printf("== blank_effective_us ==\n");
    RUN(test_blank_eff_disabled_is_zero);
    RUN(test_blank_eff_pcnt_composes);

    printf("== pcnt_blank_wide ==\n");
    RUN(test_bwide_cool_phantoms_wide);
    RUN(test_bwide_warm_phantoms_narrow);
    RUN(test_bwide_mixed_proportional);
    RUN(test_bwide_removed_exceeds_blanked_clamps);
    RUN(test_bwide_pcnt_exceeds_isr_pileup);
    RUN(test_bwide_blanking_off_is_noop);
    RUN(test_bwide_zero_cycle);

    printf("== lorawan_codec ==\n");
    RUN(test_lw_hex_decode);
    RUN(test_lw_eui_from_hex);
    RUN(test_lw_pack_version);
    RUN(test_lw_build_port1);
    RUN(test_lw_build_port2);
    RUN(test_lw_sched_interval);
    RUN(test_lw_sched_port2_due);
    RUN(test_lw_sched_fixed_dr);
    RUN(test_lw_sched_frame_is_ours);
    RUN(test_rxobs_all_timeouts);
    RUN(test_rxobs_own_dl_rx1_first_repeat);
    RUN(test_rxobs_foreign_frame_repeats_go_on);
    RUN(test_rxobs_foreign_then_own_dl);
    RUN(test_rxobs_own_malformed_ends_exchange);
    RUN(test_rxobs_own_dl_rx2_last_repeat);
    RUN(test_rxobs_single_tx);
    RUN(test_rxobs_single_tx_foreign);
    RUN(test_rxobs_rx_window_missed);
    RUN(test_rxobs_nested_mac_only);
    RUN(test_rxobs_nested_mac_only_nbtrans1);
    RUN(test_rxobs_foreign_on_last_repeat);
    RUN(test_rxobs_unread_frame_repeats_go_on);
    RUN(test_rxobs_unread_frames_two_repeats);
    RUN(test_rxobs_tx_timeout);
    RUN(test_rxobs_unread_both_windows);
    RUN(test_rxobs_nbtrans_above_max_rep);
    RUN(test_rxobs_truncation_keeps_verdict);
    RUN(test_rxobs_join_accept_not_ignored);
    RUN(test_lw_err_text);
    RUN(test_lw_next_uplink_line);
    RUN(test_lw_downlink_unusable);
    RUN(test_lw_restore_verdict);
    RUN(test_lw_boot_line_states);
    RUN(test_lw_boot_line_variants);
    RUN(test_radio_turn_rule);

    printf("== env_api_format ==\n");
    RUN(test_env_api_format_typical);
    RUN(test_env_api_format_invalid_field_is_null);
    RUN(test_env_api_format_nan_is_null_and_clears_flag);
    RUN(test_env_api_format_inf_is_null);
    RUN(test_env_api_format_over_width_cap_is_null);
    RUN(test_env_api_format_pressure_is_pascals);
    RUN(test_env_api_format_never_sampled_sentinel);
    RUN(test_env_api_format_rejects_small_buffer);
    RUN(test_env_api_format_worst_case_fits);
    RUN(test_env_api_format_rejects_bad_id);

    printf("== env_api_parse ==\n");
    RUN(test_env_api_parse_typical);
    RUN(test_env_api_parse_null_clears_flag);
    RUN(test_env_api_parse_false_flag_zeroes_value);
    RUN(test_env_api_parse_missing_key_fails);
    RUN(test_env_api_parse_truncated_fails);
    RUN(test_env_api_parse_garbage_fails);
    RUN(test_env_api_parse_reordered_keys);
    RUN(test_env_api_parse_tolerates_extra_and_missing_ts);
    RUN(test_env_api_parse_never_sampled_sentinel);
    RUN(test_env_api_parse_rejects_oversize_id);
    RUN(test_env_api_parse_rejects_over_width_cap_number);
    RUN(test_env_api_parse_rejects_negative_age);

    printf("== env_api round-trip ==\n");
    RUN(test_env_api_roundtrip_typical);
    RUN(test_env_api_roundtrip_mixed_validity);
    RUN(test_env_api_roundtrip_never_sampled);
    RUN(test_env_api_roundtrip_nan_becomes_invalid);

    printf("== speaker_logic (tick defer) ==\n");
    RUN(test_tick_defer_zero_periods_fires_next_poll);
    RUN(test_tick_defer_one_period_holds_then_fires);
    RUN(test_tick_defer_does_not_refire_without_request);
    RUN(test_tick_defer_idle_state_never_fires);
    RUN(test_tick_defer_n_periods_fires_on_n_plus_one);
    RUN(test_tick_defer_request_restarts_countdown);
    RUN(test_tick_defer_sustained_requests_starve_the_tick);
    RUN(test_tick_defer_clear_cancels_pending);
    RUN(test_tick_defer_reusable_after_clear);

    printf("== tls_logic: fingerprint_hex ==\n");
    RUN(test_fingerprint_hex_32_bytes);
    RUN(test_fingerprint_hex_uppercase_and_colons);
    RUN(test_fingerprint_hex_exact_fit);
    RUN(test_fingerprint_hex_too_small_fails_and_empties);
    RUN(test_fingerprint_hex_zero_len_fails);

    printf("== tls_logic: https_redirect_location ==\n");
    RUN(test_redirect_basic);
    RUN(test_redirect_strips_port);
    RUN(test_redirect_keeps_query);
    RUN(test_redirect_rejects_empty_host);
    RUN(test_redirect_rejects_port_only_host);
    RUN(test_redirect_rejects_ipv6_literal);
    RUN(test_redirect_rejects_uri_without_slash);
    RUN(test_redirect_truncation_fails_and_empties);
    RUN(test_redirect_exact_fit);
    RUN(test_redirect_host_trailing_dot_passes_through);
    RUN(test_redirect_host_with_space_passes_through);
    RUN(test_redirect_host_with_at_passes_through);
    RUN(test_redirect_null_args_fail);

    printf("== tls_logic: civil_to_epoch / generalized time ==\n");
    RUN(test_civil_epoch_origin);
    RUN(test_civil_epoch_2026_01_01);
    RUN(test_civil_epoch_leap_day);
    RUN(test_civil_epoch_time_of_day);
    RUN(test_generalized_time_format);
    RUN(test_generalized_time_800_days_later);
    RUN(test_generalized_time_roundtrip);
    RUN(test_generalized_time_buffer_too_small);
    RUN(test_generalized_time_negative_fails);

    // V2.8.5 helpers
    RUN(test_parse_long_strict_accepts_numbers);
    RUN(test_parse_long_strict_rejects_garbage);
    RUN(test_parse_float_strict);
    RUN(test_tick_interval_plain);
    RUN(test_tick_interval_across_wrap);
    RUN(test_fmt_kilo4_bands);
    RUN(test_fmt_kilo4_9950_boundary_stays_4_chars);
    RUN(test_tz_offset_add_colon);
    RUN(test_tz_offset_add_colon_refuses_unsafe);
    RUN(test_html_esc_zero_bufsz_writes_nothing);

    printf("== html_esc contract (V2.8.8) ==\n");
    RUN(test_html_esc_worst_case_fits);
    RUN(test_html_esc_one_byte_short_drops_last_char);
    RUN(test_html_esc_fuzz_contract);

    printf("== history.c (V2.8.8) ==\n");
    RUN(test_history_first_tick_only_primes);
    RUN(test_history_samples_every_60s_not_sooner);
    RUN(test_history_cpm5_cpm15_ramp);
    RUN(test_history_clamps_huge_minute);
    RUN(test_history_top_bit_delta_reads_zero);
    RUN(test_history_source_switch_reprimes);
    RUN(test_history_hourly_rollup);
    RUN(test_history_minute_ring_wraps_oldest_first);
    RUN(test_history_ms_clock_wrap);
    RUN(test_history_inert_when_disabled);
    RUN(test_live_needs_10s_span);
    RUN(test_live_window_is_60s);
    RUN(test_live_stall_keeps_prestall_base);
    RUN(test_live_high_rate_no_overflow);
    RUN(test_live_ring_holds_60s_at_900ms_cadence);
    RUN(test_live_sample_gate_900ms);
    RUN(test_live_ms_clock_wrap);
    RUN(test_live_source_switch_and_reset_restart);
    RUN(test_live_subtrahend_step_restarts);
    RUN(test_live_falling_total_restarts);

    printf("== config schema (review 2.1 C3a) ==\n");
    RUN(test_schema_expands);
    RUN(test_schema_keys_fit_nvs_and_form);
    RUN(test_schema_keys_unique);
    RUN(test_schema_numeric_defaults_within_bounds);
    RUN(test_schema_integer_bounds_fit_target_types);
    RUN(test_schema_string_defaults_fit);
    RUN(test_schema_every_key_on_config_form);
    RUN(test_config_form_fields_are_schema_keys);

    printf("== config_form (review 2.1 C3b) ==\n");
    RUN(test_form_roundtrip_unchanged);
    RUN(test_form_preclears_bools);
    RUN(test_form_url_decodes);
    RUN(test_form_oled_step);
    RUN(test_form_strict_numeric);
    RUN(test_form_save_restart);
    RUN(test_form_11bg_forces_ht20);
    RUN(test_form_caps_force_off);
    RUN(test_form_ftp_ps_preserved);
    RUN(test_form_tube_off_clears);
    RUN(test_form_pcnt_beats_guard);
    RUN(test_form_mqtt_mode_b_needs_ca);
    RUN(test_form_rejected_list_truncates);
    RUN(test_form_lora_hex);
    RUN(test_form_revc_forced_v2_without_switch);
    RUN(test_form_revc_v3_with_switch);
    RUN(test_form_revc_out_of_range_rejected);
    RUN(test_form_revc_fixed_board_stores_3);
    RUN(test_form_lora_hp_forced_off_without_fem);
    RUN(test_form_lora_hp_kept_with_fem);
    RUN(test_lw_pa_offset);
    RUN(test_lw_mac_dev_status_pair);
    RUN(test_lw_mac_link_adr_fixed_plan);
    RUN(test_lw_mac_dynamic_plan_settings);
    RUN(test_lw_mac_time_duty_tx_param);
    RUN(test_lw_mac_unknown_incomplete_truncated);
    RUN(test_lw_mac_dynamic_cntl_answerless_mhz);
    RUN(test_rxobs_note_mac_frames);
    RUN(test_rxobs_accepted_hit);

    printf("== ota_guard_logic (V2.8.20) ==\n");
    RUN(test_ota_guard_needs_confirm_states);
    RUN(test_ota_guard_probation_window_plus_120s);
    RUN(test_ota_guard_retry_spacing_60s);

    printf("\n");
    if (g_ran == 0) {
        printf("NO TESTS MATCHED FILTER '%s'\n", g_filter ? g_filter : "");
        return 2;
    }
    if (g_failures == 0) {
        printf("ALL %d TESTS PASS\n", g_ran);
        return 0;
    }
    printf("%d OF %d TEST(S) FAILED\n", g_failures, g_ran);
    return 1;
}
