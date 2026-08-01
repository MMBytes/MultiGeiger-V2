// Host-side unit tests for the pure helpers in main/util.h.
//
// Run on a regular host C compiler (gcc on Linux/CI, MSYS2/MinGW or
// WSL on Windows) — no ESP-IDF, no FreeRTOS, no hardware needed.
//
// Build + run locally: `_test.cmd` from the repo root (Windows) or
// `gcc -I main -Wall -Wextra -Werror -o test/run test/test_main.c &&
// ./test/run` (Linux/macOS). CI runs it in `.github/workflows/build.yml`
// via the `host-test` job.
//
// Scope (V2.4.1+ T1 first foothold): the 5 pure functions that live
// in main/util.h. Anything that depends on IDF / FreeRTOS / hardware
// (NVS, HTTP handlers, sensor drivers, transmission orchestrator) is
// out of scope for this first round — would need on-target Unity or
// QEMU.
//
// Convention: each test_xxx() returns 1 on pass, 0 on fail. The runner
// prints PASS/FAIL with the test name and tallies the failures. Non-
// zero exit code if any test failed (so CI marks the job red).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util.h"
#include "tube_logic.h"   // clamp_u32, gmc_classify, guard_effective_us
#include "lorawan_codec.h"   // lw_hex_decode, lw_eui_from_hex, lw_pack_version, lw_build_port1/2

static int g_failures = 0;

#define EXPECT_STREQ(actual, expected)                                       \
    do {                                                                     \
        if (strcmp((actual), (expected)) != 0) {                             \
            printf("    expected '%s', got '%s'\n", (expected), (actual));   \
            return 0;                                                        \
        }                                                                    \
    } while (0)

#define EXPECT_INT(actual, expected)                                         \
    do {                                                                     \
        long _a = (long)(actual);                                            \
        long _e = (long)(expected);                                          \
        if (_a != _e) {                                                      \
            printf("    expected %ld, got %ld\n", _e, _a);                   \
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

static int test_clamp_u32_below_max(void) {
    EXPECT_INT(clamp_u32(0), 0);
    EXPECT_INT(clamp_u32(190), 190);
    EXPECT_INT(clamp_u32(UINT32_MAX - 1), (long)(UINT32_MAX - 1));
    return 1;
}

static int test_clamp_u32_at_max(void) {
    EXPECT_INT(clamp_u32((uint64_t)UINT32_MAX), (long)UINT32_MAX);
    return 1;
}

static int test_clamp_u32_above_max_saturates(void) {
    // The >71.6-min wrap case: a bare (uint32_t) cast would wrap to a small
    // value; clamp_u32 must saturate to UINT32_MAX so the edge reads as
    // "maximally separated" and never spuriously trips the dead-time guard.
    EXPECT_INT(clamp_u32((uint64_t)UINT32_MAX + 1), (long)UINT32_MAX);
    EXPECT_INT(clamp_u32((uint64_t)UINT32_MAX + 1000000), (long)UINT32_MAX);
    EXPECT_INT(clamp_u32(0xFFFFFFFFFFFFFFFFull), (long)UINT32_MAX);
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
// Runner
// ----------------------------------------------------------------------------

#define RUN(test_fn)                                                         \
    do {                                                                     \
        if (test_fn()) {                                                     \
            printf("  PASS  %s\n", #test_fn);                                \
        } else {                                                             \
            printf("  FAIL  %s\n", #test_fn);                                \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

int main(void) {
    printf("== safe_strcpy ==\n");
    RUN(test_safe_strcpy_basic);
    RUN(test_safe_strcpy_truncates);
    RUN(test_safe_strcpy_empty_src);
    RUN(test_safe_strcpy_zero_dstsz);
    RUN(test_safe_strcpy_dstsz_1);

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

    printf("\n");
    if (g_failures == 0) {
        printf("ALL TESTS PASS\n");
        return 0;
    }
    printf("%d TEST(S) FAILED\n", g_failures);
    return 1;
}
