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

    printf("\n");
    if (g_failures == 0) {
        printf("ALL TESTS PASS\n");
        return 0;
    }
    printf("%d TEST(S) FAILED\n", g_failures);
    return 1;
}
