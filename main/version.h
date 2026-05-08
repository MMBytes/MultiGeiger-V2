#pragma once
// Bump before build; commit after successful flash.
// V2.3.5 — TLS handshake speedups:
//   - TLS 1.3 protocol support enabled alongside 1.2. Saves 1 round trip
//     (1-RTT vs 2-RTT) on every new handshake. mbedTLS 4.x falls back to
//     1.2 against servers that don't support 1.3.
//   - Client-side TLS session ticket protocol support unlocked
//     (CONFIG_ESP_TLS_CLIENT_SESSION_TICKETS). Auto-caching across
//     esp_http_client_init() calls still needs a follow-up code change.
//   - mbedTLS built with -O2 (CONFIG_MBEDTLS_COMPILER_OPTIMIZATION_PERF)
//     instead of -Os. ~10–20 % faster software ECC, biggest win on the
//     FeatherS3-D where the LX7 I-cache absorbs the bigger code.
//   - No source-code changes; defaults overlay only. Both boards build
//     clean with the new optimisations.
#define VERSION_STR "V2.3.5"
