/** @file
 *  @brief Implementation of tls_cert.h — see the header for the contract.
 *
 *  mbedTLS in ESP-IDF 6.0.x is 4.1 (TF-PSA-Crypto). Two consequences shape
 *  this file:
 *    1. Key generation goes through PSA (`psa_generate_key`), then the key is
 *       copied into a legacy PK context with `mbedtls_pk_copy_from_psa()` so
 *       the X.509 writer and `mbedtls_pk_write_key_pem()` can use it. The
 *       old `mbedtls_ecp_gen_key()` is private API in 4.x.
 *    2. The X.509 writer no longer takes an RNG callback; PSA supplies it.
 *       `psa_crypto_init()` is already run by IDF at system-init time
 *       (components/mbedtls/port/esp_psa_crypto_init.c), so nothing here
 *       has to call it.
 */

#include "tls_cert.h"

#if HAL_HAS_HTTPS

#include <stdarg.h>            // va_list — summary_append
#include <stdio.h>
#include <stdlib.h>            // malloc/free — re-issue staging buffers
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "esp_random.h"        // esp_fill_random — certificate serial
#include "esp_system.h"        // esp_reset_reason — reconcile loop guard
#include "esp_timer.h"         // esp_timer_get_time — keygen duration in the log
#include "nvs.h"

#include "mbedtls/asn1.h"      // mbedtls_asn1_sequence for the EKU extension, MBEDTLS_OID_SIZE
#include "mbedtls/oid.h"       // MBEDTLS_OID_SERVER_AUTH
#include "mbedtls/pk.h"
#include "mbedtls/platform_util.h"  // mbedtls_platform_zeroize — scrub the staged key
#include "mbedtls/x509.h"      // mbedtls_x509_parse_subject_alt_name, KU bits
#include "mbedtls/x509_crt.h"
#include "psa/crypto.h"

#include "ntp.h"               // ntp_time_valid — is time(NULL) trustworthy?
#include "tls_logic.h"         // fingerprint_hex, civil_to_epoch, epoch_to_generalized_time

static const char *TAG = "tls_cert";

// --- Storage -----------------------------------------------------------------
// Own NVS namespace, deliberately NOT part of config_t / config_save(): a
// /config POST must never rewrite the key, and a future "factory reset" of
// the user config must not silently change the certificate the user trusted.
static const char *NS         = "geiger_tls";
static const char *KEY_CRT    = "crt";
static const char *KEY_KEY    = "key";
static const char *KEY_ISSUED = "issued";     // u32 unix epoch of the last RE-ISSUE; 0 after creation or when the clock was not valid

// A P-256 cert with three SANs + BC + KU + EKU is ~750 B of PEM; a SEC1 P-256
// key is ~230 B. Both well under the 4000-byte nvs_set_str ceiling. Static so
// the pointers handed to esp_https_server stay valid for the firmware
// lifetime (it copies them anyway).
#define CRT_PEM_MAX  1024
#define KEY_PEM_MAX   512
#define FP_STR_LEN   (32 * 3)          // 95 chars + NUL

static char   s_crt[CRT_PEM_MAX];
static char   s_key[KEY_PEM_MAX];
static char   s_fp[FP_STR_LEN];
static bool   s_ready = false;

// V2.8.1: one-line boot summary for the syslog banner. Every line this module
// logs at boot predates the syslog client (it needs a LAN address, which only
// arrives after the AP window), and the reconcile verdict lands in the same
// main-loop tick as syslog_init(), a few lines BEFORE the socket opens — so
// on the server none of it ever appeared, and on the node the first HTTPS
// boot takes its ring with it when it reboots to load the re-issued pair.
// Each decision point appends its verdict here; syslog_init() emits it next to
// the firmware banner once the socket is live. Written from the main task
// only (tls_cert_ensure and tls_cert_reconcile both run there).
// 288: the longest honest line is a first enable — generated + fingerprint +
// re-issued + new fingerprint — at ~205 bytes, plus an "NVS load failed"
// prefix. 200 clipped exactly the segment that says a reboot is coming.
#define BOOT_SUMMARY_MAX 288
static char s_boot_summary[BOOT_SUMMARY_MAX] = "";

static void summary_append(const char *fmt, ...) {
    size_t used = strlen(s_boot_summary);
    if (used >= sizeof(s_boot_summary) - 3) return;
    if (used) {
        s_boot_summary[used++] = ';';
        s_boot_summary[used++] = ' ';
        s_boot_summary[used] = 0;
    }
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_boot_summary + used, sizeof(s_boot_summary) - used, fmt, ap);
    va_end(ap);
}

// Validity. 800 days: Apple refuses TLS server certificates valid for more
// than 825 days (iOS 13 / macOS 10.15 onwards, private roots included), so
// the 30-year self-signed habit is out. When the wall clock is not yet
// trustworthy (first cold boot, RTC at 1970) the window starts at a fixed
// 2026-01-01 instead of time(NULL).
#define CERT_LIFETIME_S     (800LL * 86400)
#define CERT_RENEW_MARGIN_S (30LL * 86400)
#define CERT_SKEW_S         (86400LL)                  // NotBefore = now - 1 day
#define CERT_FIXED_START_S  civil_to_epoch(2026, 1, 1, 0, 0, 0)
#define REISSUE_LOOP_GUARD_S 600                       // see tls_cert_reconcile()

// --- Fingerprint + parse helpers -------------------------------------------------

/** Parse a PEM cert into @p crt (caller frees). Returns ESP_OK or logs why not. */
static esp_err_t parse_pem(mbedtls_x509_crt *crt, const char *pem, size_t len_incl_nul) {
    mbedtls_x509_crt_init(crt);
    int rc = mbedtls_x509_crt_parse(crt, (const unsigned char *)pem, len_incl_nul);
    if (rc != 0) {
        ESP_LOGW(TAG, "certificate PEM does not parse (-0x%04x)", (unsigned)-rc);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

/** Fill @p fp_out with SHA-256(DER) of an already-parsed cert. */
static esp_err_t fingerprint_of(const mbedtls_x509_crt *crt, char *fp_out, size_t fp_sz) {
    uint8_t hash[32];
    size_t  olen = 0;
    psa_status_t st = psa_hash_compute(PSA_ALG_SHA_256, crt->raw.p, crt->raw.len,
                                       hash, sizeof(hash), &olen);
    if (st != PSA_SUCCESS || olen != sizeof(hash)) {
        ESP_LOGE(TAG, "psa_hash_compute failed (%d)", (int)st);
        return ESP_FAIL;
    }
    fingerprint_hex(hash, sizeof(hash), fp_out, fp_sz);
    return ESP_OK;
}

/** True if the parsed cert's SAN list carries @p ip_be as an iPAddress. */
static bool san_has_ip(const mbedtls_x509_crt *crt, uint32_t ip_be) {
    for (const mbedtls_x509_sequence *cur = &crt->subject_alt_names;
         cur != NULL && cur->buf.p != NULL; cur = cur->next) {
        mbedtls_x509_subject_alternative_name san;
        memset(&san, 0, sizeof(san));
        // Returns MBEDTLS_ERR_X509_FEATURE_UNAVAILABLE for SAN types the
        // parser does not model (none of ours); skip those rather than fail.
        if (mbedtls_x509_parse_subject_alt_name(&cur->buf, &san) != 0) continue;
        if (san.type == MBEDTLS_X509_SAN_IP_ADDRESS &&
            san.san.unstructured_name.len == 4 &&
            memcmp(san.san.unstructured_name.p, &ip_be, 4) == 0) {
            return true;
        }
    }
    return false;
}

/** Confirm the stored private key belongs to @p crt.
 *
 *  save_to_nvs() writes `crt` then `key` and nvs_set_str persists each one
 *  immediately, so a power loss between the two leaves a NEW certificate
 *  beside the OLD key. Without this check the mismatch only surfaces when
 *  httpd_ssl_start fails on the next boot — the node then falls back to plain
 *  HTTP (design D9) and NEVER self-heals, because the stored pair still parses.
 *  Catching it here costs one extra browser trust prompt (the regenerated cert
 *  has a new fingerprint) and buys back HTTPS.
 *
 *  mbedTLS 4.1 (tf-psa-crypto): mbedtls_pk_parse_key() takes no RNG arguments
 *  (pk.h:725) and mbedtls_pk_check_pair() takes just the two contexts
 *  (pk.h:694) — both lost the RNG parameters they had in 3.x.
 */
static esp_err_t check_key_matches(const mbedtls_x509_crt *crt) {
    mbedtls_pk_context pk;
    mbedtls_pk_init(&pk);
    int rc = mbedtls_pk_parse_key(&pk, (const unsigned char *)s_key, strlen(s_key) + 1,
                                  NULL, 0);
    if (rc == 0) rc = mbedtls_pk_check_pair(&crt->pk, &pk);
    mbedtls_pk_free(&pk);
    if (rc != 0) {
        ESP_LOGW(TAG, "stored key does not match the stored certificate (-0x%04x) — regenerating",
                 (unsigned)-rc);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

/** UTC epoch seconds of an mbedtls_x509_time (fields are already validated). */
static int64_t x509_time_to_epoch(const mbedtls_x509_time *t) {
    return civil_to_epoch(t->year, (unsigned)t->mon, (unsigned)t->day,
                          (unsigned)t->hour, (unsigned)t->min, (unsigned)t->sec);
}

// --- NVS load / save -------------------------------------------------------------

static esp_err_t load_from_nvs(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;               // first boot: namespace absent
    size_t clen = sizeof(s_crt);
    size_t klen = sizeof(s_key);
    err = nvs_get_str(h, KEY_CRT, s_crt, &clen);
    if (err == ESP_OK) err = nvs_get_str(h, KEY_KEY, s_key, &klen);
    nvs_close(h);
    if (err != ESP_OK) return err;

    mbedtls_x509_crt crt;
    err = parse_pem(&crt, s_crt, strlen(s_crt) + 1);
    if (err == ESP_OK) err = fingerprint_of(&crt, s_fp, sizeof(s_fp));
    if (err == ESP_OK) err = check_key_matches(&crt);
    mbedtls_x509_crt_free(&crt);
    return err;
}

/** Persist the pair in @p crt / @p key — which may be the live statics or a
 *  re-issue's staging buffers — plus the re-issue timestamp. */
static esp_err_t save_to_nvs(const char *crt, const char *key, uint32_t issued_epoch) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_str(h, KEY_CRT, crt);
    if (err == ESP_OK) err = nvs_set_str(h, KEY_KEY, key);
    if (err == ESP_OK) err = nvs_set_u32(h, KEY_ISSUED, issued_epoch);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static uint32_t issued_from_nvs(void) {
    nvs_handle_t h;
    uint32_t issued = 0;
    if (nvs_open(NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, KEY_ISSUED, &issued);     // stays 0 if absent
        nvs_close(h);
    }
    return issued;
}

static void log_nvs_usage(void) {
    // The nvs partition is 0x6000 — SIX 4096-byte pages, one of them held
    // back as the reserved page. Cert + key + issued take ~33 entries next
    // to the config namespace; if this ever reads close to full, the D9
    // fallback is the first symptom.
    //
    // available_entries is the headline figure, NOT free_entries: nvs.h
    // documents free_entries as "includes also reserved entries", so a
    // partition with no usable space left still reports a healthy-looking
    // free count. available_entries is what a write can actually consume.
    nvs_stats_t st;
    if (nvs_get_stats(NULL, &st) == ESP_OK) {
        ESP_LOGI(TAG, "NVS: %u of %u entries used, %u available (%u free incl. the reserved page)",
                 (unsigned)st.used_entries, (unsigned)st.total_entries,
                 (unsigned)st.available_entries, (unsigned)st.free_entries);
    }
}

// --- Generation ----------------------------------------------------------------

/** Generate key + self-signed cert into the caller's buffers and persist them.
 *
 *  The outputs are explicit so a re-issue can write into STAGING buffers while
 *  the live statics keep serving the certificate :443 is actually presenting
 *  until the deferred reboot. tls_cert_ensure() passes the statics themselves
 *  (first boot: nothing is serving yet).
 *
 *  @param crt_out       Certificate PEM out; >= CRT_PEM_MAX.
 *  @param crt_sz        sizeof(*crt_out).
 *  @param key_out       Private key PEM out; >= KEY_PEM_MAX.
 *  @param key_sz        sizeof(*key_out).
 *  @param fp_out        Fingerprint string out; >= FP_STR_LEN.
 *  @param fp_sz         sizeof(*fp_out).
 *  @param chip_id       CN and dNSName SAN.
 *  @param sta_ip_be     0 = no STA SAN (first boot, address unknown).
 *  @param stamp_issued  true only for a re-issue from tls_cert_reconcile();
 *                       creation stores 0 so the loop guard cannot mistake a
 *                       first boot for a reboot loop.
 */
static esp_err_t generate(char *crt_out, size_t crt_sz, char *key_out, size_t key_sz,
                          char *fp_out, size_t fp_sz,
                          const char *chip_id, uint32_t sta_ip_be, bool stamp_issued) {
    const int64_t t0 = esp_timer_get_time();
    esp_err_t result = ESP_FAIL;
    int rc;

    // 1. Keypair via PSA. EXPORT is required by mbedtls_pk_copy_from_psa();
    //    SIGN_HASH + ECDSA(SHA-256) is what the copied PK context will do.
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&attr, 256);
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_SIGN_HASH);
    psa_set_key_algorithm(&attr, PSA_ALG_ECDSA(PSA_ALG_SHA_256));
    mbedtls_svc_key_id_t kid = MBEDTLS_SVC_KEY_ID_INIT;
    psa_status_t st = psa_generate_key(&attr, &kid);
    psa_reset_key_attributes(&attr);
    if (st != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_generate_key failed (%d)", (int)st);
        // Same contract as the epilogue: never leave a half-written pair for
        // the getters to expose. Nothing is initialised yet, so zero here and
        // return rather than jumping into the mbedTLS teardown labels.
        memset(crt_out, 0, crt_sz);
        memset(key_out, 0, key_sz);
        if (fp_sz > 0) fp_out[0] = 0;
        return ESP_FAIL;
    }

    mbedtls_pk_context pk;
    mbedtls_pk_init(&pk);
    rc = mbedtls_pk_copy_from_psa(kid, &pk);
    psa_destroy_key(kid);                        // PK holds its own copy now
    if (rc != 0) {
        ESP_LOGE(TAG, "mbedtls_pk_copy_from_psa failed (-0x%04x)", (unsigned)-rc);
        goto out_pk;
    }
    rc = mbedtls_pk_write_key_pem(&pk, (unsigned char *)key_out, key_sz);
    if (rc != 0) {
        ESP_LOGE(TAG, "mbedtls_pk_write_key_pem failed (-0x%04x)", (unsigned)-rc);
        goto out_pk;
    }

    // 2. Self-signed X.509 v3. Subject == issuer, subject key == issuer key.
    //    CA:true + keyCertSign because iOS and Android will only install a
    //    CA certificate as a trust anchor; a self-signed LEAF cannot be
    //    trusted there at all. Harmless for a self-signed cert (it can only
    //    "issue" itself).
    mbedtls_x509write_cert crt;
    mbedtls_x509write_crt_init(&crt);
    mbedtls_x509write_crt_set_version(&crt, MBEDTLS_X509_CRT_VERSION_3);
    mbedtls_x509write_crt_set_md_alg(&crt, MBEDTLS_MD_SHA256);
    mbedtls_x509write_crt_set_subject_key(&crt, &pk);
    mbedtls_x509write_crt_set_issuer_key(&crt, &pk);

    char dn[48];                                 // "CN=esp32-4294967295" fits
    snprintf(dn, sizeof(dn), "CN=%s", chip_id);
    if ((rc = mbedtls_x509write_crt_set_subject_name(&crt, dn)) != 0 ||
        (rc = mbedtls_x509write_crt_set_issuer_name(&crt, dn))  != 0) {
        ESP_LOGE(TAG, "set_subject/issuer_name failed (-0x%04x)", (unsigned)-rc);
        goto out_crt;
    }

    // Serial: 16 random bytes, forced positive (top bit clear) and non-zero
    // (RFC 5280 wants a positive INTEGER; some validators reject 0).
    unsigned char serial[16];
    esp_fill_random(serial, sizeof(serial));
    serial[0] = (unsigned char)((serial[0] & 0x7F) | 0x01);

    // Validity window (UTC GeneralizedTime). time(NULL) only when NTP says
    // the clock is sane; otherwise the fixed 2026-01-01 start.
    const bool    clock_ok = ntp_time_valid();
    const int64_t nb = clock_ok ? (int64_t)time(NULL) - CERT_SKEW_S : CERT_FIXED_START_S;
    const int64_t na = nb + CERT_LIFETIME_S;
    char nb_s[15], na_s[15];
    if (!epoch_to_generalized_time(nb, nb_s, sizeof(nb_s)) ||
        !epoch_to_generalized_time(na, na_s, sizeof(na_s))) {
        ESP_LOGE(TAG, "validity formatting failed");
        goto out_crt;
    }

    if ((rc = mbedtls_x509write_crt_set_serial_raw(&crt, serial, sizeof(serial))) != 0 ||
        (rc = mbedtls_x509write_crt_set_validity(&crt, nb_s, na_s))                 != 0 ||
        (rc = mbedtls_x509write_crt_set_basic_constraints(&crt, 1, 0))              != 0 ||
        (rc = mbedtls_x509write_crt_set_key_usage(&crt, MBEDTLS_X509_KU_DIGITAL_SIGNATURE |
                                                        MBEDTLS_X509_KU_KEY_CERT_SIGN))   != 0) {
        ESP_LOGE(TAG, "serial/validity/BC/KU failed (-0x%04x)", (unsigned)-rc);
        goto out_crt;
    }

    // Extended key usage: id-kp-serverAuth. Chrome/Firefox treat a cert
    // without it as unusable for TLS server auth even after the user trusts
    // it, so this is not optional.
    mbedtls_asn1_sequence eku = {
        .buf  = { .tag = MBEDTLS_ASN1_OID,
                  .len = MBEDTLS_OID_SIZE(MBEDTLS_OID_SERVER_AUTH),
                  .p   = (unsigned char *)MBEDTLS_OID_SERVER_AUTH },
        .next = NULL,
    };
    if ((rc = mbedtls_x509write_crt_set_ext_key_usage(&crt, &eku)) != 0) {
        ESP_LOGE(TAG, "set_ext_key_usage failed (-0x%04x)", (unsigned)-rc);
        goto out_crt;
    }

    // SANs: the chip id as dNSName (a hosts-file entry or future mDNS name),
    // the AP-mode address (the ONE address known in advance), and — once
    // tls_cert_reconcile() knows it — the STA address people actually browse
    // to. Chrome/Edge/Safari check the name even for a trusted certificate,
    // so without the STA entry "trust once" never sticks outside Firefox.
    static const unsigned char ap_ip[4] = { 192, 168, 4, 1 };
    mbedtls_x509_san_list san_sta = {
        .node = { .type = MBEDTLS_X509_SAN_IP_ADDRESS,
                  .san.unstructured_name = { .tag = MBEDTLS_ASN1_OCTET_STRING,
                                             .len = 4,
                                             .p   = (unsigned char *)&sta_ip_be } },
        .next = NULL,
    };
    mbedtls_x509_san_list san_ip = {
        .node = { .type = MBEDTLS_X509_SAN_IP_ADDRESS,
                  .san.unstructured_name = { .tag = MBEDTLS_ASN1_OCTET_STRING,
                                             .len = sizeof(ap_ip),
                                             .p   = (unsigned char *)ap_ip } },
        .next = (sta_ip_be != 0) ? &san_sta : NULL,
    };
    mbedtls_x509_san_list san_dns = {
        .node = { .type = MBEDTLS_X509_SAN_DNS_NAME,
                  .san.unstructured_name = { .tag = MBEDTLS_ASN1_IA5_STRING,
                                             .len = strlen(chip_id),
                                             .p   = (unsigned char *)chip_id } },
        .next = &san_ip,
    };
    if ((rc = mbedtls_x509write_crt_set_subject_alternative_name(&crt, &san_dns)) != 0) {
        ESP_LOGE(TAG, "set_subject_alternative_name failed (-0x%04x)", (unsigned)-rc);
        goto out_crt;
    }

    // 3. Sign + PEM-encode in one call. ECDSA signatures are randomised, so
    //    the fingerprint MUST come from this exact output (parsed back below),
    //    never from a second _der() call.
    rc = mbedtls_x509write_crt_pem(&crt, (unsigned char *)crt_out, crt_sz);
    if (rc != 0) {
        ESP_LOGE(TAG, "mbedtls_x509write_crt_pem failed (-0x%04x)", (unsigned)-rc);
        goto out_crt;
    }

    {
        mbedtls_x509_crt parsed;
        result = parse_pem(&parsed, crt_out, strlen(crt_out) + 1);
        if (result == ESP_OK) result = fingerprint_of(&parsed, fp_out, fp_sz);
        mbedtls_x509_crt_free(&parsed);
    }
    if (result == ESP_OK) {
        result = save_to_nvs(crt_out, key_out,
                             (stamp_issued && clock_ok) ? (uint32_t)time(NULL) : 0);
    }
    if (result == ESP_OK) {
        const long long gen_ms = (long long)((esp_timer_get_time() - t0) / 1000);
        ESP_LOGI(TAG, "generated P-256 key + self-signed cert in %lld ms (%u B cert, %u B key, valid %s..%s%s)",
                 gen_ms, (unsigned)strlen(crt_out), (unsigned)strlen(key_out), nb_s, na_s,
                 sta_ip_be ? ", STA address included" : ", no STA address yet");
        summary_append("%s in %lld ms (valid %.8s..%.8s%s)",
                       stamp_issued ? "re-issued" : "generated", gen_ms, nb_s, na_s,
                       sta_ip_be ? ", STA addr" : ", AP addr only");
    } else {
        ESP_LOGE(TAG, "post-generation step failed: %s", esp_err_to_name(result));
    }

out_crt:
    mbedtls_x509write_crt_free(&crt);
out_pk:
    mbedtls_pk_free(&pk);
    if (result != ESP_OK) {
        // Never leave a half-written pair behind for the getters to expose.
        memset(crt_out, 0, crt_sz);
        memset(key_out, 0, key_sz);
        if (fp_sz > 0) fp_out[0] = 0;
    }
    return result;
}

// --- Public API --------------------------------------------------------------

esp_err_t tls_cert_ensure(const char *chip_id) {
    if (s_ready) return ESP_OK;
    if (!chip_id || chip_id[0] == 0) return ESP_ERR_INVALID_ARG;

    log_nvs_usage();
    esp_err_t err = load_from_nvs();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "certificate loaded from NVS, SHA-256 %s", s_fp);
        summary_append("loaded from NVS");
    } else {
        if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "NVS load failed (%s) — regenerating", esp_err_to_name(err));
            summary_append("NVS load failed (%s)", esp_err_to_name(err));
        }
        // Straight into the live statics: no server is running yet, so there
        // is nothing to keep consistent and no reader to tear.
        err = generate(s_crt, sizeof(s_crt), s_key, sizeof(s_key), s_fp, sizeof(s_fp),
                       chip_id, 0, false);
        if (err == ESP_OK) ESP_LOGI(TAG, "certificate SHA-256 %s", s_fp);
        else summary_append("generation FAILED (%s) — plain HTTP", esp_err_to_name(err));
    }
    s_ready = (err == ESP_OK);
    if (s_ready) summary_append("SHA-256 %.23s", s_fp);   // first 8 bytes identify it
    return err;
}

esp_err_t tls_cert_reconcile(const char *chip_id, uint32_t sta_ip_be, bool *reissued) {
    if (reissued) *reissued = false;
    if (!s_ready || !chip_id || sta_ip_be == 0) return ESP_ERR_INVALID_STATE;

    mbedtls_x509_crt crt;
    esp_err_t err = parse_pem(&crt, s_crt, strlen(s_crt) + 1);
    if (err != ESP_OK) { mbedtls_x509_crt_free(&crt); return err; }

    const bool    has_ip   = san_has_ip(&crt, sta_ip_be);
    const int64_t valid_to = x509_time_to_epoch(&crt.valid_to);
    mbedtls_x509_crt_free(&crt);

    const int64_t now      = ntp_time_valid() ? (int64_t)time(NULL) : 0;
    const bool    expiring = (now != 0) && (valid_to - now < CERT_RENEW_MARGIN_S);
    if (has_ip && !expiring) {
        // now == 0 means the clock is not trustworthy (a future caller that
        // skips main.c's NTP gate); a "days left" from epoch 0 would be a lie.
        if (now != 0) {
            ESP_LOGI(TAG, "certificate names the current address, %lld days left — no re-issue",
                     (long long)((valid_to - now) / 86400));
            summary_append("names the current address, %lld days left", (long long)((valid_to - now) / 86400));
        } else {
            ESP_LOGI(TAG, "certificate names the current address, clock not yet valid — no re-issue");
            summary_append("names the current address, clock not valid");
        }
        return ESP_OK;
    }

    // Loop guard (design D5): a re-issue leads to a reboot; if the address
    // is different AGAIN within minutes of the last RE-ISSUE on a software
    // reset, the DHCP server is handing out a fresh lease every boot and
    // re-issuing would loop forever.
    //
    // The guard only ever sees re-issues: generate() stamps KEY_ISSUED when
    // called with stamp_issued=true, which only happens below, and stores 0
    // on first-time creation. Without that distinction the first boot after
    // an OTA would trip the guard and never add the STA address — an OTA
    // reboot is ESP_RST_SW and the RTC carries a valid clock across it, so a
    // certificate CREATED seconds ago would be indistinguishable from one
    // re-issued seconds ago.
    const uint32_t issued = issued_from_nvs();
    if (now != 0 && issued != 0 && (now - (int64_t)issued) < REISSUE_LOOP_GUARD_S &&
        esp_reset_reason() == ESP_RST_SW) {
        ESP_LOGE(TAG, "address changed again %llds after the last re-issue — refusing to loop. "
                 "Give this node a DHCP reservation.", (long long)(now - (int64_t)issued));
        summary_append("address changed again %llds after a re-issue — REFUSED (DHCP reservation?)",
                       (long long)(now - (int64_t)issued));
        return ESP_OK;
    }

    ESP_LOGW(TAG, "re-issuing certificate: %s%s", has_ip ? "" : "current address missing from SAN",
             expiring ? (has_ip ? "expires within 30 days" : " + expires within 30 days") : "");
    // STAGING buffers, not the live statics: the running TLS server presents
    // the OLD certificate until the deferred reboot, so overwriting s_crt/s_key
    // here would make /cert.pem and the fingerprint on / advertise a
    // certificate :443 is not using — and a concurrent reader could catch a
    // half-written PEM. NVS gets the NEW pair; memory keeps the OLD one until
    // the reboot loads it back. On failure nothing in memory changed at all.
    char *crt_stage = malloc(CRT_PEM_MAX);
    char *key_stage = malloc(KEY_PEM_MAX);
    char  fp_stage[FP_STR_LEN];
    if (!crt_stage || !key_stage) {
        // One of the two may have succeeded; scrub it before it goes back to
        // the heap. Nothing was generated into it yet, but the rule is the
        // same on every path and the compiler cannot optimise this call away.
        if (crt_stage) mbedtls_platform_zeroize(crt_stage, CRT_PEM_MAX);
        if (key_stage) mbedtls_platform_zeroize(key_stage, KEY_PEM_MAX);
        free(crt_stage);
        free(key_stage);
        ESP_LOGE(TAG, "no heap for the re-issue staging buffers — keeping the current certificate");
        summary_append("re-issue skipped: no heap");
        return ESP_ERR_NO_MEM;
    }
    err = generate(crt_stage, CRT_PEM_MAX, key_stage, KEY_PEM_MAX, fp_stage, sizeof(fp_stage),
                   chip_id, sta_ip_be, true);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "certificate SHA-256 %s (effective after reboot)", fp_stage);
        summary_append("new SHA-256 %.23s after reboot", fp_stage);
        if (reissued) *reissued = true;
    } else {
        summary_append("re-issue FAILED (%s) — keeping the current certificate", esp_err_to_name(err));
    }
    // key_stage held the new PRIVATE key in cleartext; free() alone leaves it
    // readable in the heap block until something else reuses it.
    mbedtls_platform_zeroize(crt_stage, CRT_PEM_MAX);
    mbedtls_platform_zeroize(key_stage, KEY_PEM_MAX);
    free(crt_stage);
    free(key_stage);
    return err;
}

const char *tls_cert_pem(void)         { return s_ready ? s_crt : NULL; }
size_t      tls_cert_pem_len(void)     { return s_ready ? strlen(s_crt) + 1 : 0; }
const char *tls_key_pem(void)          { return s_ready ? s_key : NULL; }
size_t      tls_key_pem_len(void)      { return s_ready ? strlen(s_key) + 1 : 0; }
const char *tls_cert_fingerprint(void) { return s_ready ? s_fp : ""; }
const char *tls_cert_boot_summary(void) { return s_boot_summary[0] ? s_boot_summary : "nothing provisioned"; }

#endif  // HAL_HAS_HTTPS
