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

#include <stdio.h>
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
static const char *KEY_ISSUED = "issued";     // u32 unix epoch, 0 if clock was not valid

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
        ESP_LOGW(TAG, "stored certificate does not parse (-0x%04x)", (unsigned)-rc);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

/** Fill s_fp with SHA-256(DER) of an already-parsed cert. */
static esp_err_t fingerprint_of(const mbedtls_x509_crt *crt) {
    uint8_t hash[32];
    size_t  olen = 0;
    psa_status_t st = psa_hash_compute(PSA_ALG_SHA_256, crt->raw.p, crt->raw.len,
                                       hash, sizeof(hash), &olen);
    if (st != PSA_SUCCESS || olen != sizeof(hash)) {
        ESP_LOGE(TAG, "psa_hash_compute failed (%d)", (int)st);
        return ESP_FAIL;
    }
    fingerprint_hex(hash, sizeof(hash), s_fp, sizeof(s_fp));
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
    if (err == ESP_OK) err = fingerprint_of(&crt);
    mbedtls_x509_crt_free(&crt);
    return err;
}

static esp_err_t save_to_nvs(uint32_t issued_epoch) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_str(h, KEY_CRT, s_crt);
    if (err == ESP_OK) err = nvs_set_str(h, KEY_KEY, s_key);
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
    // The nvs partition is 0x6000 (three pages, one reserved). Cert + key +
    // issued take ~33 entries next to the config namespace; if this ever
    // reads close to full, the D9 fallback is the first symptom.
    nvs_stats_t st;
    if (nvs_get_stats(NULL, &st) == ESP_OK) {
        ESP_LOGI(TAG, "NVS: %u of %u entries used (%u free)",
                 (unsigned)st.used_entries, (unsigned)st.total_entries, (unsigned)st.free_entries);
    }
}

// --- Generation ----------------------------------------------------------------

/** Generate key + self-signed cert into s_key/s_crt and persist. @p sta_ip_be
 *  0 = no STA SAN (first boot, address unknown). */
static esp_err_t generate(const char *chip_id, uint32_t sta_ip_be) {
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
    rc = mbedtls_pk_write_key_pem(&pk, (unsigned char *)s_key, sizeof(s_key));
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
    rc = mbedtls_x509write_crt_pem(&crt, (unsigned char *)s_crt, sizeof(s_crt));
    if (rc != 0) {
        ESP_LOGE(TAG, "mbedtls_x509write_crt_pem failed (-0x%04x)", (unsigned)-rc);
        goto out_crt;
    }

    {
        mbedtls_x509_crt parsed;
        result = parse_pem(&parsed, s_crt, strlen(s_crt) + 1);
        if (result == ESP_OK) result = fingerprint_of(&parsed);
        mbedtls_x509_crt_free(&parsed);
    }
    if (result == ESP_OK) result = save_to_nvs(clock_ok ? (uint32_t)time(NULL) : 0);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "generated P-256 key + self-signed cert in %lld ms (%u B cert, %u B key, valid %s..%s%s)",
                 (long long)((esp_timer_get_time() - t0) / 1000),
                 (unsigned)strlen(s_crt), (unsigned)strlen(s_key), nb_s, na_s,
                 sta_ip_be ? ", STA address included" : ", no STA address yet");
    } else {
        ESP_LOGE(TAG, "post-generation step failed: %s", esp_err_to_name(result));
    }

out_crt:
    mbedtls_x509write_crt_free(&crt);
out_pk:
    mbedtls_pk_free(&pk);
    if (result != ESP_OK) {
        // Never leave a half-written pair behind for the getters to expose.
        memset(s_crt, 0, sizeof(s_crt));
        memset(s_key, 0, sizeof(s_key));
        s_fp[0] = 0;
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
    } else {
        if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "NVS load failed (%s) — regenerating", esp_err_to_name(err));
        }
        err = generate(chip_id, 0);
        if (err == ESP_OK) ESP_LOGI(TAG, "certificate SHA-256 %s", s_fp);
    }
    s_ready = (err == ESP_OK);
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
        ESP_LOGI(TAG, "certificate names the current address, %lld days left — no re-issue",
                 (long long)((valid_to - now) / 86400));
        return ESP_OK;
    }

    // Loop guard (design D5): a re-issue leads to a reboot; if the address
    // is different AGAIN within minutes of the last issue on a software
    // reset, the DHCP server is handing out a fresh lease every boot and
    // re-issuing would loop forever.
    const uint32_t issued = issued_from_nvs();
    if (now != 0 && issued != 0 && (now - (int64_t)issued) < REISSUE_LOOP_GUARD_S &&
        esp_reset_reason() == ESP_RST_SW) {
        ESP_LOGE(TAG, "address changed again %llds after the last re-issue — refusing to loop. "
                 "Give this node a DHCP reservation.", (long long)(now - (int64_t)issued));
        return ESP_OK;
    }

    ESP_LOGW(TAG, "re-issuing certificate: %s%s", has_ip ? "" : "current address missing from SAN",
             expiring ? (has_ip ? "expires within 30 days" : " + expires within 30 days") : "");
    err = generate(chip_id, sta_ip_be);
    if (err != ESP_OK) {
        // generate() zeroed the statics; reload the previous credential so
        // the running server and /cert.pem stay consistent until reboot.
        s_ready = false;
        if (load_from_nvs() == ESP_OK) s_ready = true;
        return err;
    }
    ESP_LOGI(TAG, "certificate SHA-256 %s (effective after reboot)", s_fp);
    if (reissued) *reissued = true;
    return ESP_OK;
}

const char *tls_cert_pem(void)         { return s_ready ? s_crt : NULL; }
size_t      tls_cert_pem_len(void)     { return s_ready ? strlen(s_crt) + 1 : 0; }
const char *tls_key_pem(void)          { return s_ready ? s_key : NULL; }
size_t      tls_key_pem_len(void)      { return s_ready ? strlen(s_key) + 1 : 0; }
const char *tls_cert_fingerprint(void) { return s_ready ? s_fp : ""; }

#endif  // HAL_HAS_HTTPS
