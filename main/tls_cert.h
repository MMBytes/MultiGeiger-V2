#pragma once

/** @file
 *  @brief Per-device TLS server credential: ECDSA P-256 key + self-signed
 *         X.509 certificate, generated on first boot and kept in NVS (V2.8.0).
 *
 *  Design: docs/superpowers/specs/2026-09-12-https-web-server-design.md,
 *  decisions D2-D6. Nothing here is compiled unless HAL_HAS_HTTPS; the
 *  Heltec V2 builds see empty inline stubs so main.c needs no #if.
 *
 *  Lifecycle: main.c calls tls_cert_ensure() once, after nvs_flash_init()
 *  and before http_server_start(). Later, once STA has an address and the
 *  clock is sane, main.c calls tls_cert_reconcile() once per boot; if that
 *  re-issues, main.c requests a reboot so the TLS server picks the new
 *  certificate up. The PEM buffers are module statics and stay valid for the
 *  life of the firmware, so http_server.c may hand the pointers straight to
 *  esp_https_server without copying (esp_https_server copies them itself).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "hal.h"

#if HAL_HAS_HTTPS

/** @brief Load the credential from NVS, or generate and store a new one.
 *
 *  Load path: reads NVS `geiger_tls/crt` + `geiger_tls/key`, parses the
 *  certificate (this doubles as an integrity check of the stored blob) and
 *  computes the fingerprint. Any failure on the load path falls through to
 *  the generate path, so a corrupt blob self-heals with one new certificate.
 *
 *  Generate path: PSA ECC secp256r1 keypair -> mbedTLS PK -> SEC1 PEM;
 *  X.509 v3 self-signed CA:true cert (CN=chip_id; SAN dNSName chip_id +
 *  iPAddress 192.168.4.1; no STA address yet — see tls_cert_reconcile();
 *  SHA-256; 800-day validity from "now" when NTP is valid, else from
 *  2026-01-01). Sub-second on every supported target; the duration is logged.
 *
 *  @param chip_id  "esp32-<num>" from main.c; becomes the CN and DNS SAN.
 *  @return ESP_OK when the getters below return valid data; otherwise an
 *          error and the getters return NULL / 0 / "" — the caller falls
 *          back to plain HTTP (design D9).
 */
esp_err_t tls_cert_ensure(const char *chip_id);

/** @brief Once per boot, after STA has an IP and the clock is sane: re-issue
 *         if the stored certificate does not name @p sta_ip_be in its SAN or
 *         expires within 30 days.
 *
 *  Why: Chrome/Edge/Safari do not suppress a name mismatch even for a
 *  trusted certificate, so the DHCP address people actually browse to must
 *  be in the SAN or "trust it once" never works. A re-issue writes the NEW
 *  pair to NVS only; the module statics — and with them the RUNNING TLS
 *  server, /cert.pem and the fingerprint on / — keep the OLD certificate, so
 *  the caller must reboot (main_request_restart()) to make it effective.
 *
 *  That reboot is NOT immediate (main_request_restart() defers until the TX
 *  worker is idle plus ~2 s), which is why the new pair is generated into
 *  heap STAGING buffers: everything the device reports stays consistent with
 *  what :443 actually presents until the reboot, and no concurrent reader of
 *  the live PEMs can see a half-written one. A failed re-issue leaves memory
 *  completely untouched.
 *
 *  Loop guard: if NVS says the current certificate was RE-ISSUED less than
 *  10 minutes ago (first-time creation does not count — it stores no
 *  timestamp) and this boot is a software reset, the call logs an error
 *  and returns ESP_OK with *reissued = false. That stops a DHCP server that
 *  hands out a new address on every boot from causing a reboot loop; the
 *  cure is a DHCP reservation, and the log line says so.
 *
 *  @param chip_id    As for tls_cert_ensure().
 *  @param sta_ip_be  Current STA IPv4 in network byte order (esp_ip4_addr.addr).
 *  @param reissued   Out: true when a new certificate was written to NVS.
 *  @return ESP_OK on a decision (either way); an error only if a needed
 *          re-issue failed, in which case the old credential stays in place.
 */
esp_err_t tls_cert_reconcile(const char *chip_id, uint32_t sta_ip_be, bool *reissued);

/** @brief Certificate PEM, NUL-terminated. NULL before tls_cert_ensure() succeeds. */
const char *tls_cert_pem(void);

/** @brief `strlen(pem) + 1` — esp-tls detects PEM by the counted NUL. 0 if none. */
size_t tls_cert_pem_len(void);

/** @brief Private key PEM (SEC1 "EC PRIVATE KEY"), NUL-terminated. NULL if none. */
const char *tls_key_pem(void);

/** @brief `strlen(key) + 1`. 0 if none. */
size_t tls_key_pem_len(void);

/** @brief SHA-256 over the certificate DER as "AA:BB:...:FF" (95 chars), or "". */
const char *tls_cert_fingerprint(void);

#else   // !HAL_HAS_HTTPS — Heltec V2 / 4 MB: plain HTTP, nothing to provision.

static inline esp_err_t tls_cert_ensure(const char *chip_id) { (void)chip_id; return ESP_ERR_NOT_SUPPORTED; }
static inline esp_err_t tls_cert_reconcile(const char *chip_id, uint32_t sta_ip_be, bool *reissued) {
    (void)chip_id; (void)sta_ip_be; if (reissued) *reissued = false; return ESP_ERR_NOT_SUPPORTED;
}
static inline const char *tls_cert_pem(void)         { return NULL; }
static inline size_t      tls_cert_pem_len(void)     { return 0; }
static inline const char *tls_key_pem(void)          { return NULL; }
static inline size_t      tls_key_pem_len(void)      { return 0; }
static inline const char *tls_cert_fingerprint(void) { return ""; }

#endif  // HAL_HAS_HTTPS
