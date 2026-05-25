// FTP log upload — native ESP-IDF.
//
// Passive FTP over BSD sockets (lwIP). All reads go through select() with a
// millisecond deadline so a half-open TCP connection (WiFi drop mid-transfer)
// cannot wedge the main loop on a blocking recv().
//
// Flow:
//   connect ctrl → 220 → USER → (331→PASS→230 | 230)
//                     → TYPE I → 200
//                     → PASV → 227 (+parse h,h,h,h,p,p)
//                     → connect data → STOR path → 125/150
//                     → write body (chunked, stall-detected) → close data
//                     → 226 (longer timeout; router may flush slowly) → QUIT
// If the write stalls, the 226 read is skipped and ctrl is closed fast
// (half-open hazard — leaving ctrl open would re-expose the recv() block).

#include "log_ftp.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"   // V2.3.15: per-upload heap drift diagnostic
#include "esp_system.h"      // V2.3.15: esp_get_free_heap_size, _minimum_free_heap_size

#include "mbedtls/ssl.h"
#include "mbedtls/error.h"
#include "mbedtls/net_sockets.h"
#include "psa/crypto.h"
// ESP-IDF 6.0 ships mbedtls 4.x which removes entropy/ctr_drbg setup from user
// code — PSA crypto is auto-initialised at system startup and the RNG is pulled
// from PSA internally; mbedtls_ssl_conf_rng() has been removed. See
// mbedtls/docs/4.0-migration-guide.md.
//
// V2.3.15: psa/crypto.h pulled in for PSA_ERROR_INSUFFICIENT_MEMORY (the -141
// = -0x008D code that mbedtls_strerror cannot decode — PSA error namespace is
// separate from mbedTLS), plus mbedtls_psa_crypto_free() / psa_crypto_init()
// for the nuclear-reset path when slot exhaustion persists.

#include "applog.h"
#include "mqtt.h"          // V2.4.14: mqtt_stop()/mqtt_is_initialized() for FTPS heap teardown
#include "ntp.h"
#include "transmission.h"
#include "util.h"

static const char *TAG = "ftp";

#define FTP_PORT              21
#define FTP_TIMEOUT_MS        10000      // 10 s per control response
#define FTP_CONFIRM_MS        15000      // 15 s for 226 — router may flush to USB
// V2.3.15: 1024 → 4096. Each TLS record carries ~29 B overhead (5 B header +
// 8 B explicit IV + 16 B AES-GCM tag); larger records mean a higher payload-
// to-overhead ratio (4 KB / 29 B ≈ 99 % vs 1 KB / 29 B ≈ 97 %), better TCP
// segment efficiency (one TLS record now spans ~3 MSS), and 4× fewer
// mbedtls_ssl_write calls per upload. Each ssl_write is a potential PSA
// pressure point because mbedTLS hands off AES-GCM record encryption to PSA
// — fewer of them means less per-cycle PSA churn. The new 16 KB lwIP TCP
// send buffer (sdkconfig.defaults) comfortably absorbs 4 KB chunks.
#define FTP_WRITE_CHUNK       4096
#define FTP_WRITE_STALL_MS    15000
#define FTP_QUIT_TIMEOUT_MS   2000
#define FTP_RETRY_COUNT       4         // retries after a failed first attempt
#define FTP_RETRY_DELAY_MS    180000    // 3 min between retries (> one TX cycle)

// V2.3.15: when this many consecutive uploads see -141 (PSA OOM), tear down
// and re-initialise the PSA crypto subsystem to clear leaked slots. Threshold
// chosen so a single transient peak doesn't trigger; 5 attempts × 3 min = 15
// min of sustained failure means there's a real slot leak we need to break.
#define FTP_PSA_OOM_RESET_THRESHOLD  5
static int  s_consecutive_psa_oom = 0;
// Set by log_tls_err whenever it observes -141, cleared at the start of every
// upload. Lets us catch PSA OOM at any point in the upload (handshake, setup,
// ssl_read, ssl_write) without each call site needing to know about the
// per-context tls.psa_oom flag.
static bool s_upload_psa_oom      = false;

// V2.3.15: set by ftp_write_buf when io_send_all returns false due to the
// 15 s wall-clock deadline (TCP-level stall, not a TLS error). Triggers
// preemptive PSA reset in the done block — empirically, mbedtls_ssl_free
// from a WANT_WRITE-then-abandon path leaks PSA crypto state in mbedTLS
// 4.x, and waiting for 5 consecutive subsequent retries to fail with
// -141 wastes 12 minutes (4 retries × 3 min apart). Resetting on the
// stall itself recovers next attempt instead of next-after-next.
static bool s_stall_observed      = false;

static const config_t *s_cfg            = NULL;
static const char     *s_chip_id        = "";
static uint32_t        s_next_upload_ms = 0;   // set in log_ftp_init from cfg
static int             s_retry_count    = 0;   // retries remaining (0 = none pending)
static uint32_t        s_retry_ms       = 0;   // wall-clock ms for next retry
// V2.4.13: set by log_ftp_pause() before OTA so an in-window scheduled
// upload can't fire mid-flash. Survives until reboot — both successful OTA
// (new firmware boots fresh) and failed OTA (user reboots manually) reset.
static bool            s_paused         = false;

// Public stats — populated at end of each upload attempt by the main task
// (FTPS runs on main, see [[feedback_main_task_runs_ftps_not_worker.md]]);
// read by the HTTP server task via log_ftp_get_stats.
//
// V2.4.1 (B1): s_last_at is int64_t — naive store is two 32-bit writes on
// the 32-bit Xtensa cores, so a cross-task reader can see torn halves
// (manifest: momentary year-2038-ish garbage timestamp on the status page
// around FTPS upload completion). Spinlock guards every read/write of the
// stats group so the reader also gets a consistent snapshot of
// have_last/last_ok/last_at/last_bytes from one upload event.
static bool          s_have_last  = false;
static bool          s_last_ok    = false;
static int64_t       s_last_at    = 0;
static uint32_t      s_last_bytes = 0;
static portMUX_TYPE  s_stats_mux  = portMUX_INITIALIZER_UNLOCKED;

void log_ftp_get_stats(log_ftp_stats_t *out) {
    if (!out) return;
    portENTER_CRITICAL(&s_stats_mux);
    out->have_last   = s_have_last;
    out->last_ok     = s_last_ok;
    out->last_at     = s_last_at;
    out->last_bytes  = s_last_bytes;
    portEXIT_CRITICAL(&s_stats_mux);
    out->next_due_ms = s_next_upload_ms;   // u32 — torn-tolerant, no lock
}

// V2.4.13: see header for rationale. One-line setter, no locking required —
// log_ftp_loop runs on the main task and only reads this flag; the only
// other writer is the explicit /update POST handler, also on a writer task
// (httpd worker) — but the flag is one byte, torn-tolerant on Xtensa, and
// a missed tick at the boundary just means one extra log_ftp_loop iteration
// before the pause takes effect (which is harmless given the 1 s loop).
void log_ftp_pause(void) {
    s_paused = true;
    ESP_LOGI(TAG, "paused — no scheduled uploads until reboot");
}

// V2.4.19: exported so the extracted periodic.c can reset the FTP-side
// PSA-OOM streak after a successful 24h refresh. See log_ftp.h doc.
void log_ftp_note_psa_refreshed(void) {
    s_consecutive_psa_oom = 0;
}

static bool wifi_up_ftp(void) {
    wifi_ap_record_t ap;
    return esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
}

// --- I/O abstraction --------------------------------------------------------
// Wraps a BSD socket optionally layered with mbedTLS. All FTP command/response
// and data-channel I/O goes through this so the plain and FTPS paths share
// the protocol code.
//
// Timeouts are applied via SO_RCVTIMEO / SO_SNDTIMEO on the underlying socket.
// For TLS reads/writes this still bounds the operation because mbedtls will
// pass WANT_READ back to us when recv() returns EAGAIN.

typedef struct {
    int  sock;
    bool tls;
    mbedtls_ssl_context ssl;
    mbedtls_net_context net;
} ftp_io_t;

// Per-upload TLS state (config + resumable session). PSA provides the RNG.
typedef struct {
    mbedtls_ssl_config       conf;
    mbedtls_ssl_session      ctrl_session;   // saved from control for data reuse
    bool                     have_session;
    bool                     psa_oom;        // V2.3.15: -141 seen during this upload
} ftp_tls_ctx_t;

// V2.3.15: PSA error codes are a separate namespace from mbedTLS's own — when a
// PSA call fails inside mbedtls_ssl_setup / mbedtls_ssl_handshake (because PSA
// runs ECDHE / HKDF / AES-GCM under the hood in mbedtls 4.x), the PSA status
// leaks back through the mbedTLS API as a bare negative integer that
// mbedtls_strerror cannot decode (it tries high=ret&0xFF80 and low=ret&0x7F,
// neither matches a known mbedTLS module so both surface as "UNKNOWN ERROR
// CODE"). PSA_ERROR_INSUFFICIENT_MEMORY = -141 = -0x008D is the slot-pool
// exhaustion code; recognising it lets us emit a clear log line ("PSA out of
// memory") instead of a useless "UNKNOWN ERROR CODE".
static inline bool is_psa_oom(int r) {
    return r == (int)PSA_ERROR_INSUFFICIENT_MEMORY;
}

static void log_tls_err(const char *where, int r) {
    if (is_psa_oom(r)) {
        ESP_LOGE(TAG, "%s: PSA out of memory (PSA_ERROR_INSUFFICIENT_MEMORY = -141)", where);
        s_upload_psa_oom = true;
    } else {
        char eb[80]; mbedtls_strerror(r, eb, sizeof(eb));
        ESP_LOGW(TAG, "%s: -0x%04x %s", where, -r, eb);
    }
}

static void io_init_plain(ftp_io_t *io, int sock) {
    io->sock = sock;
    io->tls  = false;
    memset(&io->ssl, 0, sizeof(io->ssl));
    memset(&io->net, 0, sizeof(io->net));
}

static void io_set_rcv_timeout(const ftp_io_t *io, uint32_t ms) {
    struct timeval tv = { .tv_sec = ms / 1000, .tv_usec = (ms % 1000) * 1000 };
    setsockopt(io->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

static void io_set_snd_timeout(const ftp_io_t *io, uint32_t ms) {
    struct timeval tv = { .tv_sec = ms / 1000, .tv_usec = (ms % 1000) * 1000 };
    setsockopt(io->sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

static void io_close(ftp_io_t *io) {
    if (io->tls) {
        // V2.3.19: properly flush close_notify before tearing down the
        // socket. mbedtls_ssl_close_notify() is non-blocking and frequently
        // returns MBEDTLS_ERR_SSL_WANT_WRITE right after a large body
        // upload while the lwIP TCP send buffer drains. Pre-V2.3.19 we
        // ignored the return value and immediately closed the TCP socket,
        // so the close_notify alert never made it onto the wire.
        //
        // RFC 8446 §6.1: TLS 1.3 servers MUST treat TCP close without
        // close_notify as a possible truncation attack. The project's
        // LAN FTPS server reacted with "426 Connection reset by peer"
        // on the data channel — the V2.3.15-deferred bug. TLS 1.2
        // servers are lenient about this for backwards compatibility
        // with old broken clients, so the bug only manifested once we
        // re-enabled TLS 1.3 in V2.3.5.
        //
        // 2 s deadline keeps us from blocking forever on a genuinely
        // stalled socket; in normal operation this loop fires once or
        // twice and exits in <50 ms.
        // V2.3.19: send our close_notify, retrying on WANT_WRITE (lwIP TCP
        // send buffer drains slowly right after a body upload). 2 s deadline.
        int64_t deadline_us = esp_timer_get_time() + 2 * 1000 * 1000;
        for (;;) {
            int rc = mbedtls_ssl_close_notify(&io->ssl);
            if (rc == 0) break;
            if (rc != MBEDTLS_ERR_SSL_WANT_READ &&
                rc != MBEDTLS_ERR_SSL_WANT_WRITE) {
                log_tls_err("close_notify", rc);
                break;
            }
            if (esp_timer_get_time() > deadline_us) {
                ESP_LOGW(TAG, "close_notify timeout — proceeding to close");
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // V2.3.22: bidirectional close — drain the server's close_notify
        // response (and any trailing TLS 1.3 NewSessionTicket post-handshake
        // messages) before tearing down TCP. V2.3.19 made our close_notify
        // reach the wire, but the server still saw "Connection reset by peer"
        // because lwIP sends TCP RST instead of FIN whenever there's unread
        // data in the receive buffer at close() time. TLS 1.3 piles up more
        // post-handshake messages than 1.2 (NewSessionTickets, server's own
        // close_notify) — diagnosed via V2.3.22-pre3 iteration trace which
        // showed the server sending 2× NewSessionTicket on the data channel
        // before its close_notify; pre-V2.3.22 we'd close on top of those
        // and the kernel would emit RST.
        //
        // Standard mature-TLS-client pattern (OpenSSL's SSL_shutdown does
        // this internally): keep reading until peer's close_notify or a
        // benign error, then close the TCP socket cleanly. 1 s deadline is
        // plenty — well-behaved peers respond within ms.
        int64_t drain_deadline_us = esp_timer_get_time() + 1 * 1000 * 1000;
        unsigned char drain_buf[64];
        int drain_iters = 0;
        int drain_tickets = 0;
        bool drain_clean = false;
        for (;;) {
            int rc = mbedtls_ssl_read(&io->ssl, drain_buf, sizeof(drain_buf));
            drain_iters++;
            if (rc == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY) { drain_clean = true; break; }
            if (rc == MBEDTLS_ERR_SSL_RECEIVED_NEW_SESSION_TICKET) {
                drain_tickets++;
                continue;
            }
            if (rc <= 0 && rc != MBEDTLS_ERR_SSL_WANT_READ &&
                           rc != MBEDTLS_ERR_SSL_WANT_WRITE) break;  // benign error / EOF
            if (esp_timer_get_time() > drain_deadline_us) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        // V2.3.24: downgraded INFO→DEBUG. Was visible at INFO from V2.3.22 to
        // confirm the bidirectional-close fix in production; with that arc
        // closed and the V2.3.24 snapshot-scratch fix removing per-upload
        // boilerplate from the ring, kept at DEBUG for future regression
        // investigation. Recompile with CONFIG_LOG_DEFAULT_LEVEL=DEBUG (or
        // call esp_log_level_set("ftp", ESP_LOG_DEBUG) from the console) to
        // see "drain not clean" the moment the close-sequence path regresses.
        ESP_LOGD(TAG, "TLS shutdown: drain iters=%d tickets=%d %s",
                 drain_iters, drain_tickets,
                 drain_clean ? "clean (PEER_CLOSE_NOTIFY)" : "no PEER_CLOSE_NOTIFY");

        // V2.3.15: belt-and-braces release of any PSA references the session
        // is still holding. mbedtls_ssl_free does this internally on a fully-
        // initialised context, but session_reset is the documented hook for
        // forcing PSA-backend resource release in mbedtls 4.x and is cheap
        // enough to call unconditionally.
        mbedtls_ssl_session_reset(&io->ssl);
        mbedtls_ssl_free(&io->ssl);
        io->tls = false;
    }
    if (io->sock >= 0) {
        // V2.3.22: explicit half-close (FIN, not RST) before the actual
        // close(). Belt-and-braces against the same kernel-RST-on-unread-
        // data behaviour the bidirectional drain above mitigates: even if
        // the drain loop missed something, shutdown(SHUT_WR) tells lwIP
        // "queue FIN now and send any remaining data, then close" — which
        // is what the server expects to see. Failure here is non-fatal;
        // close() below is the actual cleanup.
        shutdown(io->sock, SHUT_WR);
        close(io->sock);
        io->sock = -1;
    }
}

// mbedTLS BIO shims — take over an already-connected fd and adapt errno to
// WANT_READ/WANT_WRITE so mbedtls retries correctly under SO_*_TIMEO.
static int io_tls_send(void *ctx, const unsigned char *buf, size_t len) {
    int fd = ((mbedtls_net_context *)ctx)->fd;
    int n = send(fd, buf, len, 0);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return MBEDTLS_ERR_SSL_WANT_WRITE;
        return MBEDTLS_ERR_NET_SEND_FAILED;
    }
    return n;
}

static int io_tls_recv(void *ctx, unsigned char *buf, size_t len) {
    int fd = ((mbedtls_net_context *)ctx)->fd;
    int n = recv(fd, buf, len, 0);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return MBEDTLS_ERR_SSL_WANT_READ;
        return MBEDTLS_ERR_NET_RECV_FAILED;
    }
    return n;  // 0 = peer closed
}

static bool tls_ctx_init(ftp_tls_ctx_t *t) {
    mbedtls_ssl_config_init(&t->conf);
    mbedtls_ssl_session_init(&t->ctrl_session);
    t->have_session = false;
    t->psa_oom      = false;

    int r = mbedtls_ssl_config_defaults(&t->conf,
                                        MBEDTLS_SSL_IS_CLIENT,
                                        MBEDTLS_SSL_TRANSPORT_STREAM,
                                        MBEDTLS_SSL_PRESET_DEFAULT);
    if (r != 0) {
        log_tls_err("ssl_conf_defaults", r);
        if (is_psa_oom(r)) t->psa_oom = true;
        return false;
    }

    // No cert verification: most LAN FTP servers use self-signed certs, and
    // the user opted into this by ticking "certificate NOT verified".
    mbedtls_ssl_conf_authmode(&t->conf, MBEDTLS_SSL_VERIFY_NONE);

    // TLS 1.2 cap. V2.3.5 enabled TLS 1.3 in the build, after which FTPS
    // uploads started failing with "USER reject: -1". V2.3.10 worked around
    // this by hard-pinning FTPS at TLS 1.2 — and at the time the symptom was
    // attributed (incorrectly) to old vsftpd/proftpd/pure-ftpd builds
    // mishandling the TLS 1.3 post-handshake protocol.
    //
    // V2.3.15 found the V2.3.5–V2.3.9 "USER reject: -1" was actually one bug
    // and the post-V2.3.15 TLS-1.3-attempt failure is a SEPARATE second bug:
    //
    // BUG 1 (FIXED): -0x7B00 = MBEDTLS_ERR_SSL_RECEIVED_NEW_SESSION_TICKET
    //   is a control-flow signal mbedTLS 4.x raises from ssl_read when a
    //   TLS 1.3 server sends a post-handshake NewSessionTicket. Our io_recv1
    //   didn't recognise the signal, returned -1, surfaced as "USER reject:
    //   -1". V2.3.15's io_recv1 now handles -0x7B00 like WANT_READ.
    //
    // BUG 2 (PARKED): Even with bug 1 fixed, TLS 1.3 against THIS specific
    //   FTPS server (project LAN, 192.168.123.1) fails with "426 Data
    //   Connection: Connection reset by peer" on the data channel after
    //   STOR. Verified independent of session reuse (with PSK or without —
    //   same 426). Most likely the server doesn't fully implement TLS 1.3
    //   on the data channel, or its require_ssl_reuse=YES policy needs the
    //   modern TLS 1.3 PSK-extension resumption that mbedtls_ssl_set_session
    //   doesn't provide (TLS 1.2-era API). TLS 1.2 + session reuse works
    //   flawlessly. See reference_ftps_tls13_investigation.md memory for
    //   full failure matrix and Options A-D for future investigation.
    //
    // The PSA-slot exhaustion paths this release also addresses (slot bump
    // 32→128, nuclear reset on stall/threshold) is a THIRD independent
    // phenomenon — triggered by write stalls leaking PSA crypto state on
    // the WANT_WRITE-then-abandon tear-down path. Three bugs in total.
    //
    // V2.3.15 ships with TLS 1.2 cap DEFAULT-ON via the /config "Limit FTPS
    // to TLS 1.2" checkbox (NVS key ftp_t12only, defaults to true). Users
    // with TLS 1.3-capable FTPS servers can untick to opt back in. HTTPS
    // targets (Madavi / SC / Radmon / OSM / aqi.eco) are unaffected — they
    // always negotiate 1.3 freely via esp_tls (separate code path).
    if (s_cfg && s_cfg->ftp_tls12_only) {
        mbedtls_ssl_conf_max_tls_version(&t->conf, MBEDTLS_SSL_VERSION_TLS1_2);
    }

    // V2.3.22-pre3: mbedTLS protocol-level debug callback removed entirely.
    // pre1 enabled it globally (crashed on HTTPS volume); pre2 scoped it to
    // FTPS only (still crashed during the FTPS handshake itself). Our
    // [pre1] iteration logs in io_close are the actionable diagnostic for
    // the close-sequence question — protocol-level mbedTLS logging would
    // only add nice-to-have detail at huge stack/volume cost.

    return true;
}

static void tls_ctx_free(ftp_tls_ctx_t *t) {
    mbedtls_ssl_session_free(&t->ctrl_session);
    mbedtls_ssl_config_free(&t->conf);
    // V2.3.22-pre3: mbedtls_debug_set_threshold(0) call removed — pre1/pre2
    // diagnostic logging dropped entirely (see tls_ctx_init for context).
}

// Wrap an already-connected socket in TLS. For the data connection, reuse
// the saved control session — vsftpd's default require_ssl_reuse=YES rejects
// data handshakes that don't resume the control session.
static bool io_upgrade_tls(ftp_io_t *io, ftp_tls_ctx_t *t, bool is_data) {
    mbedtls_ssl_init(&io->ssl);
    int r = mbedtls_ssl_setup(&io->ssl, &t->conf);
    if (r != 0) {
        log_tls_err("ssl_setup", r);
        if (is_psa_oom(r)) t->psa_oom = true;
        mbedtls_ssl_free(&io->ssl);
        return false;
    }

    io->net.fd = io->sock;
    mbedtls_ssl_set_bio(&io->ssl, &io->net, io_tls_send, io_tls_recv, NULL);

    if (is_data && t->have_session) {
        // Non-fatal on failure — server may accept a fresh session.
        int sr = mbedtls_ssl_set_session(&io->ssl, &t->ctrl_session);
        if (sr != 0) ESP_LOGW(TAG, "set_session: -0x%04x (continuing)", -sr);
    }

    while ((r = mbedtls_ssl_handshake(&io->ssl)) != 0) {
        if (r != MBEDTLS_ERR_SSL_WANT_READ && r != MBEDTLS_ERR_SSL_WANT_WRITE) {
            log_tls_err(is_data ? "data TLS handshake" : "ctrl TLS handshake", r);
            if (is_psa_oom(r)) t->psa_oom = true;
            // V2.3.15: session_reset before free — same belt-and-braces as
            // io_close. Releases any PSA refs the partial-handshake state
            // is holding before we drop the context.
            mbedtls_ssl_session_reset(&io->ssl);
            mbedtls_ssl_free(&io->ssl);
            return false;
        }
    }

    if (!is_data) {
        // Save control session for data-channel reuse.
        if (mbedtls_ssl_get_session(&io->ssl, &t->ctrl_session) == 0) {
            t->have_session = true;
        }
    }

    // Surface what was actually negotiated. Useful when a future FTPS server
    // misbehaves at the handshake or post-handshake stage — without this it's
    // a guessing game whether the issue is version mismatch, cipher mismatch,
    // session resumption, or something else. mbedtls_ssl_get_version() and
    // _ciphersuite() return strings owned by mbedTLS; do not free.
    //
    // V2.3.24: downgraded INFO→DEBUG. Two of these per upload (ctrl + data)
    // is repetitive boilerplate once the TLS arc is settled; bring back at
    // INFO via esp_log_level_set("ftp", ESP_LOG_DEBUG) when investigating
    // a TLS regression on a new server.
    const char *ver  = mbedtls_ssl_get_version(&io->ssl);
    const char *ciph = mbedtls_ssl_get_ciphersuite(&io->ssl);
    ESP_LOGD(TAG, "TLS %s on %s channel: cipher=%s",
             ver ? ver : "?", is_data ? "data" : "ctrl", ciph ? ciph : "?");

    io->tls = true;
    return true;
}

// Blocking 1-byte read. Returns 1 on success, 0 on timeout (SO_RCVTIMEO fired),
// -1 on error/close. Logs the precise mbedTLS error code on failure so future
// FTPS-over-TLS issues can be diagnosed without a packet capture (V2.3.10
// added this — without it, "USER reject: -1" left no breadcrumb to identify
// whether the server sent close-notify, dropped the socket, or returned
// malformed application data).
static int io_recv1(ftp_io_t *io, char *out) {
    if (io->tls) {
        for (;;) {
            int r = mbedtls_ssl_read(&io->ssl, (unsigned char *)out, 1);
            if (r == 1) return 1;
            if (r == MBEDTLS_ERR_SSL_WANT_READ || r == MBEDTLS_ERR_SSL_WANT_WRITE) continue;
            // V2.3.15 (second-pass diagnostic discovery): MBEDTLS_ERR_SSL_RECEIVED_
            // NEW_SESSION_TICKET (-0x7B00) is NOT an error — it's a control-flow
            // signal mbedTLS 4.x raises from ssl_read when a TLS 1.3 server sends
            // a post-handshake NewSessionTicket. The application is invited to
            // optionally call mbedtls_ssl_get_session() to save the ticket for
            // future resumption, then call ssl_read again to receive actual
            // application data. We don't save it (yet — V2.3.16 session-resumption
            // work will be the natural place to harvest it); just continue the
            // read loop. This is the actual root cause of the V2.3.5–V2.3.9
            // "USER reject: -1" symptom that we incorrectly attributed to PSA
            // exhaustion — at that time io_recv1 didn't decode mbedTLS errors
            // (V2.3.10 added that diagnostic) so -0x7B00 surfaced as a bare -1.
            if (r == MBEDTLS_ERR_SSL_RECEIVED_NEW_SESSION_TICKET) {
                // V2.3.24: downgraded INFO→DEBUG. Twice per upload — pure
                // boilerplate now that TLS 1.3 ticket handling is settled.
                // Re-enable via esp_log_level_set("ftp", ESP_LOG_DEBUG) when
                // investigating a session-resumption regression.
                ESP_LOGD(TAG, "TLS 1.3 NewSessionTicket received (ignored — V2.3.16 will save)");
                continue;
            }
            if (r == MBEDTLS_ERR_SSL_TIMEOUT) return 0;
            // log_tls_err recognises -141 (PSA_ERROR_INSUFFICIENT_MEMORY) — the
            // separate "ssl_read fails because PSA slot pool is exhausted" path,
            // distinct from the NewSessionTicket signal handled above.
            log_tls_err("ssl_read", r);
            return -1;
        }
    }
    int r = recv(io->sock, out, 1, 0);
    if (r == 1) return 1;
    if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
    if (r < 0) ESP_LOGW(TAG, "recv: errno=%d (%s)", errno, strerror(errno));
    return -1;
}

// Blocking write-all. Returns true if the full buffer was sent within the
// per-chunk SO_SNDTIMEO / FTP_WRITE_STALL_MS budget.
static bool io_send_all(ftp_io_t *io, const void *buf, size_t len) {
    const unsigned char *p = (const unsigned char *)buf;
    size_t sent = 0;
    // Deadline resets each time bytes are actually written. WANT_WRITE spins
    // without making progress, so SO_SNDTIMEO (which fires EAGAIN → WANT_WRITE)
    // never terminates the loop on its own — we need a wall-clock guard here.
    int64_t deadline_us = esp_timer_get_time() + (int64_t)FTP_WRITE_STALL_MS * 1000;
    while (sent < len) {
        int n;
        if (io->tls) {
            n = mbedtls_ssl_write(&io->ssl, p + sent, len - sent);
            if (n == MBEDTLS_ERR_SSL_WANT_READ || n == MBEDTLS_ERR_SSL_WANT_WRITE) {
                if (esp_timer_get_time() > deadline_us) {
                    ESP_LOGW(TAG, "io_send_all: TLS stall at %u/%u bytes",
                             (unsigned)sent, (unsigned)len);
                    return false;
                }
                continue;
            }
            // V2.3.15 (defensive): in mbedTLS 4.x, NewSessionTicket arrives via
            // server→client reads, so this should never appear from ssl_write.
            // Handle it like WANT_READ anyway — if mbedTLS ever surfaces it on
            // the write path (e.g. internal poll for inbound records before
            // sending), continuing keeps the symmetry with io_recv1.
            if (n == MBEDTLS_ERR_SSL_RECEIVED_NEW_SESSION_TICKET) continue;
        } else {
            n = send(io->sock, p + sent, len - sent, 0);
        }
        if (n <= 0) {
            // For TLS path n is a negative MBEDTLS_ERR_* (or PSA) code; for
            // plain it's -1 with errno set. Log both shapes so the cause is
            // visible without cross-referencing. V2.3.15: route TLS errors
            // through log_tls_err so PSA OOM (-141) is decoded explicitly
            // instead of "UNKNOWN ERROR CODE".
            if (io->tls && n < 0) {
                char where[48];
                snprintf(where, sizeof(where),
                         "io_send_all at %u/%u",
                         (unsigned)sent, (unsigned)len);
                log_tls_err(where, n);
            } else {
                ESP_LOGW(TAG, "io_send_all: err at %u/%u (n=%d errno=%d)",
                         (unsigned)sent, (unsigned)len, n, errno);
            }
            return false;
        }
        sent += (size_t)n;
        deadline_us = esp_timer_get_time() + (int64_t)FTP_WRITE_STALL_MS * 1000;
    }
    return true;
}

// Read a complete FTP response (handles multi-line "code-" continuations).
// Copies the final line into out_last (may be NULL). Returns the 3-digit
// status code, or -1 on timeout/close/error. Timeout is applied via
// SO_RCVTIMEO on the underlying socket — each byte must arrive within ms.
static int ftp_read_response(ftp_io_t *io, uint32_t timeout_ms, char *out_last, size_t out_sz) {
    char line[512];
    size_t ll = 0;
    bool multiline = false;
    char expected[4] = { 0 };
    if (out_last && out_sz) out_last[0] = 0;

    io_set_rcv_timeout(io, timeout_ms);

    for (;;) {
        char c;
        int r = io_recv1(io, &c);
        if (r <= 0) return -1;
        if (c == '\r') continue;
        if (c == '\n') {
            line[ll] = 0;
            if (ll >= 3 &&
                line[0] >= '0' && line[0] <= '9' &&
                line[1] >= '0' && line[1] <= '9' &&
                line[2] >= '0' && line[2] <= '9') {
                int code = (line[0] - '0') * 100 + (line[1] - '0') * 10 + (line[2] - '0');
                if (!multiline) {
                    if (ll >= 4 && line[3] == '-') {
                        multiline = true;
                        expected[0] = line[0];
                        expected[1] = line[1];
                        expected[2] = line[2];
                        expected[3] = 0;
                    } else {
                        if (out_last && out_sz) safe_strcpy(out_last, line, out_sz);
                        return code;
                    }
                } else if (strncmp(line, expected, 3) == 0 && ll >= 4 && line[3] == ' ') {
                    if (out_last && out_sz) safe_strcpy(out_last, line, out_sz);
                    return code;
                }
            }
            ll = 0;
        } else if (ll < sizeof(line) - 1) {
            line[ll++] = c;
        }
    }
}

// Connect by hostname. Non-blocking connect + select() gives us a hard timeout.
static int ftp_connect_host(const char *host, int port, uint32_t timeout_ms) {
    struct addrinfo hints = { 0 };
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    char port_s[8];
    snprintf(port_s, sizeof(port_s), "%d", port);
    struct addrinfo *res = NULL;
    int err = getaddrinfo(host, port_s, &hints, &res);
    if (err != 0 || !res) {
        ESP_LOGW(TAG, "DNS %s failed: %d", host, err);
        if (res) freeaddrinfo(res);
        return -1;
    }
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        freeaddrinfo(res);
        return -1;
    }
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    int rc = connect(sock, res->ai_addr, res->ai_addrlen);
    if (rc < 0 && errno != EINPROGRESS) {
        close(sock);
        freeaddrinfo(res);
        return -1;
    }
    fd_set ws;
    FD_ZERO(&ws);
    FD_SET(sock, &ws);
    struct timeval tv = { .tv_sec = timeout_ms / 1000, .tv_usec = (timeout_ms % 1000) * 1000 };
    int sel = select(sock + 1, NULL, &ws, NULL, &tv);
    if (sel <= 0) {
        ESP_LOGW(TAG, "connect %s: timeout", host);
        close(sock);
        freeaddrinfo(res);
        return -1;
    }
    int so_error = 0;
    socklen_t slen = sizeof(so_error);
    getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &slen);
    if (so_error != 0) {
        ESP_LOGW(TAG, "connect %s: so_error=%d", host, so_error);
        close(sock);
        freeaddrinfo(res);
        return -1;
    }
    fcntl(sock, F_SETFL, flags);  // back to blocking; reads still go through select()
    freeaddrinfo(res);
    return sock;
}

// PASV-derived numeric-IP connect (h1,h2,h3,h4 packed big-endian).
static int ftp_connect_pasv(uint32_t ip_be, int port, uint32_t timeout_ms) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    struct sockaddr_in sa = { 0 };
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(port);
    sa.sin_addr.s_addr = ip_be;
    int rc = connect(sock, (struct sockaddr *)&sa, sizeof(sa));
    if (rc < 0 && errno != EINPROGRESS) {
        close(sock);
        return -1;
    }
    fd_set ws;
    FD_ZERO(&ws);
    FD_SET(sock, &ws);
    struct timeval tv = { .tv_sec = timeout_ms / 1000, .tv_usec = (timeout_ms % 1000) * 1000 };
    int sel = select(sock + 1, NULL, &ws, NULL, &tv);
    if (sel <= 0) {
        ESP_LOGW(TAG, "PASV connect: timeout");
        close(sock);
        return -1;
    }
    int so_error = 0;
    socklen_t slen = sizeof(so_error);
    getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &slen);
    if (so_error != 0) {
        ESP_LOGW(TAG, "PASV connect: so_error=%d", so_error);
        close(sock);
        return -1;
    }
    fcntl(sock, F_SETFL, flags);
    return sock;
}

static bool ftp_send_cmd(ftp_io_t *io, const char *cmd) {
    if (!io_send_all(io, cmd, strlen(cmd))) return false;
    return io_send_all(io, "\r\n", 2);
}

// Parses the parenthesised tuple from "227 Entering Passive Mode (h,h,h,h,p,p)".
static bool parse_pasv(const char *line, uint32_t *ip_be, int *port) {
    const char *p = strchr(line, '(');
    if (!p) return false;
    int h1, h2, h3, h4, p1, p2;
    if (sscanf(p, "(%d,%d,%d,%d,%d,%d", &h1, &h2, &h3, &h4, &p1, &p2) != 6) return false;
    if (h1 < 0 || h1 > 255 || h2 < 0 || h2 > 255 ||
        h3 < 0 || h3 > 255 || h4 < 0 || h4 > 255 ||
        p1 < 0 || p1 > 255 || p2 < 0 || p2 > 255) return false;
    *port = p1 * 256 + p2;
    *ip_be = ((uint32_t)h1) | ((uint32_t)h2 << 8) |
             ((uint32_t)h3 << 16) | ((uint32_t)h4 << 24);
    return true;
}

// Chunked write — send() / mbedtls_ssl_write() blocks, but SO_SNDTIMEO caps
// each chunk at FTP_WRITE_STALL_MS so a half-open socket cannot hang forever.
static bool ftp_write_buf(ftp_io_t *io, const char *buf, size_t len) {
    io_set_snd_timeout(io, FTP_WRITE_STALL_MS);
    size_t written = 0;
    while (written < len) {
        size_t chunk = len - written;
        if (chunk > FTP_WRITE_CHUNK) chunk = FTP_WRITE_CHUNK;
        if (!io_send_all(io, buf + written, chunk)) {
            ESP_LOGW(TAG, "write stalled after %u/%u bytes",
                     (unsigned)written, (unsigned)len);
            // V2.3.15: arm preemptive PSA reset in done block (stall path
            // leaks crypto state in mbedTLS 4.x).
            s_stall_observed = true;
            return false;
        }
        written += chunk;
    }
    return true;
}

static bool do_ftp_upload(void) {
    if (!s_cfg->ftp_enabled || s_cfg->ftp_host[0] == 0) return false;
    if (!wifi_up_ftp()) {
        ESP_LOGW(TAG, "skip: WiFi down");
        return false;
    }
    if (!ntp_time_valid()) {
        ESP_LOGW(TAG, "skip: NTP not synced (would give 1970 filename)");
        return false;
    }

    // V2.3.15: arm the per-upload diagnostic flags. log_tls_err sets
    // s_upload_psa_oom on -141; ftp_write_buf sets s_stall_observed on a
    // 15 s wall-clock stall.
    s_upload_psa_oom = false;
    s_stall_observed = false;

    // V2.3.15: heap drift diagnostic — log free heap, lifetime min, and the
    // largest contiguous block before each upload. PSA in mbedTLS 4.x stores
    // key material on the heap (hybrid keystore — the slot table is static
    // but the keys themselves are malloc'd), so a slow PSA leak shows up as
    // monotonically decreasing min_free over hours. Pair with the post-
    // upload line in done: to spot per-upload deltas as well.
    ESP_LOGI(TAG, "Pre-upload heap: free=%u min_free=%u largest=%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    // Build remote filename: geiger_<chip>_YYYY-MM-DDTHHMMSS.log.
    // NOTE: the "FTP uploading" log line below must come BEFORE applog_snapshot()
    // so that line is included in this batch — if we logged after, we'd drop it
    // whenever the ring wrapped.
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);
    char name[96];
    snprintf(name, sizeof(name),
             "geiger_%s_%04d-%02d-%02dT%02d%02d%02d.log",
             s_chip_id,
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec);

    char path[256];
    if (s_cfg->ftp_path[0]) {
        size_t plen = strlen(s_cfg->ftp_path);
        const char *sep = (plen > 0 && s_cfg->ftp_path[plen - 1] == '/') ? "" : "/";
        snprintf(path, sizeof(path), "%s%s%s", s_cfg->ftp_path, sep, name);
    } else {
        snprintf(path, sizeof(path), "%s", name);
    }

    // V2.3.16: parse optional :port suffix from ftp_host (default FTP_PORT
    // = 21 if absent or malformed). Uses strchr (rightmost colon would matter
    // only for IPv6; we're IPv4-only via getaddrinfo's AF_INET hint, so a
    // single colon is unambiguously host:port). Hostname/IP goes into a local
    // buffer; the original config string isn't modified.
    char host_buf[64];
    int  ftp_port_use = FTP_PORT;
    {
        const char *colon = strchr(s_cfg->ftp_host, ':');
        if (colon) {
            size_t host_len = (size_t)(colon - s_cfg->ftp_host);
            if (host_len >= sizeof(host_buf)) host_len = sizeof(host_buf) - 1;
            memcpy(host_buf, s_cfg->ftp_host, host_len);
            host_buf[host_len] = 0;
            long p = strtol(colon + 1, NULL, 10);
            if (p >= 1 && p <= 65535) ftp_port_use = (int)p;
            // Else: malformed port → fall back silently to default 21.
        } else {
            safe_strcpy(host_buf, s_cfg->ftp_host, sizeof(host_buf));
        }
    }

    ESP_LOGI(TAG, "FTP%s uploading %s to %s:%d as '%s'",
             s_cfg->ftp_tls ? "S" : "",
             path, host_buf, ftp_port_use,
             s_cfg->ftp_user[0] ? s_cfg->ftp_user : "anonymous");

    // V2.3.16: zero-copy streaming snapshot. Captures segment pointers into the
    // ring memory; no body buffer is allocated. Saves the equivalent of
    // ring-size DRAM during the upload window — ~60 KB on Heltec (huge given
    // the V2.3.15 min_free pressure), ~1 MB PSRAM on FeatherS3-D.
    //
    // V2.3.24: snapshot now returns up to three segments — see applog.h. The
    // first segment is a scratch-buffer copy of the danger zone (where the
    // writer's next ring_append() lands), preventing the torn-line corruption
    // observed at file head from V2.3.16-V2.3.23 once the ring had wrapped.
    applog_stream_t stream;
    if (!applog_stream_begin(&stream)) {
        ESP_LOGE(TAG, "applog_stream_begin failed (applog not initialised?)");
        return false;
    }
    size_t body_len = stream.len_a + stream.len_b + stream.len_c;

    ftp_io_t      ctrl = { .sock = -1 };
    ftp_io_t      data = { .sock = -1 };
    ftp_tls_ctx_t tls;
    bool          tls_ready = false;
    bool          ok        = false;
    bool          write_ok  = false;
    char          last[512];
    char          cmd[288];
    int           code;

    if (s_cfg->ftp_tls) {
        if (!tls_ctx_init(&tls)) {
            ESP_LOGW(TAG, "tls_ctx_init failed");
            applog_stream_end();
            return false;
        }
        tls_ready = true;
    }

    // If WiFi power save is active, disable it for the transfer. With
    // WIFI_PS_MIN_MODEM, TCP ACKs from the NAS are delayed by up to one DTIM
    // interval, filling the lwIP send buffer and causing send() to block for
    // SO_SNDTIMEO (15 s). Skipped when the user has already disabled PS
    // globally, or when the per-FTP override has been unticked in /config.
    // Restored to the configured setting at done:.
    bool ps_override = !s_cfg->wifi_ps_disabled && s_cfg->ftp_ps_disabled;
    if (ps_override)
        esp_wifi_set_ps(WIFI_PS_NONE);

    int ctrl_sock = ftp_connect_host(host_buf, ftp_port_use, FTP_TIMEOUT_MS);
    if (ctrl_sock < 0) { goto done; }
    io_init_plain(&ctrl, ctrl_sock);

    if (ftp_read_response(&ctrl, FTP_TIMEOUT_MS, last, sizeof(last)) != 220) {
        ESP_LOGW(TAG, "no 220 greeting"); goto done;
    }

    if (s_cfg->ftp_tls) {
        if (!ftp_send_cmd(&ctrl, "AUTH TLS")) { ESP_LOGW(TAG, "AUTH TLS send"); goto done; }
        code = ftp_read_response(&ctrl, FTP_TIMEOUT_MS, last, sizeof(last));
        if (code != 234) { ESP_LOGW(TAG, "AUTH TLS reject: %d", code); goto done; }
        if (!io_upgrade_tls(&ctrl, &tls, /*is_data=*/false)) goto done;
    }

    const char *user = s_cfg->ftp_user[0] ? s_cfg->ftp_user : "anonymous";
    snprintf(cmd, sizeof(cmd), "USER %s", user);
    if (!ftp_send_cmd(&ctrl, cmd)) { ESP_LOGW(TAG, "USER send"); goto done; }
    code = ftp_read_response(&ctrl, FTP_TIMEOUT_MS, last, sizeof(last));
    if (code == 331) {
        const char *pw = s_cfg->ftp_password[0] ? s_cfg->ftp_password : "anonymous@";
        snprintf(cmd, sizeof(cmd), "PASS %s", pw);
        if (!ftp_send_cmd(&ctrl, cmd)) { ESP_LOGW(TAG, "PASS send"); goto done; }
        code = ftp_read_response(&ctrl, FTP_TIMEOUT_MS, last, sizeof(last));
        if (code != 230) { ESP_LOGW(TAG, "PASS reject: %d", code); goto done; }
    } else if (code != 230) {
        ESP_LOGW(TAG, "USER reject: %d", code); goto done;
    }

    if (s_cfg->ftp_tls) {
        // PBSZ 0 is required after AUTH TLS and before PROT. Value ignored for stream.
        if (!ftp_send_cmd(&ctrl, "PBSZ 0")) goto done;
        code = ftp_read_response(&ctrl, FTP_TIMEOUT_MS, last, sizeof(last));
        if (code != 200) { ESP_LOGW(TAG, "PBSZ reject: %d", code); goto done; }
        if (!ftp_send_cmd(&ctrl, "PROT P")) goto done;
        code = ftp_read_response(&ctrl, FTP_TIMEOUT_MS, last, sizeof(last));
        if (code != 200) { ESP_LOGW(TAG, "PROT P reject: %d", code); goto done; }
    }

    if (!ftp_send_cmd(&ctrl, "TYPE I")) goto done;
    if (ftp_read_response(&ctrl, FTP_TIMEOUT_MS, last, sizeof(last)) != 200) {
        ESP_LOGW(TAG, "TYPE I reject"); goto done;
    }

    if (!ftp_send_cmd(&ctrl, "PASV")) goto done;
    if (ftp_read_response(&ctrl, FTP_TIMEOUT_MS, last, sizeof(last)) != 227) {
        ESP_LOGW(TAG, "PASV reject"); goto done;
    }
    uint32_t data_ip = 0;
    int      data_port = 0;
    if (!parse_pasv(last, &data_ip, &data_port)) {
        ESP_LOGW(TAG, "PASV parse: %s", last); goto done;
    }

    int data_sock = ftp_connect_pasv(data_ip, data_port, FTP_TIMEOUT_MS);
    if (data_sock < 0) { ESP_LOGW(TAG, "data connect"); goto done; }
    io_init_plain(&data, data_sock);

    snprintf(cmd, sizeof(cmd), "STOR %s", path);
    if (!ftp_send_cmd(&ctrl, cmd)) goto done;
    code = ftp_read_response(&ctrl, FTP_TIMEOUT_MS, last, sizeof(last));
    if (code != 125 && code != 150) {
        ESP_LOGW(TAG, "STOR reject: %d", code); goto done;
    }

    if (s_cfg->ftp_tls) {
        // Wrap data in TLS AFTER STOR's 150 Ready response — the server now
        // expects a TLS handshake on the data socket, reusing the control
        // session (required by vsftpd's default require_ssl_reuse=YES).
        if (!io_upgrade_tls(&data, &tls, /*is_data=*/true)) goto done;

        // V2.3.15: data-channel handshake done; release the saved ctrl session
        // blob now to free the PSA key references it's still holding. The data
        // path has already extracted what it needs (mbedtls_ssl_set_session
        // copied the relevant state into the data SSL context). Holding the
        // blob until tls_ctx_free at the end of upload extends PSA-slot lifetime
        // by the ~30 s body-write duration for no benefit. Re-init so
        // tls_ctx_free's session_free is a safe no-op.
        if (tls.have_session) {
            mbedtls_ssl_session_free(&tls.ctrl_session);
            mbedtls_ssl_session_init(&tls.ctrl_session);
            tls.have_session = false;
        }
    }

    // V2.3.16: send segments directly from the ring (no body buffer alloc).
    // V2.3.24: up to three segments now in chronological order — seg_a is
    // the scratch-buffer copy of the danger zone (or the full pre-wrap
    // content), seg_b is the ring tail past the danger zone (wrapped only),
    // seg_c is the newer pre-wrap half (wrapped only). ftp_write_buf is
    // unchanged.
    write_ok = true;
    if (stream.len_a > 0) {
        write_ok = ftp_write_buf(&data, stream.seg_a, stream.len_a);
    }
    if (write_ok && stream.len_b > 0) {
        write_ok = ftp_write_buf(&data, stream.seg_b, stream.len_b);
    }
    if (write_ok && stream.len_c > 0) {
        write_ok = ftp_write_buf(&data, stream.seg_c, stream.len_c);
    }
    io_close(&data);

    if (!write_ok) {
        // Half-open hazard: skip the 226 read and close ctrl fast. Waiting
        // for 226 after a stalled transfer can pin lwip_recv() for hours.
        ESP_LOGW(TAG, "write stalled — skipping 226 confirm");
        goto done;
    }

    code = ftp_read_response(&ctrl, FTP_CONFIRM_MS, last, sizeof(last));
    if (code != 226) {
        ESP_LOGW(TAG, "STOR confirm: %d (%s)", code, last);
        goto done;
    }
    ok = true;
    ftp_send_cmd(&ctrl, "QUIT");
    ftp_read_response(&ctrl, FTP_QUIT_TIMEOUT_MS, last, sizeof(last));

done:
    // Restore WiFi PS if it was disabled for the transfer.
    if (ps_override)
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    io_close(&data);
    io_close(&ctrl);
    // V2.3.15: snapshot the PSA-OOM flag BEFORE tls_ctx_free zeros it out.
    // OR in s_upload_psa_oom so OOM events from io_recv1 / io_send_all
    // (post-handshake reads/writes — the V2.3.5–V2.3.9 NewSessionTicket path)
    // also feed the consecutive-OOM counter, not just handshake-time events.
    bool psa_oom_observed = (tls_ready && tls.psa_oom) || s_upload_psa_oom;
    if (tls_ready) tls_ctx_free(&tls);
    applog_stream_end();
    if (ok) ESP_LOGI(TAG, "FTP%s upload OK (%u bytes)",
                     s_cfg->ftp_tls ? "S" : "", (unsigned)body_len);
    else    ESP_LOGW(TAG, "FTP%s upload failed",
                     s_cfg->ftp_tls ? "S" : "");

    // Publish to stats. last_bytes only updated on success so a failed retry
    // doesn't blank the previous-good byte count visible on /. V2.4.1 (B1):
    // wrap the group write to pair with the spinlock'd read in
    // log_ftp_get_stats — int64_t s_last_at needs cross-task atomicity.
    int64_t now = (int64_t)time(NULL);
    portENTER_CRITICAL(&s_stats_mux);
    s_have_last = true;
    s_last_ok   = ok;
    s_last_at   = now;
    if (ok) s_last_bytes = (uint32_t)body_len;
    portEXIT_CRITICAL(&s_stats_mux);

    // V2.3.15: PSA recovery decision.
    //
    // Two trigger paths feed the same nuclear reset:
    //   (a) Slot OOM threshold: counted across consecutive uploads. Default
    //       threshold 5 (≈15 min sustained failure). Catches gradual leaks.
    //   (b) Write-stall preemptive: fires on the SAME upload that stalled,
    //       skipping the wait. Empirically the WANT_WRITE-then-abandon path
    //       leaks PSA crypto state in mbedTLS 4.x — without this preemptive
    //       trigger, the next 4 retries (3 min apart) all fail with -141 at
    //       the data-channel handshake, wasting 12 min before the (a) path
    //       finally fires.
    //
    // Both gated on tx_is_idle() — calling mbedtls_psa_crypto_free while
    // the HTTPS worker is mid-handshake on CPU1 would corrupt its state.
    // Sub-microsecond race window accepted (worst case: one HTTPS retry).
    bool        should_reset = false;
    const char *reset_reason = NULL;

    if (psa_oom_observed) {
        s_consecutive_psa_oom++;
        ESP_LOGE(TAG, "PSA OOM observed; consecutive count = %d/%d",
                 s_consecutive_psa_oom, FTP_PSA_OOM_RESET_THRESHOLD);
        if (s_consecutive_psa_oom >= FTP_PSA_OOM_RESET_THRESHOLD) {
            should_reset = true;
            reset_reason = "consecutive PSA OOM threshold reached";
        }
    } else if (ok) {
        // Healthy upload — clear the counter so a transient peak doesn't
        // accumulate over hours.
        s_consecutive_psa_oom = 0;
    }

    if (s_stall_observed && !should_reset) {
        should_reset = true;
        reset_reason = "preemptive (write stall leaks PSA state in mbedTLS 4.x)";
    }

    if (should_reset && tx_is_idle()) {
        ESP_LOGW(TAG, "PSA crypto subsystem reset: %s", reset_reason);
        mbedtls_psa_crypto_free();
        psa_status_t ps = psa_crypto_init();
        if (ps == PSA_SUCCESS) {
            ESP_LOGI(TAG, "psa_crypto_init: ok — slot pool reset to empty");
            s_consecutive_psa_oom = 0;
        } else {
            ESP_LOGE(TAG, "psa_crypto_init failed: %d (PSA in unknown state)",
                     (int)ps);
        }
    } else if (should_reset) {
        ESP_LOGW(TAG, "PSA reset deferred (worker busy): %s", reset_reason);
    }

    // V2.3.15: post-upload heap snapshot. Compare against the pre-upload
    // line to spot per-upload deltas; watch min_free over hours/days for
    // slow PSA-backed leaks.
    ESP_LOGI(TAG, "Post-upload heap: free=%u min_free=%u largest=%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    return ok;
}

void log_ftp_init(const char *chip_id, const config_t *cfg) {
    s_chip_id         = chip_id ? chip_id : "";
    s_cfg             = cfg;
    // First upload fires one full interval after boot — same cadence as
    // subsequent uploads. Previously hardcoded to 1 h regardless of config.
    uint32_t interval_min = (cfg && cfg->ftp_interval_min >= 1) ? cfg->ftp_interval_min : 60;
    s_next_upload_ms  = interval_min * 60000UL;
}

void log_ftp_loop(uint32_t now_ms) {
    // V2.4.19: the 24h PSA crypto refresh (V2.3.23) + 24h gratuitous
    // ARP safety-net (V2.4.19) that used to live here as the first
    // block of this function moved to `periodic.c`. Both run on the
    // main task too and behaviour is preserved exactly. main.c calls
    // periodic_loop() immediately before log_ftp_loop() each tick.
    //
    // The FTP-failure-driven PSA refresh below (post-upload error
    // recovery) stays here because it's intrinsically tied to FTP
    // upload state.

    if (!s_cfg || !s_cfg->ftp_enabled) return;

    bool is_scheduled = ((int32_t)(now_ms - s_next_upload_ms) >= 0);
    bool is_retry     = (!is_scheduled && s_retry_count > 0 &&
                         (int32_t)(now_ms - s_retry_ms) >= 0);

    if (!is_scheduled && !is_retry) return;

    // V2.4.13: paused by OTA teardown — drop the scheduled/retry attempt
    // silently. No state changes (next_upload_ms / retry_count untouched)
    // so on a failed OTA + manual reboot, the schedule resumes naturally.
    if (s_paused) {
        ESP_LOGD(TAG, "skip: paused (OTA teardown)");
        return;
    }

    // Don't run while the TX worker is busy — FTP + TLS POSTs competing for
    // the WiFi link and heap have been observed to overlap cleanly, but a
    // slower FTP could push sensor.community / Radmon into retries. Retry
    // next loop tick (1 s) when the worker goes idle.
    if (!tx_is_idle()) return;

    uint32_t interval_min = s_cfg->ftp_interval_min;
    if (interval_min < 1) interval_min = 60;
    uint32_t interval_ms = interval_min * 60000UL;

    if (is_scheduled) {
        // Advance the regular schedule from this attempt — interval counts from
        // first try, not from retries. Also clears any stale retry state from
        // the previous interval (whether it eventually succeeded or not).
        s_next_upload_ms = now_ms + interval_ms;
        s_retry_count    = 0;
        s_retry_ms       = 0;
    } else {
        ESP_LOGI(TAG, "FTP: retry %d/%d",
                 FTP_RETRY_COUNT - s_retry_count + 1, FTP_RETRY_COUNT);
    }

    // V2.4.14: tear down MQTT before the FTPS upload to free its TLS state
    // (~18-25 KB on Heltec V2). Without this, the FTPS handshake + MQTT
    // session compete for ~13 KB of headroom, with observed min_free
    // dropping to 1.1 KB during the upload peak (esp32-176432 2026-05-19).
    // Even when FTPS succeeds, MQTT keep-alives can fail with errno=11
    // (EAGAIN — kernel buffer exhaustion) mid-upload, manifesting as
    // "No PING_RESP" disconnects 4 min later.
    //
    // mqtt_stop() is a clean DISCONNECT (LWT does NOT fire — broker
    // keeps the retained "online" availability), and main.c's main-loop
    // poll re-inits MQTT within ~1 s of FTPS completion. HA sees no
    // availability flap; subscribers may miss state publishes for the
    // 5-30 s upload window which is acceptable for a 150 s TX cadence.
    //
    // Same pattern as V2.4.13's OTA teardown but gated on FTPS start
    // instead of POST /update. Heltec V2 lifts min_free during upload
    // from ~1 KB to ~40 KB; FeatherS3-D / QT Py the teardown is harmless
    // overhead (heap was never tight).
    bool mqtt_was_running = mqtt_is_initialized();
    if (mqtt_was_running) {
        ESP_LOGI(TAG, "FTPS prep: stopping MQTT to free TLS state");
        mqtt_stop();
    }

    bool ok = do_ftp_upload();

    if (ok) {
        s_retry_count = 0;
        s_retry_ms    = 0;
    } else if (is_scheduled) {
        // First attempt of this interval failed — arm the retry sequence.
        s_retry_count = FTP_RETRY_COUNT;
        s_retry_ms    = now_ms + FTP_RETRY_DELAY_MS;
        ESP_LOGW(TAG, "FTP: upload failed — %d retries scheduled (~%d min apart)",
                 FTP_RETRY_COUNT, FTP_RETRY_DELAY_MS / 60000);
    } else {
        // A retry failed.
        s_retry_count--;
        if (s_retry_count > 0) {
            s_retry_ms = now_ms + FTP_RETRY_DELAY_MS;
            ESP_LOGW(TAG, "FTP: retry failed — %d remaining", s_retry_count);
        } else {
            s_retry_ms = 0;
            ESP_LOGW(TAG, "FTP: all retries exhausted");
        }
    }
}
