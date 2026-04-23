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

#include "mbedtls/ssl.h"
#include "mbedtls/error.h"
#include "mbedtls/net_sockets.h"
// ESP-IDF 6.0 ships mbedtls 4.x which removes entropy/ctr_drbg setup from user
// code — PSA crypto is auto-initialised at system startup and the RNG is pulled
// from PSA internally; mbedtls_ssl_conf_rng() has been removed. See
// mbedtls/docs/4.0-migration-guide.md.

#include "applog.h"
#include "ntp.h"
#include "transmission.h"

static const char *TAG = "ftp";

#define FTP_PORT              21
#define FTP_TIMEOUT_MS        10000      // 10 s per control response
#define FTP_CONFIRM_MS        15000      // 15 s for 226 — router may flush to USB
#define FTP_WRITE_CHUNK       1024
#define FTP_WRITE_STALL_MS    15000
#define FTP_QUIT_TIMEOUT_MS   2000

static const config_t *s_cfg           = NULL;
static const char     *s_chip_id       = "";
static uint32_t        s_next_upload_ms = 0;   // set in log_ftp_init from cfg

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
} ftp_tls_ctx_t;

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
        mbedtls_ssl_close_notify(&io->ssl);
        mbedtls_ssl_free(&io->ssl);
        io->tls = false;
    }
    if (io->sock >= 0) {
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

    int r = mbedtls_ssl_config_defaults(&t->conf,
                                        MBEDTLS_SSL_IS_CLIENT,
                                        MBEDTLS_SSL_TRANSPORT_STREAM,
                                        MBEDTLS_SSL_PRESET_DEFAULT);
    if (r != 0) { ESP_LOGW(TAG, "ssl conf defaults: -0x%04x", -r); return false; }

    // No cert verification: most LAN FTP servers use self-signed certs, and
    // the user opted into this by ticking "certificate NOT verified".
    mbedtls_ssl_conf_authmode(&t->conf, MBEDTLS_SSL_VERIFY_NONE);
    return true;
}

static void tls_ctx_free(ftp_tls_ctx_t *t) {
    mbedtls_ssl_session_free(&t->ctrl_session);
    mbedtls_ssl_config_free(&t->conf);
}

// Wrap an already-connected socket in TLS. For the data connection, reuse
// the saved control session — vsftpd's default require_ssl_reuse=YES rejects
// data handshakes that don't resume the control session.
static bool io_upgrade_tls(ftp_io_t *io, ftp_tls_ctx_t *t, bool is_data) {
    mbedtls_ssl_init(&io->ssl);
    int r = mbedtls_ssl_setup(&io->ssl, &t->conf);
    if (r != 0) { ESP_LOGW(TAG, "ssl_setup: -0x%04x", -r); mbedtls_ssl_free(&io->ssl); return false; }

    io->net.fd = io->sock;
    mbedtls_ssl_set_bio(&io->ssl, &io->net, io_tls_send, io_tls_recv, NULL);

    if (is_data && t->have_session) {
        // Non-fatal on failure — server may accept a fresh session.
        int sr = mbedtls_ssl_set_session(&io->ssl, &t->ctrl_session);
        if (sr != 0) ESP_LOGW(TAG, "set_session: -0x%04x (continuing)", -sr);
    }

    while ((r = mbedtls_ssl_handshake(&io->ssl)) != 0) {
        if (r != MBEDTLS_ERR_SSL_WANT_READ && r != MBEDTLS_ERR_SSL_WANT_WRITE) {
            char eb[80]; mbedtls_strerror(r, eb, sizeof(eb));
            ESP_LOGW(TAG, "TLS handshake: -0x%04x %s", -r, eb);
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

    io->tls = true;
    return true;
}

// Blocking 1-byte read. Returns 1 on success, 0 on timeout (SO_RCVTIMEO fired),
// -1 on error/close.
static int io_recv1(ftp_io_t *io, char *out) {
    if (io->tls) {
        for (;;) {
            int r = mbedtls_ssl_read(&io->ssl, (unsigned char *)out, 1);
            if (r == 1) return 1;
            if (r == MBEDTLS_ERR_SSL_WANT_READ || r == MBEDTLS_ERR_SSL_WANT_WRITE) continue;
            if (r == MBEDTLS_ERR_SSL_TIMEOUT) return 0;
            return -1;
        }
    }
    int r = recv(io->sock, out, 1, 0);
    if (r == 1) return 1;
    if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
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
        } else {
            n = send(io->sock, p + sent, len - sent, 0);
        }
        if (n <= 0) {
            ESP_LOGW(TAG, "io_send_all: err at %u/%u (n=%d errno=%d)",
                     (unsigned)sent, (unsigned)len, n, errno);
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
                        if (out_last && out_sz) {
                            strncpy(out_last, line, out_sz - 1);
                            out_last[out_sz - 1] = 0;
                        }
                        return code;
                    }
                } else if (strncmp(line, expected, 3) == 0 && ll >= 4 && line[3] == ' ') {
                    if (out_last && out_sz) {
                        strncpy(out_last, line, out_sz - 1);
                        out_last[out_sz - 1] = 0;
                    }
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

    // Build remote filename: geiger_<chip>_YYYY-MM-DDTHHMMSS.log.
    // NOTE: the "FTP: uploading" log line below must come BEFORE applog_snapshot()
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

    ESP_LOGI(TAG, "FTP%s: uploading %s to %s:%d as '%s'",
             s_cfg->ftp_tls ? "S" : "",
             path, s_cfg->ftp_host, FTP_PORT,
             s_cfg->ftp_user[0] ? s_cfg->ftp_user : "anonymous");

    size_t body_len = 0;
    char *body = applog_snapshot(&body_len);
    if (!body) {
        ESP_LOGE(TAG, "applog_snapshot oom");
        return false;
    }

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
            free(body);
            return false;
        }
        tls_ready = true;
    }

    int ctrl_sock = ftp_connect_host(s_cfg->ftp_host, FTP_PORT, FTP_TIMEOUT_MS);
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
    }

    write_ok = ftp_write_buf(&data, body, body_len);
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
    io_close(&data);
    io_close(&ctrl);
    if (tls_ready) tls_ctx_free(&tls);
    free(body);
    if (ok) ESP_LOGI(TAG, "FTP%s upload OK (%u bytes)",
                     s_cfg->ftp_tls ? "S" : "", (unsigned)body_len);
    else    ESP_LOGW(TAG, "FTP%s upload failed",
                     s_cfg->ftp_tls ? "S" : "");
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
    if (!s_cfg || !s_cfg->ftp_enabled) return;
    if ((int32_t)(now_ms - s_next_upload_ms) < 0) return;

    // Don't run while the TX worker is busy — FTP + TLS POSTs competing for
    // the WiFi link and heap have been observed to overlap cleanly, but a
    // slower FTP could push sensor.community / Radmon into retries. Retry
    // next loop tick (1 s) when the worker goes idle.
    if (!tx_is_idle()) return;

    uint32_t interval_min = s_cfg->ftp_interval_min;
    if (interval_min < 1) interval_min = 60;     // guard against zero
    uint32_t interval_ms = interval_min * 60000UL;

    // Regardless of success/failure, schedule next from now — a failing server
    // shouldn't cause us to hammer it on every loop tick.
    (void)do_ftp_upload();
    s_next_upload_ms = now_ms + interval_ms;
}
