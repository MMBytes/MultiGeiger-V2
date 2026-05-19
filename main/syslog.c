// V2.4.15: UDP syslog client — RFC 3164 framing.
//
// Hooked into applog_vprintf so every ESP_LOG line that lands in the ring
// also gets sent out via UDP. See syslog.h for design rationale + heap cost.
//
// Why we DON'T call ESP_LOG anywhere in this file's hot path (syslog_emit):
// applog_vprintf holds its mutex while calling us, and ESP_LOG would
// re-enter that path, deadlocking (or at minimum recursing through
// vprintf). syslog_init can ESP_LOG safely since it runs once at startup,
// outside applog's mutex.

#include "syslog.h"

#include <string.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "esp_log.h"

static const char *TAG = "syslog";

static int                s_sock = -1;
static struct sockaddr_in s_addr;
static char               s_hostname[32] = "geiger";
// Re-entrancy guard. Set while syslog_emit is mid-call so any ESP_LOG that
// somehow fires from within sendto's call chain (lwIP error path? unlikely
// but defensive) returns immediately instead of recursing.
static bool               s_in_emit = false;

void syslog_init(const char *host, uint16_t port, const char *hostname) {
    if (!host || !host[0] || port == 0) {
        ESP_LOGI(TAG, "disabled (host=%s port=%u)",
                 host ? host : "<null>", (unsigned)port);
        return;
    }
    if (s_sock >= 0) {
        ESP_LOGW(TAG, "init called twice — ignoring");
        return;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
        return;
    }

    memset(&s_addr, 0, sizeof(s_addr));
    s_addr.sin_family = AF_INET;
    s_addr.sin_port   = htons(port);

    // Accept either IPv4 dotted-quad or hostname. inet_aton succeeds on the
    // numeric path; gethostbyname is the DNS fallback. DNS lookup is one-
    // shot at init — if the server IP changes the user must reboot. This
    // matches the FTPS / Madavi behaviour for symmetry.
    if (inet_aton(host, &s_addr.sin_addr) == 0) {
        struct hostent *he = gethostbyname(host);
        if (!he || he->h_length <= 0 || !he->h_addr) {
            ESP_LOGE(TAG, "resolve failed: %s", host);
            close(sock);
            return;
        }
        memcpy(&s_addr.sin_addr, he->h_addr, (size_t)he->h_length);
    }

    if (hostname && hostname[0]) {
        size_t n = strnlen(hostname, sizeof(s_hostname) - 1);
        memcpy(s_hostname, hostname, n);
        s_hostname[n] = 0;
    }

    // Atomic publish — set s_sock LAST so syslog_emit's NULL-check is a
    // safe barrier. Without this, a concurrent vprintf could observe a
    // valid s_sock but stale s_addr.
    s_sock = sock;

    ESP_LOGI(TAG, "started — host=%s port=%u hostname=%s",
             host, (unsigned)port, s_hostname);
}

void syslog_stop(void) {
    if (s_sock < 0) return;
    int sock = s_sock;
    s_sock = -1;   // hide it from concurrent vprintf BEFORE close
    close(sock);
    ESP_LOGI(TAG, "stopped");
}

bool syslog_is_initialized(void) {
    return s_sock >= 0;
}

void syslog_emit(const char *line, size_t len) {
    if (s_sock < 0 || s_in_emit) return;
    if (!line || len == 0) return;

    s_in_emit = true;

    // Severity from the ESP_LOG level prefix. ESP_LOG output looks like:
    //   "I (HH:MM:SS.mmm) tag: text\n"
    // so the first char encodes the level. Default to info on anything else.
    int severity;
    switch (line[0]) {
        case 'E': severity = 3; break;   // error
        case 'W': severity = 4; break;   // warning
        case 'D': severity = 7; break;   // debug
        case 'V': severity = 7; break;   // verbose -> debug
        default:  severity = 6; break;   // info
    }
    int priority = 16 * 8 + severity;    // facility = local0

    // RFC 3164 timestamp "Mmm DD HH:MM:SS". We use device wall-clock when
    // it's plausibly real (post-NTP-or-RTC-carryover); rsyslog re-stamps
    // anyway based on receive time, but a real timestamp is friendlier
    // when scrolling old log files. Pre-NTP, fall back to a benign fixed
    // string — rsyslog still parses correctly and uses receive time.
    char ts[16];
    time_t now = time(NULL);
    if (now > 1735689600L) {            // 2025-01-01 — clock is real
        struct tm tm;
        localtime_r(&now, &tm);
        if (strftime(ts, sizeof(ts), "%b %e %H:%M:%S", &tm) == 0) {
            // strftime overflow shouldn't happen with our format, but
            // defensively fall back if it does.
            memcpy(ts, "Jan  1 00:00:00", 16);
        }
    } else {
        memcpy(ts, "Jan  1 00:00:00", 16);
    }

    // Strip trailing newlines — rsyslog adds its own.
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
        len--;
    }
    if (len == 0) { s_in_emit = false; return; }

    // RFC 3164 format: "<pri>TIMESTAMP HOSTNAME TAG: MSG". 600 bytes covers
    // the longest realistic ESP_LOG line (~512 chars) plus the ~40-byte
    // framing overhead.
    char buf[600];
    int n = snprintf(buf, sizeof(buf), "<%d>%s %s geiger: %.*s",
                     priority, ts, s_hostname, (int)len, line);
    if (n > 0 && n < (int)sizeof(buf)) {
        // MSG_DONTWAIT: non-blocking. If lwIP's TX queue is full we'd
        // rather drop the packet than block applog (which holds its mutex
        // while calling us). UDP send is normally sub-millisecond.
        (void)sendto(s_sock, buf, (size_t)n, MSG_DONTWAIT,
                     (struct sockaddr *)&s_addr, sizeof(s_addr));
    }

    s_in_emit = false;
}
