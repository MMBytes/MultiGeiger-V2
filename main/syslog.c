// V2.4.15: UDP syslog client — RFC 5424 framing (RFC 3164 pre-V2.5.27).
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
#include "esp_system.h"        // esp_reset_reason / esp_get_free_heap_size
#include "esp_idf_version.h"   // esp_get_idf_version
#include "esp_chip_info.h"     // esp_chip_info — model / rev / cores
#include "esp_heap_caps.h"     // largest-free-block (fragmentation baseline)

#include "version.h"           // VERSION_STR
#include "sysinfo.h"           // reset_reason_str
#include "config.h"            // CFG_HOSTNAME_MAX — s_hostname mirrors cfg->wifi_hostname
#include "hal.h"               // BOARD_NAME
#include "coredump.h"          // coredump_have_dump
#include "display.h"           // display_backend_str — panel line in banner
#include "ntp.h"               // ntp_time_valid — the clock-sane gate
#include "esp_ota_ops.h"       // esp_ota_get_running_partition — boot slot in banner

static const char *TAG = "syslog";

static int                s_sock = -1;
static struct sockaddr_in s_addr;
// Sized CFG_HOSTNAME_MAX + 1 like the cfg field it mirrors (config_fields.def)
// — a bare [32] silently truncated 32-char hostnames to 31 (V2.6.24 fix).
static char               s_hostname[CFG_HOSTNAME_MAX + 1] = "geiger";
// Re-entrancy guard. Set while syslog_emit is mid-call so any ESP_LOG that
// somehow fires from within sendto's call chain (lwIP error path? unlikely
// but defensive) returns immediately instead of recursing.
static bool               s_in_emit = false;
// V2.5.29: cumulative UDP send accounting. Incremented in emit_packet (under
// applog's mutex — no atomics needed); read via syslog_get_stats() from the TX
// cycle. A non-zero drop count = device-side loss: sendto() is MSG_DONTWAIT
// with no queue behind it, so anything it rejects is gone.
//
// V2.6.28: the dominant observed cause on this fleet is a DOWN LINK, not buffer
// pressure. Once WiFi drops there is no route, sendto() fails immediately on
// every emit, and the counter steps by the number of lines logged during the
// outage — so drops arrive as one burst, then stay flat. Observed on
// esp32-5965048: 163 drops in a single 2m25s AP channel-change outage across a
// 9-day run, otherwise zero (the prior boot logged 160 the same way).
//
// Burst pressure is NOT hypothetical — it was the pre-pacing failure mode:
// V2.5.29 confirmed the boot config dump outrunning the WiFi/lwIP drain on the
// tight-heap heltec and fixed it by yielding after every line (see
// config_log_summary() in config.c). It has not recurred since. So read the
// counter accordingly: a step implies "check for a disconnect around then",
// not "the log is too chatty". Losses are not permanent — applog's ring keeps
// the lines for /log and the scheduled FTPS upload.
static uint32_t           s_tx_count   = 0;
static uint32_t           s_drop_count = 0;

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
        const struct hostent *he = gethostbyname(host);
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

    // V2.5.22: boot summary as the FIRST line the server sees. The real boot
    // banner (version / board / chip / reset reason) is logged before WiFi +
    // syslog come up, so it never reaches the server — leaving the firmware
    // version and reset reason invisible to server-side forensics (the gap that
    // once hid an OTA behind an unexplained count-rate jump). Now that the
    // socket is live, emit a one-line summary BEFORE "started" so it leads every
    // device's server-side log. (syslog_init runs once at startup, outside
    // applog's mutex, so ESP_LOG here is safe — see file header.)
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    const char *model = chip_model_str(chip.model);
    // V2.5.28: which OTA slot did we actually boot from? Pairs with the OTA
    // "boot set to <label>" success line — if the banner's Partition matches,
    // the OTA stuck; if it shows the other slot, the bootloader rolled back.
    const esp_partition_t *run_part = esp_ota_get_running_partition();
    ESP_LOGI("boot",
             "Firmware %s (IDF %s) - Reset reason: %s - Partition: %s - Board: %s - "
             "Chip: %s rev v%d.%d (%d cores) - Coredump: %s - "
             "Free heap: %lu B (largest %lu B)",
             VERSION_STR, esp_get_idf_version(),
             reset_reason_str(esp_reset_reason()),
             run_part ? run_part->label : "?",
             BOARD_NAME,
             model, chip.revision / 100, chip.revision % 100, chip.cores,
             coredump_have_dump() ? "PRESENT (/coredump.elf)" : "none",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    // V2.6.30: attached display panel as a second banner line. The probe
    // verdict is logged by display.c long before WiFi + syslog exist, so —
    // like the banner above — it never reaches the server on its own.
    // display_setup() has always completed by syslog_init() time (main.c
    // boots the display before WiFi), so the string is final here.
    ESP_LOGI("boot", "Display: %s", display_backend_str());

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

void syslog_get_stats(uint32_t *sent, uint32_t *dropped) {
    if (sent)    *sent    = s_tx_count;
    if (dropped) *dropped = s_drop_count;
}

// V2.4.16: build one RFC 5424 frame from an already-coalesced full line and
// send it as a single UDP packet. Internal helper; the public entry-point
// `syslog_emit()` below feeds full lines here after fragment accumulation.
//
// Static send buffer (not stack): applog_vprintf holds its mutex while
// calling us, so this is single-threaded by construction. Pre-V2.4.16
// this 600-byte buffer was on the stack — combined with config_get's
// ~4 KB of local html_esc[] arrays + the per-frame ESP_LOG machinery,
// it pushed the 8 KB httpd task over its stack limit, panicking on
// /config (and intermittently /log) with `LoadStoreError` in
// `vPortYieldFromInt`. Moving to BSS eliminates the stack contribution.
static void emit_packet(const char *line, size_t len) {
    // V2.5.20 (review R10): 600 → 1200 B so a full LOG_LINE_MAX (1024 B)
    // applog line + the RFC 5424 header fits in one frame. Still well under
    // the 1500 B LAN MTU (no IP fragmentation). BSS, single-threaded by
    // construction (applog mutex) — same justification as before.
    static char s_emit_buf[1200];

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

    // V2.5.27: RFC 5424 framing so a pre-NTP line can carry a NILVALUE ("-")
    // timestamp. RFC 3164 (the pre-V2.5.27 framing) has no nil-timestamp form,
    // so the pre-sync path emitted a well-formed but bogus "Jan  1 00:00:00" —
    // a collector trusting the reported time filed it under the wrong day
    // (e.g. 2026-01-01T00:00:00), hiding every pre-sync boot line (banner +
    // config dump) from date-scoped queries. With RFC 5424, "-" tells the
    // collector "no reliable time" and it falls back to receive time on its
    // own — correct on a stock rsyslog, no server-side template change needed.
    //
    // V2.5.28: once the clock is real (ntp_time_valid()), emit LOCAL RFC 3339
    // with the numeric UTC offset (e.g. 2026-06-13T20:45:50+10:00) — the same
    // TZ-string-driven local time the status page / NTP line show, so on a
    // collector you don't control the header matches the in-message device
    // time. ntp_localtime_str() owns that one-strftime format. Pre-sync stays
    // NILVALUE "-" so the collector falls back to its own receive time.
    const char *ts = ntp_time_valid() ? ntp_localtime_str() : "-";

    // Strip trailing newlines — rsyslog adds its own.
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
        len--;
    }
    if (len == 0) return;

    // RFC 5424: <PRI>1 SP TIMESTAMP SP HOSTNAME SP APP-NAME SP PROCID SP
    //           MSGID SP STRUCTURED-DATA SP MSG. APP-NAME "geiger"; PROCID,
    //           MSGID and STRUCTURED-DATA are all NILVALUE ("-").
    int n = snprintf(s_emit_buf, sizeof(s_emit_buf),
                     "<%d>1 %s %s geiger - - - %.*s",
                     priority, ts, s_hostname, (int)len, line);
    if (n > 0) {
        // V2.5.20 (review R10): clamp-and-send on truncation. Pre-V2.5.20 a
        // frame longer than the buffer was silently DROPPED (the `n < sizeof`
        // guard) — a truncated log line on the server beats a missing one.
        if (n >= (int)sizeof(s_emit_buf)) n = (int)sizeof(s_emit_buf) - 1;
        // MSG_DONTWAIT: non-blocking. If lwIP's TX queue is full we'd
        // rather drop the packet than block applog (which holds its
        // mutex while calling us). UDP send is normally sub-ms.
        //
        // V2.5.29: count the result (was discarded) — previously INVISIBLE.
        // V2.6.28: a failure here is usually a DOWN LINK (no route, so every
        // emit during the outage fails), not lwIP pbuf-pool exhaustion — see
        // the s_drop_count declaration for the field evidence and for the
        // V2.5.29 boot-dump burst-pressure history.
        // Count only; the report is logged from the TX cycle via
        // syslog_get_stats() — NEVER ESP_LOG here (it would re-enter applog's
        // non-recursive mutex from the emit path → deadlock).
        if (sendto(s_sock, s_emit_buf, (size_t)n, MSG_DONTWAIT,
                   (struct sockaddr *)&s_addr, sizeof(s_addr)) < 0) {
            s_drop_count++;
        } else {
            s_tx_count++;
        }
    }
}

void syslog_emit(const char *line, size_t len) {
    // V2.5.29: applog_vprintf() now reassembles the IDF-v6 vprintf fragments
    // into one COMPLETE logical line before calling us (so the /log ring gets
    // the same reassembly and the two surfaces agree), so we just frame + send.
    // Pre-V2.5.29 we re-joined fragments here in a static accumulator — which
    // left the ring spliced. emit_packet() strips the trailing newline.
    //
    // The s_in_emit guard stays: it stops an ESP_LOG fired from within sendto's
    // call chain (lwIP error path) from re-entering and recursing.
    if (s_sock < 0 || s_in_emit) return;
    if (!line || len == 0) return;

    s_in_emit = true;
    emit_packet(line, len);
    s_in_emit = false;
}
