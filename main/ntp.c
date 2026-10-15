#include "ntp.h"

#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "lwip/ip_addr.h" // ipaddr_ntoa_r, IPADDR_STRLEN_MAX (V2.8.9 sync source)
#include "util.h"        // tz_offset_add_colon (V2.8.5)

static const char *TAG = "ntp";

// 2026-01-01 00:00 UTC — threshold for "time is valid". Bumped from
// 2025-01-01 in V2.5.27: the firmware only ever runs from 2026 onward, so a
// clock reading any 2025 date is necessarily a stale/bad sync, not a real
// "now" — rejecting it (→ wait for a proper sync) is the safer sanity floor.
#define EPOCH_2026 1767225600L

static volatile bool     sync_pending    = false;
// Sync timestamp stored as uint32_t offset from EPOCH_2026 (seconds).
// uint32_t = single atomic store/load on 32-bit Xtensa — written by the SNTP
// lwIP timer callback, read by ntp_poll() on the main task; no mutex needed.
static volatile uint32_t sync_tv_sec_off = 0;

// Boot epoch stored as a uint32_t offset from EPOCH_2026 (seconds).
// uint32_t = single atomic store/load on 32-bit Xtensa — no mutex needed.
// 0 means "not yet captured". Refreshed on every SNTP sync (hourly) so a
// bad first-sync timestamp self-corrects and crystal drift stays < 1 s/sync.
static volatile uint32_t s_boot_epoch_off = 0;

// Number of SNTP server slots ntp_setup() actually filled (0..3), clamped to
// CONFIG_LWIP_SNTP_MAX_SERVERS because lwIP silently drops any slot index at
// or above it (a no-op while every board sets it to 3; it only matters if the
// Kconfig is ever lowered). Written once per ntp_setup() and read by
// ntp_poll(), both on the main task.
static uint8_t s_slots = 0;

// Signature is dictated by IDF's `sntp_set_time_sync_notification_cb_t`,
// which uses non-const `struct timeval *`. We don't mutate *tv.
// cppcheck-suppress constParameterCallback
static void sync_cb(struct timeval *tv) {
    if (tv->tv_sec > EPOCH_2026)
        sync_tv_sec_off = (uint32_t)(tv->tv_sec - EPOCH_2026);
    sync_pending = true;
    // Update boot epoch on every sync — eliminates crystal drift and allows
    // recovery from a bad first-sync timestamp. EPOCH_2026 rejects stale
    // server times; the inner guard stops a negative offset when uptime_s
    // exceeds (tv_sec - EPOCH_2026) (e.g. very long crystal-only run).
    if (tv->tv_sec > EPOCH_2026) {
        uint32_t uptime_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
        time_t   epoch    = tv->tv_sec - (time_t)uptime_s;
        if (epoch > EPOCH_2026) {
            s_boot_epoch_off = (uint32_t)(epoch - EPOCH_2026);
        }
    }
}

void ntp_set_timezone(const char *tz_posix) {
    if (tz_posix && tz_posix[0]) {
        setenv("TZ", tz_posix, 1);
        tzset();
    }
}

void ntp_setup(const char *s1, const char *s2, const char *s3, const char *tz_posix) {
    ntp_set_timezone(tz_posix);

    if (esp_sntp_enabled()) {
        esp_sntp_stop();
    }
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);

    // Pack non-empty servers into consecutive slots starting at 0.
    uint8_t slot = 0;
    const char *servers[3] = { s1, s2, s3 };
    for (int i = 0; i < 3; i++) {
        if (servers[i] && servers[i][0]) {
            esp_sntp_setservername(slot++, servers[i]);
        }
    }
    s_slots = (slot > CONFIG_LWIP_SNTP_MAX_SERVERS)
              ? (uint8_t)CONFIG_LWIP_SNTP_MAX_SERVERS : slot;
    sntp_set_time_sync_notification_cb(sync_cb);
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP started (%u server(s): %s | %s | %s; TZ=%s)",
             (unsigned)slot,
             (s1 && s1[0]) ? s1 : "-",
             (s2 && s2[0]) ? s2 : "-",
             (s3 && s3[0]) ? s3 : "-",
             (tz_posix && tz_posix[0]) ? tz_posix : "(unchanged)");
}

bool ntp_time_valid(void) {
    // NOTE: also called from syslog.c emit_packet(), which runs inside
    // applog_vprintf under a NON-recursive mutex. Must NOT ESP_LOG here (nor
    // add any callee that does) — a log from this path re-enters applog and
    // self-deadlocks before the s_in_emit guard. Keep it pure.
    time_t now;
    time(&now);
    return now > EPOCH_2026;
}

// --- Sync source -------------------------------------------------------------

// Describe which configured server answered the sync, e.g.
// "via pool.example (203.0.113.7) [reach FF/00/00]".
//
// lwIP keeps an RFC 5905 reachability shift register per server slot: shifted
// left when a request is SENT to that slot, bit 0 set when a valid reply
// ARRIVES. A receive timeout or Kiss-of-Death moves lwIP to the next slot
// after the send already shifted a 0 into the failing one. A slot whose name
// fails to resolve is abandoned WITHOUT a send, so it keeps a stale bit 0 —
// but sntp_request() zeroed its address before the lookup and nothing
// restores it, so "bit 0 set AND address resolved" leaves exactly the slot
// that answered. lwIP exposes no "current server" getter
// (sntp_current_server is file-static in sntp.c), so this is the only public
// way to name the source. lwIP itself credits any accepted reply to the
// current slot (SNTP_CHECK_RESPONSE = 0), so a late reply from a slot it just
// left is attributed to the new one (or prints "?" if the new slot's lookup
// is still in flight) — that is lwIP's view, reported as is.
//
// Must run AFTER sntp_recv() has returned: it calls the sync callback (via
// sntp_process) BEFORE it sets bit 0, so asking from sync_cb would see the
// answering slot as unanswered. ntp_poll() on the main task is always later.
// If zero or several slots show bit 0 (e.g. the main task raced the tcpip
// task on a dual-core chip), print "?" rather than guess; the register
// bytes are always printed so the reader can judge.
//
// Reads lwIP state without the tcpip lock (CONFIG_LWIP_TCPIP_CORE_LOCKING is
// off): the u8 registers are single-byte loads, and the resolved address is
// only rewritten when the next request goes out (an hour later in POLL mode).
static void describe_source(char *out, size_t outsz) {
    char   reach[16];          // "FF/FF/FF" worst case for 3 slots
    size_t rn  = 0;
    int    src = -1;
    int    hits = 0;
    reach[0] = '\0';
    for (uint8_t i = 0; i < s_slots; i++) {
        uint8_t r = esp_sntp_getreachability(i);
        int w = snprintf(reach + rn, sizeof(reach) - rn, "%s%02X", i ? "/" : "", r);
        if (w > 0) rn += (size_t)w;
        if (rn >= sizeof(reach)) rn = sizeof(reach) - 1;   // clamp on truncation
        // Unresolved address = stale bit 0 left by a DNS-failure failover.
        if ((r & 0x01) && !ip_addr_isany(esp_sntp_getserver(i))) {
            if (src < 0) src = i;
            hits++;
        }
    }

    if (hits != 1) {
        snprintf(out, outsz, "via ? [reach %s]", reach);
        return;
    }

    const char *name = esp_sntp_getservername((uint8_t)src);
    char ip[IPADDR_STRLEN_MAX];
    if (!ipaddr_ntoa_r(esp_sntp_getserver((uint8_t)src), ip, sizeof(ip))) ip[0] = '\0';
    // A server configured as an IP literal would print the same text twice.
    if (name && name[0] && ip[0] && strcmp(name, ip) != 0) {
        snprintf(out, outsz, "via %s (%s) [reach %s]", name, ip, reach);
    } else {
        snprintf(out, outsz, "via %s [reach %s]",
                 (name && name[0]) ? name : (ip[0] ? ip : "?"), reach);
    }
}

void ntp_poll(void) {
    if (!sync_pending) return;
    sync_pending = false;
    // Sized for a 63-char server name (CFG_NTP_HOST_MAX) + IPv6 text
    // (IPADDR_STRLEN_MAX = 46; every board enables IPv6) + the reach field:
    // 133 bytes worst case. snprintf truncates rather than overflows.
    char src[160];
    describe_source(src, sizeof(src));
    char buf[32];
    uint32_t off = sync_tv_sec_off;
    time_t t = off ? (EPOCH_2026 + (time_t)off) : (time_t)time(NULL);
    // V2.5.20 (review): localtime_r — the non-reentrant localtime() returns a
    // shared static struct tm; every other call site in the tree already uses
    // the _r form, so make these two stragglers consistent.
    struct tm tm_local;
    localtime_r(&t, &tm_local);
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S %Z", &tm_local);
    ESP_LOGI(TAG, "sync OK: %s %s", buf, src);
}

const char *ntp_localtime_str(char *buf, size_t bufsz) {
    // NOTE: called from syslog.c emit_packet() on every emitted line, under
    // applog's NON-recursive mutex. Must NOT ESP_LOG here (nor add a callee
    // that does) — it would re-enter applog and self-deadlock. time() /
    // localtime_r() / strftime() are all log-free; keep it that way.
    if (!buf || bufsz == 0) return "";
    time_t t;
    time(&t);
    struct tm tm_local;
    localtime_r(&t, &tm_local);
    // Local RFC 3339 with the numeric UTC offset that the TZ string + tzset()
    // resolved (DST-aware: %z is +1000 in AEST, +1100 in AEDT). strftime emits
    // the offset without the colon RFC 3339 requires ("...+1000"), so splice
    // it in: "...+1000" -> "...+10:00" (util.h, host-tested). strftime
    // returns 0 and leaves the contents unspecified when the result does not
    // fit, so terminate explicitly in that case.
    size_t n = strftime(buf, bufsz, "%Y-%m-%dT%H:%M:%S%z", &tm_local);
    if (n == 0) { buf[0] = '\0'; return buf; }
    tz_offset_add_colon(buf, n, bufsz);
    return buf;
}

time_t ntp_boot_epoch(void) {
    uint32_t off = s_boot_epoch_off;
    return off ? (EPOCH_2026 + (time_t)off) : 0;
}

unsigned long ntp_uptime_s(void) {
    uint32_t off = s_boot_epoch_off;
    if (!off) return (unsigned long)(esp_timer_get_time() / 1000000LL);
    time_t uptime = (time_t)time(NULL) - (EPOCH_2026 + (time_t)off);
    return uptime > 0 ? (unsigned long)uptime : 0UL;
}
