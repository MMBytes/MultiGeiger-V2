#include "ntp.h"

#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_timer.h"

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
        int64_t uptime_s = esp_timer_get_time() / 1000000LL;
        time_t  epoch    = tv->tv_sec - (time_t)uptime_s;
        if (epoch > EPOCH_2026) {
            s_boot_epoch_off = (uint32_t)(epoch - EPOCH_2026);
        }
    }
}

void ntp_setup(const char *s1, const char *s2, const char *s3, const char *tz_posix) {
    if (tz_posix && tz_posix[0]) {
        setenv("TZ", tz_posix, 1);
        tzset();
    }

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

void ntp_poll(void) {
    if (!sync_pending) return;
    sync_pending = false;
    char buf[32];
    uint32_t off = sync_tv_sec_off;
    time_t t = off ? (EPOCH_2026 + (time_t)off) : (time_t)time(NULL);
    // V2.5.20 (review): localtime_r — the non-reentrant localtime() returns a
    // shared static struct tm; every other call site in the tree already uses
    // the _r form, so make these two stragglers consistent.
    struct tm tm_local;
    localtime_r(&t, &tm_local);
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S %Z", &tm_local);
    ESP_LOGI(TAG, "sync OK: %s", buf);
}

const char *ntp_localtime_str(void) {
    // NOTE: called from syslog.c emit_packet() on every emitted line, under
    // applog's NON-recursive mutex. Must NOT ESP_LOG here (nor add a callee
    // that does) — it would re-enter applog and self-deadlock. time() /
    // localtime_r() / strftime() are all log-free; keep it that way.
    static char buf[40];
    time_t t;
    time(&t);
    struct tm tm_local;
    localtime_r(&t, &tm_local);
    // Local RFC 3339 with the numeric UTC offset that the TZ string + tzset()
    // resolved (DST-aware: %z is +1000 in AEST, +1100 in AEDT). strftime emits
    // the offset without the colon RFC 3339 requires ("...+1000"), so splice
    // it in: "...+1000" -> "...+10:00".
    size_t n = strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", &tm_local);
    if (n >= 5 && (buf[n - 5] == '+' || buf[n - 5] == '-')) {
        buf[n + 1] = '\0';
        buf[n]     = buf[n - 1];   // shift offset minutes right by one
        buf[n - 1] = buf[n - 2];
        buf[n - 2] = ':';          // colon between offset hours and minutes
    }
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
