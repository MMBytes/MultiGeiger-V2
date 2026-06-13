#include "ntp.h"

#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "esp_log.h"
#include "esp_sntp.h"

static const char *TAG = "ntp";

// 2026-01-01 00:00 UTC — threshold for "time is valid". Bumped from
// 2025-01-01 in V2.5.27: the firmware only ever runs from 2026 onward, so a
// clock reading any 2025 date is necessarily a stale/bad sync, not a real
// "now" — rejecting it (→ wait for a proper sync) is the safer sanity floor.
#define EPOCH_2026 1767225600L

static volatile bool     sync_pending = false;
static volatile time_t   sync_tv_sec  = 0;

// Signature is dictated by IDF's `sntp_set_time_sync_notification_cb_t`,
// which uses non-const `struct timeval *`. We don't mutate *tv.
// cppcheck-suppress constParameterCallback
static void sync_cb(struct timeval *tv) {
    sync_tv_sec = tv->tv_sec;
    sync_pending = true;
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
    time_t now;
    time(&now);
    return now > EPOCH_2026;
}

void ntp_poll(void) {
    if (!sync_pending) return;
    sync_pending = false;
    char buf[32];
    time_t t = sync_tv_sec;
    // V2.5.20 (review): localtime_r — the non-reentrant localtime() returns a
    // shared static struct tm; every other call site in the tree already uses
    // the _r form, so make these two stragglers consistent.
    struct tm tm_local;
    localtime_r(&t, &tm_local);
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S %Z", &tm_local);
    ESP_LOGI(TAG, "sync OK: %s", buf);
}

const char *ntp_localtime_str(void) {
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
