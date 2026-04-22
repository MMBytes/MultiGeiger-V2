#include "ntp.h"

#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "esp_log.h"
#include "esp_sntp.h"

static const char *TAG = "ntp";

// 2025-01-01 00:00 UTC — threshold for "time is valid".
#define EPOCH_2025 1735689600L

static volatile bool     sync_pending = false;
static volatile time_t   sync_tv_sec  = 0;

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
    return now > EPOCH_2025;
}

void ntp_poll(void) {
    if (!sync_pending) return;
    sync_pending = false;
    char buf[32];
    time_t t = sync_tv_sec;
    struct tm *ti = localtime(&t);
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S %Z", ti);
    ESP_LOGI(TAG, "sync OK: %s", buf);
}

const char *ntp_localtime_str(void) {
    static char buf[32];
    time_t t;
    time(&t);
    struct tm *ti = localtime(&t);
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", ti);
    return buf;
}
