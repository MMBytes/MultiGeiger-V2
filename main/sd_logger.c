#include "sd_logger.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "hal.h"
#include "gnss.h"
#include "neopixel.h"
#include "sd_card.h"
#include "telemetry.h"
#include "util.h"

// Status-snapshot state: read unconditionally by sd_logger_get_status() (the
// /status page renders the same fields on every board), so these stay
// outside the HAL_HAS_SD_CARD gate below even though only the gated code
// ever writes them on a non-SD board (where they simply stay at their
// zero/empty init values forever).
static FILE       *s_f;
static char        s_filename[64];
static uint32_t    s_rows;
static uint32_t    s_fail_streak;
static int         s_last_err;

#if HAL_HAS_SD_CARD
static const char *s_chip_id = "";   // written by sd_logger_init(), read by create_file()
#endif

/** @brief One-time init. Stores the chip-id string used in filenames
 *  (static storage — main.c's g_chip_id). Called unconditionally from
 *  main.c regardless of board, so this function — unlike everything below —
 *  must exist on every board; the chip-id is simply never read on boards
 *  without a card. */
void sd_logger_init(const char *chip_id) {
#if HAL_HAS_SD_CARD
    s_chip_id = (chip_id != NULL) ? chip_id : "";
#else
    (void)chip_id;   // no SD card on this board: nothing to stamp filenames with
#endif
}

// Everything below is a pure implementation detail of sd_logger_cycle()'s
// HAL_HAS_SD_CARD-gated body (§ below) — gating it here too, not just that
// body, avoids -Wunused-function/-Wunused-variable on every non-SD board
// (confirmed via the heltec_v2 build, which has HAL_HAS_SD_CARD=0).
#if HAL_HAS_SD_CARD

static const char *TAG = "sd_logger";

// 3 consecutive failed cycles → red NeoPixel + ESP_LOGE (spec §7). The
// counter resets on the first subsequent success.
#define SD_FAIL_STREAK_ALERT 3

#define CELL_MAX  24    // one formatted number
#define ROW_MAX   (7 * CELL_MAX + TELEMETRY_MAX_COLUMNS * (CELL_MAX + 1) + 8)

static char        s_path[96];               // SD_MOUNT_POINT + '/' + filename
static uint16_t    s_order[TELEMETRY_MAX_COLUMNS];
static size_t      s_ncols;

// Defensive helper: vsnprintf into a buffer at offset n with bounds check.
// Returns updated n. Stops growing once the buffer is full (further calls
// no-op) — the non-negotiable fix for the bare `n += snprintf(...)`
// accumulation pattern, which lets one truncating call push n past the
// buffer size and turn the next call's `sz - n` into a huge size_t (i.e. an
// out-of-bounds write). Same clamp as http_server.c::append_safe /
// transmission.c::tx_append / mqtt.c::APPEND; not shared via util.h because
// each of those already carries its own file-local copy (house convention —
// see transmission.c:334-337).
__attribute__((format(printf, 4, 5)))
static int append_safe(char *out, size_t sz, int n, const char *fmt, ...) {
    if (n < 0 || (size_t)n >= sz) return (int)sz;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(out + n, sz - n, fmt, ap);
    va_end(ap);
    if (w < 0) return n;
    n += w;
    if ((size_t)n > sz) n = (int)sz;
    return n;
}

static int order_cmp(const void *a, const void *b) {
    const telemetry_desc_t *da = telemetry_get(*(const uint16_t *)a);
    const telemetry_desc_t *db = telemetry_get(*(const uint16_t *)b);
    return strcmp(da->header, db->header);
}

static void fail_cycle(int err) {
    s_last_err = err;
    s_fail_streak++;
    if (s_fail_streak == SD_FAIL_STREAK_ALERT) {
        // Solid red is deliberately loud — a standalone node has no other
        // way to tell a passer-by its whole reason for existing has stopped.
        ESP_LOGE(TAG, "%lu consecutive SD failures (last err %d)",
                 (unsigned long)s_fail_streak, err);
        // V2.6.19 (final review A1): neopixel_set_alert(), not
        // neopixel_set_rgb() — the latter is a one-shot the very next tube
        // pulse would wipe to black within seconds at background rate.
        neopixel_set_alert(64, 0, 0);
    }
}

static void close_file_for_remount(void) {
    if (s_f != NULL) {
        fclose(s_f);   // FILE* is stale once the card is gone — never reuse
        s_f = NULL;
    }
    sd_card_unmount();
    // Force a NEW file on recovery (spec §7): filename state reset here,
    // rebuilt from the (GPS-synced) clock at the next successful cycle.
    s_filename[0] = '\0';
    s_path[0]     = '\0';
}

/** Build sorted column order + write the header row. Returns false on I/O error. */
static bool create_file(void) {
    time_t now = time(NULL);
    struct tm tm_utc;
    gmtime_r(&now, &tm_utc);

    char stamp[20];
    strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm_utc);
    snprintf(s_filename, sizeof(s_filename), "%s_%s.csv", s_chip_id, stamp);
    snprintf(s_path, sizeof(s_path), "%s/%s", SD_MOUNT_POINT, s_filename);

    // Sorted column order (spec §4): pure function of the attached sensor
    // set — stable across boots, OTAs, and init-order refactors, so a
    // node's files always merge. Sorted ONCE per file, reused every row.
    s_ncols = telemetry_count();
    for (size_t i = 0; i < s_ncols; i++) s_order[i] = (uint16_t)i;
    qsort(s_order, s_ncols, sizeof(s_order[0]), order_cmp);

    s_f = fopen(s_path, "a");
    if (s_f == NULL) {
        ESP_LOGE(TAG, "fopen(%s): errno %d", s_path, errno);
        return false;
    }

    char row[ROW_MAX];
    int n = 0;
    row[0] = '\0';
    n = append_safe(row, sizeof(row), n,
                    "DateTime UTC,Uptime [s],GPS Lat,GPS Lon,GPS Alt [m],GPS Sats,GPS HDOP");
    for (size_t i = 0; i < s_ncols; i++) {
        n = append_safe(row, sizeof(row), n, ",%s", telemetry_get(s_order[i])->header);
    }
    append_safe(row, sizeof(row), n, "\n");
    if (fputs(row, s_f) == EOF || fflush(s_f) == EOF || fsync(fileno(s_f)) != 0) {
        ESP_LOGE(TAG, "header write failed: errno %d", errno);
        return false;
    }
    s_rows = 0;
    ESP_LOGI(TAG, "created %s (%u sensor columns)", s_filename, (unsigned)s_ncols);
    return true;
}

static bool write_row(void) {
    char row[ROW_MAX];
    int n = 0;
    row[0] = '\0';

    time_t now = time(NULL);
    struct tm tm_utc;
    gmtime_r(&now, &tm_utc);
    char stamp[24];
    strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);

    n = append_safe(row, sizeof(row), n, "%s,%llu", stamp,
                    (unsigned long long)(esp_timer_get_time() / 1000000LL));

    // GPS cells: empty on fix loss, while DateTime above keeps running on
    // the (GPS-synced) system clock — spec §5.
    gnss_fix_t gf = { 0 };
    if (gnss_get_fix(&gf) && gf.valid) {
        n = append_safe(row, sizeof(row), n, ",%.6f,%.6f,%.0f,%u,%.1f",
                        gf.lat, gf.lon, (double)gf.alt_m,
                        (unsigned)gf.sats, (double)gf.hdop);
    } else {
        n = append_safe(row, sizeof(row), n, ",,,,,");
    }

    for (size_t i = 0; i < s_ncols; i++) {
        const telemetry_desc_t *d = telemetry_get(s_order[i]);
        char cell[CELL_MAX];
        if (d->read(cell, sizeof(cell), d->arg)) {
            n = append_safe(row, sizeof(row), n, ",%s", cell);
        } else {
            n = append_safe(row, sizeof(row), n, ",");
        }
    }
    append_safe(row, sizeof(row), n, "\n");

    if (fputs(row, s_f) == EOF || fflush(s_f) == EOF || fsync(fileno(s_f)) != 0) {
        ESP_LOGE(TAG, "row write failed: errno %d", errno);
        return false;
    }
    s_rows++;
    return true;
}

#endif  // HAL_HAS_SD_CARD (append_safe .. write_row)

void sd_logger_cycle(void) {
#if HAL_HAS_SD_CARD
    // Mount (retries once per cycle after failure/card-pull — spec §7).
    if (!sd_card_mounted()) {
        esp_err_t err = sd_card_mount();
        if (err != ESP_OK) {
            fail_cycle((int)err);
            return;
        }
    }

    // File creation gate (spec §5): wait for the GPS-synced clock so the
    // filename and all rows share one valid UTC time base.
    if (s_f == NULL) {
        if (!gnss_clock_synced()) {
            ESP_LOGI(TAG, "waiting for GNSS clock sync before creating file");
            return;   // deliberately NOT a failure — antenna may just be cold
        }
        if (!create_file()) {
            close_file_for_remount();
            fail_cycle(errno);
            return;
        }
    }

    if (!write_row()) {
        close_file_for_remount();
        fail_cycle(errno);
        return;
    }

    if (s_fail_streak >= SD_FAIL_STREAK_ALERT) {
        neopixel_set_alert(0, 0, 0);   // recovery: clear the sticky red alert
    }
    s_fail_streak = 0;
    s_last_err    = 0;
#endif
}

void sd_logger_get_status(sd_logger_status_t *out) {
    out->mounted      = sd_card_mounted();
    out->file_open    = (s_f != NULL);
    out->rows_written = s_rows;
    out->fail_streak  = s_fail_streak;
    out->last_err     = s_last_err;
    safe_strcpy(out->filename, s_filename, sizeof(out->filename));
}
