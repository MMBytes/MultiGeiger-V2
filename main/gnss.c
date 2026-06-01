// V2.5.8 — Generic GNSS receiver driver. See gnss.h for the design doc and
// the PA1010D / MAX-M10S transport differences.

#include "gnss.h"
#include "ntp.h"

#include <string.h>
#include <stdlib.h>
#include <sys/time.h>      // struct timeval, settimeofday() for clock discipline
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"   // vTaskDelay (NMEA auto-detect sniff)

static const char *TAG = "gnss";

// --- I²C addresses ----------------------------------------------------------
#define ADDR_UBLOX    0x42   // u-blox DDC (MAX-M10S)
#define ADDR_PA1010D  0x10   // CDTop PA1010D (shares 0x10 with VEML7700)

// u-blox DDC registers.
#define UBX_REG_LEN_HI 0xFD  // bytes-available, high byte
#define UBX_REG_STREAM 0xFF  // data-stream register

// Clock discipline: re-set the system clock from GNSS only when it drifts at
// least this far from the fix UTC. Below this we leave it alone — RMC arrives
// at 1 Hz and writing settimeofday() every second injects sub-second jitter
// into everything that reads the clock. The RC oscillator drifts far less
// than this over the inter-set interval, so the clock stays GNSS-led without
// the churn.
#define DISCIPLINE_DRIFT_S   2

// How long after the last GNSS clock-set we still report GNSS as the active
// time source on /status (after which, with no fresh fix, SNTP is effectively
// back in charge).
#define TIME_SOURCE_TTL_MS   (5 * 60 * 1000)

// Plausibility floor for a GNSS-supplied year (see parse_datetime). The NMEA
// 2-digit year already caps the range at 2000-2099; rejecting anything before
// this also rules out the cold-start "year 00" glitch. A date that passes is
// trusted enough to drive settimeofday(), so keep this current.
#define GNSS_MIN_YEAR   2020

// Largest NMEA sentence we keep (spec max payload is 82 incl. CRLF; round up).
// Prefixed to avoid colliding with POSIX <limits.h> LINE_MAX.
#define GNSS_LINE_MAX 100

// --- Transport abstraction --------------------------------------------------
//
// Both chips hand us NMEA text; they differ only in how bytes come off I²C.
// read_chunk() returns the number of bytes placed in `buf` (>=0), or -1 on a
// bus error. `idle_byte` is the chip's no-data padding so the drain loop can
// stop early instead of spinning on filler.
typedef struct {
    const char *name;
    uint8_t     addr;
    uint8_t     idle_byte;
    int (*read_chunk)(uint8_t *buf, size_t max);
} gnss_transport_t;

static i2c_master_dev_handle_t s_dev      = NULL;
static const gnss_transport_t *s_tp       = NULL;
static bool                    s_ready     = false;

// --- Latest fix snapshot (written by poll task, read by HTTP task) ----------
static portMUX_TYPE s_fix_mux = portMUX_INITIALIZER_UNLOCKED;
static gnss_fix_t   s_fix     = { 0 };
static int64_t      s_last_clock_set_ms = 0;   // 0 = never

// --- Transport implementations ----------------------------------------------

// PA1010D: plain current-address read. The device always returns `max` bytes,
// padding with 0x0A (LF) when it has no fresh sentence — the '$'-anchored
// accumulator ignores that filler.
static int pa1010d_read_chunk(uint8_t *buf, size_t max) {
    if (i2c_master_receive(s_dev, buf, max, 100) != ESP_OK) return -1;
    return (int)max;
}

// u-blox DDC: read the 16-bit bytes-available count from 0xFD/0xFE, then read
// that many (capped at `max`) from the 0xFF stream register. Idle filler is
// 0xFF. Returns 0 when nothing is queued so the drain loop stops promptly.
static int ublox_read_chunk(uint8_t *buf, size_t max) {
    uint8_t reg = UBX_REG_LEN_HI;
    uint8_t cnt[2];
    if (i2c_master_transmit_receive(s_dev, &reg, 1, cnt, sizeof(cnt), 100) != ESP_OK) {
        return -1;
    }
    uint16_t avail = ((uint16_t)cnt[0] << 8) | cnt[1];
    if (avail == 0 || avail == 0xFFFF) return 0;   // 0xFFFF = bus held / no data

    size_t want = (avail < max) ? avail : max;
    uint8_t stream = UBX_REG_STREAM;
    if (i2c_master_transmit_receive(s_dev, &stream, 1, buf, want, 100) != ESP_OK) {
        return -1;
    }
    return (int)want;
}

static const gnss_transport_t TRANSPORTS[] = {
    { "MAX-M10S", ADDR_UBLOX,   0xFF, ublox_read_chunk   },
    { "PA1010D",  ADDR_PA1010D, 0x0A, pa1010d_read_chunk },
};

// --- NMEA parsing -----------------------------------------------------------

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// Validate "$....*HH" XOR checksum. `s` starts at '$', NUL-terminated.
static bool checksum_ok(const char *s) {
    const char *star = strchr(s, '*');
    if (!star || star[1] == 0 || star[2] == 0) return false;
    int hi = hexval(star[1]), lo = hexval(star[2]);
    if (hi < 0 || lo < 0) return false;
    uint8_t cs = 0;
    for (const char *p = s + 1; p < star; p++) cs ^= (uint8_t)*p;
    return cs == (uint8_t)((hi << 4) | lo);
}

// Convert NMEA ddmm.mmmm (lat) / dddmm.mmmm (lon) + hemisphere to decimal
// degrees. Minutes are always the two right-most integer digits in both
// encodings, so degrees = value / 100 works uniformly for lat and lon.
static double nmea_to_deg(const char *field, char hemi) {
    if (!field || !field[0]) return 0.0;
    double raw     = atof(field);                  // e.g. 4237.1234
    int    whole   = (int)(raw / 100.0);           // degrees part
    double minutes = raw - (double)whole * 100.0;  // remaining mm.mmmm
    double out     = (double)whole + minutes / 60.0;
    if (hemi == 'S' || hemi == 'W') out = -out;
    return out;
}

// Split `line` (a mutable copy) into comma-separated fields. The trailing
// "*HH" checksum is cut off the last field. Returns the field count.
static int split_fields(char *line, const char *fields[], int max_fields) {
    // Drop checksum suffix.
    char *star = strchr(line, '*');
    if (star) *star = 0;

    int n = 0;
    char *p = line;
    fields[n++] = p;
    while (*p && n < max_fields) {
        if (*p == ',') {
            *p = 0;
            fields[n++] = p + 1;
        }
        p++;
    }
    return n;
}

// Parse exactly two ASCII digits at p[0..1]. Returns -1 if either character
// is not 0-9 — important because atoi() silently accepts garbage (atoi("1x")
// == 1), and this value can reach settimeofday() (see parse_datetime).
static int two_digits(const char *p) {
    if (p[0] < '0' || p[0] > '9' || p[1] < '0' || p[1] > '9') return -1;
    return (p[0] - '0') * 10 + (p[1] - '0');
}

// Parse hhmmss(.sss) + ddmmyy into a UTC Unix epoch. Returns false on a blank
// time/date (no fix yet) OR on any out-of-range / non-digit field.
//
// This validation is a SECURITY/AVAILABILITY guard, not just hygiene: a
// checksum-valid NMEA sentence only proves wire integrity, not sanity — RF
// multipath / a glitching module can present a self-consistent garbage date.
// Since the parsed value flows straight into settimeofday() (GPS-primary
// clock), an unchecked bad date could yank the wall clock outside TLS
// cert-validity windows and break every HTTPS/MQTT upload until a good fix
// re-disciplines it. Reject anything implausible so a bad fix is a no-op, not
// a clock-corrupting event. `t_field` = "hhmmss.sss", `d_field` = "ddmmyy".
static bool parse_datetime(const char *t_field, const char *d_field, time_t *out) {
    if (!t_field || strlen(t_field) < 6) return false;
    if (!d_field || strlen(d_field) < 6) return false;

    int hh  = two_digits(t_field);
    int mm  = two_digits(t_field + 2);
    int ss  = two_digits(t_field + 4);
    int day = two_digits(d_field);
    int mon = two_digits(d_field + 2);
    int yy  = two_digits(d_field + 4);
    if (hh < 0 || mm < 0 || ss < 0 || day < 0 || mon < 0 || yy < 0) return false;

    // Range-validate every component (ss==60 allowed for leap seconds). The
    // NMEA 2-digit year already caps the range at 2000-2099; additionally
    // reject anything before GNSS_MIN_YEAR to catch the "year 00" cold-start
    // glitch — keeps the accepted window tight enough not to break TLS.
    if (hh > 23 || mm > 59 || ss > 60)                 return false;
    if (mon < 1 || mon > 12 || day < 1 || day > 31)    return false;
    if (yy + 2000 < GNSS_MIN_YEAR)                     return false;

    struct tm tm = { 0 };
    tm.tm_hour = hh;
    tm.tm_min  = mm;
    tm.tm_sec  = ss;
    tm.tm_mday = day;
    tm.tm_mon  = mon - 1;      // 0-based
    tm.tm_year = yy + 100;     // years since 1900 → 20yy

    time_t e = timegm(&tm);    // interpret tm as UTC (newlib provides timegm)
    if (e <= 0) return false;
    *out = e;
    return true;
}

// Apply GNSS UTC to the system clock when it has drifted past the threshold,
// and stamp the "GNSS is the time source" TTL.
static void discipline_clock(time_t gnss_utc, int64_t now_ms) {
    time_t sys = time(NULL);
    long drift = (long)(sys - gnss_utc);
    if (drift < 0) drift = -drift;

    if (s_last_clock_set_ms == 0 || drift >= DISCIPLINE_DRIFT_S) {
        struct timeval tv = { .tv_sec = gnss_utc, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        ESP_LOGI(TAG, "clock disciplined from GNSS (drift was %lds)", drift);
    }
    // GNSS owns the clock as long as fixes keep coming. Wrap the 64-bit store
    // in the fix mux so the HTTP task's read in gnss_time_is_source() can't
    // tear on a 32-bit core. (The read of it just above is on this same poll
    // task — the only writer — so it needs no lock.)
    taskENTER_CRITICAL(&s_fix_mux);
    s_last_clock_set_ms = now_ms;
    taskEXIT_CRITICAL(&s_fix_mux);
}

static void parse_rmc(const char *const fields[], int nf, int64_t now_ms) {
    // $..RMC,time,status,lat,N/S,lon,E/W,spd,cog,date,...
    if (nf < 10) return;
    bool valid = (fields[2][0] == 'A');

    // Do all parsing (nmea_to_deg/atof, parse_datetime/timegm) BEFORE taking
    // the spinlock — they're pure, but taskENTER_CRITICAL disables interrupts
    // on-core, so keep only the struct stores inside the section.
    double lat = 0.0, lon = 0.0;
    time_t utc = 0;
    bool   have_utc = false;
    if (valid) {
        lat = nmea_to_deg(fields[3], fields[4][0]);
        lon = nmea_to_deg(fields[5], fields[6][0]);
        have_utc = parse_datetime(fields[1], fields[9], &utc);
    }

    taskENTER_CRITICAL(&s_fix_mux);
    s_fix.valid = valid;
    if (valid) {
        s_fix.lat = lat;
        s_fix.lon = lon;
        if (have_utc) s_fix.utc = utc;
    }
    time_t utc_for_clock = s_fix.utc;
    taskEXIT_CRITICAL(&s_fix_mux);

    if (valid && utc_for_clock > 0) discipline_clock(utc_for_clock, now_ms);  // GPS-primary
}

static void parse_gga(const char *const fields[], int nf) {
    // $..GGA,time,lat,N/S,lon,E/W,quality,numSV,HDOP,alt,M,...
    if (nf < 10) return;
    int quality = atoi(fields[6]);

    taskENTER_CRITICAL(&s_fix_mux);
    s_fix.fix_3d = (quality >= 1);
    s_fix.sats   = (uint8_t)atoi(fields[7]);
    s_fix.hdop   = (float)atof(fields[8]);
    s_fix.alt_m  = (float)atof(fields[9]);
    taskEXIT_CRITICAL(&s_fix_mux);
}

// Dispatch one validated, NUL-terminated "$...*HH" sentence.
static void parse_sentence(char *line, int64_t now_ms) {
    if (line[0] != '$') return;
    if (!checksum_ok(line)) return;

    const char *fields[24];
    int nf = split_fields(line, fields, 24);
    if (nf < 1 || strlen(fields[0]) < 6) return;

    // fields[0] retains the leading '$', so it's "$" + talker(2) + type(3),
    // e.g. "$GPRMC" / "$GNRMC" / "$GNGGA". Skip the '$' AND the 2-char talker
    // (+3) to land on the 3-char sentence type, so both single-GPS (GP) and
    // multi-constellation (GN) talkers match. (Was +2 — an off-by-one that
    // dropped every sentence because the '$' was counted as part of the
    // talker; surfaced on first real-hardware fix.)
    const char *type = fields[0] + 3;
    if      (!strncmp(type, "RMC", 3)) parse_rmc(fields, nf, now_ms);
    else if (!strncmp(type, "GGA", 3)) parse_gga(fields, nf);
}

// --- Line accumulator -------------------------------------------------------
static char s_line[GNSS_LINE_MAX];
static int  s_len = 0;
static bool s_cap = false;

static void feed_byte(uint8_t b, int64_t now_ms) {
    if (b == '$') {                 // (re)start of a sentence
        s_cap = true;
        s_len = 0;
        s_line[s_len++] = '$';
        return;
    }
    if (!s_cap) return;             // junk / UBX binary / idle filler — ignore
    if (b == '\r' || b == '\n') {   // end of sentence
        if (s_len > 0) {
            s_line[s_len] = 0;
            parse_sentence(s_line, now_ms);
        }
        s_cap = false;
        s_len = 0;
        return;
    }
    if (s_len < GNSS_LINE_MAX - 1) {
        s_line[s_len++] = (char)b;
    } else {                        // overrun — drop and resync on next '$'
        s_cap = false;
        s_len = 0;
    }
}

// V2.5.10: confirm a device at the shared 0x10 address is actually a GNSS
// (streaming NMEA) and not a VEML7700 ambient-light sensor — both ACK at 0x10
// and both swallow register writes, so address + write-ACK can't tell them
// apart. We read raw bytes and look for ONE checksum-valid "$...*HH" sentence
// within a short window. A VEML7700 returns opaque register bytes
// (00 00 FF FF...) that can never pass an NMEA checksum, so this cannot
// false-positive. The window covers a module that hasn't emitted its first
// sentence yet at boot (idle-fill until the first fix-independent sentence).
static bool sniff_nmea(i2c_master_dev_handle_t dev) {
    int64_t deadline = esp_timer_get_time() + 1500 * 1000;   // ~1.5 s
    char line[GNSS_LINE_MAX];
    int  len = 0;
    bool cap = false;
    while (esp_timer_get_time() < deadline) {
        uint8_t buf[64];
        if (i2c_master_receive(dev, buf, sizeof(buf), 200) == ESP_OK) {
            for (int i = 0; i < 64; i++) {
                uint8_t b = buf[i];
                if (b == '$') {
                    cap = true; len = 0; line[len++] = '$';
                } else if (!cap) {
                    continue;
                } else if (b == '\r' || b == '\n') {
                    if (len > 0) { line[len] = 0; if (checksum_ok(line)) return true; }
                    cap = false; len = 0;
                } else if (len < GNSS_LINE_MAX - 1) {
                    line[len++] = (char)b;
                } else {
                    cap = false; len = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return false;
}

// --- Public API -------------------------------------------------------------

esp_err_t gnss_init(i2c_master_bus_handle_t bus) {
    if (s_ready) return ESP_OK;
    if (!bus) return ESP_ERR_INVALID_ARG;

    // Auto-detect: probe u-blox 0x42 first (unambiguous — nothing else lives
    // there), then PA1010D 0x10. 0x10 is shared with the VEML7700 light
    // sensor, so for that address we must SNIFF for live NMEA before claiming
    // it; a VEML7700 fails the sniff and is left for the ambient-light probe.
    for (size_t i = 0; i < sizeof(TRANSPORTS) / sizeof(TRANSPORTS[0]); i++) {
        const gnss_transport_t *tp = &TRANSPORTS[i];
        if (i2c_master_probe(bus, tp->addr, 50) != ESP_OK) continue;

        i2c_device_config_t devcfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = tp->addr,
            .scl_speed_hz    = 400000,   // both parts support 400 kHz
        };
        i2c_master_dev_handle_t dev = NULL;
        esp_err_t err = i2c_master_bus_add_device(bus, &devcfg, &dev);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "%s ACK'd at 0x%02X but add_device failed: %s",
                     tp->name, tp->addr, esp_err_to_name(err));
            continue;
        }

        // Disambiguate the shared 0x10 address; 0x42 binds on ACK.
        if (tp->addr == ADDR_PA1010D && !sniff_nmea(dev)) {
            ESP_LOGI(TAG, "0x%02X ACK'd but no NMEA in 1.5s — not a GNSS "
                          "(likely VEML7700), leaving for light-sensor probe",
                     tp->addr);
            i2c_master_bus_rm_device(dev);
            continue;
        }

        s_dev   = dev;
        s_tp    = tp;
        s_ready = true;
        ESP_LOGI(TAG, "%s auto-detected at 0x%02X (GPS-primary time source)",
                 tp->name, tp->addr);
        return ESP_OK;
    }

    // Quiet (debug) — auto-detect now runs on every node, so "not found" is
    // the normal case for boards without a GNSS, not a warning.
    ESP_LOGD(TAG, "no GNSS receiver found (probed 0x%02X, 0x%02X)",
             ADDR_UBLOX, ADDR_PA1010D);
    return ESP_ERR_NOT_FOUND;
}

bool gnss_present(void) {
    return s_ready;
}

void gnss_poll(void) {
    if (!s_ready) return;

    int64_t now_ms = esp_timer_get_time() / 1000;

    // Drain in bounded chunks. Stop early on an all-idle chunk (no fresh data)
    // or a bus error; the cap bounds worst-case time on the service tick.
    uint8_t chunk[64];
    for (int iter = 0; iter < 8; iter++) {
        int got = s_tp->read_chunk(chunk, sizeof(chunk));
        if (got <= 0) break;

        bool all_idle = true;
        for (int j = 0; j < got; j++) {
            if (chunk[j] != s_tp->idle_byte) all_idle = false;
            feed_byte(chunk[j], now_ms);
        }
        if (all_idle) break;
    }
}

bool gnss_get_fix(gnss_fix_t *out) {
    if (!s_ready || !out) return false;
    taskENTER_CRITICAL(&s_fix_mux);
    *out = s_fix;
    taskEXIT_CRITICAL(&s_fix_mux);
    return true;
}

const char *gnss_chip_name(void) {
    return s_ready ? s_tp->name : "none";
}

uint8_t gnss_i2c_addr(void) {
    return s_ready ? s_tp->addr : 0;
}

bool gnss_time_is_source(void) {
    if (!s_ready) return false;
    // Snapshot the 64-bit timestamp under the fix mux — written by the poll
    // task in discipline_clock(); a bare read here (HTTP task) could tear on a
    // 32-bit core and mislabel the /status time source for one tick.
    taskENTER_CRITICAL(&s_fix_mux);
    int64_t last = s_last_clock_set_ms;
    taskEXIT_CRITICAL(&s_fix_mux);
    if (last == 0) return false;
    int64_t now_ms = esp_timer_get_time() / 1000;
    return (now_ms - last) < TIME_SOURCE_TTL_MS;
}
