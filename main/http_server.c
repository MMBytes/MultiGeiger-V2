#include "http_server.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_chip_info.h"     // V2.3.13: chip-model lookup for OTA target validation
#include "esp_app_format.h"    // V2.3.13: ESP_CHIP_ID_* constants for OTA validation
#include "esp_app_desc.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "lwip/ip4_addr.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "mbedtls/base64.h"

#include "version.h"
#include "applog.h"
#include "diag.h"              // V2.4.32: diag_log_heap (net-RAM split at OTA-prep)
#include "coredump.h"          // V2.4.18: panic dump availability + summary + /coredump.elf
#include "hal.h"
#include "pm_sensor.h"
#include "env_sensor.h"
#include "noise_sensor.h"
#include "display.h"           // V2.3.30: live-apply OLED brightness via display_set_contrast
#include "als.h"               // V2.3.29: ambient light sensor (FeatherS3-D)
#include "veml7700.h"          // V2.3.30: I²C ambient light sensor (any board)
#include "gnss.h"              // V2.5.8: I²C GNSS receiver (PA1010D / MAX-M10S)
#include "tube.h"
#include "tube_pcnt.h"         // V2.5.16: release PCNT comb DRAM in OTA teardown
#include "history.h"            // V2.5.6: CPM history for the /status graph
#include "transmission.h"
#include "log_ftp.h"
#include "main_status.h"       // V2.4.1 (A4): consolidated status snapshot
#include "mqtt.h"              // V2.4.4: MQTT connection state for /status row
#include "syslog.h"            // V2.4.15: tear down UDP socket in OTA teardown
#include "sysinfo.h"           // V2.4.26: reset_reason_str / chip_model_str (also used by mqtt.c)
#include "ntp.h"               // ntp_time_valid — the clock-sane gate
#include "util.h"              // V2.4.1+ (T1): ct_memcmp, html_esc, url_decode, safe_strcpy

static const char *TAG = "http";

static httpd_handle_t s_server   = NULL;
static config_t      *s_cfg      = NULL;
static const char    *s_chip_id  = "";
static char           s_mac_str[18] = "??:??:??:??:??:??";   // filled at start
// V2.4.1 (A9): the pre-V2.4.1 `static volatile bool s_restart_requested`
// + `http_server_restart_requested()` polled-flag pair was replaced with
// an event-bit notification — handlers now call main_request_restart()
// from main_status.h, which both sets a persistent flag and wakes the
// main loop's xEventGroupWaitBits via EV_RESTART. Drops "click Save and
// restart" -> reboot latency from up to 1 s to ~µs.

// --- Access log --------------------------------------------------------------
// Logs every incoming request with its URI and client IP. Called at the top of
// every handler, before auth checks, so unauthorised attempts are visible too.

static void peer_ipstr(httpd_req_t *req, char *out, size_t outsz) {
    if (outsz == 0) return;
    out[0] = '?';
    out[1] = 0;
    int sockfd = httpd_req_to_sockfd(req);
    if (sockfd < 0) return;
    struct sockaddr_storage addr;
    socklen_t len = sizeof(addr);
    if (getpeername(sockfd, (struct sockaddr *)&addr, &len) != 0) return;
    if (addr.ss_family == AF_INET) {
        inet_ntop(AF_INET,
                  &((struct sockaddr_in *)&addr)->sin_addr,
                  out, outsz);
    } else if (addr.ss_family == AF_INET6) {
        inet_ntop(AF_INET6,
                  &((struct sockaddr_in6 *)&addr)->sin6_addr,
                  out, outsz);
    }
}

static void log_access(httpd_req_t *req, const char *what) {
    char ipstr[48];
    peer_ipstr(req, ipstr, sizeof(ipstr));
    ESP_LOGI(TAG, "%s from %s", what, ipstr);
}

// --- Auth --------------------------------------------------------------------

// V2.4.1+ (T1): ct_memcmp moved to util.h as a static inline so the host-
// side test runner under `test/` can include it directly. Same constant-
// time semantics as the V2.3.33 (B2) original.

// Returns true if Authorization header is "Basic base64(admin:<ap_password>)".
// On failure, sends 401 + WWW-Authenticate and returns false. The caller must
// return ESP_OK immediately without sending anything further.
//
// Distinguishes two failure modes: header absent (expected first-touch from
// a fresh browser — logged as INFO) vs. header present but wrong credentials
// (a real bad-password attempt — logged as WARN so it stands out in /log).
static bool check_auth(httpd_req_t *req) {
    char header[160];
    size_t hlen = httpd_req_get_hdr_value_len(req, "Authorization");
    bool header_present = (hlen > 0 && hlen < sizeof(header));
    if (!header_present) goto unauth;
    if (httpd_req_get_hdr_value_str(req, "Authorization", header, sizeof(header)) != ESP_OK) {
        goto unauth;
    }
    if (strncmp(header, "Basic ", 6) != 0) goto unauth;

    char userpass[128];
    int n = snprintf(userpass, sizeof(userpass), "admin:%s", s_cfg->ap_password);
    if (n <= 0 || (size_t)n >= sizeof(userpass)) goto unauth;

    unsigned char expected[200];
    size_t enc_len = 0;
    if (mbedtls_base64_encode(expected, sizeof(expected) - 1, &enc_len,
                              (const unsigned char *)userpass, strlen(userpass)) != 0) {
        goto unauth;
    }
    expected[enc_len] = 0;
    // V2.3.33: length check first, then constant-time byte compare. Length
    // mismatch is non-secret (derivable from the credential layout) so the
    // early exit is fine; the byte comparison must be timing-safe.
    size_t got_len = strlen(header + 6);
    if (got_len != enc_len) goto unauth;
    if (ct_memcmp(header + 6, expected, enc_len) != 0) goto unauth;
    return true;

unauth:
    if (header_present) {
        char ipstr[48];
        peer_ipstr(req, ipstr, sizeof(ipstr));
        ESP_LOGW(TAG, "auth failed for %s from %s", req->uri, ipstr);
    }
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"MultiGeiger\"");
    httpd_resp_send(req, NULL, 0);
    return false;
}

// --- CSRF guard for state-changing POST handlers ----------------------------
//
// V2.3.33: defends against cross-site request forgery on /config, /update,
// /reboot. Basic-auth credentials are cached per-origin by browsers and
// auto-attached to any subsequent request to the same origin — including
// cross-origin form POSTs from a malicious page the admin happens to visit
// in the same browser session. Without this guard, an attacker page can
// `<form action="http://device/config" method=POST>` with arbitrary fields,
// the browser attaches the cached Authorization header, and the request
// succeeds silently.
//
// Defence: require the request's Origin header to match Host (preferred —
// all modern browsers set Origin on cross-origin POST and on most
// same-origin POSTs); fall back to Referer match on host prefix. If
// neither header is present, deny — programmatic clients (curl, scripts)
// can pass `-H "Origin: http://<device>:<port>"` to satisfy the check.
//
// Caller must `return ESP_OK` immediately when this returns false.
static bool check_same_origin(httpd_req_t *req) {
    char host[64];
    if (httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK) {
        goto deny;
    }
    size_t host_len = strlen(host);

    char origin[128];
    if (httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin)) == ESP_OK) {
        // Origin format: "scheme://host[:port]" with no path. After stripping
        // the scheme prefix, what remains must equal Host exactly.
        const char *p = strstr(origin, "://");
        if (!p) goto deny;
        p += 3;
        if (strcmp(p, host) == 0) return true;
        goto deny;
    }

    char referer[160];
    if (httpd_req_get_hdr_value_str(req, "Referer", referer, sizeof(referer)) == ESP_OK) {
        // Referer format: "scheme://host[:port]/path?query#frag". After
        // stripping the scheme, the host portion (up to the first '/', '?',
        // '#' or NUL terminator) must equal Host.
        const char *p = strstr(referer, "://");
        if (!p) goto deny;
        p += 3;
        if (strncmp(p, host, host_len) == 0) {
            char next = p[host_len];
            if (next == '/' || next == 0 || next == '?' || next == '#') {
                return true;
            }
        }
        goto deny;
    }

deny:
    {
        char ipstr[48];
        peer_ipstr(req, ipstr, sizeof(ipstr));
        ESP_LOGW(TAG, "CSRF check failed for POST %s from %s", req->uri, ipstr);
    }
    httpd_resp_set_status(req, "403 Forbidden");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_send(req, "Cross-origin POST refused (CSRF protection). "
                        "Submit from the device's own pages.",
                    HTTPD_RESP_USE_STRLEN);
    return false;
}

// --- Security response headers ----------------------------------------------
//
// V2.3.33: clickjacking protection for the admin UI. `DENY` blocks framing
// from any origin (the device never frames itself either). Applied to
// success responses on /config, /update, /reboot — the pages browsers
// actually render. Error responses funnelled through `httpd_resp_send_err`
// build their own minimal response and aren't normally framed in attacks.
static void set_security_headers(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
}

// --- HTML escape + URL decode + hex_nibble ----------------------------------
// V2.4.1+ (T1): moved to util.h as static inline so the host-side test
// runner under `test/` can include them directly. Same semantics as before
// (html_esc V2.0+, hex_nibble V2.0+, url_decode V2.4.1 B6 RFC-strict).

// --- GET / (status, no auth) -------------------------------------------------

// V2.5.33: format_uptime() moved to util.h (static inline) so transmission.c's
// heap-guard reboot log line can reuse the exact /status uptime formatting.

// Decode wifi_auth_mode_t to a short label. Names match what most APs use in
// their admin UI (the IDF enum names like AUTH_WPA2_PSK are too jargon-y).
static const char *wifi_auth_str(wifi_auth_mode_t m) {
    switch (m) {
        case WIFI_AUTH_OPEN:            return "Open";
        case WIFI_AUTH_WEP:             return "WEP";
        case WIFI_AUTH_WPA_PSK:         return "WPA-PSK";
        case WIFI_AUTH_WPA2_PSK:        return "WPA2-PSK";
        case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2-PSK";
        case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-Enterprise";
        case WIFI_AUTH_WPA3_PSK:        return "WPA3-PSK";
        case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3-PSK";
        default:                        return "?";
    }
}

// Compact phy-mode summary like "bgn" or "ax". Each capability bit becomes a
// letter; absent radios just drop out (no padding). IDF 6.0 exposes 11a/ac/ax
// flags too — prepended with their own letters where relevant.
static void wifi_phy_str(const wifi_ap_record_t *ap, char *out, size_t sz) {
    char buf[16];
    int n = 0;
    if (ap->phy_11b)  buf[n++] = 'b';
    if (ap->phy_11g)  buf[n++] = 'g';
    if (ap->phy_11n)  buf[n++] = 'n';
    if (ap->phy_11a)  buf[n++] = 'a';
    if (ap->phy_11ac) { buf[n++] = 'a'; buf[n++] = 'c'; }
    if (ap->phy_11ax) { buf[n++] = 'a'; buf[n++] = 'x'; }
    if (ap->phy_lr)   buf[n++] = 'L';      // ESP-NOW long-range, rare
    buf[n] = 0;
    if (n == 0) snprintf(out, sz, "?");
    else        snprintf(out, sz, "%s", buf);
}

static void format_net_info(char *out, size_t sz) {
    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode);

    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    wifi_ap_record_t ap;
    bool sta_up = (sta != NULL) && (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);

    // ap_name is a user-editable config field, so escape it once for whichever
    // Network branch renders below — same treatment as the joined SSID
    // (ssid_esc) and the /config page (e_apn). 96 B matches the /config buffer;
    // html_esc() is bounded so it truncates rather than overflows.
    char ap_name_esc[96];
    html_esc(s_cfg->ap_name, ap_name_esc, sizeof(ap_name_esc));

    if (sta_up) {
        esp_netif_ip_info_t ip = { 0 };
        esp_netif_get_ip_info(sta, &ip);
        esp_netif_dns_info_t d1 = { 0 }, d2 = { 0 };
        esp_netif_get_dns_info(sta, ESP_NETIF_DNS_MAIN,   &d1);
        esp_netif_get_dns_info(sta, ESP_NETIF_DNS_BACKUP, &d2);
        char ip_s[16], gw_s[16], nm_s[16], d1_s[16], d2_s[16];
        esp_ip4addr_ntoa(&ip.ip,      ip_s, sizeof(ip_s));
        esp_ip4addr_ntoa(&ip.gw,      gw_s, sizeof(gw_s));
        esp_ip4addr_ntoa(&ip.netmask, nm_s, sizeof(nm_s));
        esp_ip4addr_ntoa(&d1.ip.u_addr.ip4, d1_s, sizeof(d1_s));
        esp_ip4addr_ntoa(&d2.ip.u_addr.ip4, d2_s, sizeof(d2_s));
        bool has_d2 = (d2.ip.u_addr.ip4.addr != 0) &&
                      (d2.ip.u_addr.ip4.addr != d1.ip.u_addr.ip4.addr);
        char ssid_esc[66];
        // SSID is up to 32 bytes plus null; escape into 65 + slack.
        char ssid_raw[33] = {0};
        memcpy(ssid_raw, ap.ssid, sizeof(ap.ssid));
        ssid_raw[32] = 0;
        html_esc(ssid_raw, ssid_esc, sizeof(ssid_esc));
        char phy[8];
        wifi_phy_str(&ap, phy, sizeof(phy));
        const char *bw = (ap.second == WIFI_SECOND_CHAN_NONE) ? "BW20" : "BW40";
        // AID lives in a separate API in IDF 6.0 (was a wifi_ap_record_t
        // member in 5.x). 0 = not associated / unknown.
        uint16_t aid = 0;
        esp_wifi_sta_get_aid(&aid);
        main_status_t st;
        main_status_snapshot(&st);
        snprintf(out, sz,
                 "<div class=\"info\"><h3>Network</h3>"
                 // AP SSID = this device's own fallback/AP-mode SSID (config
                 // ap_name); distinct from the "SSID:" below = the network it's
                 // joined to. Lives in the Network block (moved out of System).
                 "<b>AP SSID:</b> %s<br>"
                 "<b>SSID:</b> %s<br>"
                 "<b>Security:</b> %s &middot; PHY %s &middot; %s &middot; channel %d<br>"
                 "<b>BSSID:</b> %02x:%02x:%02x:%02x:%02x:%02x &nbsp; AID %u<br>"
                 "<b>RSSI:</b> %d dBm<br>"
                 "<b>IP:</b> %s<br>"
                 "<b>Gateway:</b> %s<br>"
                 "<b>Network Mask:</b> %s<br>"
                 "<b>DNS:</b> %s%s%s<br>"
                 "<b>Reconnects:</b> %lu since boot"
                 "</div>",
                 ap_name_esc,
                 ssid_esc,
                 wifi_auth_str(ap.authmode), phy, bw, (int)ap.primary,
                 ap.bssid[0], ap.bssid[1], ap.bssid[2],
                 ap.bssid[3], ap.bssid[4], ap.bssid[5],
                 (unsigned)aid,
                 (int)ap.rssi,
                 ip_s, gw_s, nm_s,
                 d1_s, has_d2 ? ", " : "", has_d2 ? d2_s : "",
                 (unsigned long)st.reconnects);
        return;
    }

    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) {
        esp_netif_t *apn = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        esp_netif_ip_info_t ip = { 0 };
        if (apn) esp_netif_get_ip_info(apn, &ip);
        char ip_s[16];
        esp_ip4addr_ntoa(&ip.ip, ip_s, sizeof(ip_s));
        snprintf(out, sz, "<div class=\"info\"><h3>Network</h3><b>AP SSID:</b> %s<br><b>IP:</b> %s (AP mode)</div>", ap_name_esc, ip_s);
        return;
    }

    snprintf(out, sz, "<div class=\"info\"><h3>Network</h3><b>Network:</b> No connection</div>");
}

// --- Device identity block ---------------------------------------------------
// All format_* helpers emit a complete self-wrapping <div class="info"> block
// so status_get can stream them sequentially via httpd_resp_send_chunk without
// needing a top-level body buffer. Empty output (e.g. tube disabled, no env
// sensor) is a single null terminator — the caller's strlen() sees 0, the
// chunk is skipped.
static void format_device(char *out, size_t sz) {
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    const char *model = chip_model_str(chip.model);
    char feat[32]; feat[0] = 0;
    if (chip.features & CHIP_FEATURE_WIFI_BGN) strncat(feat, "WiFi ", sizeof(feat)-strlen(feat)-1);
    if (chip.features & CHIP_FEATURE_BLE)      strncat(feat, "BLE ",  sizeof(feat)-strlen(feat)-1);
    if (chip.features & CHIP_FEATURE_BT)       strncat(feat, "BT ",   sizeof(feat)-strlen(feat)-1);
    bool emb_flash = (chip.features & CHIP_FEATURE_EMB_FLASH) != 0;
    bool emb_psram = (chip.features & CHIP_FEATURE_EMB_PSRAM) != 0;
    if (emb_flash) strncat(feat, "EmbFlash ", sizeof(feat)-strlen(feat)-1);
    if (emb_psram) strncat(feat, "EmbPSRAM ", sizeof(feat)-strlen(feat)-1);
    // Strip trailing space.
    size_t fl = strlen(feat); if (fl && feat[fl-1] == ' ') feat[fl-1] = 0;

    const esp_app_desc_t *app = esp_app_get_description();
    const char *fw_date = app ? app->date : __DATE__;
    const char *fw_time = app ? app->time : __TIME__;

#if HAL_HAS_ANTENNA_SWITCH
    const char *antenna = s_cfg->use_external_antenna ? "EXTERNAL u.FL" : "internal PCB";
#else
    const char *antenna = "(N/A — internal only)";
#endif

    snprintf(out, sz,
        "<div class=\"info\"><h3>Device</h3>"
        "<b>Chip ID:</b> %s<br>"
        "<b>MAC:</b> %s<br>"
        "<b>Board:</b> %s<br>"
        "<b>Chip:</b> %s rev v%d.%d &middot; %d cores &middot; %s<br>"
        "<b>Memory:</b> %lu MB flash%s<br>"
        "<b>Firmware:</b> %s &nbsp; (built %s %s)<br>"
        "<b>Antenna:</b> %s"
        "</div>",
        s_chip_id,
        s_mac_str,
        BOARD_NAME,
        model, chip.revision / 100, chip.revision % 100, chip.cores, feat[0] ? feat : "?",
        (unsigned long)(flash_size / (1024 * 1024)),
#if HAL_HAS_PSRAM
        " &middot; 8 MB PSRAM",
#else
        "",
#endif
        VERSION_STR, fw_date, fw_time,
        antenna);
}

// --- System block ------------------------------------------------------------
// Uptime, reset reason, heap, NTP last-sync.
static void format_ago(int64_t now_unix, int64_t past_unix, char *out, size_t sz) {
    if (past_unix <= 0 || now_unix < past_unix) { snprintf(out, sz, "never"); return; }
    long s = (long)(now_unix - past_unix);
    if (s < 60)        snprintf(out, sz, "%lds ago", s);
    else if (s < 3600) snprintf(out, sz, "%ldm %lds ago", s / 60, s % 60);
    else if (s < 86400)snprintf(out, sz, "%ldh %ldm ago", s / 3600, (s / 60) % 60);
    else               snprintf(out, sz, "%ldd %ldh ago", s / 86400, (s / 3600) % 24);
}

static void format_wallclock(int64_t unix_t, char *out, size_t sz) {
    if (unix_t <= 0) { snprintf(out, sz, "—"); return; }
    time_t t = (time_t)unix_t;
    struct tm tm;
    localtime_r(&t, &tm);
    strftime(out, sz, "%Y-%m-%d %H:%M:%S", &tm);
}

static void format_system(char *out, size_t sz, unsigned long uptime_s, time_t now_wall) {
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_free  = esp_get_minimum_free_heap_size();
    uint32_t max_alloc = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    char uptime_buf[32];
    format_uptime(uptime_s, uptime_buf, sizeof(uptime_buf));

    // NTP last-sync. sntp_get_sync_status() returns COMPLETED briefly after
    // each callback then resets to RESET — which is misleading on a page that
    // refreshes infrequently. Better: just check whether the wall clock looks
    // sane (same predicate ntp_time_valid uses) and show the most recent
    // localtime sample.
    // now_wall is captured once in status_get so "clock now" and "Started+Uptime"
    // are rooted in the same timestamp — their arithmetic is always consistent.
    time_t now = now_wall;
    char ntp_line[96];
    if (ntp_time_valid()) {     // clock is real (NTP-synced or sane RTC carry-over)
        char ts[24];
        format_wallclock((int64_t)now, ts, sizeof(ts));
        snprintf(ntp_line, sizeof(ntp_line), "synced &middot; clock now %s", ts);
    } else {
        snprintf(ntp_line, sizeof(ntp_line), "<span style='color:#c80'>not synced yet</span>");
    }

    // V2.5.22: append the boot wall-clock to the Uptime line. Boot epoch is
    // captured once at first NTP sync (ntp_boot_epoch()) so it never drifts:
    // recomputing now-uptime on every page load walks ~1s per few hours because
    // esp_timer counts raw crystal ticks while time(NULL) is NTP-slewed.
    char started_suffix[48] = "";
    time_t boot_epoch = ntp_boot_epoch();
    if (boot_epoch) {
        char started[24];
        format_wallclock((int64_t)boot_epoch, started, sizeof(started));
        snprintf(started_suffix, sizeof(started_suffix), " (Started %s)", started);
    }

    // V2.4.18: core dump line. Three render modes:
    //   - no dump:  "none"
    //   - dump:     "yes &middot; <SZ> bytes &middot; task=<NAME> PC=0x...
    //                <reason> [download] [erase]"
    // The download link is always shown when a dump exists; the erase
    // button submits via a tiny form to /coredump_erase (CSRF-checked
    // POST) and reloads.
    // V2.4.22: cd_summary[320] + cd_line[640] = ~960 B moved from stack
    // to BSS. Same V2.4.20 fix pattern — large buffers on the httpd task's
    // 8 KB stack are fragile, and format_system is called from the `/`
    // handler which already pays the static-ified buf[1600] tax. Safe
    // because esp_http_server uses a single thread (httpd_thread runs all
    // URI handlers serially via select), so format_system is never
    // re-entered. Both branches of the if/else below always populate
    // cd_line before it's read, so no per-call zero-init needed.
    static char cd_summary[320];
    coredump_get_summary_html(cd_summary, sizeof(cd_summary));
    static char cd_line[640];
    if (coredump_have_dump()) {
        // Inline form for the erase button keeps the System block
        // self-contained — no JS, no separate page. The hidden submit
        // POSTs to /coredump_erase with same-origin so the CSRF check
        // succeeds; basic-auth is auto-attached by the browser since
        // the user already authenticated for /status.
        snprintf(cd_line, sizeof(cd_line),
            "<b>Core dump:</b> %s &middot; "
            "<a href=\"/coredump.elf\">download .elf</a> &middot; "
            "<form method=\"POST\" action=\"/coredump_erase\" "
            "style=\"display:inline\" "
            "onsubmit=\"return confirm('Erase core dump?');\">"
            "<button type=\"submit\">erase</button></form><br>",
            cd_summary);
    } else {
        snprintf(cd_line, sizeof(cd_line),
                 "<b>Core dump:</b> %s<br>", cd_summary);
    }

    // V2.4.28: I²C sensor-read-error counter. Climbs when env / PM / noise /
    // light reads fail at the top-level call site in do_tx_cycle. A
    // monotonically rising value points at marginal supply, flaky bus, or
    // a failing sensor — pair it with `reset_reason: BROWNOUT` to confirm
    // power vs bus suspicion.
    main_status_t st;
    main_status_snapshot(&st);

    snprintf(out, sz,
        "<div class=\"info\"><h3>System</h3>"
        "<b>Uptime:</b> %s%s<br>"
        "<b>Reset reason:</b> %s<br>"
        "%s"                                  // core-dump line (V2.4.18)
        "<b>Free heap:</b> %lu bytes<br>"
        "<b>Min free heap:</b> %lu bytes<br>"
        "<b>Max allocation:</b> %lu bytes<br>"
        "<b>I²C errors:</b> %lu since boot<br>"
        "<b>NTP:</b> %s<br>"
        // V2.4.9: resolved display layout mode. Shows what display.c
        // picked at boot (e.g. "auto (resolved: rotation)" or
        // "radiation (forced)") so the user can confirm the runtime
        // decision without digging into /log.
        "<b>Display layout:</b> %s"
        "</div>",
        uptime_buf, started_suffix,
        reset_reason_str(esp_reset_reason()),
        cd_line,
        (unsigned long)free_heap,
        (unsigned long)min_free,
        (unsigned long)max_alloc,
        (unsigned long)st.i2c_errors,
        ntp_line,
        display_mode_str());
}

// --- Cycle block -------------------------------------------------------------
static void format_cycle(char *out, size_t sz, unsigned long uptime_ms) {
    main_status_t st;
    main_status_snapshot(&st);

    char wall[32], ago[24];
    format_wallclock(st.last_cycle_at, wall, sizeof(wall));
    format_ago((int64_t)time(NULL), st.last_cycle_at, ago, sizeof(ago));

    // Next cycle = last + tx_interval. If we haven't run yet, "—".
    char next_line[64];
    if (st.last_cycle_ms == 0) {
        snprintf(next_line, sizeof(next_line), "—");
    } else {
        long remaining_ms = (long)(st.last_cycle_ms + s_cfg->tx_interval_ms) - (long)uptime_ms;
        if (remaining_ms < 0) remaining_ms = 0;
        snprintf(next_line, sizeof(next_line), "in %lds", remaining_ms / 1000);
    }

    snprintf(out, sz,
        "<div class=\"info\"><h3>Cycle</h3>"
        "<b>Cycle #:</b> %lu<br>"
        "<b>Last dt:</b> %lu ms<br>"
        "<b>Interval:</b> %lu ms<br>"
        "<b>Last cycle:</b> %s &nbsp; (%s)<br>"
        "<b>Next cycle:</b> %s"
        "</div>",
        (unsigned long)st.cycles,
        (unsigned long)st.last_dt_ms,
        (unsigned long)s_cfg->tx_interval_ms,
        wall, ago,
        next_line);
}

// --- Radiation block (only when tube is enabled) -----------------------------
static void format_radiation(char *out, size_t sz) {
    if (!tube_is_enabled()) { out[0] = 0; return; }

    main_status_t st;
    main_status_snapshot(&st);

    // HV pulses per minute over the last cycle window. dt_ms == 0 means no
    // cycle has run yet; show a dash so we don't divide by zero. V2.4.27:
    // uses `last_hv_pulses_delta` (per-cycle) instead of `last_hv_pulses`
    // (cumulative-since-boot) — the previous form divided the cumulative
    // counter by one cycle's duration, producing a number that grew
    // unboundedly with uptime instead of the intended pulses/min rate.
    char hv_line[64];
    if (st.last_dt_ms > 0) {
        float hv_per_min = (float)st.last_hv_pulses_delta * 60000.0f / (float)st.last_dt_ms;
        snprintf(hv_line, sizeof(hv_line), "%.1f / min  (cumulative %lu)",
                 hv_per_min, (unsigned long)st.last_hv_pulses);
    } else {
        snprintf(hv_line, sizeof(hv_line), "—  (cumulative %lu)",
                 (unsigned long)st.last_hv_pulses);
    }

    // V2.5.16: width-filter indicator — only shown when the filter is active.
    // Makes it obvious on the page that the CPM above is POST-filter, and what
    // the pre-filter value was, so the effect is visible without the syslog.
    char filt_line[128];
    if (st.pcnt_filtering) {
        snprintf(filt_line, sizeof(filt_line),
                 "<b>PCNT width filter:</b> ON @%lu ns &mdash; raw CPM %lu "
                 "&rarr; filtered %lu<br>",
                 (unsigned long)st.pcnt_filter_width_ns,
                 (unsigned long)st.last_cpm_raw, (unsigned long)st.last_cpm);
    } else {
        filt_line[0] = 0;
    }

    snprintf(out, sz,
        "<div class=\"info\"><h3>Radiation</h3>"
        "<b>Tube:</b> enabled &middot; %s%s<br>"
        "<b>CPM:</b> %lu<br>"
        "%s"
        "<b>Dose rate:</b> %.3f µSv/h<br>"
        "<b>HV pulses:</b> %s"
        "</div>",
        tube_type_name(s_cfg->tube_type),   // V2.6.1: show the selected tube
        st.last_hv_error ? " &middot; <span style='color:#c00;font-weight:bold'>HV ERROR</span>" : "",
        (unsigned long)st.last_cpm, filt_line, st.last_usvph,
        hv_line);
}

// --- Environment block -------------------------------------------------------
static void format_environment(char *out, size_t sz) {
    if (!env_sensor_present()) { out[0] = 0; return; }
    main_status_t st;
    main_status_snapshot(&st);
    if (!st.have_env) {
        snprintf(out, sz,
            "<div class=\"info\"><h3>Environment</h3>"
            "<b>Sensor:</b> %s<br>"
            "awaiting first cycle..."
            "</div>", env_sensor_name());
        return;
    }
    snprintf(out, sz,
        "<div class=\"info\"><h3>Environment</h3>"
        "<b>Sensor:</b> %s<br>"
        "<b>Temperature:</b> %.2f °C<br>"
        "<b>Humidity:</b> %.2f %%<br>"
        "<b>Pressure:</b> %.2f hPa"
        "</div>",
        env_sensor_name(),
        st.env_t,
        st.env_h,
        st.env_p / 100.0f);
}

// Defensive append helper — defined below near format_uploads; forward-declared
// here so the multi-segment formatters above it (ALS / GNSS / PM) can use the
// clamped accumulation instead of bare `n += snprintf` (V2.5.20 review R4:
// on truncation the bare pattern pushes n past sz and `sz - n` underflows).
__attribute__((format(printf, 4, 5)))
static int append_safe(char *out, size_t sz, int n, const char *fmt, ...);

// --- Ambient light block ----------------------------------------------------
// Two possible sources, both shown when present:
//   * VEML7700 (I²C, any board, fixed 0x10) — accurate lux + raw ALS + white
//   * ALS-PT19 (FeatherS3-D analog, GPIO 4) — rough lux from voltage divider
//
// Reads happen on every /status request — VEML7700 is two short I²C reads
// (~1 ms total at 400 kHz), ALS-PT19 is a ~10 µs ADC oneshot. Fresh data
// for a page that's only loaded when the user opens it. The whole block
// is skipped if neither sensor is present.
static void format_als(char *out, size_t sz) {
    bool have_veml = veml7700_present();
    bool have_pt19 = als_present();
    if (!have_veml && !have_pt19) { out[0] = 0; return; }

    int n = append_safe(out, sz, 0, "<div class=\"info\"><h3>Ambient light</h3>");

    if (have_veml) {
        uint16_t als_raw = 0, white_raw = 0;
        float    lux = 0.0f;
        if (veml7700_read(&als_raw, &white_raw, &lux) == ESP_OK) {
            n = append_safe(out, sz, n,
                "<b>Sensor:</b> VEML7700 (I²C, 0x10)<br>"
                "<b>Reading:</b> %.1f lux (raw ALS=%u, white=%u, %s)",
                (double)lux, (unsigned)als_raw, (unsigned)white_raw,
                als_brightness_label(lux));
        } else {
            n = append_safe(out, sz, n,
                "<b>Sensor:</b> VEML7700 (I²C, 0x10)<br>read failed");
        }
        if (have_pt19) n = append_safe(out, sz, n, "<br><br>");
    }

    if (have_pt19) {
        uint32_t mv = 0;
        float    lux = 0.0f;
        if (als_read(NULL, &mv, &lux) == ESP_OK) {
            n = append_safe(out, sz, n,
                "<b>Sensor:</b> ALS-PT19 (analog, GPIO 4)<br>"
                "<b>Reading:</b> %lu mV (~%d lux, %s)",
                (unsigned long)mv, (int)lux, als_brightness_label(lux));
        } else {
            n = append_safe(out, sz, n,
                "<b>Sensor:</b> ALS-PT19 (analog, GPIO 4)<br>read failed");
        }
    }

    append_safe(out, sz, n, "</div>");
}

// --- GNSS / position block --------------------------------------------------
// V2.5.8: shown only when a GNSS receiver was auto-detected at boot. Reads the
// cached fix snapshot — no I²C here; the receiver is drained on the main
// service task. Until the first valid fix we show the acquisition state
// (satellite count climbs as the receiver locks on).
static void format_gnss(char *out, size_t sz) {
    if (!gnss_present()) { out[0] = 0; return; }

    gnss_fix_t f;
    gnss_get_fix(&f);

    int n = append_safe(out, sz, 0,
        "<div class=\"info\"><h3>GNSS / Position</h3>"
        "<b>Sensor:</b> %s (I²C, 0x%02X)<br>",
        gnss_chip_name(), gnss_i2c_addr());

    // Unique chip ID — MAX-M10S only (UBX-SEC-UNIQID); empty for the PA1010D.
    const char *serial = gnss_serial();
    if (serial[0]) {
        n = append_safe(out, sz, n, "<b>Serial:</b> 0x%s<br>", serial);
    }

    if (!f.valid) {
        n = append_safe(out, sz, n,
            "<b>Fix:</b> acquiring… (%u satellites visible)", (unsigned)f.sats);
    } else {
        char utc[32] = "—";
        if (f.utc > 0) {
            struct tm tm_utc;
            gmtime_r(&f.utc, &tm_utc);
            strftime(utc, sizeof(utc), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
        }
        n = append_safe(out, sz, n,
            "<b>Fix:</b> %s, %u satellites, HDOP %.1f<br>"
            "<b>Position:</b> %.6f, %.6f "
            "(<a href=\"https://www.openstreetmap.org/?mlat=%.6f&amp;mlon=%.6f"
            "#map=15/%.6f/%.6f\" target=\"_blank\" rel=\"noopener\">map</a>)<br>"
            "<b>Altitude:</b> %.0f m MSL<br>"
            "<b>UTC:</b> %s",
            f.fix_3d ? "3D" : "2D", (unsigned)f.sats, (double)f.hdop,
            f.lat, f.lon, f.lat, f.lon, f.lat, f.lon,
            (double)f.alt_m, utc);
    }

    // V2.5.11: no "system time source" line — GNSS no longer sets the clock
    // (NTP is the sole source). The UTC above is the receiver's reported time.
    append_safe(out, sz, n, "</div>");
}

// --- Noise block -------------------------------------------------------------
static void format_noise(char *out, size_t sz) {
    if (!noise_sensor_present()) { out[0] = 0; return; }
    noise_sample_t n;
    if (noise_sensor_get_last_sample(&n) != ESP_OK) {
        snprintf(out, sz,
            "<div class=\"info\"><h3>Noise</h3>"
            "<b>Sensor:</b> %s<br>"
            "first window still integrating..."
            "</div>", noise_sensor_name());
        return;
    }
    snprintf(out, sz,
        "<div class=\"info\"><h3>Noise</h3>"
        "<b>Sensor:</b> %s (%s)<br>"
        "<b>LAeq:</b> %.1f dB(A)<br>"
        "<b>LA min / max:</b> %.1f / %.1f dB(A)"
        "</div>",
        noise_sensor_name(), noise_sensor_version(),
        n.laeq, n.la_min, n.la_max);
}

// --- MQTT block --------------------------------------------------------------
//
// V2.4.4 (Phase 3): /status row for the MQTT client added in V2.4.2. Skipped
// when the user hasn't enabled MQTT — keeps the page tight on Madavi-only
// devices. Reads only from mqtt.c's connect-state flag + publish counter,
// never touches the broker from this HTTP-handler context (no I/O blocking).
static void format_mqtt(char *out, size_t sz) {
    if (!s_cfg->mqtt_enable) { out[0] = 0; return; }

    const bool connected = mqtt_is_connected();
    const uint32_t pubs  = mqtt_publish_count();
    const char *state_html = connected
        ? "<span style='color:#080'>connected</span>"
        : "<span style='color:#c00'>disconnected</span>";
    const char *broker_html = (s_cfg->mqtt_broker[0])
        ? s_cfg->mqtt_broker
        : "<i>(not set)</i>";

    // V2.4.6: TLS status row. Compact one-liner — full config lives on /config.
    const char *tls_html;
    if (!s_cfg->mqtt_tls_enable) {
        tls_html = "<span style='color:#888'>off (plain MQTT)</span>";
    } else {
        switch (s_cfg->mqtt_tls_mode) {
            case 0:  tls_html = "<span style='color:#080'>on &mdash; Mode A (Mozilla CA bundle)</span>"; break;
            case 1:  tls_html = "<span style='color:#080'>on &mdash; Mode B (custom CA cert)</span>";    break;
            case 2:  tls_html = "<span style='color:#c80'>on &mdash; Mode D (skip verification)</span>"; break;
            default: tls_html = "<span style='color:#c00'>on &mdash; unknown mode</span>";              break;
        }
    }

    snprintf(out, sz,
        "<div class=\"info\"><h3>MQTT</h3>"
        "<b>Broker:</b> %s:%lu<br>"
        "<b>State:</b> %s<br>"
        "<b>Publishes since boot:</b> %lu<br>"
        "<b>Topic prefix:</b> <code>%s</code><br>"
        "<b>HA Discovery:</b> %s<br>"
        "<b>TLS:</b> %s"
        "</div>",
        broker_html, (unsigned long)s_cfg->mqtt_port,
        state_html,
        (unsigned long)pubs,
        s_cfg->mqtt_topic_prefix[0] ? s_cfg->mqtt_topic_prefix : "<i>(empty)</i>",
        s_cfg->mqtt_ha_discovery ? "enabled" : "disabled",
        tls_html);
}

// --- Uploads block -----------------------------------------------------------
//
// Per-target succeeded/attempted + last_rc + breaker state, plus FTPS line.
// Targets that aren't enabled in config are omitted (keeps the block tight).
static bool target_enabled(tx_target_id_t id) {
    switch (id) {
        case TX_TARGET_MADAVI:  return s_cfg->send_madavi;
        case TX_TARGET_SENSORC: return s_cfg->send_sensorc;
        case TX_TARGET_RADMON:  return s_cfg->send_radmon;
        case TX_TARGET_OSM:     return s_cfg->send_osm;
        case TX_TARGET_OSM_STAGING: return s_cfg->send_osm_staging;
        case TX_TARGET_AQI:     return s_cfg->send_aqi;
        case TX_TARGET_GMC:        return s_cfg->send_gmc;
        case TX_TARGET_THINGSPEAK: return s_cfg->send_thingspeak;
        case TX_TARGET_THINGSPEAK_PM: return s_cfg->send_thingspeak_pm;
        default:                return false;
    }
}

// Defensive helper: snprintf into a buffer at offset n with bounds check.
// Returns updated n. Stops growing once the buffer is full (further calls
// no-op). Prevents the `sz - n` underflow if a caller miscounted.
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

// --- CPM history graph data (V2.5.6) -----------------------------------------
// Emits the minute + hour rings as a JS object for the inline SVG graph
// (STATUS_GRAPH). Empty when the tube is disabled — the dust node has no
// radiation history. Values are CPM, only the valid ring entries, oldest..newest.
static void format_history_data(char *out, size_t sz) {
    if (!tube_is_enabled()) { out[0] = 0; return; }
    history_snapshot_t h;
    history_get(&h);
    int n = append_safe(out, sz, 0, "<script>var H={min:[");
    for (uint8_t i = 0; i < h.min_count; i++)
        n = append_safe(out, sz, n, "%s%u", i ? "," : "", (unsigned)h.cpm_min[i]);
    n = append_safe(out, sz, n, "],hour:[");
    for (uint8_t i = 0; i < h.hour_count; i++)
        n = append_safe(out, sz, n, "%s%u", i ? "," : "", (unsigned)h.cpm_hour[i]);
    append_safe(out, sz, n, "]};</script>");
}

static void format_uploads(char *out, size_t sz) {
    int n = append_safe(out, sz, 0,
        "<div class=\"info\"><h3>Uploads</h3>"
        "<table class=u>"
        "<tr><th align=left>Target</th>"
        "<th class=ar>OK / Attempted</th>"
        "<th class=ar>Last rc</th>"
        "<th class=ar>Breaker</th></tr>");

    for (int i = 0; i < TX_TARGET_COUNT; i++) {
        if (!target_enabled(i)) continue;
        tx_target_stats_t s;
        tx_get_stats(i, &s);
        const char *breaker_html =
            (s.breaker_open_cycles == 0) ? "<span class=g>closed</span>" : NULL;
        char breaker_buf[80];
        if (!breaker_html) {
            snprintf(breaker_buf, sizeof(breaker_buf),
                     "<b class=o>open (%d cyc)</b>",
                     s.breaker_open_cycles);
            breaker_html = breaker_buf;
        }
        const char *rc_class =
            (s.attempted == 0)                            ? "d" :
            (s.last_rc >= 200 && s.last_rc < 300)         ? "g" :
            (s.last_rc == -1)                             ? "r" :
                                                            "o";
        char rc_buf[16];
        if (s.attempted == 0) snprintf(rc_buf, sizeof(rc_buf), "—");
        else                  snprintf(rc_buf, sizeof(rc_buf), "%d", s.last_rc);

        n = append_safe(out, sz, n,
            "<tr><td>%s</td>"
            "<td class=ar>%lu / %lu</td>"
            "<td class=\"ar %s\">%s</td>"
            "<td class=ar>%s</td></tr>",
            tx_target_name(i),
            (unsigned long)s.succeeded, (unsigned long)s.attempted,
            rc_class, rc_buf,
            breaker_html);
    }
    n = append_safe(out, sz, n, "</table>");

    // FTPS line — separate from the table because it has its own cadence.
    if (s_cfg->ftp_enabled) {
        log_ftp_stats_t f;
        log_ftp_get_stats(&f);
        char wall[32], ago[24], next_buf[32];
        if (f.have_last) {
            format_wallclock(f.last_at, wall, sizeof(wall));
            format_ago((int64_t)time(NULL), f.last_at, ago, sizeof(ago));
        } else {
            snprintf(wall, sizeof(wall), "never");
            ago[0] = 0;
        }
        // next_due_ms is monotonic uptime ms. Convert to a duration string
        // mirroring format_ago's H/M/S split (V2.3.24 — was "in Nseconds"
        // only, awkward at hourly cadence).
        unsigned long uptime_ms = (unsigned long)(esp_timer_get_time() / 1000LL);
        long remaining_ms = (long)(f.next_due_ms - uptime_ms);
        if (remaining_ms < 0) remaining_ms = 0;
        long rs = remaining_ms / 1000;
        if      (rs < 60)    snprintf(next_buf, sizeof(next_buf), "in %lds", rs);
        else if (rs < 3600)  snprintf(next_buf, sizeof(next_buf), "in %ldm %lds", rs / 60, rs % 60);
        else if (rs < 86400) snprintf(next_buf, sizeof(next_buf), "in %ldh %ldm", rs / 3600, (rs / 60) % 60);
        else                 snprintf(next_buf, sizeof(next_buf), "in %ldd %ldh", rs / 86400, (rs / 3600) % 24);

        const char *result_html;
        if (!f.have_last)         result_html = "—";
        else if (f.last_ok)       result_html = "<span class=g>OK</span>";
        else                      result_html = "<span class=r>FAIL</span>";

        n = append_safe(out, sz, n,
            "<br><b>FTP%s:</b> last %s%s%s &middot; %lu bytes &middot; result %s &middot; next %s",
            s_cfg->ftp_tls ? "S" : "",
            wall,
            f.have_last ? " (" : "",
            f.have_last ? ago  : "",
            (unsigned long)f.last_bytes,
            result_html,
            next_buf);
        if (f.have_last) {
            n = append_safe(out, sz, n, ")");
        }
    }
    append_safe(out, sz, n, "</div>");
}

// Favicon — radiation trefoil ☢ (U+2622) on yellow, served as SVG.
static const char s_favicon_svg[] =
    "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'>"
    "<rect width='100' height='100' rx='14' fill='#FFE000'/>"
    "<text x='50' y='82' font-size='80' text-anchor='middle' fill='#111'>"
    "\xe2\x98\xa2"   // UTF-8 for ☢
    "</text></svg>";

static esp_err_t favicon_get(httpd_req_t *req) {
    httpd_resp_set_type(req, "image/svg+xml");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=86400");
    return httpd_resp_send(req, s_favicon_svg, HTTPD_RESP_USE_STRLEN);
}

// PM sensor status block. Emits a self-contained <div class="info"> if a PM
// sensor is connected, otherwise an empty string (caller drops it cleanly into
// the page). Reads only from the cache populated by the cycle thread — never
// touches I²C from this HTTP-handler context.
static void format_pm_info(char *out, size_t sz) {
    if (!pm_sensor_present()) {
        out[0] = 0;
        return;
    }
    pm_sensor_status_t st;
    bool have_status = (pm_sensor_get_last_status(&st) == ESP_OK);
    pm_sample_t pm;
    bool have_sample = (pm_sensor_get_last_sample(&pm) == ESP_OK);

    // Pretty status badges with colour. Red for hard faults so they jump out;
    // orange for the soft "fan speed warning"; green for OK. CSS is inline
    // because the parent page only carries minimal styling.
    const char *fan_html =
        !have_status            ? "<span style='color:#888'>unknown</span>" :
        st.fan_fail             ? "<span style='color:#c00;font-weight:bold'>FAULT</span>" :
        st.fan_speed_warn       ? "<span style='color:#c80;font-weight:bold'>SPEED WARN</span>" :
                                  "<span style='color:#080'>OK</span>";
    const char *laser_html =
        !have_status            ? "<span style='color:#888'>unknown</span>" :
        st.laser_fail           ? "<span style='color:#c00;font-weight:bold'>FAULT</span>" :
                                  "<span style='color:#080'>OK</span>";

    int n = append_safe(out, sz, 0,
        "<div class=\"info\"><h3>Particulate matter</h3>"
        "<b>Sensor:</b> %s<br>"
        "<b>Fan:</b> %s<br>"
        "<b>Laser:</b> %s<br>"
        "<b>Status raw:</b> <code>0x%08lx</code><br>",
        pm_sensor_name(), fan_html, laser_html,
        have_status ? (unsigned long)st.raw : 0UL);

    if (have_sample) {
        n = append_safe(out, sz, n,
            "<b>PM1.0 / PM2.5 / PM4.0 / PM10:</b> "
            "%.1f / %.1f / %.1f / %.1f µg/m³<br>"
            "<b>Typical particle size:</b> %.2f µm<br>",
            pm.pm1_0, pm.pm2_5, pm.pm4_0, pm.pm10, pm.typ_size_um);
    } else {
        n = append_safe(out, sz, n,
            "<b>PM readings:</b> awaiting first cycle...<br>");
    }
    append_safe(out, sz, n, "</div>");
}

// Static page chrome — wrapper HTML that doesn't change between requests.
// Sent as string literals via httpd_resp_send_chunk → no per-request copy.
static const char STATUS_HEAD[] =
    "<!doctype html><html><head><meta charset=\"utf-8\">"
    "<link rel=\"icon\" type=\"image/svg+xml\" href=\"/favicon.ico\">"
    "<title>MultiGeiger</title>"
    "<style>body{font-family:sans-serif;max-width:680px;margin:20px auto;padding:0 10px}"
    "h1{color:#333}h3{margin:0 0 6px 0;color:#444}a{color:#0066cc}"
    ".info{background:#f5f5f5;border:1px solid #ddd;padding:10px;border-radius:4px;margin:10px 0}"
    ".u{border-collapse:collapse;font-size:.95em}.ar{text-align:right;padding-left:10px}"
    ".g{color:#080}.r{color:#c00}.o{color:#c80}.d{color:#888}"
    "</style></head><body>";
// V2.3.32: split into HEAD + TAIL so status_get can inject the optional
// per-chip Madavi graphs link between them when madavi uploads are enabled.
// Resolved once per page render (no JS): if Madavi is toggled on/off in
// /config, the link only appears/disappears on the next page load.
static const char STATUS_LINKS_HEAD[] =
    "<p><a href=\"/config\">&#9881; Configuration</a> (requires password)</p>"
    "<p><a href=\"/update\">&#11014; Firmware Update (OTA)</a> (requires password)</p>"
    "<p><a href=\"/log\">&#128221; View log buffer</a></p>";
static const char STATUS_LINKS_TAIL[] =
    "</body></html>";

// V2.5.6: CPM-history graph chrome — static SVG + hand-drawn JS (no external
// lib; offline device). Drawing runs in the browser; the device only serves
// this markup + the per-request data object `H` (emitted by format_history_data
// just before this chunk). `Hh` is the SVG height — must NOT shadow the data
// object `H`. Sent verbatim via send_chunk, so `%` is literal.
static const char STATUS_GRAPH[] =
    "<div class='info'><h3>CPM history</h3>"
    "<svg id='hg' viewBox='0 0 320 140' style='width:100%;height:auto;"
    "background:#111;border-radius:4px;font-family:sans-serif'>"
    "<g id='hgrid'></g><g id='hxlab'></g><g id='hylab'></g>"
    "<polyline id='hpl' fill='none' stroke='#4caf50' stroke-width='1.5' points=''/>"
    "<text id='hlbl' x='316' y='11' fill='#888' font-size='9' text-anchor='end'></text>"
    "</svg><br>"
    "<button onclick=\"hdraw('min')\">60 min</button> "
    "<button onclick=\"hdraw('hour')\">24 h</button>"
    "<script>function hdraw(s){"
    "var d=(s=='hour'?H.hour:H.min);"
    "var gr=document.getElementById('hgrid'),yl=document.getElementById('hylab'),"
    "xl=document.getElementById('hxlab'),pl=document.getElementById('hpl'),"
    "lb=document.getElementById('hlbl');"
    "if(!d||!d.length){lb.textContent='no data yet';pl.setAttribute('points','');"
    "gr.innerHTML='';yl.innerHTML='';xl.innerHTML='';return;}"
    "var mx=Math.max.apply(null,d),mn=Math.min.apply(null,d),sp=(mx-mn)||1,n=d.length;"
    "var W=320,Hh=140,L=24,R=6,T=10,B=20,x0=L,x1=W-R,y0=T,y1=Hh-B;"
    "function X(i){return x0+i*(x1-x0)/(n>1?n-1:1);}"
    "function Y(v){return y1-(v-mn)*(y1-y0)/sp;}"
    // horizontal gridlines at every 1 CPM (skipped when the range is huge so we
    // don't emit hundreds of <line>s); emphasise + label round steps.
    "var st=sp>40?10:sp>12?5:1,mok=sp<=45,g='',yt='';"
    "for(var v=Math.ceil(mn);v<=Math.floor(mx);v++){"
    "var bg=(v%st==0);if(!bg&&!mok)continue;var y=Y(v).toFixed(1);"
    "g+='<line x1=\"'+x0+'\" y1=\"'+y+'\" x2=\"'+x1+'\" y2=\"'+y+'\" stroke=\"#fff\" stroke-opacity=\"'+(bg?0.18:0.05)+'\" stroke-width=\"0.5\"/>';"
    "if(bg)yt+='<text x=\"'+(x0-3)+'\" y=\"'+(Y(v)+3).toFixed(1)+'\" fill=\"#888\" font-size=\"8\" text-anchor=\"end\">'+v+'</text>';"
    "}"
    "gr.innerHTML=g;yl.innerHTML=yt;"
    "pl.setAttribute('points',d.map(function(v,i){return X(i).toFixed(1)+','+Y(v).toFixed(1);}).join(' '));"
    // x-axis reading times — newest point = now (viewer's clock), evenly spaced.
    "var ms=(s=='hour'?3600000:60000),nw=Date.now(),xt='',tk=Math.min(5,n);"
    "for(var k=0;k<tk;k++){"
    "var i=Math.round(k*(n-1)/(tk>1?tk-1:1)),t=new Date(nw-(n-1-i)*ms);"
    "var hh=('0'+t.getHours()).slice(-2)+':'+('0'+t.getMinutes()).slice(-2),xx=X(i).toFixed(1);"
    "xt+='<line x1=\"'+xx+'\" y1=\"'+y0+'\" x2=\"'+xx+'\" y2=\"'+y1+'\" stroke=\"#fff\" stroke-opacity=\"0.05\" stroke-width=\"0.5\"/>';"
    "xt+='<text x=\"'+xx+'\" y=\"'+(Hh-7)+'\" fill=\"#888\" font-size=\"8\" text-anchor=\"middle\">'+hh+'</text>';"
    "}"
    "xl.innerHTML=xt;"
    "lb.textContent=(s=='hour'?'last '+n+' h':'last '+n+' min');}"
    "hdraw('min');</script></div>";

// Streaming-friendly send: skip if format function emitted nothing (the
// "block hidden" case for radiation/env/noise/PM). Returns false on
// transport error — caller bails immediately.
static bool send_block(httpd_req_t *req, const char *buf) {
    size_t len = strlen(buf);
    if (len == 0) return true;
    return httpd_resp_send_chunk(req, buf, len) == ESP_OK;
}

static esp_err_t status_get(httpd_req_t *req) {
    log_access(req, "GET /");
    // Single shared scratch buffer reused across all blocks. Sized for the
    // worst-case block (uploads with 5 enabled targets ≈ 1.4 KB). Stream each
    // chunk via httpd chunked transfer-encoding — same pattern V2.3.17 used
    // for /log to avoid heap-pressure transient peaks.
    //
    // V2.4.22: moved from stack to BSS. 1.6 KB on the httpd task's 8 KB
    // stack stacks with config_get's escape buffers and format_system's
    // cd_summary/cd_line on adjacent /status renders. Safe because
    // esp_http_server uses a single thread (httpd_thread runs all URI
    // handlers serially), so this handler is never re-entered; the entire
    // function fills buf then sends each block before next overwrite.
    static char buf[1600];
    int64_t       now_us    = esp_timer_get_time();
    // uptime_ms stays crystal-based (esp_timer) because format_cycle compares
    // it against last_cycle_ms which is also esp_timer — mixing time bases
    // there would corrupt the "Next cycle in Xs" display.
    unsigned long uptime_ms = (unsigned long)(now_us / 1000LL);
    // Capture wall clock once so "clock now" in format_system and "Started +
    // Uptime" below are always rooted in the same timestamp.
    time_t        now_wall   = time(NULL);
    time_t        boot_ep    = ntp_boot_epoch();
    time_t        uptime_t   = boot_ep
        ? now_wall - boot_ep
        : (time_t)(now_us / 1000000LL);
    unsigned long uptime_s   = uptime_t > 0 ? (unsigned long)uptime_t : 0UL;

    httpd_resp_set_type(req, "text/html; charset=utf-8");

    if (httpd_resp_send_chunk(req, STATUS_HEAD, sizeof(STATUS_HEAD) - 1) != ESP_OK) goto fail;

    // Title — small, dynamic, fits trivially.
    int n = snprintf(buf, sizeof(buf), "<h1>&#9762; MultiGeiger %s</h1>", VERSION_STR);
    if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) goto fail;

    format_device     (buf, sizeof(buf)); if (!send_block(req, buf)) goto fail;
    format_net_info   (buf, sizeof(buf)); if (!send_block(req, buf)) goto fail;
    format_system     (buf, sizeof(buf), uptime_s, now_wall);  if (!send_block(req, buf)) goto fail;
    format_cycle      (buf, sizeof(buf), uptime_ms); if (!send_block(req, buf)) goto fail;
    format_radiation  (buf, sizeof(buf)); if (!send_block(req, buf)) goto fail;
    // V2.5.6: CPM-history graph — radiation nodes only. Data object first, then
    // the static SVG+JS chrome that draws it.
    if (tube_is_enabled()) {
        format_history_data(buf, sizeof(buf)); if (!send_block(req, buf)) goto fail;
        if (httpd_resp_send_chunk(req, STATUS_GRAPH, sizeof(STATUS_GRAPH) - 1) != ESP_OK) goto fail;
    }
    format_environment(buf, sizeof(buf)); if (!send_block(req, buf)) goto fail;
    format_als        (buf, sizeof(buf)); if (!send_block(req, buf)) goto fail;
    format_gnss       (buf, sizeof(buf)); if (!send_block(req, buf)) goto fail;
    format_noise      (buf, sizeof(buf)); if (!send_block(req, buf)) goto fail;
    format_pm_info    (buf, sizeof(buf)); if (!send_block(req, buf)) goto fail;
    format_uploads    (buf, sizeof(buf)); if (!send_block(req, buf)) goto fail;
    format_mqtt       (buf, sizeof(buf)); if (!send_block(req, buf)) goto fail;

    if (httpd_resp_send_chunk(req, STATUS_LINKS_HEAD, sizeof(STATUS_LINKS_HEAD) - 1) != ESP_OK) goto fail;

    // V2.3.32: optional per-chip Madavi graph link. Only emitted if the
    // user has Madavi uploads enabled — there's no point linking to a graph
    // page that has no data behind it. The dashboard ID + path are baked in
    // from the user's confirmed working URL on 2026-05-16; if Madavi ever
    // restructures the Grafana URL space we'll need to refresh it here.
    if (s_cfg && s_cfg->send_madavi && s_chip_id && s_chip_id[0]) {
        n = snprintf(buf, sizeof(buf),
            "<p><a href=\"https://api-rrd.madavi.de:3000/grafana/d/q87EBfWGk/"
            "temperature-humidity-pressure?var-chipID=%s\" target=\"_blank\" "
            "rel=\"noopener\">&#128202; Madavi graphs for %s</a></p>",
            s_chip_id, s_chip_id);
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) goto fail;
    }

    if (httpd_resp_send_chunk(req, STATUS_LINKS_TAIL, sizeof(STATUS_LINKS_TAIL) - 1) != ESP_OK) goto fail;
    httpd_resp_send_chunk(req, NULL, 0);   // end-of-stream sentinel
    return ESP_OK;
fail:
    // Best-effort terminate the chunked stream so the socket isn't half-open.
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_FAIL;
}

// --- GET /config (auth'd form) -----------------------------------------------

// Bumped from 6144 in V2.3.3 — adding the openSenseMap + aqi.eco rows pushed
// the worst-case formatted length past the old ceiling. V2.3.33: bumped 8192
// → 16384 after silent snprintf truncation surfaced as missing page tail on
// devices with longer field values (longer SSIDs/FTP paths/tokens consumed
// the remaining budget; cut point varied per board because field lengths
// varied). 16 KB gives ~8 KB headroom; truncation is now also logged at
// ERROR level (see config_get) so any future near-miss is loud, not silent.
//
// V2.4.6: per-board buffer size — moved to hal.h. Heltec V2 stays at 16 KB
// (tight internal-DRAM budget), FeatherS3-D / QT Py bump to 32 KB to leave
// room for the MQTT TLS PEM textarea + future config sections without
// stressing heap on PSRAM-backed boards.
#define CFG_FORM_BUF_SIZE HAL_CFG_FORM_BUF_SIZE

static esp_err_t config_get(httpd_req_t *req) {
    log_access(req, "GET /config");
    if (!check_auth(req)) return ESP_OK;

    char *body = malloc(CFG_FORM_BUF_SIZE);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_OK;
    }

    // Escape every string field for safe use in value="..." attributes.
    //
    // V2.4.22: all e_* arrays moved from stack to BSS — collectively
    // ~4.3 KB, over half the httpd task's 8 KB stack budget when
    // simultaneously in scope. Same V2.4.20 fix pattern. Safe because
    // esp_http_server uses a single thread (httpd_thread runs all URI
    // handlers serially via select), so config_get is never re-entered.
    // Each html_esc() call below fully overwrites its target buffer
    // before any reader (the final big snprintf) sees it. BSS cost
    // ~4.3 KB once, replacing 4.3 KB off every config_get invocation.
    static char e_ssid[96], e_pw[192], e_chip[96], e_ru[96], e_rp[192];
    static char e_ntp1[192], e_ntp2[192], e_ntp3[192], e_ap[96];
    static char e_tz[160];
    static char e_apn[96], e_host[96];
    static char e_fhost[192], e_fuser[96], e_fpw[192], e_fpath[192];
    static char e_osm[80], e_osm_tok[160], e_aqi[160];
    static char e_osm_st[80], e_osm_st_tok[160];   // V2.5.26: OSM staging
    // V2.4.4: MQTT fields. e_mhost generously sized — html_esc 4x worst case
    // (every byte → "&amp;" or similar) over CFG_MQTT_HOST_MAX=63 = ~256;
    // e_mpfx similarly over CFG_MQTT_PFX_MAX=31.
    static char e_mhost[256], e_muser[160], e_mpw[256], e_mpfx[128];
    // V2.4.15: syslog host esc buffer. CFG_SYSLOG_HOST_MAX=63 × 4 = ~256.
    static char e_slh[256];
    // V2.4.6: MQTT TLS PEM cert. html_esc worst-case is ~6x for a textarea
    // because every '<' becomes "&lt;" / '"' becomes "&quot;" / '&' becomes
    // "&amp;" — but real PEM is mostly base64 alnum (no escaping needed) +
    // line markers. 8 KB headroom over CFG_MQTT_CA_CERT_MAX=2400 covers
    // even pathological inputs without truncation.
    char *e_mca = malloc(8192);
    if (!e_mca) {
        free(body);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_OK;
    }
    html_esc(s_cfg->wifi_ssid,     e_ssid, sizeof(e_ssid));
    html_esc(s_cfg->wifi_password, e_pw,   sizeof(e_pw));
    html_esc(s_chip_id,            e_chip, sizeof(e_chip));  // read-only display
    html_esc(s_cfg->radmon_user,   e_ru,   sizeof(e_ru));
    html_esc(s_cfg->radmon_password, e_rp, sizeof(e_rp));
    html_esc(s_cfg->ntp_server,    e_ntp1, sizeof(e_ntp1));
    html_esc(s_cfg->ntp_server2,   e_ntp2, sizeof(e_ntp2));
    html_esc(s_cfg->ntp_server3,   e_ntp3, sizeof(e_ntp3));
    html_esc(s_cfg->tz_posix,      e_tz,   sizeof(e_tz));
    html_esc(s_cfg->ap_password,   e_ap,   sizeof(e_ap));
    html_esc(s_cfg->ap_name,       e_apn,  sizeof(e_apn));
    html_esc(s_cfg->wifi_hostname, e_host, sizeof(e_host));
    html_esc(s_cfg->ftp_host,      e_fhost, sizeof(e_fhost));
    html_esc(s_cfg->ftp_user,      e_fuser, sizeof(e_fuser));
    html_esc(s_cfg->ftp_password,  e_fpw,   sizeof(e_fpw));
    html_esc(s_cfg->ftp_path,      e_fpath, sizeof(e_fpath));
    html_esc(s_cfg->osm_box_id,       e_osm,     sizeof(e_osm));
    html_esc(s_cfg->osm_access_token, e_osm_tok, sizeof(e_osm_tok));
    html_esc(s_cfg->osm_staging_box_id, e_osm_st,     sizeof(e_osm_st));
    html_esc(s_cfg->osm_staging_token,  e_osm_st_tok, sizeof(e_osm_st_tok));
    html_esc(s_cfg->aqi_token,        e_aqi,     sizeof(e_aqi));
    // V2.5.1: GMCMap + ThingSpeak. Numeric/hex fields, but escape for safety.
    char e_gmc_aid[CFG_USER_NAME_MAX * 3 + 4];
    char e_gmc_gid[CFG_USER_NAME_MAX * 3 + 4];
    char e_ts_key [CFG_TOKEN_MAX     * 3 + 4];
    char e_ts_pm_key [CFG_TOKEN_MAX  * 3 + 4];   // V2.5.4: ThingSpeak PM key
    html_esc(s_cfg->gmc_account_id,     e_gmc_aid, sizeof(e_gmc_aid));
    html_esc(s_cfg->gmc_geiger_id,      e_gmc_gid, sizeof(e_gmc_gid));
    html_esc(s_cfg->thingspeak_api_key, e_ts_key,  sizeof(e_ts_key));
    html_esc(s_cfg->thingspeak_pm_api_key, e_ts_pm_key, sizeof(e_ts_pm_key));
    html_esc(s_cfg->mqtt_broker,       e_mhost, sizeof(e_mhost));
    html_esc(s_cfg->mqtt_user,         e_muser, sizeof(e_muser));
    html_esc(s_cfg->mqtt_password,     e_mpw,   sizeof(e_mpw));
    html_esc(s_cfg->mqtt_topic_prefix, e_mpfx,  sizeof(e_mpfx));
    html_esc(s_cfg->mqtt_tls_ca,       e_mca,   8192);
    html_esc(s_cfg->syslog_host,       e_slh,   sizeof(e_slh));

    // V2.3.30: build the display-brightness <option> list dynamically — OFF
    // (0 %) followed by 10 % through 100 % in 10 % steps. Builder keeps the
    // main snprintf args list sane (one extra %s instead of eleven "selected"
    // vs "" args). V2.3.32: added OFF entry — turns the SerLCD backlight
    // fully dark (panel pixels still drawn but invisible) and powers the
    // OLED panel down via 0xAE (zero-current state). See display_set_contrast.
    // V2.4.22: br_opts also static — same justification as the e_* block above.
    static char br_opts[512];
    int br_n = 0;
    br_n += snprintf(br_opts + br_n, sizeof(br_opts) - br_n,
                     "<option value=\"0\"%s>OFF</option>",
                     (s_cfg->oled_brightness_pct == 0) ? " selected" : "");
    for (int v = 10; v <= 100; v += 10) {
        br_n += snprintf(br_opts + br_n, sizeof(br_opts) - br_n,
                         "<option value=\"%d\"%s>%d%%</option>",
                         v,
                         (s_cfg->oled_brightness_pct == v) ? " selected" : "",
                         v);
    }

    // V2.6.1: tube-type dropdown options built from the shared tube_types table
    // (single source of truth) instead of hardcoded <option> lines + per-option
    // selected-marker args. append_safe keeps the accumulation truncation-safe.
    char tube_opts[384];
    int tube_n = 0;
    for (uint32_t t = 0; t < TUBE_TYPE_COUNT; t++) {
        tube_n = append_safe(tube_opts, sizeof(tube_opts), tube_n,
                             "<option value=\"%lu\"%s>%s%s</option>",
                             (unsigned long)t,
                             (s_cfg->tube_type == t) ? " selected" : "",
                             tube_type_name(t), tube_type_menu_suffix(t));
    }

    int n = snprintf(body, CFG_FORM_BUF_SIZE,
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<title>Config — MultiGeiger V2</title>"
        "<style>body{font-family:system-ui;max-width:40em;margin:2em auto;padding:0 1em}"
        "label{display:block;margin-top:.8em}"
        "input[type=text],input[type=password],input[type=number]{width:100%%;padding:.4em;box-sizing:border-box}"
        "input[type=submit]{padding:.6em 1.2em;margin-top:1.2em;font-size:1em}"
        ".chk{display:inline-block;margin-right:1em;margin-top:.4em}"
        // V2.5.7: indent each TX target's config fields under its enable checkbox.
        ".cfg{margin:.1em 0 .9em 1.7em;padding-left:.8em;border-left:2px solid #ccc}"
        ".cfg label{margin-top:.5em}"
        // V2.3.24: browsers' user-agent stylesheet shrinks <code> to ~85 % of
        // body text. Override so the chip-id / MAC values stay visually equal
        // to the surrounding label text — monospace is the distinguishing cue,
        // not size.
        "code{font-size:1em}"
        // V2.3.24: red asterisk class for "requires reboot" field markers.
        // Paired with two submit buttons (Save / Save and restart) so the
        // user picks the right one based on whether they touched any of the
        // marked fields.
        ".r{color:#c00;font-weight:bold}"
        // Stack the two submit buttons with a small gap; suppress the
        // .8em label-style top margin browsers apply when an input lives
        // inside a non-label parent.
        ".btns{margin-top:1.2em}.btns input{margin-right:.6em}</style>"
        "</head><body><h1>Configuration</h1>"
        "<p><span class=\"r\">*</span> requires reboot to take effect &mdash; "
        "use <b>Save and restart</b> at the bottom when changing these.</p>"
        "<form method=\"post\" action=\"/config\">"
        "<h3>Network</h3>"
        "<label>WiFi SSID <span class=\"r\">*</span>"
        "<input type=\"text\" name=\"wifi_ssid\" value=\"%s\"></label>"
        "<label>WiFi password <span class=\"r\">*</span>"
        "<input type=\"password\" name=\"wifi_pw\" value=\"%s\"></label>"
        "<label>DHCP hostname (visible in router) <span class=\"r\">*</span>"
        "<input type=\"text\" name=\"wifi_host\" value=\"%s\" maxlength=\"32\"></label>"
        "<label>AP SSID (used in AP / fallback mode) <span class=\"r\">*</span>"
        "<input type=\"text\" name=\"ap_name\" value=\"%s\" maxlength=\"32\"></label>"
        // V2.5.30: moved here (below AP SSID) from the old "Other" section.
        "<label>Web admin and access point password"
        "<input type=\"password\" name=\"ap_pw\" value=\"%s\"></label>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"wifi_11bg\" "
        "id=\"wifi_11bg\" onchange=\"syncHt20()\" %s> Limit to 802.11b/g <span class=\"r\">*</span></label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"wifi_ht20\" "
        "id=\"wifi_ht20\" %s> Limit to 20MHz <span class=\"r\">*</span></label></div>"
        "<script>function syncHt20(){"
        "var bg=document.getElementById('wifi_11bg');"
        "var ht=document.getElementById('wifi_ht20');"
        "if(bg.checked){ht.checked=true;ht.disabled=true;}"
        "else{ht.disabled=false;}"
        "}syncHt20();</script>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"wifi_ps_dis\" "
        "id=\"wifi_ps_dis\" onchange=\"syncFtpPs()\" %s> "
        "Disable WiFi power save (always-on radio; may reduce mesh re-keying drops) "
        "<span class=\"r\">*</span></label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"wifi_ext_a\" "
        "id=\"wifi_ext_a\" %s %s> Use External Antenna Port <span class=\"r\">*</span>"
        "%s</label></div>"
        "<p>Chip ID (auto-derived from MAC): <code>%s</code><br>"
        "MAC: <code>%s</code></p>"
        "<h3>Hardware</h3>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"tube_en\" "
        "id=\"tube_en\" onchange=\"syncTube()\" %s> "
        "Enable Geiger tube (HV pump, pulse counter, radiation uploads). "
        "Uncheck for non-Geiger deployments &mdash; disables HV/ISR/gptimer at boot "
        "and skips Madavi geiger POST, sensor.community X-PIN 19, and Radmon. "
        "<span class=\"r\">*</span></label></div>"
        // V2.6.1: tube type — picks the cps→µSv/h dose conversion factor (the only
        // tube-dependent term; CPM is unchanged). Options are pre-built into
        // `tube_opts` from the shared tube_types table above. Live-applied (no `*`):
        // dose is recomputed each cycle from the saved tube_type.
        "<label>Geiger tube type "
        "<select name=\"tube_type\" id=\"tube_type\">"
        "%s"
        "</select> <small>(sets the CPM&rarr;&micro;Sv/h factor; CPM unaffected; "
        "live on Save)</small></label>"
        // V2.5.16: PCNT pulse-width filter — indented under the tube enable like
        // the TX sub-options, and tube-gated via syncTube() (greyed when the
        // tube is off; it can't run without count pulses).
        "<div class=\"cfg\">"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"pcnt_filt\" "
        "id=\"pcnt_filt\" onchange=\"syncGuard()\" %s> "
        "PCNT pulse-width filter &mdash; drops count-line pulses narrower than the "
        "width below (removes the ESP32-S3 narrow-pulse over-count). When ON it "
        "<b>changes the counted CPM</b> (dose/uploads use the filtered count); the log "
        "still shows the full <code>DIAG</code>/<code>PCNT</code> data and a "
        "<code>FILTER:</code> line with the pre-filter CPM. "
        "<span class=\"r\">*</span></label></div>"
        "<label>Filter width (ns, 250&ndash;12000; ~4000 = 4&micro;s) "
        "<input type=\"text\" inputmode=\"numeric\" name=\"pcnt_filt_w\" "
        "id=\"pcnt_filt_w\" value=\"%lu\"> <span class=\"r\">*</span></label>"
        // V2.5.30: dead-time guard — checkbox + window, indented under the tube
        // enable alongside the PCNT filter. MUTUALLY EXCLUSIVE with PCNT: syncGuard()
        // greys+unticks this when PCNT is on (pcnt_filter wins — it makes the PCNT
        // hardware path authoritative, bypassing the ISR guard). Live-applied (no
        // `*`). Time-domain twin of the width filter: collapses 1-5ms re-triggers.
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"dt_guard\" "
        "id=\"dt_guard\" %s> "
        "Dead-time guard &mdash; collapses 1-5ms afterpulse/re-trigger trains to one "
        "count (time domain; reaches what the width filter can't). <b>Mutually "
        "exclusive with the PCNT width filter above</b> &mdash; if PCNT is on it "
        "takes the count and this is forced off.</label></div>"
        "<label>Guard window (&micro;s, 200&ndash;20000; ~3000 typical) "
        "<input type=\"text\" inputmode=\"numeric\" name=\"dt_guard_us\" "
        "id=\"dt_guard_us\" value=\"%lu\"></label></div>"
        // V2.5.19: I²C pin-out route toggle. Board-gated like the antenna switch
        // (greyed + force-off on boards without HAL_HAS_I2C_PINOUT_SWITCH). 3 %s
        // slots: disabled-attr, checked-attr, trailing note.
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"i2c_pinout\" "
        "id=\"i2c_pinout\" %s %s> Route I&sup2;C to the pin-out pads "
        "(QT Py: SDA/SCL pads IO4/IO33 instead of the STEMMA QT connector) "
        "<span class=\"r\">*</span>%s</label></div>"
        // V2.5.30: heap-guard floor moved here to the BOTTOM of the Hardware
        // section (was in "Other"). No asterisk — read live each TX cycle by
        // tx_heap_guard() (V2.5.18), so it applies on plain Save (no reboot).
        "<label>Heap-guard auto-reboot floor (KB, 0 = off)"
        "<input type=\"text\" inputmode=\"numeric\" name=\"heap_guard\" value=\"%lu\"></label>"
        // V2.5.33: confirm window now configurable (was a hard-coded 5).
        "<label>Heap-guard confirm cycles"
        "<input type=\"text\" inputmode=\"numeric\" name=\"hg_confirm\" value=\"%lu\"></label>"
        // V2.5.10: GNSS receiver is auto-detected at boot (no toggle) — a
        // MAX-M10S (0x42) or PA1010D (0x10) is found automatically; nothing to
        // configure here. See the "GNSS / Position" card on /status.
        // V2.5.7: each target = main enable (+ inline HTTPS) at the left margin,
        // with its config fields indented in a .cfg block. Station altitude +
        // sea-level toggle live under sensor.community (its only consumer — the
        // old standalone "BME280 (environmental)" section was retired).
        "<h3>Transmission targets</h3>"
        // V2.5.30: TX interval moved here to the TOP of Transmission targets
        // (was in "Other").
        "<label>Sensor data upload interval (ms) <span class=\"r\">*</span>"
        "<input type=\"text\" inputmode=\"numeric\" name=\"tx_int_ms\" value=\"%lu\"></label>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_mad\" %s> Madavi</label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"mad_https\" %s> HTTPS</label></div><br>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_sc\" %s> sensor.community</label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"sc_https\" %s> HTTPS</label></div>"
        "<div class=\"cfg\">"
        "<label>Station altitude (m above sea level)"
        "<input type=\"text\" inputmode=\"decimal\" name=\"alt_m\" value=\"%.1f\"></label>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_sl\" %s> "
        "Send pressure-at-sealevel</label></div></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_rad\" id=\"send_rad\" %s> "
        "Radmon &mdash; radiation-only</label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"rad_https\" %s> HTTPS</label></div>"
        "<div class=\"cfg\">"
        "<label>Radmon user<input type=\"text\" name=\"rad_user\" value=\"%s\"></label>"
        "<label>Radmon password<input type=\"password\" name=\"rad_pw\" value=\"%s\"></label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_osm\" %s> "
        "openSenseMap (HTTPS only)</label></div>"
        "<div class=\"cfg\">"
        "<label>Box ID (24-char MongoDB ObjectId &mdash; per-device on opensensemap.org)"
        "<input type=\"text\" name=\"osm_box\" value=\"%s\" maxlength=\"25\"></label>"
        "<label>Access Token (optional &mdash; only if your box has authentication enabled)"
        "<input type=\"password\" name=\"osm_tok\" value=\"%s\" maxlength=\"64\"></label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_osm_st\" %s> "
        "openSenseMap STAGING (HTTPS only)</label></div>"
        "<div class=\"cfg\">"
        "<label>Staging Box ID (beta &mdash; per-device on staging.opensensemap.org)"
        "<input type=\"text\" name=\"osm_st_box\" value=\"%s\" maxlength=\"25\"></label>"
        "<label>Staging Access Token (optional)"
        "<input type=\"password\" name=\"osm_st_tok\" value=\"%s\" maxlength=\"64\"></label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_aqi\" %s> "
        "aqi.eco (HTTPS only)</label></div>"
        "<div class=\"cfg\">"
        "<label>aqi.eco token"
        "<input type=\"text\" name=\"aqi_tok\" value=\"%s\" maxlength=\"64\"></label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_gmc\" id=\"send_gmc\" %s> "
        "GMCMap (gmcmap.com, HTTP only) &mdash; radiation-only</label></div>"
        "<div class=\"cfg\">"
        "<label>Account ID"
        "<input type=\"text\" name=\"gmc_aid\" value=\"%s\" maxlength=\"32\"></label>"
        "<label>Geiger Counter ID"
        "<input type=\"text\" name=\"gmc_gid\" value=\"%s\" maxlength=\"32\"></label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_ts\" id=\"send_ts\" %s> "
        "ThingSpeak &mdash; radiation-only</label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"ts_https\" %s> HTTPS</label></div>"
        "<div class=\"cfg\">"
        "<label>Channel Write API Key"
        "<input type=\"password\" name=\"ts_key\" value=\"%s\" maxlength=\"64\"></label></div>"
        // V2.5.4: ThingSpeak (Particulate Matter) — independent channel for the
        // SPS30 dust node. field1-4=PM1.0/2.5/4.0/10, 5-7=T/H/P, 8=typ. size.
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_ts_pm\" %s> "
        "ThingSpeak (Particulate Matter) &mdash; SPS30 dust node (separate channel)</label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"ts_pm_https\" %s> HTTPS</label></div>"
        "<div class=\"cfg\">"
        "<label>Channel Write API Key"
        "<input type=\"password\" name=\"ts_pm_key\" value=\"%s\" maxlength=\"64\"></label></div>"
        "<h3>FTP log upload</h3>"
        "<p>Periodically uploads the in-memory log ring (same content as "
        "<a href=\"/log\">/log</a>) to a LAN FTP server. Passive mode. "
        "Leave user/password empty for anonymous login.</p>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"ftp_en\" %s> "
        "Enable FTP log upload</label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"ftp_tls\" %s> "
        "Use explicit TLS (AUTH TLS on port 21) &mdash; certificate NOT verified</label></div>"
        "<label>FTP host (or host:port if non-default)<input type=\"text\" name=\"ftp_host\" value=\"%s\"></label>"
        "<label>FTP user (blank = anonymous)<input type=\"text\" name=\"ftp_user\" value=\"%s\"></label>"
        "<label>FTP password<input type=\"password\" name=\"ftp_pw\" value=\"%s\"></label>"
        "<label>Remote directory (e.g. /geiger)<input type=\"text\" name=\"ftp_path\" value=\"%s\"></label>"
        "<label>Upload interval (minutes) (Max 10090)<input type=\"text\" inputmode=\"numeric\" name=\"ftp_int\" value=\"%lu\"></label>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"ftp_ps_dis\" "
        "id=\"ftp_ps_dis\" %s> Disable WiFi power save during FTP transfer "
        "(prevents DTIM-delayed TCP ACKs; auto-cleared if WiFi PS is already disabled above)</label></div>"
        "<script>function syncFtpPs(){"
        "var w=document.getElementById('wifi_ps_dis');"
        "var f=document.getElementById('ftp_ps_dis');"
        "if(w.checked){f.checked=false;f.disabled=true;}"
        "else{f.disabled=false;}"
        "}syncFtpPs();"
        // V2.5.4: radiation upload targets are meaningless without the tube.
        // Grey + force-uncheck Radmon/GMCMap/ThingSpeak when "Enable Geiger
        // tube" is off (server-side enforcement in config_post mirrors this).
        "function syncTube(){"
        "var t=document.getElementById('tube_en');"
        "var a=['send_rad','send_gmc','send_ts','pcnt_filt','pcnt_filt_w'];"
        "for(var i=0;i<a.length;i++){var e=document.getElementById(a[i]);"
        "if(!e)continue;"
        "if(t.checked){e.disabled=false;}"
        "else{e.checked=false;e.disabled=true;}}"
        "syncGuard();"
        "}"
        // V2.5.30: dead-time guard is mutually exclusive with the PCNT width filter
        // (pcnt_filter wins — it makes the PCNT path authoritative, bypassing the
        // ISR guard). Greyed + unticked when the tube is off OR PCNT is on. Mirrors
        // the server-side force-clear in config_post.
        "function syncGuard(){"
        "var t=document.getElementById('tube_en');"
        "var p=document.getElementById('pcnt_filt');"
        "var g=document.getElementById('dt_guard');"
        "var gw=document.getElementById('dt_guard_us');"
        "if(!t.checked||p.checked){g.checked=false;g.disabled=true;gw.disabled=true;}"
        "else{g.disabled=false;gw.disabled=false;}"
        "}"
        "syncTube();</script>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"ftp_t12only\" %s> "
        "Limit FTPS to TLS 1.2 (only tick if your FTPS server can't handle TLS 1.3)</label></div>"
        "<h3>MQTT (Home Assistant / Mosquitto)</h3>"
        "<p>Publish-only MQTT 3.1.1 client. Per TX cycle, sends one JSON state "
        "message to <code>&lt;prefix&gt;/&lt;chip-id&gt;/state</code> with all "
        "present-sensor readings. If the device drops abruptly (power loss, "
        "WiFi failure, crash), the broker publishes <code>offline</code> to "
        "<code>&lt;prefix&gt;/&lt;chip-id&gt;/availability</code> on the "
        "device's behalf (MQTT Last-Will-and-Testament), so Home Assistant "
        "flips the entity to <i>unavailable</i> within ~90 seconds instead "
        "of showing the stale last reading as fresh. If <i>HA Discovery</i> "
        "is on, retained config payloads land at "
        "<code>homeassistant/sensor/geiger_&lt;chip-id&gt;/&hellip;/config</code> "
        "on every reconnect so Home Assistant auto-creates the entities.</p>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"mqtt_en\" %s> "
        "Enable MQTT publishing <span class=\"r\">*</span></label></div>"
        "<label>Broker host (or IP) <span class=\"r\">*</span>"
        "<input type=\"text\" name=\"mqtt_brk\" value=\"%s\" maxlength=\"63\"></label>"
        "<label>Broker port <span class=\"r\">*</span>"
        "<input type=\"text\" inputmode=\"numeric\" name=\"mqtt_port\" value=\"%lu\"></label>"
        "<label>Username (optional)"
        "<input type=\"text\" name=\"mqtt_user\" value=\"%s\" maxlength=\"32\"></label>"
        "<label>Password (optional)"
        "<input type=\"password\" name=\"mqtt_pw\" value=\"%s\" maxlength=\"64\"></label>"
        "<label>Topic prefix <span class=\"r\">*</span>"
        "<input type=\"text\" name=\"mqtt_pfx\" value=\"%s\" maxlength=\"31\"></label>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"mqtt_ha\" %s> "
        "Publish Home Assistant Discovery payloads "
        "<span class=\"r\">*</span></label></div>"
        // V2.4.6: MQTT TLS rows. Master enable + mode dropdown + CA textarea.
        // PEM textarea is conditionally relevant (only Mode B uses it) but
        // always rendered to keep the form layout stable across mode changes;
        // an unused PEM is just dead bytes in NVS.
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"mqtt_tls\" %s> "
        "Use TLS to broker (mqtts:// — change port to 8883 if applicable) "
        "<span class=\"r\">*</span></label></div>"
        "<label>TLS trust mode <span class=\"r\">*</span>"
        "<select name=\"mqtt_tls_m\">"
        "<option value=\"0\"%s>A &mdash; Mozilla CA bundle (Let&#39;s Encrypt etc.)</option>"
        "<option value=\"1\"%s>B &mdash; Custom CA cert (paste PEM below)</option>"
        "<option value=\"2\"%s>D &mdash; Skip server verification (LAN only!)</option>"
        "</select></label>"
        "<label>CA cert PEM (Mode B only) <span class=\"r\">*</span>"
        "<textarea name=\"mqtt_tls_ca\" rows=\"8\" maxlength=\"2400\" "
        "style=\"width:100%%;font-family:monospace;font-size:0.85em;box-sizing:border-box\" "
        "placeholder=\"-----BEGIN CERTIFICATE-----&#10;...&#10;-----END CERTIFICATE-----\""
        ">%s</textarea></label>"
        // V2.4.15: UDP syslog client. Plaintext UDP — LAN-only. rsyslog on
        // the broker host (or any syslog receiver) catches every ESP_LOG
        // line emitted after STA connect. Boot logs stay in the on-device
        // ring buffer (visible via /log). See [[reference_syslog_pi_setup]].
        "<h3>Syslog (UDP)</h3>"
        "<p style=\"font-size:0.85em;color:#666;line-height:1.4\">"
        "Per-line UDP shipping (RFC 5424) of every device log entry to a LAN "
        "syslog server (e.g. <code>rsyslog</code> on the same Pi running the "
        "MQTT broker). Plaintext — for trusted-LAN use only. Tiny heap "
        "footprint (~0 KB persistent) and zero retry/buffer state, making it "
        "the cheapest log-shipping path on Heltec V2.</p>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"syslog_en\" %s> "
        "Enable syslog forwarding <span class=\"r\">*</span></label></div>"
        "<label>Syslog server host (or IP) <span class=\"r\">*</span>"
        "<input type=\"text\" name=\"syslog_h\" value=\"%s\" maxlength=\"63\"></label>"
        "<label>Syslog UDP port <span class=\"r\">*</span>"
        "<input type=\"text\" inputmode=\"numeric\" name=\"syslog_p\" value=\"%lu\"></label>"
        "<h3>Tick, LED and display</h3>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"sp_tick\" %s> "
        "Speaker tick on each GM pulse <span class=\"r\">*</span></label></div><br>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"led_tick\" %s> "
        "LED flash on each GM pulse <span class=\"r\">*</span></label></div><br>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"play_sound\" %s> "
        "Play boot chirp <span class=\"r\">*</span></label></div><br>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"show_disp\" %s> "
        "Enable Display <span class=\"r\">*</span></label></div>"
        // V2.4.9: display layout mode dropdown. AUTO uses panel-based rule
        // (SerLCD or SSD1309 → rotation; SSD1306 → radiation). Explicit
        // overrides bypass the auto rule entirely.
        "<label>Display layout <span class=\"r\">*</span>"
        "<select name=\"disp_mode\">"
        "<option value=\"0\"%s>Auto (panel-based: small OLED &rarr; radiation, big OLED / SerLCD &rarr; rotation)</option>"
        "<option value=\"1\"%s>Radiation only (Heltec-style single page)</option>"
        "<option value=\"2\"%s>Rotation (Env / PM / Number / Uploads / System)</option>"
        "</select></label>"
        "<label>Display brightness "
        "<select name=\"oled_bright\">%s</select>"
        " <small>(live — applies on Save without reboot)</small></label>"
        "<h3>Time</h3>"
        "<label>NTP server 1 <span class=\"r\">*</span>"
        "<input type=\"text\" name=\"ntp\" value=\"%s\"></label>"
        "<label>NTP server 2 (optional) <span class=\"r\">*</span>"
        "<input type=\"text\" name=\"ntp2\" value=\"%s\"></label>"
        "<label>NTP server 3 (optional) <span class=\"r\">*</span>"
        "<input type=\"text\" name=\"ntp3\" value=\"%s\"></label>"
        "<label>Timezone (POSIX TZ) <span class=\"r\">*</span>"
        "<input type=\"text\" name=\"tz_posix\" value=\"%s\" maxlength=\"47\">"
        "<small>e.g. <code>AEST-10AEDT,M10.1.0,M4.1.0/3</code> (Sydney), "
        "<code>CET-1CEST,M3.5.0,M10.5.0/3</code> (Germany), "
        "<code>UTC0</code> (UTC). See <code>man tzset</code>.</small></label>"
        // V2.5.30: ap_pw moved to Network (below AP SSID), tx_int_ms to the top of
        // Transmission targets, heap_guard to the bottom of Hardware — leaving this
        // section Time-only (header renamed from "Other" to "Time" above).
        // V2.3.24: two submit buttons. The clicked button's name=value is the
        // only one included in the POST body (standard HTML form behaviour),
        // so the handler distinguishes via the "save_restart" key. Plain
        // "Save" leaves the device running with the new NVS values applied
        // live to the fields that read s_cfg per cycle / per request — see
        // asterisk legend at the top of the form for which fields don't.
        "<div class=\"btns\">"
        "<input type=\"submit\" name=\"save\" value=\"Save\">"
        "<input type=\"submit\" name=\"save_restart\" value=\"Save and restart\">"
        "</div>"
        "</form>"
        "<h3>Reboot</h3>"
        "<form method=\"post\" action=\"/reboot\" "
        "onsubmit=\"return confirm('Reboot the device now?');\">"
        "<input type=\"submit\" value=\"Reboot now\">"
        "</form>"
        "<p><a href=\"/\">Back to status</a> &middot; "
        "<a href=\"/update\">Firmware update</a></p>"
        "</body></html>",
        e_ssid, e_pw, e_host, e_apn, e_ap,   // V2.5.30: e_ap (ap_pw) moved to Network
        s_cfg->wifi_11bg_only   ? "checked" : "",
        s_cfg->wifi_ht20_only   ? "checked" : "",
        s_cfg->wifi_ps_disabled ? "checked" : "",
#if HAL_HAS_ANTENNA_SWITCH
        "",                                                  // not disabled on this board
        s_cfg->use_external_antenna ? "checked" : "",        // current state
        "",                                                  // no trailing note
#else
        "disabled",                                          // greyed out
        "",                                                  // never checked on this board
        " <small>(not available on this board)</small>",
#endif
        e_chip, s_mac_str,
        s_cfg->tube_enabled ? "checked" : "",
        tube_opts,                              // V2.6.1: table-driven tube dropdown
        s_cfg->pcnt_filter  ? "checked" : "",   // V2.5.16: indented under tube_en
        (unsigned long)s_cfg->pcnt_filter_width_ns,  // V2.5.16: filter width input
        s_cfg->deadtime_guard ? "checked" : "",      // V2.5.30: guard enable checkbox
        (unsigned long)s_cfg->deadtime_guard_us,     // V2.5.30: guard window µs
#if HAL_HAS_I2C_PINOUT_SWITCH
        "",                                          // not disabled on this board
        s_cfg->i2c_pinout ? "checked" : "",          // current state
        "",                                          // no trailing note
#else
        "disabled",                                  // greyed out
        "",                                          // never checked on this board
        " <small>(not available on this board)</small>",
#endif
        (unsigned long)s_cfg->heap_guard_floor_kb,   // V2.5.30: bottom of Hardware
        (unsigned long)s_cfg->heap_guard_confirm_cycles,  // V2.5.33: confirm cycles box
        (unsigned long)s_cfg->tx_interval_ms,        // V2.5.30: top of Transmission targets
        s_cfg->send_madavi  ? "checked" : "",
        s_cfg->madavi_https ? "checked" : "",
        s_cfg->send_sensorc ? "checked" : "",
        s_cfg->sensorc_https ? "checked" : "",
        (double)s_cfg->station_altitude_m,            // V2.5.7: moved under sensor.community
        s_cfg->send_sealevel_pressure ? "checked" : "",
        s_cfg->send_radmon  ? "checked" : "",
        s_cfg->radmon_https ? "checked" : "",
        e_ru, e_rp,
        s_cfg->send_osm ? "checked" : "",
        e_osm,
        e_osm_tok,
        s_cfg->send_osm_staging ? "checked" : "",
        e_osm_st,
        e_osm_st_tok,
        s_cfg->send_aqi ? "checked" : "",
        e_aqi,
        s_cfg->send_gmc ? "checked" : "",
        e_gmc_aid,
        e_gmc_gid,
        s_cfg->send_thingspeak  ? "checked" : "",
        s_cfg->thingspeak_https ? "checked" : "",
        e_ts_key,
        s_cfg->send_thingspeak_pm  ? "checked" : "",
        s_cfg->thingspeak_pm_https ? "checked" : "",
        e_ts_pm_key,
        s_cfg->ftp_enabled ? "checked" : "",
        s_cfg->ftp_tls     ? "checked" : "",
        e_fhost, e_fuser, e_fpw, e_fpath,
        (unsigned long)s_cfg->ftp_interval_min,
        s_cfg->ftp_ps_disabled ? "checked" : "",
        s_cfg->ftp_tls12_only  ? "checked" : "",
        // V2.4.4: MQTT row format args (must match order of %s/%lu/%s/%s/%s/%s in the form HTML above).
        s_cfg->mqtt_enable ? "checked" : "",
        e_mhost,
        (unsigned long)s_cfg->mqtt_port,
        e_muser,
        e_mpw,
        e_mpfx,
        s_cfg->mqtt_ha_discovery ? "checked" : "",
        // V2.4.6: MQTT TLS row format args (5 in order: enable, then three
        // <option selected> markers for the trust-mode dropdown 0/1/2, then
        // the CA cert PEM body).
        s_cfg->mqtt_tls_enable ? "checked" : "",
        s_cfg->mqtt_tls_mode == 0 ? " selected" : "",
        s_cfg->mqtt_tls_mode == 1 ? " selected" : "",
        s_cfg->mqtt_tls_mode == 2 ? " selected" : "",
        e_mca,
        // V2.4.15: syslog row format args (3 in order: enable / host / port).
        s_cfg->syslog_enable ? "checked" : "",
        e_slh,
        (unsigned long)s_cfg->syslog_port,
        s_cfg->speaker_tick ? "checked" : "",
        s_cfg->led_tick     ? "checked" : "",
        s_cfg->play_sound   ? "checked" : "",
        s_cfg->show_display ? "checked" : "",
        // V2.4.9: display layout mode dropdown (3 args — selected markers
        // for the three options Auto/Radiation/Rotation).
        s_cfg->display_mode == 0 ? " selected" : "",
        s_cfg->display_mode == 1 ? " selected" : "",
        s_cfg->display_mode == 2 ? " selected" : "",
        br_opts,
        e_ntp1, e_ntp2, e_ntp3, e_tz);   // V2.5.30: ap_pw/tx_int/heap_guard relocated above

    // V2.3.33: snprintf returns the would-have-been length on truncation,
    // not the bytes actually written. If n >= buffer size the page tail was
    // cut — log loudly so the issue surfaces in serial instead of silently
    // producing a broken HTML page. Clamp the send length to the buffer
    // to avoid reading past it.
    if (n >= (int)CFG_FORM_BUF_SIZE) {
        ESP_LOGE(TAG, "config page truncated: needed %d bytes, buffer is %u; "
                 "increase CFG_FORM_BUF_SIZE", n, (unsigned)CFG_FORM_BUF_SIZE);
        n = CFG_FORM_BUF_SIZE - 1;
    }

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    set_security_headers(req);
    esp_err_t err = httpd_resp_send(req, body, n > 0 ? n : 0);
    free(body);
    free(e_mca);
    return err;
}

// --- POST /config (parse form, save, flag restart) --------------------------
//
// V2.4.1: the per-field plumbing (pre-clear bools + dispatch) is now
// generated from the schema in config_fields.def via the helpers
// config_post_preclear_bools() and config_post_apply_field() in config.c.
// This handler owns only the parts that DON'T fit the schema:
//   * HTTP body slurp + URL-form parsing
//   * Special-case key handling (`save_restart`, `oled_bright` step check)
//   * Cross-field invariants applied after the dispatch (wifi_11bg → ht20,
//     antenna force-clear on boards lacking hardware, ftp_ps preserve when
//     global PS is off)

static esp_err_t config_post(httpd_req_t *req) {
    log_access(req, "POST /config");
    if (!check_auth(req)) return ESP_OK;
    if (!check_same_origin(req)) return ESP_OK;

    // V2.4.1 (C5): content_len is size_t in esp_http_server. Hold it
    // as size_t throughout to avoid silent narrowing on huge values.
    // Hard cap blocks abuse; recv loop uses size_t for position
    // and the (signed) int return value of httpd_req_recv for error.
    //
    // V2.4.6: cap raised from 4096 to 12288 to fit the URL-encoded MQTT
    // TLS CA cert (PEM up to CFG_MQTT_CA_CERT_MAX ≈ 2400 raw, ~5 KB
    // URL-encoded) plus all other form fields (~5-6 KB). 12 KB sits well
    // under both the 16 KB Heltec form buffer and the 32 KB PSRAM-board
    // form buffer — POST and GET are separate transient allocations.
    size_t total = req->content_len;
    if (total == 0 || total > 12288) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body size out of range");
        return ESP_OK;
    }
    char *buf = malloc(total + 1);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_OK;
    }
    size_t received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, buf + received, total - received);
        if (r <= 0) {
            free(buf);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_OK;
        }
        received += (size_t)r;
    }
    buf[total] = 0;

    // V2.3.24: which submit button was clicked? "save_restart" key only
    // appears when the user clicked "Save and restart"; plain "save" appears
    // when they clicked "Save" (NVS persists, no reboot triggered). Default
    // is no-restart so a hand-crafted POST that omits both keys also stays
    // running.
    bool restart_after_save = false;

    // V2.5.34: collect the form keys of fields that matched but whose value was
    // out of range (so the prior value was kept). Reported on the result page +
    // logged, instead of the prior silent no-save that made an out-of-range entry
    // (e.g. ftp_int > its max) look like nothing happened. Only known field keys
    // ever land here (set inside the schema dispatch), so the contents are a fixed
    // allowlist — safe to echo into the result HTML without escaping.
    char rejected[160];
    rejected[0] = 0;
    int rej_n = 0;

    // Start from the current config; pre-clear every bool (forms only POST
    // ticked checkboxes, so an absent key means "unticked"). Schema-derived
    // — see config.c::config_post_preclear_bools.
    config_t next = *s_cfg;
    config_post_preclear_bools(&next);

    char *p = buf;
    while (*p) {
        char *eq = strchr(p, '=');
        if (!eq) break;
        *eq = 0;
        char *val = eq + 1;
        char *amp = strchr(val, '&');
        if (amp) *amp = 0;
        url_decode(val);

        // V2.3.30: oled_bright needs a STEP validator (0 or 10..100 step 10)
        // that's tighter than the schema's generic [0,100] envelope. Handle
        // it FIRST so the generic dispatch never sees this key.
        // V2.3.32: 0 (OFF) accepted — display_set_contrast interprets as
        // panel-dark (OLED 0xAE) / backlight-off (SerLCD).
        bool oor = false;   // matched a known field but value out of range/step
        if (strcmp(p, "oled_bright") == 0) {
            long v = strtol(val, NULL, 10);
            if (v == 0 || (v >= 10 && v <= 100 && (v % 10) == 0)) {
                next.oled_brightness_pct = (uint8_t)v;
            } else {
                oor = true;   // V2.5.34: out-of-step value kept prior — report it
            }
        }
        // Generic schema dispatch. Returns true if `p` matched a known
        // field; out-of-range numerics keep prior value and set `oor`.
        else if (!config_post_apply_field(&next, p, val, &oor)) {
            // Non-schema control keys.
            if (strcmp(p, "save_restart") == 0) restart_after_save = true;
            // Plain "save" and any unknown keys are silently ignored.
        }

        // V2.5.34: accumulate any rejected field's key (comma-separated).
        if (oor) {
            rej_n = append_safe(rejected, sizeof(rejected), rej_n,
                                "%s%s", rej_n ? ", " : "", p);
        }

        if (!amp) break;
        p = amp + 1;
    }
    free(buf);

    // --- Cross-field invariants — applied after dispatch ----------------

    // 802.11b/g channels are always 20 MHz — HT40 only exists under 11n.
    // A disabled checkbox doesn't POST, so the form may send wifi_ht20=0
    // even while the UI showed it ticked; enforce the invariant here so
    // the stored state matches what the user saw.
    if (next.wifi_11bg_only) next.wifi_ht20_only = true;

    // External-antenna switch: silently force-disable on boards without the
    // hardware. Defence-in-depth — the UI already greys the checkbox, but a
    // hand-crafted POST could still set it.
#if !HAL_HAS_ANTENNA_SWITCH
    next.use_external_antenna = false;
#endif

    // V2.5.19: same defence-in-depth for the I²C pin-out route — force-disable
    // on boards without the alternate pads (UI already greys it).
#if !HAL_HAS_I2C_PINOUT_SWITCH
    next.i2c_pinout = false;
#endif

    // ftp_ps_dis is greyed out (and force-unchecked) in the UI when the
    // global wifi_ps_dis is ticked, so the form won't POST it. Preserve the
    // previously saved value rather than clobbering it to false — that way
    // the user's preference survives a round-trip through "global PS off".
    if (next.wifi_ps_disabled) next.ftp_ps_disabled = s_cfg->ftp_ps_disabled;

    // V2.5.4: the radiation-only upload targets (Radmon, GMCMap, ThingSpeak)
    // are meaningless without the Geiger tube. The UI greys + force-unchecks
    // them when "Enable Geiger tube" is off (syncTube()); enforce the same
    // invariant here so a hand-crafted POST can't enable them, and so the
    // stored config stays honest. Unlike ftp_ps_dis we do NOT preserve the
    // prior choice — "tube off" genuinely turns these off, not just hides them.
    if (!next.tube_enabled) {
        next.send_radmon = false;
        next.send_gmc = false;
        next.send_thingspeak = false;
        next.pcnt_filter = false;   // V2.5.16: width filter needs count pulses
        next.deadtime_guard = false; // V2.5.30: dead-time guard needs count pulses
    }

    // V2.5.30: dead-time guard is mutually exclusive with pcnt_filter — the guard
    // runs in the GMC ISR but pcnt_filter makes the PCNT hardware path (which the
    // guard can't reach) authoritative for the uploaded count. pcnt_filter WINS;
    // mirror the UI's syncGuard() greying so a hand-crafted POST can't set both.
    if (next.pcnt_filter) {
        next.deadtime_guard = false;
    }

    *s_cfg = next;
    esp_err_t err = config_save(s_cfg);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_OK;
    }

    // V2.3.30: live-apply OLED/SerLCD brightness (no reboot for this field).
    // No-op if the panel is dark (show_display=false) or if the value didn't
    // change. ~1 ms (OLED) or ~10 ms (SerLCD) of I²C traffic.
    display_set_contrast(s_cfg->oled_brightness_pct);

    // V2.5.3: live-apply MQTT discovery for upload-target enable changes. The
    // TX path picks up g_cfg next cycle, but MQTT's cached enable flags + the
    // HA-discovery entities only refresh in mqtt_init() — so without this a
    // target toggled via plain "Save" wouldn't show its HA entities until a
    // reconnect/reboot. No-op when MQTT isn't running.
    if (mqtt_is_initialized()) {
        mqtt_apply_config(s_cfg);
    }

    // V2.5.30 (review #4): live-apply the dead-time guard. The GMC ISR reads the
    // window (a volatile uint32) on every edge, so a /config change takes effect
    // immediately — no reboot, unlike the hardware-latched pcnt_filter. No-op when
    // the tube wasn't started at boot (the count ISR isn't installed). When tube
    // is off, the value was already force-cleared to 0 above.
    if (tube_is_enabled()) {
        tube_set_guard_us(config_effective_guard_us(s_cfg));
    }

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    set_security_headers(req);
    // V2.5.34: single render path for both result pages — they differ only in the
    // <h1> + body text, so pick those with a ternary and build the page once. The
    // out-of-range notice (when any field was kept) is appended inline, so there's
    // no separate warn[] buffer. Previously an out-of-range value was kept silently
    // with no feedback, so a too-large ftp_int looked like the save did nothing.
    if (rejected[0]) {
        ESP_LOGW(TAG, "config POST: out-of-range value(s) NOT saved (kept prior): %s",
                 rejected);
    }
    if (restart_after_save) main_request_restart();
    ESP_LOGI(TAG, "config saved via POST — %s",
             restart_after_save ? "restart flagged" : "no restart requested");

    const char *h1   = restart_after_save ? "Saved. Restarting..." : "Saved.";
    const char *body = restart_after_save
        ? "<p>Device will restart in ~2 seconds. Your browser will drop the "
          "connection; reconnect to the new WiFi settings if you changed them.</p>"
          "<p><a href=\"/\">Back to status</a></p>"
        : "<p>New settings persisted to NVS and applied live. If you changed "
          "any field marked with <span style=\"color:#c00;font-weight:bold\">*</span> "
          "(reboot-required) the new value won't take effect until the next "
          "restart.</p>"
          "<p><a href=\"/config\">Back to configuration</a> &middot; "
          "<a href=\"/\">Back to status</a></p>";

    char page[1024];
    int n = append_safe(page, sizeof(page), 0,
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<title>Saved</title></head><body><h1>%s</h1>", h1);
    if (rejected[0]) {
        n = append_safe(page, sizeof(page), n,
            "<p style=\"color:#c00;font-weight:bold\">&#9888; These fields were out "
            "of range and were NOT saved (previous value kept): %s</p>", rejected);
    }
    append_safe(page, sizeof(page), n, "%s</body></html>", body);
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

// --- POST /reboot (manual restart button) ----------------------------------

static esp_err_t reboot_post(httpd_req_t *req) {
    log_access(req, "POST /reboot");
    if (!check_auth(req)) return ESP_OK;
    if (!check_same_origin(req)) return ESP_OK;
    main_request_restart();
    ESP_LOGW(TAG, "manual reboot requested via /reboot");
    const char *ok =
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<title>Rebooting</title></head><body>"
        "<h1>Rebooting...</h1>"
        "<p>Device will restart in ~2 seconds.</p>"
        "<p><a href=\"/\">Back to status</a></p>"
        "</body></html>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    set_security_headers(req);
    return httpd_resp_send(req, ok, HTTPD_RESP_USE_STRLEN);
}

// --- GET /coredump.elf (download dump) + POST /coredump_erase (clear) -------
//
// V2.4.18: panic-dump retrieval over the air. Pairs with the 64 KB
// `coredump` partition added to partitions.csv / partitions_4mb.csv and
// with CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y. ESP-IDF writes the dump
// during the panic handler; this endpoint streams the partition bytes
// out so the operator can decode them off-device with:
//   espcoredump.py info_corefile -t elf -c oatlands.elf build_<board>/geiger_v2.elf
//
// Both endpoints are basic-auth gated. The GET is not CSRF-checked (it's
// idempotent, read-only); the POST erase is CSRF-checked like /config,
// /update, /reboot.
//
// Why a dedicated endpoint and not just GET /partition?: we need to stop
// at the actual dump SIZE (from esp_core_dump_image_get), not the full
// 64 KB partition — otherwise the client downloads 64 KB of mostly 0xFF.
// And we need the content-type / disposition headers so browsers
// download-as-file rather than try to render.

static esp_err_t coredump_get(httpd_req_t *req) {
    log_access(req, "GET /coredump.elf");
    if (!check_auth(req)) return ESP_OK;
    if (!coredump_have_dump()) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No core dump present");
        return ESP_OK;
    }
    httpd_resp_set_type(req, "application/octet-stream");
    // Filename baked from chip-id so multi-device captures don't collide
    // on the operator's download folder. ASCII only — Content-Disposition
    // RFC 6266 quoted-string with non-ASCII would need RFC 5987 encoding.
    char disp[96];
    snprintf(disp, sizeof(disp),
             "attachment; filename=\"coredump_%s.elf\"",
             s_chip_id[0] ? s_chip_id : "device");
    httpd_resp_set_hdr(req, "Content-Disposition", disp);
    esp_err_t r = coredump_stream_to_http(req);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "/coredump.elf stream failed: %s", esp_err_to_name(r));
        // httpd has already started the response — there's no clean way
        // to send_err at this point. Return ESP_OK so the framework
        // tears the response down without complaining.
        return ESP_OK;
    }
    return ESP_OK;
}

static esp_err_t coredump_erase_post(httpd_req_t *req) {
    log_access(req, "POST /coredump_erase");
    if (!check_auth(req)) return ESP_OK;
    if (!check_same_origin(req)) return ESP_OK;
    esp_err_t r = coredump_erase();
    if (r != ESP_OK) {
        char msg[96];
        snprintf(msg, sizeof(msg), "erase failed: %s", esp_err_to_name(r));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, msg);
        return ESP_OK;
    }
    // Redirect back to status. 303 See Other is the right code for
    // POST→GET redirection per RFC 7231 §6.4.4.
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    set_security_headers(req);
    return httpd_resp_send(req, NULL, 0);
}

// --- GET /update (upload form) ----------------------------------------------
// Uses XHR to POST the raw .bin as the request body (Content-Type:
// application/octet-stream). Avoids multipart parsing on the device.

// V2.3.14: board-specific upload-form prompt. V2.3.16-pre2: distinguish 4 MB
// (knock-off) vs 8 MB (genuine) Heltec since the binaries are NOT inter-
// changeable (4 MB image asserts at boot on 8 MB hardware and vice versa).
// CMakeLists.txt defines BOARD_HELTEC_V2_4MB IN ADDITION TO BOARD_HELTEC_V2
// for the 4 MB variant — order the #if chain so the more-specific 4MB check
// fires first. Literal-string concatenation resolves at compile time, zero
// runtime cost. Bold red styling inline so we don't add a CSS rule for one
// element.
#if BOARD_HELTEC_V2_4MB
    #define UPLOAD_PROMPT_BOARD "<b style=\"color:red\">Heltec WiFi Kit 32 (4MB)</b>"
#elif BOARD_HELTEC_V2
    #define UPLOAD_PROMPT_BOARD "<b style=\"color:red\">Heltec WiFi Kit 32 (8MB)</b>"
#elif BOARD_FEATHERS3_D
    #define UPLOAD_PROMPT_BOARD "<b style=\"color:red\">FeatherS3</b>"
#elif BOARD_ADAFRUIT_QTPY_ESP32_PICO
    #define UPLOAD_PROMPT_BOARD "<b style=\"color:red\">Adafruit QT Py ESP32-PICO</b>"
#elif BOARD_SEEED_XIAO_ESP32S3
    // V2.5.19: the XIAO target shipped in V2.4.25 but was never given a label
    // branch here, so it fell through to "(unknown board)" on the OTA page —
    // which, by elimination, was the only way to identify a XIAO build.
    #define UPLOAD_PROMPT_BOARD "<b style=\"color:red\">Seeed XIAO ESP32-S3</b>"
#else
    #define UPLOAD_PROMPT_BOARD "<b style=\"color:red\">(unknown board)</b>"
#endif

static esp_err_t update_get(httpd_req_t *req) {
    log_access(req, "GET /update");
    if (!check_auth(req)) return ESP_OK;
    static const char page[] =
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<title>OTA — MultiGeiger V2</title>"
        "<style>body{font-family:system-ui;max-width:40em;margin:2em auto;padding:0 1em}"
        "progress{width:100%;height:1.4em;margin-top:1em}"
        "#msg{margin-top:1em;white-space:pre-wrap}</style>"
        "</head><body><h1>Firmware update</h1>"
        // Compile-time string-literal concatenation — VERSION_STR is a
        // #define so the version is baked into this static page at build
        // time. Zero runtime cost. version.h's include here means a tag
        // bump auto-rebuilds this TU.
        "<p><b>Current firmware:</b> <code>" VERSION_STR "</code></p>"
        "<p>Select a firmware .bin for " UPLOAD_PROMPT_BOARD ".</p>"
        "<input type=\"file\" id=\"f\" accept=\".bin\">"
        "<button id=\"go\" onclick=\"upload()\">Upload</button>"
        "<progress id=\"p\" value=\"0\" max=\"1\"></progress>"
        "<div id=\"msg\"></div>"
        "<p><a href=\"/\">Back to status</a></p>"
        "<script>"
        "function upload(){"
        " var f=document.getElementById('f').files[0];"
        " if(!f){document.getElementById('msg').textContent='No file selected';return;}"
        " var go=document.getElementById('go');go.disabled=true;"
        " var x=new XMLHttpRequest();"
        " x.upload.onprogress=function(e){if(e.lengthComputable)document.getElementById('p').value=e.loaded/e.total;};"
        " x.onload=function(){document.getElementById('msg').textContent=x.status+' '+x.responseText;go.disabled=false;};"
        " x.onerror=function(){document.getElementById('msg').textContent='Upload failed';go.disabled=false;};"
        " x.open('POST','/update');"
        " x.setRequestHeader('Content-Type','application/octet-stream');"
        " x.send(f);"
        "}"
        "</script></body></html>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    set_security_headers(req);
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

// --- POST /update (stream body into OTA partition) --------------------------

#define OTA_CHUNK 1024

// V2.4.24: split the auth + flag-management front-end into a thin wrapper
// so the ~250-line body doesn't have to set/clear main_ota_in_progress
// at every one of its ~17 return paths. The wrapper sets the flag once
// after auth/CSRF pass, calls the inner function, and clears the flag
// before returning regardless of inner's outcome. See main_status.h
// main_ota_begin() doc for the WHY (gates the main loop's TX-cycle
// scheduler so OTA gets the WiFi airtime to itself).
static esp_err_t update_post_inner(httpd_req_t *req);

static esp_err_t update_post(httpd_req_t *req) {
    log_access(req, "POST /update");
    if (!check_auth(req)) return ESP_OK;
    if (!check_same_origin(req)) return ESP_OK;
    main_ota_begin();
    esp_err_t result = update_post_inner(req);
    main_ota_end();
    return result;
}

static esp_err_t update_post_inner(httpd_req_t *req) {
    // V2.4.13: reclaim heap BEFORE the OTA receive/write loop. Heltec V2
    // (4MB) with V2.4.11+ ran at min_free ~13 KB at steady state (MQTT TLS
    // session + esp_crt_bundle + WiFi stack); not enough headroom for
    // esp_ota_write's scratch buffers — caused OTA OOM observed 2026-05-19
    // on esp32-176432. Three-step teardown:
    //   1) Drain the TX worker so a Madavi HTTPS POST in flight doesn't
    //      compete with the OTA recv loop on TLS state.
    //   2) Pause FTPS scheduling so the next due upload doesn't fire
    //      mid-OTA (FTPS holds ~15-20 KB during its TLS handshake).
    //   3) Stop+destroy MQTT to free its ~18-25 KB TLS session.
    // Effect: lifts min_free back to ~50 KB on Heltec V2, plenty for OTA.
    // On FeatherS3-D the teardown is harmless — heap was never tight.
    // Scoped to POST only: GET /update (the form render) leaves network
    // state intact, so just viewing the page has zero impact.
    // V2.4.32: snapshot the net-stack RAM split at OTA start — this is the
    // failure moment for the long-uptime inbound-stall (esp32-5965048,
    // 2026-05-30). Logged before the teardown below so we capture the degraded
    // state with WiFi / MQTT / FTP still up.
    diag_log_heap("OTA prep");

    // V2.4.32: wait up to 60s (was 10s) for the TX worker to go idle. A
    // sensor.community / openSenseMap retry storm is up to 4 × ~15s ≈ 60s, so a
    // 10s budget guaranteed we'd "proceed anyway" mid-storm and let the OTA
    // write contend with an in-flight TLS upload. 60s covers the worst-case
    // retry run so the OTA almost always starts on a quiet radio.
    ESP_LOGI(TAG, "OTA prep: waiting for TX worker idle (up to 60s)");
    int spins = 0;
    while (!tx_is_idle() && spins++ < 600) vTaskDelay(pdMS_TO_TICKS(100));
    if (spins >= 600) {
        ESP_LOGW(TAG, "OTA prep: TX still busy after 60s — proceeding anyway");
    }
    log_ftp_pause();
    mqtt_stop();
    // V2.5.28: syslog deliberately stays UP through the whole flash (was
    // syslog_stop() here pre-V2.5.28). The teardown's heap win is mqtt_stop
    // (~50 KB TLS) + log_ftp_pause; the UDP socket is only a few hundred
    // bytes, so keeping it open is negligible and lets the entire OTA trace
    // (progress / verify / SUCCESS / FAILED) reach the syslog server —
    // including failures, which used to be invisible server-side.
    tube_pcnt_stop(); // V2.5.16: release the opt-in PCNT comb's internal DRAM
                      // (no-op when pcnt_filter is off — the common case)
    // V2.4.17: tell the main-loop poll NOT to re-init MQTT/syslog. Without
    // this the poll re-armed both within ~1 s of the stops above, undoing
    // the V2.4.13 heap-freeing intent during the bulk of the OTA write.
    // log_ftp_pause is already sticky; MQTT and syslog needed the equivalent
    // gate. Flag is set-only — device reboots on OTA success; on failure,
    // user must manually /reboot to restore services.
    main_suspend_services();
    ESP_LOGI(TAG, "OTA prep: heap free=%u min=%u largest=%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));

    // V2.4.1 (C5): content_len is size_t; hold as size_t throughout.
    size_t total = req->content_len;
    if (total == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty body");
        return ESP_OK;
    }

    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA slot");
        return ESP_OK;
    }

    // V2.3.33: bound the upload size by the OTA partition before doing any
    // work. esp_ota_begin would fail on out-of-range image_size anyway,
    // but checking here lets us reject early with a clear error and
    // prevents a malicious client from claiming a giant content_len and
    // slowloris-ing the recv loop one chunk at a time.
    if (total > target->size) {
        ESP_LOGE(TAG, "OTA refused: claimed body %u > partition size %lu",
                 (unsigned)total, (unsigned long)target->size);
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "Image (%u bytes) larger than OTA partition (%lu bytes)",
                 (unsigned)total, (unsigned long)target->size);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "OTA: %u bytes -> partition %s @ 0x%lx",
             (unsigned)total, target->label, (unsigned long)target->address);

    esp_ota_handle_t ota = 0;
    esp_err_t err = esp_ota_begin(target, total, &ota);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA FAILED: esp_ota_begin — %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_OK;
    }

    char *buf = malloc(OTA_CHUNK);
    if (!buf) {
        esp_ota_abort(ota);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_OK;
    }

    size_t received = 0;
    // V2.4.13: weak-WiFi resilience — retry transient recv timeouts instead
    // of aborting the whole OTA. Combined with the 30 s recv_wait_timeout
    // bump in http_server_start, this gives up to 5 × 30 s = 150 s of TCP
    // outage tolerance per chunk before we give up. Only retries on
    // HTTPD_SOCK_ERR_TIMEOUT (-3); HTTPD_SOCK_ERR_FAIL (-1) and peer-close
    // (0) are still fatal — there's no recovering from those. The retry
    // counter resets on every successful recv so a long upload over a
    // generally-OK link with occasional hiccups doesn't drain the budget.
    int recv_retries = 0;
    const int RECV_MAX_RETRIES = 5;
    // V2.5.28: per-flash telemetry — wall-clock start + a 128 KB progress gate.
    const int64_t t_start  = esp_timer_get_time();
    size_t        next_log = 128 * 1024;
    while (received < total) {
        size_t want = total - received;
        if (want > OTA_CHUNK) want = OTA_CHUNK;
        int r = httpd_req_recv(req, buf, want);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            if (recv_retries < RECV_MAX_RETRIES) {
                recv_retries++;
                ESP_LOGW(TAG, "recv timeout at %u/%u — retry %d/%d",
                         (unsigned)received, (unsigned)total,
                         recv_retries, RECV_MAX_RETRIES);
                continue;
            }
            ESP_LOGE(TAG, "OTA FAILED: recv timed out %d× at %u/%u KB",
                     RECV_MAX_RETRIES, (unsigned)(received / 1024), (unsigned)(total / 1024));
            free(buf);
            esp_ota_abort(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv timeout");
            return ESP_OK;
        }
        if (r <= 0) {
            ESP_LOGE(TAG, "OTA FAILED: recv at %u/%u KB (r=%d)",
                     (unsigned)(received / 1024), (unsigned)(total / 1024), r);
            free(buf);
            esp_ota_abort(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_OK;
        }
        recv_retries = 0;   // got bytes — reset the retry budget
        err = esp_ota_write(ota, buf, (size_t)r);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA FAILED: esp_ota_write at %u KB — %s",
                     (unsigned)(received / 1024), esp_err_to_name(err));
            free(buf);
            esp_ota_abort(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
            return ESP_OK;
        }
        received += (size_t)r;
        if (received >= next_log) {
            ESP_LOGI(TAG, "OTA progress: %u/%u KB (%u%%)",
                     (unsigned)(received / 1024), (unsigned)(total / 1024),
                     (unsigned)(received * 100 / total));
            next_log += 128 * 1024;
        }
    }
    free(buf);

    const int64_t recv_ms = (esp_timer_get_time() - t_start) / 1000;
    ESP_LOGI(TAG, "OTA receive complete: %u KB in %llds (%u KB/s)",
             (unsigned)(total / 1024), (long long)(recv_ms / 1000),
             recv_ms > 0 ? (unsigned)((uint64_t)(total / 1024) * 1000 / recv_ms) : 0);

    err = esp_ota_end(ota);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA FAILED: esp_ota_end — %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_OK;
    }
    ESP_LOGI(TAG, "OTA image written + finalized (esp_ota_end ok) — verifying");

    // V2.3.13: validate the just-written app's chip_id and project_name BEFORE
    // committing the boot partition. The bootloader does its own chip-ID check
    // on the next boot and rolls back automatically on mismatch — but that's
    // an ugly recovery (one wasted OTA cycle, confusing reboot). Catching it
    // here gives the user a clean HTTP 400 with explanation while leaving the
    // current firmware untouched. Doesn't catch same-chip-different-board
    // mistakes (e.g. WiFi LoRa 32 V2 vs WiFi Kit 32 V2 — both ESP32) but
    // covers the common cross-family case (heltec_v2 ↔ feathers3_d).
    //
    // Two reads are needed because the chip_id and project_name live in
    // different structures within the binary:
    //   - chip_id lives in esp_image_header_t at partition offset 0
    //     (NOT in esp_app_desc_t, which is what tripped the V2.3.13 first
    //     build attempt). Read via esp_partition_read directly.
    //   - project_name + version + date live in esp_app_desc_t, accessible
    //     via esp_ota_get_partition_description.
    esp_image_header_t img_hdr = {0};
    err = esp_partition_read(target, 0, &img_hdr, sizeof(img_hdr));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_partition_read(image header) failed: %s",
                 esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Could not read uploaded image header");
        return ESP_OK;
    }

    esp_app_desc_t new_desc = {0};
    err = esp_ota_get_partition_description(target, &new_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_get_partition_description failed: %s",
                 esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Could not read uploaded image descriptor");
        return ESP_OK;
    }

    // Project-name check first: catches "user uploaded a totally unrelated
    // ESP-IDF firmware by mistake". CMakeLists.txt sets project(geiger_v2)
    // so every legitimate build's app descriptor carries that name.
    if (strcmp(new_desc.project_name, "geiger_v2") != 0) {
        ESP_LOGE(TAG, "OTA refused: project_name='%.32s' (expected 'geiger_v2')",
                 new_desc.project_name);
        // 256-byte buffer is comfortable: max payload is the 32-char
        // project_name field plus ~130 chars of static text + NUL.
        char msg[256];
        snprintf(msg, sizeof(msg),
                 "OTA refused: uploaded binary's project_name is '%.32s', "
                 "expected 'geiger_v2'. This does not look like a "
                 "MultiGeiger-V2 firmware build.",
                 new_desc.project_name);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
        return ESP_OK;
    }

    // Chip-ID check: catches heltec_v2 (ESP32) vs feathers3_d (ESP32-S3)
    // cross-family mistakes. Map the running chip's esp_chip_model_t value
    // into the image header's ESP_CHIP_ID_* enum for direct comparison.
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    esp_chip_id_t expected_chip_id;
    const char  *expected_board;
    // V2.5.20 (review R6): the error label now uses the RUNNING build's
    // BOARD_NAME instead of hardcoded board names — a QT Py used to report
    // itself as "heltec_v2 (ESP32)" and a XIAO as "feathers3_d (ESP32-S3)"
    // in the refusal message. Only one case can match the firmware actually
    // running, so BOARD_NAME is always the right device label.
    switch (chip.model) {
        case CHIP_ESP32:
            expected_chip_id = ESP_CHIP_ID_ESP32;
            expected_board   = BOARD_NAME " (ESP32)";
            break;
        case CHIP_ESP32S3:
            expected_chip_id = ESP_CHIP_ID_ESP32S3;
            expected_board   = BOARD_NAME " (ESP32-S3)";
            break;
        default:
            // Future-proofing: if someone ports the firmware to ESP32-C3/C6/H2
            // without updating this switch, fall through and trust the
            // bootloader as the safety net.
            expected_chip_id = ESP_CHIP_ID_INVALID;
            expected_board   = "(unrecognised chip family)";
            break;
    }

    if (expected_chip_id != ESP_CHIP_ID_INVALID &&
        img_hdr.chip_id != expected_chip_id) {
        ESP_LOGE(TAG, "OTA refused: binary chip_id=0x%04x (version=%.32s), "
                 "device chip_id=0x%04x (expected=%s)",
                 (unsigned)img_hdr.chip_id, new_desc.version,
                 (unsigned)expected_chip_id, expected_board);
        // 384-byte buffer accommodates the longest expected_board (~24 chars),
        // 32-char version field, 4-char hex chip_id, and ~230 chars of static
        // text + NUL with comfortable margin.
        char msg[384];
        snprintf(msg, sizeof(msg),
                 "OTA refused: this device is %s but the uploaded binary "
                 "(version %.32s) is for chip_id 0x%04x. Check that you "
                 "uploaded the correct geiger_v2_<board>.bin file from the "
                 "GitHub release for THIS board family.",
                 expected_board, new_desc.version, (unsigned)img_hdr.chip_id);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "OTA verified: project=%.32s version=%.32s built=%.16s "
             "chip_id=0x%04x — committing", new_desc.project_name,
             new_desc.version, new_desc.date, (unsigned)img_hdr.chip_id);

    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA FAILED: esp_ota_set_boot_partition — %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_OK;
    }

    main_request_restart();
    ESP_LOGW(TAG, "OTA SUCCESS: %s -> %.32s (%u bytes) — boot set to %s, rebooting in ~2s",
             VERSION_STR, new_desc.version, (unsigned)total, target->label);
    // V2.5.28: syslog send is fire-and-forget (MSG_DONTWAIT). The main loop's
    // 2 s pre-restart delay already covers the flush, but yield briefly here
    // too so the SUCCESS line is on the wire before this handler returns.
    vTaskDelay(pdMS_TO_TICKS(100));

    const char *ok = "OTA OK — restarting in ~2s";
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    set_security_headers(req);
    return httpd_resp_send(req, ok, HTTPD_RESP_USE_STRLEN);
}

// --- GET /log (auth'd, streams the ring buffer as text/plain) ---------------

#define LOG_CHUNK 2048

// V2.3.17: streams directly from the ring via applog_stream_begin/end — no
// 60 KB malloc. Pre-V2.3.17 used applog_snapshot which mallocs the entire
// ring as one contiguous buffer (Content-Length-friendly response). Found
// 2026-05-10 via FTP_Investigation log analysis: every browser hit on /log
// caused a 60 KB transient peak; combined with concurrent Radmon retry
// storms or other heap-heavy events, peak demand stacked to ~95 KB and
// momentarily dropped free heap to single-digit-KB territory (min_free=316
// observed). Streaming via chunked Transfer-Encoding eliminates the body
// malloc entirely. Browser doesn't care about missing Content-Length —
// chunked is HTTP/1.1 standard. Memory cost: ~0 KB (just the existing
// 2 KB chunk-sized stack copy in httpd_resp_send_chunk).
//
// V2.3.24: snapshot returns up to three segments now (scratch copy of the
// danger zone + tail remainder + newer pre-wrap half) — see applog.h. The
// loop bound just bumped from 2 to 3.
static esp_err_t log_get(httpd_req_t *req) {
    log_access(req, "GET /log");
    // V2.3.33: /log is no longer auth-gated. The ring buffer is diagnostic
    // output (boot banner, WiFi/upload status, sensor readings) — same
    // class of information the device already broadcasts to Madavi /
    // sensor.community / Grafana publicly. Removing the prompt makes it
    // a one-click "view log" link from the unauth'd /status page.

    applog_stream_t s;
    if (!applog_stream_begin(&s)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "applog uninit");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "text/plain; charset=utf-8");

    // Send each segment in LOG_CHUNK-sized pieces, in chronological order:
    // seg_a (oldest — V2.3.24 scratch copy when wrapped, ring otherwise),
    // seg_b (tail remainder past the scratch copy when wrapped),
    // seg_c (newer pre-wrap half when wrapped). httpd_resp_send_chunk
    // handles HTTP/1.1 chunked-encoding framing.
    const char  *segs[3]  = { s.seg_a, s.seg_b, s.seg_c };
    const size_t sizes[3] = { s.len_a, s.len_b, s.len_c };
    for (int i = 0; i < 3; i++) {
        size_t sent = 0;
        while (sent < sizes[i]) {
            size_t n = sizes[i] - sent;
            if (n > LOG_CHUNK) n = LOG_CHUNK;
            if (httpd_resp_send_chunk(req, segs[i] + sent, n) != ESP_OK) {
                applog_stream_end();
                return ESP_FAIL;
            }
            sent += n;
        }
    }
    httpd_resp_send_chunk(req, NULL, 0);  // terminate
    applog_stream_end();
    return ESP_OK;
}

// --- Server bring-up ---------------------------------------------------------

void http_server_start(config_t *cfg, const char *chip_id) {
    s_cfg     = cfg;
    s_chip_id = chip_id ? chip_id : "";

    // STA MAC — burned into eFuse, same across boots. Displayed on / and
    // /config so the user can identify the board without serial access.
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        snprintf(s_mac_str, sizeof(s_mac_str),
                 "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
    hc.stack_size  = 8192;               // room for form+base64 on one stack
    hc.max_uri_handlers = 12;            // / /favicon.ico /config GET+POST /update GET+POST /reboot /log /coredump.elf /coredump_erase
    hc.lru_purge_enable = true;
    // CRITICAL — DO NOT change esp_http_server's threading model without
    // first reverting the static-buffer pattern used in V2.4.20 + V2.4.22.
    // We rely on the IDF default that ONE httpd thread processes all
    // URI handlers serially via select(). Several handlers (status_get,
    // config_get, format_system, applog_vprintf) hold large `static`
    // scratch buffers that are race-free ONLY under that assumption. If
    // a future IDF release introduces per-connection worker threads — or
    // we ever set HTTPD_DEFAULT_CONFIG flags that spawn them — every one
    // of those statics needs to become a mutex-guarded shared buffer or
    // a per-handler heap alloc. See V2.4.22 CHANGELOG for the audit list.
    // V2.4.13: bump per-recv-call timeout 5 s → 30 s for weak-WiFi OTA
    // resilience. The OTA POST streams ~1.2 MB in ~1200 recv calls; the
    // default 5 s window meant a single TCP gap >5 s killed the entire
    // upload mid-flash. 30 s rides through most transient drops while
    // leaving plenty of headroom against a truly dead connection (the
    // retry-on-timeout loop in update_post then gives 6× more on top).
    // Harmless for other routes: the timeout only ticks during ACTIVE
    // recv calls (request body reads); idle keep-alive connections don't
    // consume it. Send-side keeps the default — responses are tiny.
    hc.recv_wait_timeout = 30;

    esp_err_t err = httpd_start(&s_server, &hc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        s_server = NULL;
        return;
    }

    static const httpd_uri_t uri_favicon = {
        .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_get,
    };
    static const httpd_uri_t uri_root = {
        .uri = "/", .method = HTTP_GET, .handler = status_get,
    };
    static const httpd_uri_t uri_config_get = {
        .uri = "/config", .method = HTTP_GET, .handler = config_get,
    };
    static const httpd_uri_t uri_config_post = {
        .uri = "/config", .method = HTTP_POST, .handler = config_post,
    };
    static const httpd_uri_t uri_update_get = {
        .uri = "/update", .method = HTTP_GET, .handler = update_get,
    };
    static const httpd_uri_t uri_update_post = {
        .uri = "/update", .method = HTTP_POST, .handler = update_post,
    };
    static const httpd_uri_t uri_reboot_post = {
        .uri = "/reboot", .method = HTTP_POST, .handler = reboot_post,
    };
    static const httpd_uri_t uri_log_get = {
        .uri = "/log", .method = HTTP_GET, .handler = log_get,
    };
    static const httpd_uri_t uri_coredump_get = {
        .uri = "/coredump.elf", .method = HTTP_GET, .handler = coredump_get,
    };
    static const httpd_uri_t uri_coredump_erase = {
        .uri = "/coredump_erase", .method = HTTP_POST, .handler = coredump_erase_post,
    };
    httpd_register_uri_handler(s_server, &uri_favicon);
    httpd_register_uri_handler(s_server, &uri_root);
    httpd_register_uri_handler(s_server, &uri_config_get);
    httpd_register_uri_handler(s_server, &uri_config_post);
    httpd_register_uri_handler(s_server, &uri_update_get);
    httpd_register_uri_handler(s_server, &uri_update_post);
    httpd_register_uri_handler(s_server, &uri_reboot_post);
    httpd_register_uri_handler(s_server, &uri_log_get);
    httpd_register_uri_handler(s_server, &uri_coredump_get);
    httpd_register_uri_handler(s_server, &uri_coredump_erase);
    ESP_LOGI(TAG, "HTTP server listening on :80 (routes: / /favicon.ico /config /update /reboot /log /coredump.elf /coredump_erase)");
}
