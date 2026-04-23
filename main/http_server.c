#include "http_server.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "mbedtls/base64.h"

#include "version.h"
#include "applog.h"

static const char *TAG = "http";

static httpd_handle_t s_server   = NULL;
static config_t      *s_cfg      = NULL;
static const char    *s_chip_id  = "";
static char           s_mac_str[18] = "??:??:??:??:??:??";   // filled at start
static volatile bool  s_restart_requested = false;

// --- Access log --------------------------------------------------------------
// Logs every incoming request with its URI and client IP. Called at the top of
// every handler, before auth checks, so unauthorised attempts are visible too.

static void log_access(httpd_req_t *req, const char *what) {
    char ipstr[48] = "?";
    int sockfd = httpd_req_to_sockfd(req);
    if (sockfd >= 0) {
        struct sockaddr_storage addr;
        socklen_t len = sizeof(addr);
        if (getpeername(sockfd, (struct sockaddr *)&addr, &len) == 0) {
            if (addr.ss_family == AF_INET) {
                inet_ntop(AF_INET,
                          &((struct sockaddr_in *)&addr)->sin_addr,
                          ipstr, sizeof(ipstr));
            } else if (addr.ss_family == AF_INET6) {
                inet_ntop(AF_INET6,
                          &((struct sockaddr_in6 *)&addr)->sin6_addr,
                          ipstr, sizeof(ipstr));
            }
        }
    }
    ESP_LOGI(TAG, "%s from %s", what, ipstr);
}

// --- Auth --------------------------------------------------------------------

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
    if (strcmp(header + 6, (char *)expected) != 0) goto unauth;
    return true;

unauth:
    if (header_present) {
        ESP_LOGW(TAG, "auth FAILED for %s", req->uri);
    }
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"MultiGeiger\"");
    httpd_resp_send(req, NULL, 0);
    return false;
}

// --- HTML escape for attribute values ---------------------------------------
// Only `&` and `"` matter in value="..." — escape both. `<` and `>` are legal
// in attribute values but we escape them too for tag-safety.

static void html_esc(const char *in, char *out, size_t bufsz) {
    size_t o = 0;
    while (*in && o + 7 < bufsz) {
        switch (*in) {
            case '&':  memcpy(out + o, "&amp;",  5); o += 5; break;
            case '"':  memcpy(out + o, "&quot;", 6); o += 6; break;
            case '<':  memcpy(out + o, "&lt;",   4); o += 4; break;
            case '>':  memcpy(out + o, "&gt;",   4); o += 4; break;
            default:   out[o++] = *in;
        }
        in++;
    }
    out[o] = 0;
}

// --- URL decode (in place) ---------------------------------------------------

static int hex_nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void url_decode(char *s) {
    char *w = s;
    while (*s) {
        if (*s == '+') {
            *w++ = ' ';
            s++;
        } else if (*s == '%' && s[1] && s[2]) {
            int hi = hex_nibble(s[1]);
            int lo = hex_nibble(s[2]);
            if (hi >= 0 && lo >= 0) {
                *w++ = (char)((hi << 4) | lo);
                s += 3;
            } else {
                *w++ = *s++;
            }
        } else {
            *w++ = *s++;
        }
    }
    *w = 0;
}

// --- GET / (status, no auth) -------------------------------------------------

static void format_uptime(unsigned long s, char *out, size_t sz) {
    unsigned long m = s / 60;
    unsigned long h = m / 60;
    unsigned long d = h / 24;
    if (d > 0) {
        snprintf(out, sz, "%lud %02luh %02lum", d, h % 24, m % 60);
    } else {
        snprintf(out, sz, "%02luh %02lum %02lus", h, m % 60, s % 60);
    }
}

static void format_net_info(char *out, size_t sz) {
    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode);

    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    wifi_ap_record_t ap;
    bool sta_up = (sta != NULL) && (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);

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
        snprintf(out, sz,
                 "<b>IP:</b> %s<br>"
                 "<b>Gateway:</b> %s<br>"
                 "<b>Network Mask:</b> %s<br>"
                 "<b>DNS:</b> %s%s%s<br>"
                 "<b>WiFi Strength:</b> %d dBm<br>",
                 ip_s, gw_s, nm_s,
                 d1_s, has_d2 ? ", " : "", has_d2 ? d2_s : "",
                 (int)ap.rssi);
        return;
    }

    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA) {
        esp_netif_t *apn = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        esp_netif_ip_info_t ip = { 0 };
        if (apn) esp_netif_get_ip_info(apn, &ip);
        char ip_s[16];
        esp_ip4addr_ntoa(&ip.ip, ip_s, sizeof(ip_s));
        snprintf(out, sz, "<b>IP:</b> %s (AP mode)<br>", ip_s);
        return;
    }

    snprintf(out, sz, "<b>Network:</b> No connection<br>");
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

static esp_err_t status_get(httpd_req_t *req) {
    log_access(req, "GET /");
    char body[1700];
    unsigned long uptime_s = (unsigned long)(esp_timer_get_time() / 1000000);
    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t max_alloc = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    char uptime_buf[32];
    format_uptime(uptime_s, uptime_buf, sizeof(uptime_buf));
    char net_info[400];
    format_net_info(net_info, sizeof(net_info));

    int n = snprintf(body, sizeof(body),
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<link rel=\"icon\" type=\"image/svg+xml\" href=\"/favicon.ico\">"
        "<title>MultiGeiger</title>"
        "<style>body{font-family:sans-serif;max-width:600px;margin:20px auto;padding:0 10px}"
        "h1{color:#333}a{color:#0066cc}"
        ".info{background:#f5f5f5;border:1px solid #ddd;padding:10px;border-radius:4px;margin:10px 0}"
        "</style></head><body>"
        "<h1>MultiGeiger %s</h1>"
        "<div class=\"info\">"
        "<b>Chip ID:</b> %s<br>"
        "<b>MAC:</b> %s<br>"
        "%s"
        "<b>AP SSID:</b> %s<br>"
        "<b>Free heap:</b> %lu bytes<br>"
        "<b>Max allocation:</b> %lu bytes<br>"
        "<b>Uptime:</b> %s"
        "</div>"
        "<p><a href=\"/config\">&#9881; Configuration</a> (requires password)</p>"
        "<p><a href=\"/update\">&#11014; Firmware Update (OTA)</a> (requires password)</p>"
        "<p><a href=\"/log\">&#128221; View log buffer</a> (requires password)</p>"
        "</body></html>",
        VERSION_STR,
        s_chip_id,
        s_mac_str,
        net_info,
        s_cfg->ap_name,
        (unsigned long)free_heap,
        (unsigned long)max_alloc,
        uptime_buf);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, body, n > 0 ? n : 0);
}

// --- GET /config (auth'd form) -----------------------------------------------

#define CFG_FORM_BUF_SIZE 6144

static esp_err_t config_get(httpd_req_t *req) {
    log_access(req, "GET /config");
    if (!check_auth(req)) return ESP_OK;

    char *body = malloc(CFG_FORM_BUF_SIZE);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_OK;
    }

    // Escape every string field for safe use in value="..." attributes.
    char e_ssid[96], e_pw[192], e_chip[96], e_ru[96], e_rp[192];
    char e_ntp1[192], e_ntp2[192], e_ntp3[192], e_ap[96];
    char e_tz[160];
    char e_apn[96], e_host[96];
    char e_fhost[192], e_fuser[96], e_fpw[192], e_fpath[192];
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

    int n = snprintf(body, CFG_FORM_BUF_SIZE,
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<title>Config — MultiGeiger V2</title>"
        "<style>body{font-family:system-ui;max-width:40em;margin:2em auto;padding:0 1em}"
        "label{display:block;margin-top:.8em}"
        "input[type=text],input[type=password],input[type=number]{width:100%%;padding:.4em;box-sizing:border-box}"
        "input[type=submit]{padding:.6em 1.2em;margin-top:1.2em;font-size:1em}"
        ".chk{display:inline-block;margin-right:1em;margin-top:.4em}</style>"
        "</head><body><h1>Configuration</h1>"
        "<form method=\"post\" action=\"/config\">"
        "<label>WiFi SSID<input type=\"text\" name=\"wifi_ssid\" value=\"%s\"></label>"
        "<label>WiFi password<input type=\"password\" name=\"wifi_pw\" value=\"%s\"></label>"
        "<label>DHCP hostname (visible in router) "
        "<input type=\"text\" name=\"wifi_host\" value=\"%s\" maxlength=\"32\"></label>"
        "<label>AP SSID (used in AP / fallback mode) "
        "<input type=\"text\" name=\"ap_name\" value=\"%s\" maxlength=\"32\"></label>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"wifi_11bg\" "
        "id=\"wifi_11bg\" onchange=\"syncHt20()\" %s> Limit to 802.11b/g</label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"wifi_ht20\" "
        "id=\"wifi_ht20\" %s> Limit to 20MHz</label></div>"
        "<script>function syncHt20(){"
        "var bg=document.getElementById('wifi_11bg');"
        "var ht=document.getElementById('wifi_ht20');"
        "if(bg.checked){ht.checked=true;ht.disabled=true;}"
        "else{ht.disabled=false;}"
        "}syncHt20();</script>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"wifi_ps_dis\" %s> "
        "Disable WiFi power save (always-on radio; may reduce mesh re-keying drops)</label></div>"
        "<p>Chip ID (auto-derived from MAC): <code>%s</code><br>"
        "MAC: <code>%s</code></p>"
        "<h3>Transmission targets</h3>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_mad\" %s> Madavi</label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"mad_https\" %s> HTTPS</label></div><br>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_sc\" %s> sensor.community</label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"sc_https\" %s> HTTPS</label></div><br>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_rad\" %s> Radmon</label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"rad_https\" %s> HTTPS</label></div>"
        "<label>Radmon user<input type=\"text\" name=\"rad_user\" value=\"%s\"></label>"
        "<label>Radmon password<input type=\"password\" name=\"rad_pw\" value=\"%s\"></label>"
        "<h3>BME280 (environmental)</h3>"
        "<label>Station altitude (m above sea level) "
        "<input type=\"number\" step=\"0.1\" name=\"alt_m\" value=\"%.1f\"></label>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"send_sl\" %s> "
        "Send pressure-at-sealevel to sensor.community</label></div>"
        "<h3>FTP log upload</h3>"
        "<p>Periodically uploads the in-memory log ring (same content as "
        "<a href=\"/log\">/log</a>) to a LAN FTP server. Passive mode. "
        "Leave user/password empty for anonymous login.</p>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"ftp_en\" %s> "
        "Enable FTP log upload</label></div>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"ftp_tls\" %s> "
        "Use explicit TLS (AUTH TLS on port 21) &mdash; certificate NOT verified</label></div>"
        "<label>FTP host<input type=\"text\" name=\"ftp_host\" value=\"%s\"></label>"
        "<label>FTP user (blank = anonymous)<input type=\"text\" name=\"ftp_user\" value=\"%s\"></label>"
        "<label>FTP password<input type=\"password\" name=\"ftp_pw\" value=\"%s\"></label>"
        "<label>Remote directory (e.g. /geiger)<input type=\"text\" name=\"ftp_path\" value=\"%s\"></label>"
        "<label>Upload interval (minutes)<input type=\"number\" name=\"ftp_int\" value=\"%lu\" min=\"1\" max=\"1440\"></label>"
        "<h3>Tick, LED and display</h3>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"sp_tick\" %s> "
        "Speaker tick on each GM pulse</label></div><br>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"led_tick\" %s> "
        "LED flash on each GM pulse</label></div><br>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"play_sound\" %s> "
        "Play boot chirp</label></div><br>"
        "<div class=\"chk\"><label><input type=\"checkbox\" name=\"show_disp\" %s> "
        "Drive OLED display</label></div>"
        "<h3>Other</h3>"
        "<label>NTP server 1<input type=\"text\" name=\"ntp1\" value=\"%s\"></label>"
        "<label>NTP server 2 (optional)<input type=\"text\" name=\"ntp2\" value=\"%s\"></label>"
        "<label>NTP server 3 (optional)<input type=\"text\" name=\"ntp3\" value=\"%s\"></label>"
        "<label>Timezone (POSIX TZ)<input type=\"text\" name=\"tz_posix\" value=\"%s\" maxlength=\"47\">"
        "<small>e.g. <code>AEST-10AEDT,M10.1.0,M4.1.0/3</code> (Sydney), "
        "<code>CET-1CEST,M3.5.0,M10.5.0/3</code> (Germany), "
        "<code>UTC0</code> (UTC). See <code>man tzset</code>.</small></label>"
        "<label>Web admin and access point password<input type=\"password\" name=\"ap_pw\" value=\"%s\"></label>"
        "<label>Sensor data upload interval (ms)<input type=\"number\" name=\"tx_int_ms\" value=\"%lu\" min=\"10000\" max=\"3600000\"></label>"
        "<input type=\"submit\" value=\"Save and restart\">"
        "</form>"
        "<h3>Reboot</h3>"
        "<form method=\"post\" action=\"/reboot\" "
        "onsubmit=\"return confirm('Reboot the device now?');\">"
        "<input type=\"submit\" value=\"Reboot now\">"
        "</form>"
        "<p><a href=\"/\">Back to status</a> &middot; "
        "<a href=\"/update\">Firmware update</a></p>"
        "</body></html>",
        e_ssid, e_pw, e_host, e_apn,
        s_cfg->wifi_11bg_only   ? "checked" : "",
        s_cfg->wifi_ht20_only   ? "checked" : "",
        s_cfg->wifi_ps_disabled ? "checked" : "",
        e_chip, s_mac_str,
        s_cfg->send_madavi  ? "checked" : "",
        s_cfg->madavi_https ? "checked" : "",
        s_cfg->send_sensorc ? "checked" : "",
        s_cfg->sensorc_https ? "checked" : "",
        s_cfg->send_radmon  ? "checked" : "",
        s_cfg->radmon_https ? "checked" : "",
        e_ru, e_rp,
        (double)s_cfg->station_altitude_m,
        s_cfg->send_sealevel_pressure ? "checked" : "",
        s_cfg->ftp_enabled ? "checked" : "",
        s_cfg->ftp_tls     ? "checked" : "",
        e_fhost, e_fuser, e_fpw, e_fpath,
        (unsigned long)s_cfg->ftp_interval_min,
        s_cfg->speaker_tick ? "checked" : "",
        s_cfg->led_tick     ? "checked" : "",
        s_cfg->play_sound   ? "checked" : "",
        s_cfg->show_display ? "checked" : "",
        e_ntp1, e_ntp2, e_ntp3, e_tz, e_ap,
        (unsigned long)s_cfg->tx_interval_ms);

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    esp_err_t err = httpd_resp_send(req, body, n > 0 ? n : 0);
    free(body);
    return err;
}

// --- POST /config (parse form, save, flag restart) --------------------------

static void assign_str(char *dst, size_t dstsz, const char *src) {
    strncpy(dst, src, dstsz - 1);
    dst[dstsz - 1] = 0;
}

static esp_err_t config_post(httpd_req_t *req) {
    log_access(req, "POST /config");
    if (!check_auth(req)) return ESP_OK;

    int total = req->content_len;
    if (total <= 0 || total > 4096) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body size out of range");
        return ESP_OK;
    }
    char *buf = malloc(total + 1);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_OK;
    }
    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, buf + received, total - received);
        if (r <= 0) {
            free(buf);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_OK;
        }
        received += r;
    }
    buf[total] = 0;

    // Work on a local copy so a parse failure can't half-apply. Booleans
    // default to false — form only includes the name when the box is checked.
    config_t next = *s_cfg;
    next.send_madavi            = false;
    next.madavi_https           = false;
    next.send_sensorc           = false;
    next.sensorc_https          = false;
    next.send_radmon            = false;
    next.radmon_https           = false;
    next.send_sealevel_pressure = false;
    next.ftp_enabled            = false;
    next.ftp_tls                = false;
    next.speaker_tick           = false;
    next.led_tick               = false;
    next.play_sound             = false;
    next.show_display           = false;
    next.wifi_11bg_only         = false;
    next.wifi_ht20_only         = false;
    next.wifi_ps_disabled       = false;

    char *p = buf;
    while (*p) {
        char *eq = strchr(p, '=');
        if (!eq) break;
        *eq = 0;
        char *val = eq + 1;
        char *amp = strchr(val, '&');
        if (amp) *amp = 0;
        url_decode(val);

        if      (!strcmp(p, "wifi_ssid")) assign_str(next.wifi_ssid,     sizeof(next.wifi_ssid),     val);
        else if (!strcmp(p, "wifi_pw"))   assign_str(next.wifi_password, sizeof(next.wifi_password), val);
        else if (!strcmp(p, "wifi_host")) assign_str(next.wifi_hostname, sizeof(next.wifi_hostname), val);
        else if (!strcmp(p, "ap_name"))   assign_str(next.ap_name,       sizeof(next.ap_name),       val);
        else if (!strcmp(p, "wifi_11bg"))  next.wifi_11bg_only   = true;
        else if (!strcmp(p, "wifi_ht20"))  next.wifi_ht20_only   = true;
        else if (!strcmp(p, "wifi_ps_dis")) next.wifi_ps_disabled = true;
        else if (!strcmp(p, "send_mad"))  next.send_madavi   = true;
        else if (!strcmp(p, "mad_https")) next.madavi_https  = true;
        else if (!strcmp(p, "send_sc"))   next.send_sensorc  = true;
        else if (!strcmp(p, "sc_https"))  next.sensorc_https = true;
        else if (!strcmp(p, "send_rad"))  next.send_radmon   = true;
        else if (!strcmp(p, "rad_https")) next.radmon_https  = true;
        else if (!strcmp(p, "rad_user"))  assign_str(next.radmon_user,     sizeof(next.radmon_user),     val);
        else if (!strcmp(p, "rad_pw"))    assign_str(next.radmon_password, sizeof(next.radmon_password), val);
        else if (!strcmp(p, "ntp1"))      assign_str(next.ntp_server,      sizeof(next.ntp_server),      val);
        else if (!strcmp(p, "ntp2"))      assign_str(next.ntp_server2,     sizeof(next.ntp_server2),     val);
        else if (!strcmp(p, "ntp3"))      assign_str(next.ntp_server3,     sizeof(next.ntp_server3),     val);
        else if (!strcmp(p, "tz_posix"))  assign_str(next.tz_posix,        sizeof(next.tz_posix),        val);
        else if (!strcmp(p, "ap_pw"))     assign_str(next.ap_password,     sizeof(next.ap_password),     val);
        else if (!strcmp(p, "tx_int_ms")) {
            long v = strtol(val, NULL, 10);
            if (v >= 10000 && v <= 3600000) next.tx_interval_ms = (uint32_t)v;
        }
        else if (!strcmp(p, "alt_m")) {
            float v = strtof(val, NULL);
            if (v >= -500.0f && v <= 9000.0f) next.station_altitude_m = v;
        }
        else if (!strcmp(p, "send_sl")) next.send_sealevel_pressure = true;
        else if (!strcmp(p, "ftp_en"))  next.ftp_enabled = true;
        else if (!strcmp(p, "ftp_tls")) next.ftp_tls     = true;
        else if (!strcmp(p, "ftp_host")) assign_str(next.ftp_host,     sizeof(next.ftp_host),     val);
        else if (!strcmp(p, "ftp_user")) assign_str(next.ftp_user,     sizeof(next.ftp_user),     val);
        else if (!strcmp(p, "ftp_pw"))   assign_str(next.ftp_password, sizeof(next.ftp_password), val);
        else if (!strcmp(p, "ftp_path")) assign_str(next.ftp_path,     sizeof(next.ftp_path),     val);
        else if (!strcmp(p, "ftp_int")) {
            long v = strtol(val, NULL, 10);
            if (v >= 1 && v <= 1440) next.ftp_interval_min = (uint32_t)v;
        }
        else if (!strcmp(p, "sp_tick"))    next.speaker_tick = true;
        else if (!strcmp(p, "led_tick"))   next.led_tick     = true;
        else if (!strcmp(p, "play_sound")) next.play_sound   = true;
        else if (!strcmp(p, "show_disp"))  next.show_display = true;

        if (!amp) break;
        p = amp + 1;
    }
    free(buf);

    // 802.11b/g channels are always 20 MHz — HT40 only exists under 11n.
    // A disabled checkbox doesn't POST, so the form may send wifi_ht20=0
    // even while the UI showed it ticked; enforce the invariant here so
    // the stored state matches what the user saw.
    if (next.wifi_11bg_only) next.wifi_ht20_only = true;

    *s_cfg = next;
    esp_err_t err = config_save(s_cfg);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_OK;
    }
    s_restart_requested = true;
    ESP_LOGI(TAG, "config saved via POST — restart flagged");

    const char *ok =
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<title>Saved</title></head><body>"
        "<h1>Saved. Restarting...</h1>"
        "<p>Device will restart in ~2 seconds. Your browser will drop the "
        "connection; reconnect to the new WiFi settings if you changed them.</p>"
        "</body></html>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, ok, HTTPD_RESP_USE_STRLEN);
}

// --- POST /reboot (manual restart button) ----------------------------------

static esp_err_t reboot_post(httpd_req_t *req) {
    log_access(req, "POST /reboot");
    if (!check_auth(req)) return ESP_OK;
    s_restart_requested = true;
    ESP_LOGW(TAG, "manual reboot requested via /reboot");
    const char *ok =
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<title>Rebooting</title></head><body>"
        "<h1>Rebooting...</h1>"
        "<p>Device will restart in ~2 seconds.</p>"
        "</body></html>";
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, ok, HTTPD_RESP_USE_STRLEN);
}

// --- GET /update (upload form) ----------------------------------------------
// Uses XHR to POST the raw .bin as the request body (Content-Type:
// application/octet-stream). Avoids multipart parsing on the device.

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
        "<p>Select a firmware .bin (from firmware_releases/ or build/geiger_v2.bin).</p>"
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
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

// --- POST /update (stream body into OTA partition) --------------------------

#define OTA_CHUNK 1024

static esp_err_t update_post(httpd_req_t *req) {
    log_access(req, "POST /update");
    if (!check_auth(req)) return ESP_OK;

    int total = req->content_len;
    if (total <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty body");
        return ESP_OK;
    }

    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA slot");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "OTA: %d bytes -> partition %s @ 0x%lx",
             total, target->label, (unsigned long)target->address);

    esp_ota_handle_t ota = 0;
    esp_err_t err = esp_ota_begin(target, total, &ota);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_OK;
    }

    char *buf = malloc(OTA_CHUNK);
    if (!buf) {
        esp_ota_abort(ota);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_OK;
    }

    int received = 0;
    while (received < total) {
        int want = total - received;
        if (want > OTA_CHUNK) want = OTA_CHUNK;
        int r = httpd_req_recv(req, buf, want);
        if (r <= 0) {
            ESP_LOGE(TAG, "recv failed at %d/%d (r=%d)", received, total, r);
            free(buf);
            esp_ota_abort(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_OK;
        }
        err = esp_ota_write(ota, buf, r);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed at %d: %s", received, esp_err_to_name(err));
            free(buf);
            esp_ota_abort(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
            return ESP_OK;
        }
        received += r;
    }
    free(buf);

    err = esp_ota_end(ota);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_OK;
    }
    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_OK;
    }

    s_restart_requested = true;
    ESP_LOGW(TAG, "OTA written (%d bytes) — restart flagged", total);

    const char *ok = "OTA OK — restarting in ~2s";
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req, ok, HTTPD_RESP_USE_STRLEN);
}

// --- GET /log (auth'd, streams the ring buffer as text/plain) ---------------

#define LOG_CHUNK 2048

static esp_err_t log_get(httpd_req_t *req) {
    log_access(req, "GET /log");
    if (!check_auth(req)) return ESP_OK;

    size_t len = 0;
    char *snap = applog_snapshot(&len);
    if (!snap) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    size_t sent = 0;
    while (sent < len) {
        size_t n = len - sent;
        if (n > LOG_CHUNK) n = LOG_CHUNK;
        if (httpd_resp_send_chunk(req, snap + sent, n) != ESP_OK) {
            free(snap);
            return ESP_FAIL;
        }
        sent += n;
    }
    httpd_resp_send_chunk(req, NULL, 0);  // terminate
    free(snap);
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
    hc.max_uri_handlers = 10;            // / /favicon.ico /config GET+POST /update GET+POST /reboot /log
    hc.lru_purge_enable = true;

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
    httpd_register_uri_handler(s_server, &uri_favicon);
    httpd_register_uri_handler(s_server, &uri_root);
    httpd_register_uri_handler(s_server, &uri_config_get);
    httpd_register_uri_handler(s_server, &uri_config_post);
    httpd_register_uri_handler(s_server, &uri_update_get);
    httpd_register_uri_handler(s_server, &uri_update_post);
    httpd_register_uri_handler(s_server, &uri_reboot_post);
    httpd_register_uri_handler(s_server, &uri_log_get);
    ESP_LOGI(TAG, "HTTP server listening on :80 (routes: / /favicon.ico /config /update /reboot /log)");
}

bool http_server_restart_requested(void) {
    return s_restart_requested;
}
