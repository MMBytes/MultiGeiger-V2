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
#include "http_server.h"       // http_server_transport_str — TLS line in banner (V2.8.1)
#include "tls_cert.h"          // tls_cert_boot_summary — TLS line in banner (V2.8.1)
#include "ntp.h"               // ntp_time_valid — the clock-sane gate
#include "esp_ota_ops.h"       // esp_ota_get_running_partition — boot slot in banner
#include "esp_flash.h"         // esp_flash_default_chip — flash line in banner (V2.8.6)
#include "esp_flash_chips/spi_flash_chip_driver.h"   // spi_flash_chip_t::name
// V2.8.6: further banner lines (Build / SoC / PSRAM / Sensors)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"     // vTaskDelay — per-line pacing of the banner
#include "esp_app_desc.h"      // esp_app_get_description, esp_app_get_elf_sha256
#include "esp_bootloader_desc.h"   // esp_bootloader_desc_t
#include "esp_secure_boot.h"   // esp_secure_boot_enabled
#include "esp_efuse.h"         // esp_efuse_get_pkg_ver, esp_efuse_is_flash_encryption_enabled
                               // (esp_flash_encrypt.h's esp_flash_encryption_enabled is deprecated in IDF 6)
#include "esp_efuse_table.h"   // ESP_EFUSE_FLASH_CAP / ESP_EFUSE_PSRAM_CAP (S3 / C5 SoC line)
#include "esp_clk_tree.h"      // esp_clk_tree_src_get_freq_hz — CPU / XTAL clock
#if HAL_HAS_PSRAM
#include "esp_psram.h"         // esp_psram_get_size
#endif
#include "env_sensor.h"        // env_sensor_present / env_sensor_name
#include "pm_sensor.h"         // pm_sensor_present / pm_sensor_name
#include "noise_sensor.h"      // noise_sensor_present / noise_sensor_name
#include "sgp41.h"             // sgp41_present
#include "veml7700.h"          // veml7700_present
#include "als.h"               // als_present
#include "gnss.h"              // gnss_present / gnss_chip_name
#include "fuel_gauge.h"        // fuel_gauge_chip_ready / fuel_gauge_present
#include "tube.h"              // tube_is_enabled

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

// --- V2.8.6: boot banner detail lines ---------------------------------------
// Hardware and build facts that until now existed only on the serial console
// (IDF's own startup prints) or in the RAM /log, so server-side forensics
// could not tell which build, bootloader, module variant or sensor set a node
// had. Each is its OWN "boot: <Key>:" line — the "boot: Firmware" line stays
// byte-identical because ~10 docs/scripts parsers match it up to "Chip:" and
// count boots by "Firmware V". All are called from syslog_init() on the main
// task after the socket is live, so every line goes straight to the server.

// Flash bus mode / clock chosen in menuconfig. Not CONFIG_ESPTOOLPY_FLASHMODE
// / _FLASHFREQ: those are the esptool image-HEADER strings, which IDF folds
// on purpose (QIO and QOUT are written as "dio", 120M as "80m") — so they
// would keep reading "dio 80m" after a switch to QIO. Today all 11 boards
// are DIO at 40 or 80 MHz.
#if CONFIG_ESPTOOLPY_FLASHMODE_QIO
#define BANNER_FLASH_MODE "qio"
#elif CONFIG_ESPTOOLPY_FLASHMODE_QOUT
#define BANNER_FLASH_MODE "qout"
#elif CONFIG_ESPTOOLPY_FLASHMODE_DIO
#define BANNER_FLASH_MODE "dio"
#elif CONFIG_ESPTOOLPY_FLASHMODE_DOUT
#define BANNER_FLASH_MODE "dout"
#elif CONFIG_ESPTOOLPY_FLASHMODE_OPI
#define BANNER_FLASH_MODE "opi"
#else
#define BANNER_FLASH_MODE "?"
#endif
#if CONFIG_ESPTOOLPY_FLASHFREQ_120M
#define BANNER_FLASH_FREQ "120m"
#else
#define BANNER_FLASH_FREQ CONFIG_ESPTOOLPY_FLASHFREQ   // only 120M is folded
#endif

// The banner grew from 5 to 9 lines sent back to back. On the tight-heap
// heltec_v2 a burst like that outran the WiFi/lwIP drain once already
// (V2.5.29, the config dump) — same remedy: a 1-tick yield after each line.
#define BANNER_PACE() vTaskDelay(pdMS_TO_TICKS(10))

// Build identity: which exact image and which bootloader. Two builds can carry
// the same VERSION_STR (an OTA of V2.8.5 -> V2.8.5 is in the 2026-10 logs),
// so the ELF SHA-256 prefix is what ties a node — and a coredump pulled from
// it — to one specific ELF. IDF keeps only CONFIG_APP_RETRIEVE_LEN_ELF_SHA hex
// digits of it in RAM (default 9, unchanged here): the same prefix the panic
// handler prints as "ELF file SHA256:", so the two match character for
// character. The bootloader is never touched by OTA, so each node runs
// whatever its first USB flash installed; IDF documents features that depend
// on its age. Read from flash with esp_ota_get_bootloader_description(NULL) —
// one ~80-byte SPI read, so cache and non-IRAM interrupts are off for its
// duration (fine on the main task; the HV tick is cache-safe since V2.8.6).
// esp_bootloader_get_description() would instead return the APP's own weak
// copy of the struct (the app's IDF version), not the bootloader's.
// Bootloaders older than the descriptor format return ESP_ERR_NOT_FOUND.
// Secure boot: on the plain ESP32 esp_secure_boot_enabled() is a build-config
// constant (false unless secure boot is compiled in); S3/C5 read the eFuse.
// Flash encryption is a live eFuse read on all three.
static void banner_build(void) {
    char elf[CONFIG_APP_RETRIEVE_LEN_ELF_SHA + 1];  // hex digits + NUL
    esp_app_get_elf_sha256(elf, sizeof(elf));
    const esp_app_desc_t *app = esp_app_get_description();

    esp_bootloader_desc_t bl;
    char bl_str[80];
    const esp_err_t bl_err = esp_ota_get_bootloader_description(NULL, &bl);
    if (bl_err == ESP_OK) {
        // idf_ver / date_time are fixed-size fields: bound the reads.
        snprintf(bl_str, sizeof(bl_str), "IDF %.32s (%.24s)",
                 bl.idf_ver, bl.date_time[0] ? bl.date_time : "no date");
    } else {
        snprintf(bl_str, sizeof(bl_str), "no descriptor (%s)", esp_err_to_name(bl_err));
    }
    ESP_LOGI("boot", "Build: ELF %s - built %.16s %.16s - bootloader %s - "
                     "secure boot %s, flash encryption %s",
             elf, app->date, app->time, bl_str,
             esp_secure_boot_enabled() ? "ON" : "off",
             esp_efuse_is_flash_encryption_enabled() ? "ON" : "off");
    BANNER_PACE();
}

// Module variant and clocks. Package version (eFuse) is printed raw: its
// numbering is per chip family (TRM eFuse tables). What is in the package
// differs by family:
//  - ESP32: esp_chip_info() derives CHIP_FEATURE_EMB_FLASH / EMB_PSRAM from
//    the package ID (e.g. PICO-V3-02, D0WDR2-V3 carry PSRAM), so those bits
//    are meaningful there and are printed as emb_flash / emb_psram.
//  - ESP32-S3 / C5: IDF never sets those two bits; in-package flash and PSRAM
//    are separate eFuse fields (FLASH_CAP, PSRAM_CAP — on the S3 e.g. PSRAM_CAP
//    {0 none, 1 8M, 2 2M, 3 16M, 4 4M}), printed raw as flash_cap / psram_cap.
//    This is what separates an N8 from an N8R8 module with the same pkg value.
// Every timing figure in this project was measured at a known CPU clock, so
// the live value is logged rather than CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ (a
// register read here, no calibration). 0 if the clock query fails.
static void banner_soc(const esp_chip_info_t *chip) {
    const uint32_t f = chip->features;
    uint32_t cpu_hz = 0, xtal_hz = 0;
    if (esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_CPU, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                                     &cpu_hz) != ESP_OK) cpu_hz = 0;
    if (esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_XTAL, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                                     &xtal_hz) != ESP_OK) xtal_hz = 0;
    char in_pkg[40];
#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C5
    uint8_t flash_cap = 0, psram_cap = 0;   // fields are 2-3 bits wide
    esp_efuse_read_field_blob(ESP_EFUSE_FLASH_CAP, &flash_cap,
                              (size_t)esp_efuse_get_field_size(ESP_EFUSE_FLASH_CAP));
    esp_efuse_read_field_blob(ESP_EFUSE_PSRAM_CAP, &psram_cap,
                              (size_t)esp_efuse_get_field_size(ESP_EFUSE_PSRAM_CAP));
    snprintf(in_pkg, sizeof(in_pkg), "flash_cap=%u psram_cap=%u",
             (unsigned)flash_cap, (unsigned)psram_cap);
#else
    snprintf(in_pkg, sizeof(in_pkg), "emb_flash=%d emb_psram=%d",
             (f & CHIP_FEATURE_EMB_FLASH) ? 1 : 0, (f & CHIP_FEATURE_EMB_PSRAM) ? 1 : 0);
#endif
    ESP_LOGI("boot", "SoC: %s pkg %lu - wifi=%d bt=%d ble=%d 802.15.4=%d %s - "
                     "CPU %lu MHz, XTAL %lu MHz",
             chip_model_str(chip->model), (unsigned long)esp_efuse_get_pkg_ver(),
             (f & CHIP_FEATURE_WIFI_BGN) ? 1 : 0, (f & CHIP_FEATURE_BT) ? 1 : 0,
             (f & CHIP_FEATURE_BLE) ? 1 : 0, (f & CHIP_FEATURE_IEEE802154) ? 1 : 0,
             in_pkg,
             (unsigned long)(cpu_hz / 1000000), (unsigned long)(xtal_hz / 1000000));
    BANNER_PACE();
}

// PSRAM as built and found. applog logs the same facts at ring setup, but
// that is before WiFi, so they reached /log only — never the server.
static void banner_psram(void) {
#if HAL_HAS_PSRAM
    ESP_LOGI("boot", "PSRAM: %u MB, mode=%s speed=%s",
             (unsigned)(esp_psram_get_size() >> 20), psram_mode_str(), psram_speed_str());
#else
    ESP_LOGI("boot", "PSRAM: none (board has none)");
#endif
    BANNER_PACE();
}

// What the boot probe found, one key per sensor class, "none" when absent —
// so a server log alone says which data a node can produce (e.g. whether it
// has its own T/RH or relies on a neighbour's). All probes have finished by
// syslog_init() time: main.c probes both I2C buses before WiFi starts.
// fuel= reports the CHIP (fuel_gauge_chip_ready), not fuel_gauge_present(),
// which also needs the user's "Battery attached" setting: a fitted gauge with
// that box unticked shows "MAX17048 (no battery set)" instead of "none".
// light= names both sources when a board has both (FeatherS3-D onboard
// ALS-PT19 plus a STEMMA VEML7700), in env_sensor_name()'s "A+B" style.
static void banner_sensors(void) {
    const bool veml = veml7700_present(), pt19 = als_present();
    const char *light = (veml && pt19) ? "VEML7700+ALS-PT19"
                      : veml ? "VEML7700" : pt19 ? "ALS-PT19" : "none";
    const char *fuel = !fuel_gauge_chip_ready() ? "none"
                     : fuel_gauge_present()     ? "MAX17048"
                                                : "MAX17048 (no battery set)";
    ESP_LOGI("boot", "Sensors: tube=%s env=%s pm=%s noise=%s voc=%s light=%s gnss=%s fuel=%s",
             tube_is_enabled() ? "on" : "off",
             env_sensor_present() ? env_sensor_name() : "none",
             pm_sensor_present() ? pm_sensor_name() : "none",
             noise_sensor_present() ? noise_sensor_name() : "none",
             sgp41_present() ? "SGP41" : "none",
             light,
             gnss_present() ? gnss_chip_name() : "none",
             fuel);
    BANNER_PACE();
}

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
    BANNER_PACE();

    // V2.8.6: build identity and module variant right after the summary.
    banner_build();
    banner_soc(&chip);

    // V2.6.30: attached display panel as a second banner line. The probe
    // verdict is logged by display.c long before WiFi + syslog exist, so —
    // like the banner above — it never reaches the server on its own.
    // display_setup() has always completed by syslog_init() time (main.c
    // boots the display before WiFi), so the string is final here.
    ESP_LOGI("boot", "Display: %s", display_backend_str());
    BANNER_PACE();

    // V2.8.6: flash part as a banner line. IDF prints only the driver family
    // ("detected chip: winbond") and only on the serial console at startup,
    // so the exact part was unknown for every deployed node. It mattered for
    // review 2.7: CONFIG_SPI_FLASH_AUTO_SUSPEND is allowed per JEDEC ID
    // (Winbond 0xEF4017 only, four GD parts, XMC-D, FM), and enabling it on an
    // unlisted part fails an assert at boot. JEDEC = manufacturer byte + 16-bit
    // device ID. chip_id and chip_drv were filled in by esp_flash_init at
    // startup, so reading them costs no flash transaction. The physical size
    // is decoded arithmetically from that same ID, but the call still brackets
    // it like any flash op (cache and non-IRAM interrupts off for a few µs,
    // other core stalled) — it sends no SPI command. "?" when the ID does not
    // follow the size-byte convention (driver returns UNSUPPORTED_CHIP).
    // The header size is what the image was built for; mode and clock are the
    // menuconfig choice (BANNER_FLASH_MODE / _FREQ above).
    {
        const esp_flash_t *fc = esp_flash_default_chip;
        uint32_t phys = 0;
        char phys_str[12] = "?";
        if (fc && esp_flash_get_physical_size(NULL, &phys) == ESP_OK && phys) {
            snprintf(phys_str, sizeof(phys_str), "%lu", (unsigned long)(phys >> 20));
        }
        ESP_LOGI("boot", "Flash: JEDEC 0x%06lX (driver %s) - %s MB physical, %lu MB header - "
                         "build mode %s %s",
                 fc ? (unsigned long)fc->chip_id : 0UL,
                 (fc && fc->chip_drv && fc->chip_drv->name) ? fc->chip_drv->name : "?",
                 phys_str, fc ? (unsigned long)(fc->size >> 20) : 0UL,
                 BANNER_FLASH_MODE, BANNER_FLASH_FREQ);
        BANNER_PACE();
    }

    banner_psram();
    banner_sensors();

    // V2.8.1: TLS verdict as a third banner line. The certificate load /
    // generation, both "listening" lines and the reconcile decision are all
    // logged before this socket exists (the reconcile even lands in the same
    // main-loop tick as this call, a few lines earlier), so without this line
    // the server never learns whether a node is on HTTPS, which certificate it
    // presents, or whether it re-issued and rebooted. On boards without HTTPS
    // the stubs make this "HTTP :80 — n/a".
    ESP_LOGI("boot", "TLS: %s — %s", http_server_transport_str(), tls_cert_boot_summary());
    BANNER_PACE();

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
    // V2.8.5: syslog's own time-string buffer. Pre-V2.8.5 ntp_localtime_str()
    // returned one static buffer shared with the SD logger (main task), so a
    // line logged from another task could pick up a half-written string.
    // BSS, not stack, for the same reason as s_emit_buf above (this runs on
    // whichever task logged, some with small stacks); single-threaded by the
    // same applog mutex.
    static char s_ts_buf[NTP_LOCALTIME_STR_LEN];
    const char *ts = ntp_time_valid() ? ntp_localtime_str(s_ts_buf, sizeof(s_ts_buf)) : "-";

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
