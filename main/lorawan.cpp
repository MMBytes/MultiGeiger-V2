// LoRaWAN uplink for BOARD_HELTEC_WIFI_LORA32_V4_R2 — the tree's ONLY C++
// translation unit, kept behind the plain-C API in lorawan.h. Whole file is
// gated on CONFIG_GEIGER_LORAWAN (set only in that board's sdkconfig): the
// CMake SRCS conditional keeps it out of other boards' builds, and this
// guard additionally yields an empty TU for the cppcheck gate's other-board
// define sets (_build.cmd scans all of main/ for every board).
#include "sdkconfig.h"
#if CONFIG_GEIGER_LORAWAN

#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "nvs.h"
#include "RadioLib.h"

extern "C" {
#include "hal.h"
#include "lorawan.h"
#include "lorawan_codec.h"
#include "config.h"
#include "applog.h"
#include "version.h"   // VERSION_STR -> s_version_packed (lw_pack_version)
}

static const char *TAG = "lorawan";

// RadioLib HAL on raw IDF-6 drivers. CONFIG_FREERTOS_HZ stays 100: delay()
// is a hybrid — whole ticks via vTaskDelay (scheduler-friendly for the
// multi-second RX-window waits), sub-tick remainder busy-waited on esp_timer
// so LoRaWAN's ms-scale window timing holds without a global tick-rate bump.
//
// Reconciled against the fetched RadioLib 7.2.1 headers
// (managed_components/jgromes__radiolib/src/Hal.h): base-class constructor
// arity/order (input, output, low, high, rising, falling) and the full
// pure-virtual list match this class exactly — no adaptation needed.
class GeigerLoRaHal : public RadioLibHal {
public:
    GeigerLoRaHal()
        : RadioLibHal(GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, 0, 1,
                      GPIO_INTR_POSEDGE, GPIO_INTR_NEGEDGE) {}

    void init() override { spiBegin(); }
    void term() override { spiEnd(); }

    void pinMode(uint32_t pin, uint32_t mode) override {
        if (pin == RADIOLIB_NC) return;
        gpio_config_t c = {};
        c.pin_bit_mask = 1ULL << pin;
        c.mode = (gpio_mode_t)mode;
        c.pull_up_en = GPIO_PULLUP_DISABLE;
        c.pull_down_en = GPIO_PULLDOWN_DISABLE;
        c.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&c);
    }
    void digitalWrite(uint32_t pin, uint32_t value) override {
        if (pin == RADIOLIB_NC) return;
        gpio_set_level((gpio_num_t)pin, value);
    }
    uint32_t digitalRead(uint32_t pin) override {
        if (pin == RADIOLIB_NC) return 0;
        return gpio_get_level((gpio_num_t)pin);
    }
    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
        if (interruptNum == RADIOLIB_NC) return;
        gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)mode);
        // ISR service may already be installed by tube.c — tolerate that.
        // The IRAM flag is REQUIRED to match tube.c's existing install (the
        // service is process-wide; a second install with mismatched flags
        // returns INVALID_STATE and leaves tube.c's flags in force anyway).
        esp_err_t e = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "isr service: %s", esp_err_to_name(e));
        }
        // ACCEPTED RISK (whole-branch review 2026-07-18): the callback
        // RadioLib registers here (its DIO1 handler) lives in flash, not
        // IRAM, while the dispatcher runs under ESP_INTR_FLAG_IRAM. A DIO1
        // edge that lands while flash cache is disabled (another task doing
        // an NVS commit or OTA write) during a Class-A RX window would fault.
        // Judged tolerable, not silently shipped: RX windows are ~1-2 s per
        // 150 s cycle (~1% duty) and the worker's own NVS session-write
        // happens AFTER sendReceive() returns (outside its own RX window), so
        // the only overlap is a user-initiated /config Save or an OTA landing
        // in that narrow window on a joined node — low probability, and the
        // failure is a clean reset, not corruption. Revisit (IRAM trampoline
        // or uplink-suppress-during-OTA) if field reports show RX-window
        // resets; the OTA teardown hook already exists for the latter.
        gpio_isr_handler_add((gpio_num_t)interruptNum,
                             reinterpret_cast<gpio_isr_t>(interruptCb), NULL);
        gpio_intr_enable((gpio_num_t)interruptNum);
    }
    void detachInterrupt(uint32_t interruptNum) override {
        if (interruptNum == RADIOLIB_NC) return;
        gpio_isr_handler_remove((gpio_num_t)interruptNum);
        gpio_intr_disable((gpio_num_t)interruptNum);
    }
    void delay(RadioLibTime_t ms) override {
        const int64_t deadline = esp_timer_get_time() + (int64_t)ms * 1000;
        const RadioLibTime_t tick_ms = portTICK_PERIOD_MS;   // 10 ms at 100 Hz
        if (ms > tick_ms) { vTaskDelay((ms - tick_ms) / tick_ms); }
        int64_t rem = deadline - esp_timer_get_time();
        if (rem > 0) { esp_rom_delay_us((uint32_t)rem); }
    }
    void delayMicroseconds(RadioLibTime_t us) override { esp_rom_delay_us((uint32_t)us); }
    RadioLibTime_t millis() override { return (RadioLibTime_t)(esp_timer_get_time() / 1000); }
    RadioLibTime_t micros() override { return (RadioLibTime_t)esp_timer_get_time(); }
    long pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) override {
        (void)pin; (void)state; (void)timeout; return 0;      // unused by SX126x
    }
    void yield() override { taskYIELD(); }

    void spiBegin() override {
        if (s_spi) return;
        spi_bus_config_t bus = {};
        bus.sclk_io_num = PIN_LORA_SCK;
        bus.mosi_io_num = PIN_LORA_MOSI;
        bus.miso_io_num = PIN_LORA_MISO;
        bus.quadwp_io_num = -1;
        bus.quadhd_io_num = -1;
        // V2.6.23-dev (T6 finding): Task 5 used SPI_DMA_DISABLED, which caps
        // spi_device_polling_transmit at 64 DATA bytes per transaction (IDF's
        // non-DMA CPU-FIFO path). RadioLib's Module::SPItransferStream (see
        // managed_components/jgromes__radiolib/src/Module.cpp) builds ONE
        // combined cmd+data buffer per SPI op and calls hal->spiTransfer()
        // on it ONCE — e.g. SX126x::readBuffer()'s FIFO read is
        // cmd(2) + up to RADIOLIB_SX126X_MAX_PACKET_LENGTH(255) + status(1)
        // = up to 258 bytes in a single spiTransfer(), far past 64. LoRaWAN
        // join-accept/session downlinks and any near-max-length uplink FIFO
        // write would silently truncate under DMA_DISABLED. SPI_DMA_CH_AUTO
        // (IDF picks a free DMA channel) plus a matching max_transfer_sz
        // removes the cap; 512 covers the FIFO's 255-byte worst case with
        // margin for future RadioLib command growth.
        bus.max_transfer_sz = 512;
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
        spi_device_interface_config_t dev = {};
        dev.clock_speed_hz = 2 * 1000 * 1000;   // SX1262 max 16 MHz; 2 MHz is plenty and forgiving
        dev.mode = 0;
        dev.spics_io_num = -1;                  // RadioLib toggles NSS itself
        dev.queue_size = 1;
        ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &s_spi));
    }
    void spiBeginTransaction() override {}
    void spiTransfer(uint8_t *out, size_t len, uint8_t *in) override {
        spi_transaction_t t = {};
        t.length = len * 8;
        t.tx_buffer = out;
        t.rx_buffer = in;
        spi_device_polling_transmit(s_spi, &t);
    }
    void spiEndTransaction() override {}
    void spiEnd() override {
        if (!s_spi) return;
        spi_bus_remove_device(s_spi);
        spi_bus_free(SPI2_HOST);
        s_spi = NULL;
    }
private:
    spi_device_handle_t s_spi = NULL;
};

static GeigerLoRaHal s_hal;
static Module  s_mod(&s_hal, PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);
static SX1262  s_radio(&s_mod);
static lorawan_status_t s_status;
static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;

// Power + radio bring-up. Returns false on hardware failure (module parks
// in LORAWAN_ST_HW_FAIL — never retried until reboot; the Geiger core is
// never hostage to the radio).
static bool radio_init(void) {
    // VFEM: power to the LoRa front end, BEFORE any SPI traffic (V1.9
    // precedent on the V4; ~100 ms settle).
    gpio_config_t vfem = {};
    vfem.pin_bit_mask = 1ULL << PIN_LORA_VFEM;
    vfem.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&vfem));
    gpio_set_level((gpio_num_t)PIN_LORA_VFEM, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Opt-in FEM lines — reworked-hardware boards ONLY (see hal.h):
    if (g_cfg.lorawan_fem_en) {
        gpio_config_t fe = {};
        fe.pin_bit_mask = 1ULL << PIN_LORA_FEM_EN;
        fe.mode = GPIO_MODE_OUTPUT;
        ESP_ERROR_CHECK(gpio_config(&fe));
        gpio_set_level((gpio_num_t)PIN_LORA_FEM_EN, 1);   // GC1109 CSD enable
        ESP_LOGW(TAG, "FEM_EN (GPIO2) driven HIGH — reworked-hardware mode");
    }
    if (g_cfg.lorawan_high_power) {
        gpio_config_t pa = {};
        pa.pin_bit_mask = 1ULL << PIN_LORA_FEM_PA;
        pa.mode = GPIO_MODE_OUTPUT;
        ESP_ERROR_CHECK(gpio_config(&pa));
        gpio_set_level((gpio_num_t)PIN_LORA_FEM_PA, 1);   // 28 dBm gain stage
        ESP_LOGW(TAG, "FEM_PA (GPIO46) driven HIGH — 28 dBm mode");
    }

    // Let RadioLib own ALL SX1262 configuration (lesson from RadioLib
    // discussion #1665: mixing manual raw commands with RadioLib breaks the
    // radio state). begin() defaults include DIO3-as-TCXO at 1.6 V, which
    // bench-worked on the V4 there. LoRaWANNode reconfigures freq/DR later.
    int16_t st = s_radio.begin();
    if (st != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "SX1262 begin failed: %d — LoRaWAN parked (HW_FAIL)", st);
        return false;
    }
    ESP_LOGI(TAG, "SX1262 init OK");
    return true;
}

// Region-table index -> short human-readable name. Index matches
// config_fields.def's lorawan_region comment (0=EU868 ... 2=AU915 default).
static const char *const REGION_NAMES[] = {
    "EU868", "US915", "AU915", "AS923", "IN865", "KR920", "EU433", "CN470"
};
#define REGION_NAME_COUNT (sizeof(REGION_NAMES) / sizeof(REGION_NAMES[0]))

const char *lorawan_region_name(uint8_t region_idx) {
    if (region_idx >= REGION_NAME_COUNT) return "?";
    return REGION_NAMES[region_idx];
}

// Config index -> RadioLib band-plan object. Order is FROZEN (the index is
// persisted in NVS via config_fields.def's lorawan_region); default index
// 2 = AU915 (user fleet), 0 = EU868 (upstream Multigeiger community).
// Names for logging reuse REGION_NAMES above (lorawan_region_name()) rather
// than a second parallel string table.
//
// Verified against the fetched RadioLib 7.2.1 headers
// (managed_components/jgromes__radiolib/src/protocols/LoRaWAN/LoRaWAN.h:495-505):
// all eight plain band objects (EU868, US915, EU433, AU915, CN470, AS923,
// KR920, IN865) exist with these exact names — no adaptation needed. AS923
// also has AS923_2/_3/_4 sub-region-frequency variants; per the task brief
// this table uses the plain AS923 object only.
static const LoRaWANBand_t *const k_bands[] = {
    &EU868, &US915, &AU915, &AS923, &IN865, &KR920, &EU433, &CN470,
};

const char *lorawan_state_name(lorawan_state_t st) {
    switch (st) {
        case LORAWAN_ST_DISABLED:  return "disabled";
        case LORAWAN_ST_NO_CONFIG: return "no_config";
        case LORAWAN_ST_HW_FAIL:   return "hw_fail";
        case LORAWAN_ST_JOINING:   return "joining";
        case LORAWAN_ST_JOINED:    return "joined";
    }
    return "?";   // unreachable for any valid lorawan_state_t value
}

void lorawan_get_status(lorawan_status_t *out) {
    if (!out) return;
    portENTER_CRITICAL(&s_status_mux);
    *out = s_status;
    portEXIT_CRITICAL(&s_status_mux);
}

// --- Credential parsing ------------------------------------------------------

typedef struct {
    uint64_t join_eui, dev_eui;
    uint8_t  app_key[16];
} lw_creds_t;

// TTN-console strings (verbatim from config) -> RadioLib types. False =>
// LORAWAN_ST_NO_CONFIG (surfaced on /status; never a crash or a join spam).
static bool parse_creds(lw_creds_t *c) {
    return lw_eui_from_hex(g_cfg.lorawan_joineui, &c->join_eui) &&
           lw_eui_from_hex(g_cfg.lorawan_deveui,  &c->dev_eui)  &&
           lw_hex_decode(g_cfg.lorawan_appkey, c->app_key, 16);
}

// --- NVS persistence ---------------------------------------------------------

// Session persistence (spec §4). Nonces after EVERY join attempt — DevNonce
// reuse across reboots permanently locks a device out of a LoRaWAN 1.0.4
// network. Session after every successful uplink cycle — lets a rebooted
// node resume without rejoin. Wear math: ~300 B blob / 150 s cycle over the
// wear-leveled 24 KB nvs partition ≈ decades; fine.
static void nvs_save_blob(const char *key, const uint8_t *buf, size_t len) {
    nvs_handle_t h;
    if (nvs_open("lorawan", NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_blob(h, key, buf, len) == ESP_OK) nvs_commit(h);
    nvs_close(h);
}
static bool nvs_load_blob(const char *key, uint8_t *buf, size_t len) {
    nvs_handle_t h;
    if (nvs_open("lorawan", NVS_READONLY, &h) != ESP_OK) return false;
    size_t got = len;
    bool ok = (nvs_get_blob(h, key, buf, &got) == ESP_OK) && got == len;
    nvs_close(h);
    return ok;
}

// --- Worker task --------------------------------------------------------------

// Queue depth 1 — a freshest-wins mailbox: the producer overwrites any
// unconsumed snapshot (see lorawan_transmit()), so the worker always pulls
// the latest window. Mirrors transmission.c's queue-depth-1 convention but
// NOT its drop-if-busy policy — see lorawan_transmit() for why they differ.
static QueueHandle_t s_q;
static volatile bool s_busy = false;
static LoRaWANNode  *s_node = NULL;

// s_version_packed: set once in lorawan_setup() (main task, at boot) via
// lw_pack_version(VERSION_STR); read (never written) by the worker task
// thereafter — a plain uint16_t load/store pair needs no locking either way.
static uint16_t s_version_packed = 0;

static void set_state(lorawan_state_t st) {
    portENTER_CRITICAL(&s_status_mux);
    s_status.state = st;
    portEXIT_CRITICAL(&s_status_mux);
}

// Classify one sendReceive() outcome under s_status_mux. House rule (memory:
// no-blocking-calls-in-critical-section): time()/ESP_LOG/RadioLib getters
// (s_radio.getRSSI()/getSNR() do a live SPI round-trip) are ALL called
// BEFORE taking the spinlock; only the plain-value stores happen inside it.
static void record_uplink_result(int16_t st, size_t dl_len, const LoRaWANEvent_t *ev) {
    int64_t now = (int64_t)time(NULL);           // captured before the lock
    float rssi = 0.0f, snr = 0.0f;
    bool have_dl = (dl_len > 0);
    if (have_dl) {
        rssi = s_radio.getRSSI();                 // SPI round-trip — before the lock
        snr  = s_radio.getSNR();
        ESP_LOGI(TAG, "downlink: port %u, %u bytes", (unsigned)ev->fPort, (unsigned)dl_len);
    }

    if (st >= RADIOLIB_ERR_NONE) {
        portENTER_CRITICAL(&s_status_mux);
        s_status.uplinks_sent++;
        s_status.last_uplink_at = now;
        if (have_dl) {
            s_status.last_dl_rssi = rssi;
            s_status.last_dl_snr  = snr;
        }
        portEXIT_CRITICAL(&s_status_mux);
    } else if (st == RADIOLIB_ERR_UPLINK_UNAVAILABLE || st == RADIOLIB_ERR_NO_CHANNEL_AVAILABLE) {
        // Duty-cycle / dwell-time guard: RadioLib itself blocked the send
        // (regulatory, not a radio/network failure) — expected occasionally
        // under AS923/EU868 duty-cycle limits, not logged as a failure.
        portENTER_CRITICAL(&s_status_mux);
        s_status.duty_skipped++;
        portEXIT_CRITICAL(&s_status_mux);
        ESP_LOGI(TAG, "duty-cycle guard: cycle skipped (%d)", st);
    } else {
        portENTER_CRITICAL(&s_status_mux);
        s_status.failed++;
        s_status.last_error = st;
        portEXIT_CRITICAL(&s_status_mux);
        ESP_LOGW(TAG, "uplink failed: %d", st);
    }
}

static void lorawan_task(void *arg) {
    (void)arg;

    // Snapshot every LoRaWAN /config field this task needs ONCE, right at
    // task entry, before any RadioLib call (including radio_init()'s
    // s_radio.begin()). A concurrent /config POST replaces g_cfg wholesale
    // (`*s_cfg = next` in http_server.c, on the httpd task) — re-reading a
    // multi-byte field (the EUI/key hex strings) mid-copy could tear.
    // lorawan_region/lorawan_subband are single bytes (can't tear) but are
    // snapshotted too, for the same "read config once, then never again"
    // discipline — this task runs for hours (join backoff alone can span
    // 60 min per retry) and must not act on a value that changed underneath
    // it mid-loop. g_cfg.lorawan_enabled was already read once in
    // lorawan_setup() on the MAIN task, before httpd starts — that read
    // predates any possible /config POST and is not repeated here.
    lw_creds_t creds;
    bool have_creds = parse_creds(&creds);
    uint8_t region  = g_cfg.lorawan_region;
    uint8_t subband = g_cfg.lorawan_subband;

    // Clamp region before it indexes k_bands[] below. config.c loads U8
    // fields from NVS with NO range check (bounds are enforced only on the
    // /config POST path), so a corrupt or foreign NVS byte >= the table size
    // would make k_bands[region] an out-of-bounds pointer read → a garbage
    // LoRaWANBand_t* into LoRaWANNode → crash EVERY boot until an NVS erase
    // (the bad value persists). Fall back to the AU915 default instead — the
    // radio must never brick the node (Geiger core stays alive regardless).
    if (region >= (sizeof(k_bands) / sizeof(k_bands[0]))) {
        ESP_LOGW(TAG, "lorawan_region %u out of range — defaulting to AU915", region);
        region = 2;
    }

    if (!radio_init()) { set_state(LORAWAN_ST_HW_FAIL); vTaskDelete(NULL); }
    if (!have_creds) {
        ESP_LOGW(TAG, "credentials missing/malformed — LoRaWAN idle (NO_CONFIG)");
        set_state(LORAWAN_ST_NO_CONFIG); vTaskDelete(NULL);
    }

    // static: LoRaWANNode has no default constructor and must outlive this
    // function's stack frame (this loop never returns in practice, but a
    // static also keeps the object off an 8 KB task stack).
    static LoRaWANNode node(&s_radio, k_bands[region], subband);
    s_node = &node;
    node.beginOTAA(creds.join_eui, creds.dev_eui, NULL, creds.app_key); // NULL nwkKey = LoRaWAN 1.0.x

    // Restore persisted state. Nonces first (join replay protection), then
    // session (rejoin-free resume). A rejected restore is self-healing: log,
    // fall through to a clean join. Buffer sized to the larger of the two
    // (SESSION_BUF_SIZE > NONCES_BUF_SIZE) and reused for both loads.
    uint8_t buf[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];
    if (nvs_load_blob("nonces", buf, RADIOLIB_LORAWAN_NONCES_BUF_SIZE)) {
        int16_t st = node.setBufferNonces(buf);
        if (st != RADIOLIB_ERR_NONE) ESP_LOGW(TAG, "nonces restore rejected: %d", st);
    }
    if (nvs_load_blob("session", buf, RADIOLIB_LORAWAN_SESSION_BUF_SIZE)) {
        int16_t st = node.setBufferSession(buf);
        if (st != RADIOLIB_ERR_NONE) ESP_LOGW(TAG, "session restore rejected: %d", st);
    }

    // Join with exponential backoff: 1,2,4,... capped 60 min (spec §4).
    // (Loop starts at 60 s — the "1,2,4..." is in minutes: 60 s, 120 s, ...)
    set_state(LORAWAN_ST_JOINING);
    uint32_t backoff_s = 60;
    for (;;) {
        int16_t st = node.activateOTAA();
        nvs_save_blob("nonces", node.getBufferNonces(), RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
        portENTER_CRITICAL(&s_status_mux); s_status.join_attempts++; portEXIT_CRITICAL(&s_status_mux);
        if (st == RADIOLIB_LORAWAN_NEW_SESSION || st == RADIOLIB_LORAWAN_SESSION_RESTORED) {
            uint32_t dev_addr = node.getDevAddr();   // RadioLib getter — before the lock
            ESP_LOGI(TAG, "%s — DevAddr %08lx",
                     st == RADIOLIB_LORAWAN_SESSION_RESTORED ? "session resumed" : "joined (OTAA)",
                     (unsigned long)dev_addr);
            portENTER_CRITICAL(&s_status_mux);
            s_status.dev_addr = dev_addr;
            portEXIT_CRITICAL(&s_status_mux);
            set_state(LORAWAN_ST_JOINED);
            break;
        }
        portENTER_CRITICAL(&s_status_mux); s_status.last_error = st; portEXIT_CRITICAL(&s_status_mux);
        ESP_LOGW(TAG, "join failed: %d — retry in %lus", st, (unsigned long)backoff_s);
        vTaskDelay(pdMS_TO_TICKS(backoff_s * 1000));
        if (backoff_s < 3600) backoff_s *= 2;
    }

    // Uplink loop: one queue slot; freshest-wins (the producer overwrites
    // via xQueueOverwrite in lorawan_transmit(), so this pulls the latest
    // window each time).
    lorawan_snapshot_t snap;
    for (;;) {
        if (xQueueReceive(s_q, &snap, portMAX_DELAY) != pdTRUE) continue;
        s_busy = true;

        uint8_t p1[10];
        lw_build_port1(p1, snap.gm_counts, snap.dt_ms, s_version_packed, snap.tube_nbr);
        // Downlink scratch. MUST be sized for the largest frame RadioLib can
        // deliver: sendReceive()'s dataDown parameter has NO size argument and
        // RadioLib writes the decrypted payload straight in (no clamp to the
        // caller's buffer), setting *dl_len afterward. A network-scheduled
        // application downlink can be up to RADIOLIB_LORAWAN_MAX_DOWNLINK_SIZE
        // (250 B) at higher data rates; a smaller buffer here would let a
        // (MIC-valid) oversized downlink smash this task's stack. 250 B on the
        // 8 KB worker stack is fine.
        uint8_t dl[RADIOLIB_LORAWAN_MAX_DOWNLINK_SIZE]; size_t dl_len = 0;
        LoRaWANEvent_t ev_dn;
        int16_t st = node.sendReceive(p1, sizeof(p1), 1, dl, &dl_len, false, NULL, &ev_dn);
        record_uplink_result(st, dl_len, &ev_dn);

        if (st >= RADIOLIB_ERR_NONE && snap.env_valid) {
            uint8_t p2[5];
            if (lw_build_port2(p2, snap.temperature_c, snap.humidity_pct, snap.pressure_pa)) {
                dl_len = 0;
                st = node.sendReceive(p2, sizeof(p2), 2, dl, &dl_len, false, NULL, &ev_dn);
                record_uplink_result(st, dl_len, &ev_dn);
            }
        }
        // Session after every uplink cycle — cheap insurance for rejoin-free
        // reboot with an FCnt the network accepts.
        nvs_save_blob("session", node.getBufferSession(), RADIOLIB_LORAWAN_SESSION_BUF_SIZE);
        s_busy = false;
    }
}

// Real setup: guards on lorawan_enabled, then creates the queue + worker
// task. Radio init/join/credential parsing all happen ON the worker task
// (lorawan_task) — this function never touches the radio or g_cfg's
// LoRaWAN string fields itself.
void lorawan_setup(void) {
    portENTER_CRITICAL(&s_status_mux);
    // Value-init (not memset): s_status carries two floats
    // (last_dl_rssi/last_dl_snr) and cppcheck's memsetClassFloat rule
    // correctly flags relying on an all-zero-bytes bit pattern for floating
    // point — this form asks the compiler for a real 0.0f assignment.
    s_status = lorawan_status_t{};
    s_status.region  = g_cfg.lorawan_region;
    s_status.subband = g_cfg.lorawan_subband;
    portEXIT_CRITICAL(&s_status_mux);

    // g_cfg.lorawan_enabled read here, once, on the MAIN task at boot,
    // before httpd starts (so no /config POST can race this read) — safe
    // per the g_cfg-snapshot rule above; not re-read anywhere else.
    if (!g_cfg.lorawan_enabled) {
        portENTER_CRITICAL(&s_status_mux);
        s_status.state = LORAWAN_ST_DISABLED;
        portEXIT_CRITICAL(&s_status_mux);
        return;
    }

    s_version_packed = lw_pack_version(VERSION_STR);

    s_q = xQueueCreate(1, sizeof(lorawan_snapshot_t));
    configASSERT(s_q);

    // Same worker-task convention as transmission.c's tx_setup(): pinned to
    // CPU1 so a multi-second join/uplink RadioLib call (blocking SPI polls +
    // RX-window waits) never starves CPU0's idle task / task watchdog.
    // BOARD_HELTEC_WIFI_LORA32_V4_R2 is ESP32-S3 (dual-core) — always the
    // pinned branch here, unlike tx_setup()'s single-core ESP32-C5 carve-out
    // (LoRaWAN is this one board only; see file header).
    BaseType_t ok = xTaskCreatePinnedToCore(
        lorawan_task, "lorawan", 8192, NULL, (tskIDLE_PRIORITY + 1), NULL, 1);
    configASSERT(ok == pdPASS);
    ESP_LOGI(TAG, "LoRaWAN worker task created (join runs in the background)");
}

// Enqueue one uplink snapshot into the depth-1 mailbox. Non-blocking and
// freshest-wins: xQueueOverwrite always replaces any still-unconsumed
// snapshot with the latest one. This only differs from a plain drop-if-busy
// send when the worker stays busy across multiple TX cycles — chiefly a long
// OTAA join, during which the worker sits in its join/backoff loop and never
// drains the queue. Overwrite guarantees the FIRST uplink after the join
// carries the current counting window, not the stale one captured when the
// join began. (Deliberately diverges from tx_transmit()'s drop-if-busy: the
// WiFi worker drains every cycle, so it never accumulates the multi-hour
// staleness a join backoff can — the depth-1 "latest value" mailbox is the
// right model here, and xQueueOverwrite is FreeRTOS's purpose-built
// primitive for it. Requires a depth-1 queue, which s_q is.)
void lorawan_transmit(const lorawan_snapshot_t *snap) {
    if (!snap) return;
    // s_q is NULL until lorawan_setup() creates it, and setup() only creates
    // it when lorawan_enabled was set AT BOOT. But do_tx_cycle() reads
    // g_cfg.lorawan_enabled LIVE, so a user who ticks "Enable LoRaWAN" and
    // presses the plain (non-restart) /config Save flips the flag true on a
    // node whose worker/queue were never created — the next TX cycle would
    // reach here with s_q == NULL and xQueueOverwrite()'s configASSERT would
    // panic. Guard it: enabling LoRaWAN is a reboot-required change (the
    // /config fields are starred), so a live enable correctly no-ops here
    // until the reboot that actually brings the worker up.
    if (!s_q) return;
    xQueueOverwrite(s_q, snap);   // depth-1: always succeeds, never blocks
}

// Idle iff the worker isn't mid-uplink AND nothing is queued for it.
// s_q NULL (LoRaWAN never armed this boot) reads as idle.
bool lorawan_is_idle(void) {
    return !s_busy && (s_q == NULL || uxQueueMessagesWaiting(s_q) == 0);
}

#endif // CONFIG_GEIGER_LORAWAN
