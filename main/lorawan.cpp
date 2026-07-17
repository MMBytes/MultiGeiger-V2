// LoRaWAN uplink for BOARD_HELTEC_WIFI_LORA32_V4_R2 — the tree's ONLY C++
// translation unit, kept behind the plain-C API in lorawan.h. Whole file is
// gated on CONFIG_GEIGER_LORAWAN (set only in that board's sdkconfig): the
// CMake SRCS conditional keeps it out of other boards' builds, and this
// guard additionally yields an empty TU for the cppcheck gate's other-board
// define sets (_build.cmd scans all of main/ for every board).
#include "sdkconfig.h"
#if CONFIG_GEIGER_LORAWAN

#include <string.h>
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
        esp_err_t e = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "isr service: %s", esp_err_to_name(e));
        }
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
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED));
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

// TEMPORARY (Task 5 smoke-test only): runs radio_init() inline on the main
// task at boot and just logs the result. Task 6 replaces this body with the
// real worker-task version — its own task, OTAA join with backoff, and the
// TX queue that lorawan_transmit()/lorawan_is_idle() actually drive.
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

    if (!g_cfg.lorawan_enabled) {
        portENTER_CRITICAL(&s_status_mux);
        s_status.state = LORAWAN_ST_DISABLED;
        portEXIT_CRITICAL(&s_status_mux);
        return;
    }

    bool ok = radio_init();
    portENTER_CRITICAL(&s_status_mux);
    s_status.state = ok ? LORAWAN_ST_JOINING : LORAWAN_ST_HW_FAIL;
    portEXIT_CRITICAL(&s_status_mux);
    if (ok) {
        ESP_LOGI(TAG, "radio_init OK (smoke test only — Task 6 adds the join/uplink worker)");
    }
}

// TEMPORARY stub — Task 6 replaces this with the real enqueue-to-worker-task
// logic. No-op for now: nothing calls lorawan_transmit() yet (that wiring is
// also Task 6's, in the main TX cycle), but the definition must exist so the
// lorawan.h `#else` (real) branch links.
void lorawan_transmit(const lorawan_snapshot_t *snap) {
    (void)snap;
}

// TEMPORARY stub — always idle until Task 6's worker task exists.
bool lorawan_is_idle(void) {
    return true;
}

#endif // CONFIG_GEIGER_LORAWAN
