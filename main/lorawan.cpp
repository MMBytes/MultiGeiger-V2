// LoRaWAN uplink for the SX1262 boards (Heltec WiFi LoRa 32 V4; V2.8.11: XIAO
// ESP32-S3 + Wio-SX1262) — the tree's ONLY C++ translation unit, kept behind
// the plain-C API in lorawan.h. Per-board radio differences come from hal.h
// (HAL_HAS_LORA_FEM, HAL_LORA_TCXO_V, PIN_LORA_RF_SW_RX). Whole file is
// gated on CONFIG_GEIGER_LORAWAN (set only in those boards' sdkconfig): the
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

#include <sys/time.h>   // settimeofday() — DeviceTimeReq network time sync

extern "C" {
#include "hal.h"
#include "lorawan.h"
#include "lorawan_codec.h"
#include "lorawan_sched.h"   // V2.8.14: SF interval table, port-2 cadence, fixed DR
#include "lorawan_rxobs.h"   // V2.8.21: RX-window observer + its log text
#include "radio_turn_rule.h" // V2.8.15: no exchange while a WiFi upload runs
#include "transmission.h"    // tx_is_idle()
#include "log_ftp.h"         // log_ftp_is_busy()
#include "tube.h"            // tube_isr_service_installed()
#include "config.h"
#include "ntp.h"       // ntp_boot_epoch() — "has SNTP ever synced" gate for time sync
#include "version.h"   // VERSION_STR -> s_version_packed (lw_pack_version)
}

static const char *TAG = "lorawan";

// --- RX-window observer --------------------------------------------------------
//
// Passive record of what the SX1262 did around ONE uplink or join request,
// fed from the HAL's spiTransfer() (lorawan_rxobs.h). Only the worker task
// drives RadioLib, so every write runs on that task; no locking needed.
// Armed by rxobs_begin() right before activateOTAA()/sendReceive(), disarmed after.
static rxobs_t s_obs;
// lorawan_rxobs.h is RadioLib-free (host-tested); its copies of the codes must match.
static_assert(LW_RXOBS_ERR_TX_TIMEOUT == RADIOLIB_ERR_TX_TIMEOUT, "rxobs code");
static_assert(LW_RXOBS_ERR_CRC_MISMATCH == RADIOLIB_ERR_CRC_MISMATCH, "rxobs code");
static_assert(LW_RXOBS_ERR_DOWNLINK_MALFORMED == RADIOLIB_ERR_DOWNLINK_MALFORMED, "rxobs code");
static_assert(LW_RXOBS_ERR_INVALID_PORT == RADIOLIB_ERR_INVALID_PORT, "rxobs code");
static_assert(LW_RXOBS_ERR_NO_RX_WINDOW == RADIOLIB_ERR_NO_RX_WINDOW, "rxobs code");
static_assert(LW_RXOBS_ERR_MIC_MISMATCH == RADIOLIB_ERR_MIC_MISMATCH, "rxobs code");
// lw_err_text()'s plain literals too: a RadioLib release that renumbers one
// must fail the build here, not mislabel a node-log line.
static_assert(RADIOLIB_ERR_UNKNOWN == -1, "lw_err_text code");
static_assert(RADIOLIB_ERR_CHIP_NOT_FOUND == -2, "lw_err_text code");
static_assert(RADIOLIB_ERR_PACKET_TOO_LONG == -4, "lw_err_text code");
static_assert(RADIOLIB_ERR_RX_TIMEOUT == -6, "lw_err_text code");
static_assert(RADIOLIB_ERR_INVALID_FREQUENCY == -12, "lw_err_text code");
static_assert(RADIOLIB_ERR_INVALID_OUTPUT_POWER == -13, "lw_err_text code");
static_assert(RADIOLIB_ERR_SPI_WRITE_FAILED == -16, "lw_err_text code");
static_assert(RADIOLIB_ERR_WRONG_MODEM == -20, "lw_err_text code");
static_assert(RADIOLIB_ERR_LORA_HEADER_DAMAGED == -24, "lw_err_text code");
static_assert(RADIOLIB_ERR_NULL_POINTER == -28, "lw_err_text code");
static_assert(RADIOLIB_ERR_INVALID_DATA_RATE == -501, "lw_err_text code");
static_assert(RADIOLIB_ERR_INVALID_TCXO_VOLTAGE == -703, "lw_err_text code");
static_assert(RADIOLIB_ERR_SPI_CMD_TIMEOUT == -705, "lw_err_text code");
static_assert(RADIOLIB_ERR_SPI_CMD_INVALID == -706, "lw_err_text code");
static_assert(RADIOLIB_ERR_SPI_CMD_FAILED == -707, "lw_err_text code");
static_assert(RADIOLIB_ERR_NETWORK_NOT_JOINED == -1101, "lw_err_text code");
static_assert(RADIOLIB_ERR_INVALID_REVISION == -1103, "lw_err_text code");
static_assert(RADIOLIB_ERR_NO_CHANNEL_AVAILABLE == -1106, "lw_err_text code");
static_assert(RADIOLIB_ERR_UPLINK_UNAVAILABLE == -1108, "lw_err_text code");
static_assert(RADIOLIB_ERR_COMMAND_QUEUE_FULL == -1109, "lw_err_text code");
static_assert(RADIOLIB_ERR_JOIN_NONCE_INVALID == -1111, "lw_err_text code");
static_assert(RADIOLIB_ERR_DWELL_TIME_EXCEEDED == -1114, "lw_err_text code");
static_assert(RADIOLIB_ERR_CHECKSUM_MISMATCH == -1115, "lw_err_text code");
static_assert(RADIOLIB_ERR_NO_JOIN_ACCEPT == -1116, "lw_err_text code");
static_assert(RADIOLIB_ERR_NONCES_DISCARDED == -1119, "lw_err_text code");
static_assert(RADIOLIB_ERR_SESSION_DISCARDED == -1120, "lw_err_text code");
static_assert(RADIOLIB_ERR_INVALID_SUBBAND == -1123, "lw_err_text code");
static_assert(RADIOLIB_ERR_MAC_ANSWERS_PENDING == -1124, "lw_err_text code");

// RadioLib HAL on raw IDF-6 drivers. CONFIG_FREERTOS_HZ stays 100: delay()
// is a hybrid — whole ticks via vTaskDelay (scheduler-friendly for the
// multi-second RX-window waits), sub-tick remainder busy-waited on esp_timer
// so LoRaWAN's ms-scale window timing holds without a global tick-rate bump.
//
// Matches RadioLib 8.0's Hal.h: base-class constructor arity/order (input,
// output, low, high, rising, falling) and the full pure-virtual list, with
// init/term/spiBegin/spiTransfer/spiEnd returning a RadioLib status code.
// RadioLib also ships an EspHal of its own; it is not used (see
// CMakeLists.txt, RADIOLIB_RELAX_RTOS_TICK).
class GeigerLoRaHal : public RadioLibHal {
public:
    GeigerLoRaHal()
        : RadioLibHal(GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, 0, 1,
                      GPIO_INTR_POSEDGE, GPIO_INTR_NEGEDGE) {}

    int16_t init() override { return spiBegin(); }
    int16_t term() override { return spiEnd(); }

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
        // Install the shared GPIO ISR service AT MOST ONCE. RadioLib calls
        // attachInterrupt() around every RX window (many times once joined),
        // and the driver's gpio_install_isr_service() logs a red ESP_LOGE
        // "GPIO isr service already installed" on every call after the first
        // — on a tube-enabled node tube.c installs it at boot, so ours would
        // always be a repeat. Without this guard that benign error spammed
        // /log + syslog on every RX window forever. The static latch collapses
        // it to a single boot attempt (INVALID_STATE still tolerated in case
        // another driver installed it first; the IRAM flag must match
        // tube.c's install — the service is process-wide).
        // V2.8.15: that one boot attempt still logged the red line once per
        // boot (the driver logs before it returns INVALID_STATE), so skip the
        // call when tube_setup() already installed the service; install here
        // only on a tube-disabled node.
        static bool s_isr_service_tried = false;
        if (!s_isr_service_tried) {
            s_isr_service_tried = true;
            if (!tube_isr_service_installed()) {
                esp_err_t e = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
                if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
                    ESP_LOGE(TAG, "isr service: %s", esp_err_to_name(e));
                }
            }
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
        // failure is a clean reset, not corruption.
        // V2.8.12 UPDATE: it happened — XIAO 2026-10-18, an OTA started mid
        // exchange while NbTrans=3 kept the radio busy ~20 s per 120 s cycle
        // (TG1WDT_SYS_RST). The OTA path is now closed by lorawan_suspend()
        // (OTA-suspension block below). Still OPEN for other flash writers
        // during an RX window (/config Save's nvs_commit, cert/config writes).
        // Candidate full fix, not done yet: an ISR-free HAL — attachInterrupt()
        // only stores pin + callback and yield() polls DIO1 and calls it from
        // task context (RadioLib only reads its downlink flag inside
        // `while (...) hal->yield()` loops, LoRaWAN.cpp receiveClassA()). Defining
        // ESP32 to get RadioLib's IRAM_ATTR is NOT an option: BuildOpt.h would
        // switch RadioLib to its Arduino-ESP32 platform branch.
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

    int16_t spiBegin() override {
        if (s_spi) return RADIOLIB_ERR_NONE;
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
        return RADIOLIB_ERR_NONE;
    }
    void spiBeginTransaction() override {}
    int16_t spiTransfer(uint8_t *out, size_t len, uint8_t *in) override {
        spi_transaction_t t = {};
        t.length = len * 8;
        t.tx_buffer = out;
        t.rx_buffer = in;
        spi_device_polling_transmit(s_spi, &t);
        // Passive RX-window observer (read-only, after the transfer).
        if (s_obs.on && len > 0) rxobs_note(&s_obs, esp_timer_get_time(), out, len, in);
        return RADIOLIB_ERR_NONE;
    }
    void spiEndTransaction() override {}
    int16_t spiEnd() override {
        if (!s_spi) return RADIOLIB_ERR_NONE;
        spi_bus_remove_device(s_spi);
        spi_bus_free(SPI2_HOST);
        s_spi = NULL;
        return RADIOLIB_ERR_NONE;
    }
private:
    spi_device_handle_t s_spi = NULL;
};

static GeigerLoRaHal s_hal;
static Module  s_mod(&s_hal, PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);
// V2.8.19: exposes the protected GetPacketStatus reply. In LoRa mode its bytes
// are RssiPkt << 16 | SnrPkt << 8 | SignalRssiPkt. RadioLib 8.0's getRSSI()
// returns RssiPkt too (#1913, fixed upstream 10-26; before, it returned
// SignalRssiPkt, which bottoms out at -127.5 dBm for weak downlinks), but
// SignalRssiPkt has no public getter: packetStatus() returns RssiPkt, the
// packet's received power as Semtech's LoRaMac-node reports it, plus
// SignalRssiPkt and SNR, all from one SPI read.
//
// An external power amplifier (Heltec V4.3's KCT8103L, ~11 dB: Meshtastic's
// measurement on the V4, +11 dB at 0-15 dBm drive, compressing to +7 dB at
// 21 dBm) is compensated by RadioLib itself: radio_init() calls
// setOutputPowerOffset() before begin(), so the LoRaWAN stack's power (= EIRP,
// the band's limit, AU915 30 dBm / EU868 16 dBm) is accepted over the SX1262's
// range shifted by the gain, and the gain is subtracted before the radio is
// programmed — the antenna gets about what the stack asked for, never more.
class GeigerSX1262 : public SX1262 {
public:
    explicit GeigerSX1262(Module *m) : SX1262(m) {}

    /** @brief RssiPkt (packet received power), SignalRssiPkt (signal estimate
     *         after despreading) in dBm and SNR in dB of the last LoRa packet,
     *         from one GetPacketStatus read. The signal estimate tells a weak
     *         signal from a high noise floor at negative SNR; it bottoms out
     *         at -127.5 dBm. SNR decoding as RadioLib's getSNR(). */
    void packetStatus(float *rssi, float *signal, float *snr) {
        const uint32_t ps = this->getPacketStatus();
        *rssi   = -0.5f * (float)((ps >> 16) & 0xFF);
        *signal = -0.5f * (float)(ps & 0xFF);
        *snr    = 0.25f * (float)(int8_t)((ps >> 8) & 0xFF);
    }
};
static GeigerSX1262 s_radio(&s_mod);
static lorawan_status_t s_status;
static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;

// V2.8.14 uplink schedule state — see the "Uplink schedule" block further down.
static uint8_t s_cycle_sf = 0;                // SF the TX-cycle interval follows; 0 = not set yet
static const LoRaWANBand_t *s_band = NULL;    // the region's band plan, set in lorawan_setup()
// Fixed data rate from /config (lorawan_dr != 0). Set in lorawan_setup()
// before the worker starts; the worker clears s_fixed_dr (atomic) if the
// region can't use that DR, falling back to ADR.
static bool    s_fixed_dr = false;
static uint8_t s_fixed_dr_value = 0;

// --- OTA suspension (V2.8.12) --------------------------------------------------
//
// s_busy: the worker is inside a radio exchange (join attempt or uplink with
// its RX windows). s_suspended: set once by lorawan_suspend() (OTA prep, httpd
// task), never cleared — the OTA ends in a reboot either way, and a failed
// OTA losing LoRaWAN until the next reboot is the safe side.
//
// Why: RadioLib's DIO1 callback lives in flash while the GPIO ISR dispatcher
// runs under ESP_INTR_FLAG_IRAM (see attachInterrupt() above). An RX-window
// edge while the OTA's flash writes have the cache disabled hangs the chip
// until the interrupt watchdog resets it — observed 2026-10-18 19:25 CEST on
// the XIAO (TG1WDT_SYS_RST 4.6 s into an OTA started mid port-2 exchange,
// NbTrans=3 keeping the radio busy ~20 s per 120 s cycle).
//
// Handshake (both sides seq-cst, Dekker-style): the worker publishes
// s_busy=true BEFORE reading s_suspended; lorawan_suspend() publishes
// s_suspended=true BEFORE reading s_busy. Whatever the interleaving, either
// the worker sees the suspension and backs out, or the OTA side sees busy and
// waits for the exchange to end — never both proceed.
static bool s_busy = false;
static bool s_suspended = false;

// V2.8.15: s_busy also keeps the exchange apart from the WiFi uploads on a
// node that uses both (radio_turn_rule.h). Same handshake, other partners:
// publish s_busy, THEN read tx_is_idle() / log_ftp_is_busy(); the WiFi
// upload round and FTPS publish their own flags, THEN read lorawan_on_air().
// The exchange yields: if a WiFi transfer runs or is queued it clears
// s_busy again and retries every TURN_POLL_MS, at most TURN_WAIT_MAX_MS,
// then goes ahead anyway (logged). A WiFi-only build never gets here; on a
// LoRaWAN node without WiFi uploads the only "transfer" is an empty round
// (WiFi up for MQTT / web / OTA, no target enabled), which runs in
// milliseconds and doesn't wait itself — at most one TURN_POLL_MS yield.
#define TURN_POLL_MS     100
#define TURN_WAIT_MAX_MS (5UL * 60UL * 1000UL)   // a round with several HTTPS targets, or FTPS, can take minutes

// Enter one radio exchange. False = suspended for OTA: do not touch the radio.
// Which WiFi transfer(s) held an exchange back, for its log line.
static const char *turn_what(bool round, bool ftps) {
    if (round && ftps) return "the upload round and the FTPS log upload";
    if (ftps) return "the FTPS log upload";
    return "the upload round";
}

static bool exchange_begin(void) {
    const int64_t t0 = esp_timer_get_time();
    bool held_by_round = false, held_by_ftps = false;   // what we waited for (log only)
    for (;;) {
        __atomic_store_n(&s_busy, true, __ATOMIC_SEQ_CST);
        if (__atomic_load_n(&s_suspended, __ATOMIC_SEQ_CST)) {
            __atomic_store_n(&s_busy, false, __ATOMIC_SEQ_CST);
            return false;
        }
        const int64_t waited_ms = (esp_timer_get_time() - t0) / 1000;
        const bool round_idle = tx_is_idle();
        const bool ftps_busy  = log_ftp_is_busy();
        if (rt_lorawan_may_start(round_idle, ftps_busy)) {
            if (waited_ms >= 1000) {
                ESP_LOGI(TAG, "waited %.1f s for %s to finish", (double)waited_ms / 1000.0,
                         turn_what(held_by_round, held_by_ftps));
            }
            return true;
        }
        held_by_round = held_by_round || !round_idle;
        held_by_ftps  = held_by_ftps || ftps_busy;
        if (waited_ms >= (int64_t)TURN_WAIT_MAX_MS) {
            ESP_LOGW(TAG, "%s still running after %lu s — going ahead anyway",
                     turn_what(held_by_round, held_by_ftps), (unsigned long)(TURN_WAIT_MAX_MS / 1000));
            return true;
        }
        __atomic_store_n(&s_busy, false, __ATOMIC_SEQ_CST);   // yield to the upload
        vTaskDelay(pdMS_TO_TICKS(TURN_POLL_MS));
    }
}
static void exchange_end(void) { __atomic_store_n(&s_busy, false, __ATOMIC_SEQ_CST); }

// Suspended worker parks here until the OTA's reboot. Never returns.
[[noreturn]] static void park_for_ota(void) {
    ESP_LOGI(TAG, "suspended for OTA — no further radio activity until reboot");
    for (;;) vTaskDelay(portMAX_DELAY);
}

#if HAL_HAS_LORA_FEM
// V2.8.19: net TX gain of the Heltec V4's PA at low drive (Meshtastic's
// heltec_v4 variant.h: +11 dB at 0-15 dBm in, compressing to +7 dB at 21 dBm).
// Using the low-drive value errs on the low side at high drive.
#define LORA_KCT_PA_GAIN_DB 11

// V2.8.19: front-end type, probed once: -1 = not yet, 0 = GC1109 (or an
// unclear reading), 1 = KCT8103L. Written only by the first
// lorawan_fem_is_kct8103l() call (main task, before the worker exists), read
// afterwards from any task.
static volatile int8_t s_fem_kct8103l = -1;

bool lorawan_fem_is_kct8103l(void) {
    if (s_fem_kct8103l >= 0) return s_fem_kct8103l == 1;
    // VFEM: power to the LoRa front end, BEFORE any SPI traffic (V1.9
    // precedent on the V4; ~100 ms settle). The probe needs it powered.
    gpio_config_t vfem = {};
    vfem.pin_bit_mask = 1ULL << PIN_LORA_VFEM;
    vfem.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&vfem));
    gpio_set_level((gpio_num_t)PIN_LORA_VFEM, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    // Read-only probe: the V4.2's GC1109 has an internal pull-down on CSD
    // (GPIO2), the V4.3's KCT8103L a pull-up — MeshCore's and Meshtastic's
    // Heltec V4 support detect it the same way. GPIO2 is read as a plain
    // input (ESP32 pulls off) before radio_init() drives it.
    gpio_config_t probe = {};
    probe.pin_bit_mask = 1ULL << PIN_LORA_FEM_EN;
    probe.mode = GPIO_MODE_INPUT;
    probe.pull_up_en = GPIO_PULLUP_DISABLE;
    probe.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&probe));
    vTaskDelay(pdMS_TO_TICKS(2));   // let a previously driven level decay
    int highs = 0;
    for (int i = 0; i < 5; i++) {
        highs += gpio_get_level((gpio_num_t)PIN_LORA_FEM_EN);
        esp_rom_delay_us(100);
    }
    ESP_LOGI(TAG, "front end: GPIO2 read HIGH %d/5 -> %s", highs,
             highs == 5 ? "KCT8103L (Heltec V4.3)" : highs == 0 ? "GC1109 (Heltec V4.2)" :
                          "unclear, treated as GC1109");
    s_fem_kct8103l = (highs == 5) ? 1 : 0;
    return s_fem_kct8103l == 1;
}

bool lorawan_fem_kct8103l_detected(void) { return s_fem_kct8103l == 1; }

// The LNA mode radio_init() actually applied: lorawan_lna is read once there,
// so a /config change only takes effect after a restart, and the log/status
// labels must follow the applied mode, not the saved setting.
static volatile bool s_lna_applied = false;

bool lorawan_dl_rssi_includes_lna(void) {
    return s_fem_kct8103l == 1 && s_lna_applied;
}
#else   // bare SX1262 (XIAO + Wio): no front end to probe
bool lorawan_fem_is_kct8103l(void) { return false; }
bool lorawan_fem_kct8103l_detected(void) { return false; }
bool lorawan_dl_rssi_includes_lna(void) { return false; }
#endif  // HAL_HAS_LORA_FEM

// Power + radio bring-up. Returns false on hardware failure (module parks
// in LORAWAN_ST_HW_FAIL — never retried until reboot; the Geiger core is
// never hostage to the radio).
static bool radio_init(void) {
#if HAL_HAS_LORA_FEM
    // V2.8.19: powers VFEM and, on the first call, probes the front-end type
    // (usually already done by main.c before speaker_setup()).
    const bool kct = lorawan_fem_is_kct8103l();

    // FEM_EN: always driven — the GC1109 must be enabled for RX (RadioLib
    // discussion #1665), so with LoRaWAN on there is no valid "FEM off"
    // configuration. Unconditional since V2.6.34: this build's pin map
    // targets the V1.10 mainboard, where GPIO2 belongs to FEM_EN alone
    // (see hal.h — the old opt-in existed only for the V1.9 double-booking).
    gpio_config_t fe = {};
    fe.pin_bit_mask = 1ULL << PIN_LORA_FEM_EN;
    fe.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&fe));
    gpio_set_level((gpio_num_t)PIN_LORA_FEM_EN, 1);   // CSD enable (GC1109 and KCT8103L)
    ESP_LOGI(TAG, "FEM_EN (GPIO2) driven HIGH (front end RX/TX enable)");

    // FEM_PA stays opt-in on the V4.2: 28 dBm is a regulatory/power choice,
    // not a hardware gate (GPIO46 is free on the V1.10 mainboard, see hal.h).
    // V2.8.19: on a V4.3 GPIO46 is unconnected and the KCT8103L always
    // transmits through its PA (CTX below), so it is left alone there.
    if (kct) {
        if (g_cfg.lorawan_high_power)
            ESP_LOGI(TAG, "28 dBm mode setting has no effect on a KCT8103L (always PA in TX)");
    } else if (g_cfg.lorawan_high_power) {
        gpio_config_t pa = {};
        pa.pin_bit_mask = 1ULL << PIN_LORA_FEM_PA;
        pa.mode = GPIO_MODE_OUTPUT;
        ESP_ERROR_CHECK(gpio_config(&pa));
        gpio_set_level((gpio_num_t)PIN_LORA_FEM_PA, 1);   // 28 dBm gain stage
        ESP_LOGW(TAG, "FEM_PA (GPIO46) driven HIGH — 28 dBm mode");
    }
#endif  // HAL_HAS_LORA_FEM — boards without it have a bare SX1262 (no power-up step)

    // Let RadioLib own ALL SX1262 configuration (lesson from RadioLib
    // discussion #1665: mixing manual raw commands with RadioLib breaks the
    // radio state). begin() also sets DIO2 as the RF switch and DIO3 as the
    // TCXO supply; the TCXO voltage comes from hal.h (Heltec V4 1.6 V,
    // Wio-SX1262 1.8 V). Configuration-structure begin(): TCXO voltage and
    // regulator are public members set BEFORE it; ConfigLoRa_t's defaults
    // (434 MHz, 125 kHz, SF9, CR 4/7, private sync word, 10 dBm, 8-symbol
    // preamble) are replaced by LoRaWANNode with the band's settings later.
#if HAL_HAS_LORA_FEM
    // The KCT8103L's PA is always in the TX path, so RadioLib programs the
    // radio ~11 dB below the EIRP the LoRaWAN stack asks for. Must come
    // before begin(): begin() sets a power, and that call computes the
    // accepted range from the offset (PhysicalLayer applyOutputPowerOffset()).
    if (kct) (void)s_radio.setOutputPowerOffset(LORA_KCT_PA_GAIN_DB);   // scalar form: cannot fail
#endif
    static_assert(HAL_LORA_TCXO_V == 1.6f || HAL_LORA_TCXO_V == 1.8f, "map HAL_LORA_TCXO_V to RadioLibTCXOVoltage_t");
    s_radio.tcxoVoltage = (HAL_LORA_TCXO_V == 1.8f) ? Voltage1V8 : Voltage1V6;
    s_radio.useRegulatorLDO = false;          // DC-DC
    ConfigLoRa_t lora_cfg;                    // all fields at their documented defaults
    int16_t st = s_radio.begin(lora_cfg);
    if (st != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "SX1262 begin failed: %d (%s) — LoRaWAN parked (HW_FAIL)", (int)st, lw_err_text(st));
        return false;
    }
    ESP_LOGI(TAG, "SX1262 init OK");
    {
        // TX power range after begin() (it applied any PA offset), for the
        // "boot: LoRaWAN:" line. Getters before the lock (house rule).
        const int8_t pmin = s_radio.getOutputPowerMin(), pmax = s_radio.getOutputPowerMax();
        portENTER_CRITICAL(&s_status_mux);
        s_status.tx_power_min = pmin;
        s_status.tx_power_max = pmax;
        portEXIT_CRITICAL(&s_status_mux);
    }
#ifdef PIN_LORA_RF_SW_RX
    // V2.8.11 (XIAO + Wio): the module's RF_SW1 input wants HIGH to receive.
    // RadioLib drives this pin HIGH in RX and LOW in TX / idle (Module.cpp
    // setRfSwitchPins), alongside DIO2. Same wiring as Meshtastic's
    // seeed_xiao_s3 variant (SX126X_RXEN 38).
    s_radio.setRfSwitchPins(PIN_LORA_RF_SW_RX, RADIOLIB_NC);
    ESP_LOGI(TAG, "RF switch: DIO2 + GPIO%d (RX high)", PIN_LORA_RF_SW_RX);
#endif
#if HAL_HAS_LORA_FEM
    // V2.8.19: on a KCT8103L RadioLib drives CTX (GPIO5) through an RF-switch
    // table next to DIO2 (= CPS): HIGH in TX (PA mode); in RX LOW = through
    // the LNA (lorawan_lna on, the default) or HIGH = LNA bypass, for sites
    // with strong transmitters nearby (Heltec wiki "LNA"); idle LOW. Before
    // V2.8.19 GPIO5 sat LOW as the speaker's N leg, so the KCT8103L never
    // entered PA mode and a V4.3 transmitted far below its rated power
    // (measured: +70 dB at the gateway once CTX was driven).
    if (kct) {
        // Range after begin(): its setOutputPower() call applied the offset.
        ESP_LOGI(TAG, "KCT8103L PA: TX power range %d..%d dBm (radio set %d dB lower, compensating the PA gain)",
                 (int)s_radio.getOutputPowerMin(), (int)s_radio.getOutputPowerMax(), LORA_KCT_PA_GAIN_DB);
        static const uint32_t ctx_pins[Module::RFSWITCH_MAX_PINS] = {
            PIN_LORA_KCT_CTX, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC };
        static const Module::RfSwitchMode_t ctx_lna[] = {
            { Module::MODE_IDLE, { 0 } },
            { Module::MODE_RX,   { 0 } },
            { Module::MODE_TX,   { 1 } },
            END_OF_MODE_TABLE };
        static const Module::RfSwitchMode_t ctx_bypass[] = {
            { Module::MODE_IDLE, { 0 } },
            { Module::MODE_RX,   { 1 } },
            { Module::MODE_TX,   { 1 } },
            END_OF_MODE_TABLE };
        const bool lna = g_cfg.lorawan_lna;
        s_lna_applied = lna;
        s_radio.setRfSwitchTable(ctx_pins, lna ? ctx_lna : ctx_bypass);
        ESP_LOGI(TAG, "KCT8103L: GPIO%d = CTX via RadioLib (TX HIGH = PA, RX %s)",
                 PIN_LORA_KCT_CTX, lna ? "LOW = LNA" : "HIGH = LNA bypass");
    }
#endif
    // Boosted RX gain (SX126x register 0x08AC = 0x96) instead of RadioLib's
    // power-saving default: about +2 dB receive sensitivity for about +1 mA
    // while receiving (Semtech datasheet; Meshtastic enables it by default).
    // persist = true also puts the register in the radio's retention list, so a
    // warm-start sleep would keep it (this firmware registers no sleep callback,
    // so RadioLib keeps the radio in standby between the uplink and the RX
    // windows anyway). Weak downlinks fail by about this margin (XIAO EU868, 2026-10).
    const int16_t gst = s_radio.setRxBoostedGainMode(true, true);
    if (gst == RADIOLIB_ERR_NONE) {
        ESP_LOGI(TAG, "SX1262: RX boosted gain on (about +2 dB receive sensitivity, about +1 mA while receiving)");
    } else {
        ESP_LOGW(TAG, "SX1262: RX boosted gain NOT set: %d (%s) — receiving with the power-saving gain",
                 (int)gst, lw_err_text(gst));
    }
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
// All eight are RadioLib's plain band objects (LoRaWAN.h: EU868, US915,
// EU433, AU915, CN470, AS923, KR920, IN865). AS923 also has AS923_2/_3/_4
// sub-region-frequency variants; this table uses the plain AS923 object only.
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
    // V2.8.14 schedule fields: derived on read, not stored in s_status.
    out->cycle_interval_ms = lorawan_cycle_interval_ms();
    out->cycle_sf          = __atomic_load_n(&s_cycle_sf, __ATOMIC_RELAXED);
    out->fixed_dr          = __atomic_load_n(&s_fixed_dr, __ATOMIC_RELAXED);
    out->fixed_dr_value    = s_fixed_dr_value;
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
// network. Session after every uplink exchange (V2.8.12; was once per cycle)
// — lets a rebooted node resume without rejoin at an FCnt the network
// accepts. Wear math: ~300 B blob, 2 exchanges per 150 s cycle, over the
// wear-leveled 24 KB nvs partition ≈ still decades; fine.
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

// Set by lorawan_forget_session() (httpd task), read by the worker's
// per-exchange session save (worker task) — single bool, atomic on this
// target either way. Once set it stays set until the imminent reboot, so
// no uplink completing between the NVS erase and the actual restart can
// re-save the just-erased session blob. The restart is NOT guaranteed to
// happen within ~2 s: main.c defers it on the WiFi TX worker's idle check,
// which on a WiFi+LoRa node can hold it for a full TLS upload cycle.
static volatile bool s_forget_pending = false;

// --- Session persistence (RadioLib 8.0 buffer callbacks) ----------------------
//
// RadioLib calls these itself: the persistence store (DevNonce/JoinNonce)
// before every join request, the session store after every uplink — also
// between NbTrans repeats, i.e. inside an exchange. The store callbacks only
// copy to RAM; the worker writes NVS after the exchange (lw8_flush()), never
// while an RX window may be open. Restore: a missing blob is handed over as
// an all-zero buffer, which RadioLib rejects (CHECKSUM_MISMATCH); the
// s_lw8_*_found flags tell that apart from a stored blob it rejected.
#define LW8_KEY_PERS "lw8_pers"
#define LW8_KEY_SESS "lw8_sess"
static uint8_t s_lw8_pers[RADIOLIB_LORAWAN_PERSISTENCE_BUF_SIZE];
static uint8_t s_lw8_sess[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];
static bool s_lw8_pers_dirty = false;
static bool s_lw8_sess_dirty = false;
// Restore log wording only (lw_restore_verdict()): what the restore callbacks found.
static bool s_lw8_pers_found  = false;   // a persistence blob was in NVS at boot
static bool s_lw8_sess_called = false;   // RadioLib got as far as the session restore
static bool s_lw8_sess_found  = false;   // ... and a session blob was in NVS
static bool s_lw8_sess_unfinished = false;   // ... saved before its first uplink: withheld

// cppcheck-suppress constParameterCallback ; signature fixed by RadioLib BufferCb_t
static void lw8_store_pers(uint8_t *buf, size_t len) {
    if (len != sizeof(s_lw8_pers)) return;
    memcpy(s_lw8_pers, buf, len);
    s_lw8_pers_dirty = true;
}
// cppcheck-suppress constParameterCallback ; signature fixed by RadioLib BufferCb_t
static void lw8_store_sess(uint8_t *buf, size_t len) {
    if (len != sizeof(s_lw8_sess)) return;
    memcpy(s_lw8_sess, buf, len);
    s_lw8_sess_dirty = true;
}
static void lw8_restore_pers(uint8_t *buf, size_t len) {
    s_lw8_pers_found = nvs_load_blob(LW8_KEY_PERS, buf, len);
    if (!s_lw8_pers_found) memset(buf, 0, len);
}
/** @brief RadioLib's session restore callback. A session still marked
 *         "activating" is handed over as empty instead (V2.8.21). RadioLib 8.0
 *         saves the session right after a join, BEFORE it marks it active;
 *         after a reset before the first uplink it would apply that blob's
 *         channel masks/DR, reject it with -1101 and leave the status at
 *         "activating" — the next join then skips createSession() (no
 *         defaults; on EU868 stale masks point at zeroed CFList channels, so
 *         a DR0 join picks frequency 0: -12). An all-zero buffer fails
 *         RadioLib's checkBufferCommon() before any state is touched, so the
 *         join starts exactly as on a fresh board; the DevNonce history (the
 *         persistence buffer) is not affected. */
static void lw8_restore_sess(uint8_t *buf, size_t len) {
    s_lw8_sess_called = true;
    s_lw8_sess_found = nvs_load_blob(LW8_KEY_SESS, buf, len);
    s_lw8_sess_unfinished = s_lw8_sess_found &&
                            buf[RADIOLIB_LORAWAN_SESSION_STATUS] == RADIOLIB_LORAWAN_SESSION_ACTIVATING;
    if (!s_lw8_sess_found || s_lw8_sess_unfinished) memset(buf, 0, len);
}

// Session forget for /lorawan_reset (http_server.c). Erases only the NVS
// blobs — the running worker's in-RAM session is untouched (it isn't built
// for live restart), so the caller reboots right after. The s_forget_pending
// flag above suppresses the worker's end-of-cycle session re-save until
// that reboot lands, making the wipe deterministic.
void lorawan_forget_session(bool wipe_nonces) {
    s_forget_pending = true;   // BEFORE the erase — closes the save/erase race
    nvs_handle_t h;
    if (nvs_open("lorawan", NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "forget session: nvs_open failed");
        return;
    }
    // ESP_ERR_NVS_NOT_FOUND (blob never written) is success for an erase.
    esp_err_t e = nvs_erase_key(h, LW8_KEY_SESS);
    if (e != ESP_OK && e != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "forget session: erase '%s': %s", LW8_KEY_SESS, esp_err_to_name(e));
    }
    if (wipe_nonces) {
        e = nvs_erase_key(h, LW8_KEY_PERS);
        if (e != ESP_OK && e != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGE(TAG, "forget session: erase '%s': %s", LW8_KEY_PERS, esp_err_to_name(e));
        }
    }
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGW(TAG, "session forgotten (%s) — fresh OTAA join on next boot",
             wipe_nonces ? "incl. DevNonce history" : "DevNonce history kept");
}

/** @brief Erase the RadioLib 7.x session blob V2.8.20 and older kept under
 *         "session" (V2.8.21). Called once the RadioLib 8.0 session is active
 *         (joined or resumed): the old session is dead on the network from
 *         then on. Left in place, a node put back on V2.8.20 (by hand, or by
 *         V2.8.20's own crash rollback) would resume it and send into a
 *         session the network no longer accepts, without ever re-joining.
 *         Without it, V2.8.20 joins afresh. Its DevNonce history ("nonces")
 *         is kept on purpose: V2.8.20 loads it independently, so that join
 *         continues the old counter, which is normally above every DevNonce
 *         V2.8.21 used since restarting at 0 (unless V2.8.21 sent more join
 *         requests than the whole earlier history) — TTN then accepts it
 *         without another reset. Not found = nothing to do, silent. */
static void lw_erase_v2820_session(void) {
    nvs_handle_t h;
    const esp_err_t o = nvs_open("lorawan", NVS_READWRITE, &h);
    if (o != ESP_OK) {
        // Logged: the old session stays, and a return to V2.8.20 would resume it.
        ESP_LOGW(TAG, "could not remove the LoRaWAN session saved by V2.8.20 or older: %s", esp_err_to_name(o));
        return;
    }
    const esp_err_t e = nvs_erase_key(h, "session");
    if (e == ESP_OK) nvs_commit(h);
    nvs_close(h);
    if (e == ESP_OK) {
        ESP_LOGI(TAG, "removed the LoRaWAN session saved by V2.8.20 or older (a return to that firmware "
                      "now joins afresh)");
    } else if (e != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "could not remove the LoRaWAN session saved by V2.8.20 or older: %s", esp_err_to_name(e));
    }
}

// Write what RadioLib handed the store callbacks during the exchange or join
// attempt that just ended (worker task, radio idle). The ONLY writer of the
// two blobs; skipped once an operator's wipe is pending, so an exchange that
// ends between the wipe and the restart can't write the session back.
static void lw8_flush(void) {
    if (s_forget_pending) return;
    if (s_lw8_pers_dirty) { s_lw8_pers_dirty = false; nvs_save_blob(LW8_KEY_PERS, s_lw8_pers, sizeof(s_lw8_pers)); }
    if (s_lw8_sess_dirty) { s_lw8_sess_dirty = false; nvs_save_blob(LW8_KEY_SESS, s_lw8_sess, sizeof(s_lw8_sess)); }
}

// --- Worker task --------------------------------------------------------------

// Queue depth 1 — a freshest-wins mailbox: the producer overwrites any
// unconsumed snapshot (see lorawan_transmit()), so the worker always pulls
// the latest window. Mirrors transmission.c's queue-depth-1 convention but
// NOT its drop-if-busy policy — see lorawan_transmit() for why they differ.
static QueueHandle_t s_q;

// --- LoRaWANNode subclass -----------------------------------------------------
//
// RadioLib 8.0's own Class A receive path is used unchanged: a frame its
// checkDownlink() rejects (not ours, unreadable) is dropped and the window
// counts as empty, so RX2 and the NbTrans repeats go on (the observer marks
// such a frame "ignored" in the log, lorawan_rxobs.h). Uses protected RadioLib
// members (channels, band, dwellTimeUp, phyLayer, nbTrans) — re-check them on
// every RadioLib update.
class GeigerNode : public LoRaWANNode {
public:
    GeigerNode(PhysicalLayer *phy, const LoRaWANBand_t *band, uint8_t subBand)
        : LoRaWANNode(phy, band, subBand) {}

    /** @brief Data rate the NEXT uplink will use (the session's current DR). */
    uint8_t uplinkDr() const { return this->channels[RADIOLIB_LORAWAN_UPLINK].dr; }

    /** @brief True if a `len`-byte application payload can go out at the
     *         current DR: the band's size limit and, on dwell-time plans
     *         (AU915/AS923: 400 ms), the air time of the whole frame (+13 B
     *         MHDR/FHDR/FPort/MIC) — the same ToA-vs-dwell arithmetic
     *         RadioLib's adrBackoff() uses. Not getMaxPayloadLen(): that one
     *         also subtracts the queued MAC answers. */
    bool payloadFits(uint8_t len) { return this->frameFits(len, 0); }

    /** @brief As payloadFits(), for `payload` bytes plus `fopts` bytes of
     *         MAC answers in the FOpts field. */
    bool frameFits(size_t payload, size_t fopts) {
        const uint8_t dr = this->uplinkDr();
        if (dr >= RADIOLIB_LORAWAN_CHANNEL_NUM_DATARATES) return false;
        if (payload + fopts > this->band->payloadLenMax[dr]) return false;
        if (!this->dwellTimeUp) return true;
        const LoRaWANDataRate_t *d = &this->band->dataRates[dr];
        return this->phyLayer->calculateTimeOnAir(d->modem, d->dr, d->pc, payload + fopts + 13) / 1000 <=
               this->dwellTimeUp;
    }

    /** @brief NbTrans the next sendReceive() will use (set by the network's
     *         LinkADRReq, reset to 1 by the ADR backoff). */
    uint8_t nbTransNow() const { return this->nbTrans; }

    /** @brief After activation: duty-cycle limit back to the band's own value
     *         (EU868: 36 s/h = 1 %) unless the network set one. RadioLib seeds
     *         every session with a synthetic DutyCycleReq (MaxDCycle 7 = 1/128
     *         for a 1 % band, 10 for 0.1 %) and replays it on every restore,
     *         so a setDutyCycle() before activation does not stick. A stored
     *         MaxDCycle other than that seed came from the network and is kept
     *         (a network request for exactly the seed value can't be told
     *         apart and is relaxed too). No-op with the limiter off or on
     *         bands without a limit. Call only while the session is active:
     *         createSession() runs before activation only.
     *  @param[out] by_network true if the limit in force was set by the network.
     *  @return the limit in force, ms of air time per hour (0 = none). */
    uint32_t applyBandDutyCycle(bool *by_network) {
        *by_network = false;
        if (!this->dutyCycleEnabled) return 0;   // limiter off: no limit in force
        if (this->band->dutyCycle == 0) {        // band without a limit: any limit is the network's
            *by_network = (this->dutyCycle != 0);
            return this->dutyCycle;
        }
        uint8_t seed = 0;   // createSession()'s MaxDCycle for this band
        if (this->band->dutyCycle == 3600) seed = 10;
        else if (this->band->dutyCycle == 36000) seed = 7;
        const uint8_t stored = this->bufferSession[RADIOLIB_LORAWAN_SESSION_DUTY_CYCLE] & 0x0F;
        if (stored == seed) this->setDutyCycle(true, 0);   // 0 = the band's value
        else *by_network = true;
        return this->dutyCycle;
    }
};

// --- Uplink schedule (V2.8.14) -------------------------------------------------
//
// One schedule per node (lorawan_sched.h): while LoRaWAN runs, main.c's TX
// rounds follow lorawan_cycle_interval_ms(), i.e. the interval for the
// spreading factor of the NEXT uplink's data rate. s_cycle_sf is written by
// lorawan_setup() (main task, before the worker exists: the join DR) and then
// only by the worker (after every join attempt and exchange); read by any task.
// (State variables s_cycle_sf / s_band / s_fixed_dr* are defined next to
// s_status above, because lorawan_get_status() reads them.)

// The LoRa spreading factor of a band's data rate. FSK / LR-FHSS rates have
// none and are fast: the SF7 row. An unused DR slot: the slowest row.
static uint8_t sf_of_dr(const LoRaWANBand_t *band, uint8_t dr) {
    if (dr >= RADIOLIB_LORAWAN_CHANNEL_NUM_DATARATES) return 12;
    const LoRaWANDataRate_t *d = &band->dataRates[dr];
    if (d->modem == RADIOLIB_MODEM_LORA) return d->dr.lora.spreadingFactor;
    if (d->modem == RADIOLIB_MODEM_NONE) return 12;
    return 7;
}

static void set_cycle_sf(uint8_t sf) { __atomic_store_n(&s_cycle_sf, sf, __ATOMIC_RELAXED); }

/** @brief set_cycle_sf() plus one log line when the uplink interval changes
 *         (or always, with @p force, right after a join or session resume), so
 *         a long gap before the next uplink reads as the schedule, not a hang.
 *         The TX round runs one interval after the previous one (main.c), so
 *         the next uplink comes at most one interval from now. Worker task only. */
static void set_cycle_sf_noted(uint8_t sf, bool force) {
    const uint8_t was = __atomic_load_n(&s_cycle_sf, __ATOMIC_RELAXED);
    set_cycle_sf(sf);
    if (sf == 0 || (sf == was && !force)) return;
    // Whole minutes: every lw_sched_interval_ms() entry is a whole number of
    // minutes (4/7/12/22/30) — keep it that way or round here.
    const unsigned min = (unsigned)(lw_sched_interval_ms(sf) / 60000u);
    if (__atomic_load_n(&s_fixed_dr, __ATOMIC_RELAXED)) {
        ESP_LOGI(TAG, "uplink interval now %u min (fixed DR%u = SF%u schedule) — the next uplink comes at most %u min "
                      "from now", min, (unsigned)s_fixed_dr_value, (unsigned)sf, min);
    } else {
        ESP_LOGI(TAG, "uplink interval now %u min (SF%u schedule, data rate set by ADR) — the next uplink comes at "
                      "most %u min from now", min, (unsigned)sf, min);
    }
}

uint32_t lorawan_cycle_interval_ms(void) {
    portENTER_CRITICAL(&s_status_mux);
    const lorawan_state_t st = s_status.state;
    portEXIT_CRITICAL(&s_status_mux);
    if (st != LORAWAN_ST_JOINING && st != LORAWAN_ST_JOINED) return 0;
    const uint8_t sf = __atomic_load_n(&s_cycle_sf, __ATOMIC_RELAXED);
    return sf ? lw_sched_interval_ms(sf) : 0;
}

// Apply the fixed data rate (ADR off), after a join or session resume and
// whenever a network command moved the DR: RadioLib's LinkADRReq handler sets
// the DR even with ADR disabled (LoRaWAN.cpp, MAC_LINK_ADR — only the channel
// mask rule depends on adrEnabled). Falls back to ADR, once and for good, if
// the region has no channel for that DR or a 10-byte port-1 frame wouldn't fit
// (dwell-time-limited plans: SF11/SF12 exceed AU915's / AS923's 400 ms).
static void apply_fixed_dr(GeigerNode &node) {
    if (!__atomic_load_n(&s_fixed_dr, __ATOMIC_RELAXED)) return;
    const uint8_t want = s_fixed_dr_value;
    const uint8_t was  = node.uplinkDr();
    node.setADR(false);   // RAM-only flag: set again after every boot / resume
    int16_t st = (was == want) ? RADIOLIB_ERR_NONE : node.setDatarate(want);
    if (st == RADIOLIB_ERR_NONE && !node.payloadFits(10)) st = RADIOLIB_ERR_DWELL_TIME_EXCEEDED;
    if (st != RADIOLIB_ERR_NONE) {
        ESP_LOGW(TAG, "fixed DR%u not usable in this region: %d (%s) — falling back to ADR",
                 (unsigned)want, (int)st, lw_err_text(st));
        __atomic_store_n(&s_fixed_dr, false, __ATOMIC_RELAXED);
        // Only the size/dwell branch gets here with the DR moved (a refused
        // setDatarate() leaves it unchanged).
        if (node.uplinkDr() != was) (void)node.setDatarate(was);
        node.setADR(true);
        return;
    }
    if (was != want) {
        ESP_LOGI(TAG, "data rate fixed at DR%u (was DR%u), ADR off", (unsigned)want, (unsigned)was);
    }
}

// s_version_packed: set once in lorawan_setup() (main task, at boot) via
// lw_pack_version(VERSION_STR); read (never written) by the worker task
// thereafter — a plain uint16_t load/store pair needs no locking either way.
static uint16_t s_version_packed = 0;

static void set_state(lorawan_state_t st) {
    portENTER_CRITICAL(&s_status_mux);
    s_status.state = st;
    portEXIT_CRITICAL(&s_status_mux);
}

// --- Link-health telemetry (V2.8.12) -----------------------------------------
//
// Worker-task-private state behind the status-line / /status fields
// (lorawan.h) and the per-uplink log lines. All of it is this-boot only.

static uint32_t lw_uptime_s(void) { return (uint32_t)(esp_timer_get_time() / 1000000LL); }

// Rolling ~24 h air time: 24 one-hour buckets indexed by uptime hour. The sum
// covers the current partial hour plus the 23 before it. Compared against
// TTN's Fair Use budget (30 s uplink air time per device per day).
static uint32_t s_air_bucket_ms[24];
static uint32_t s_air_hour = 0;    // uptime hour that s_air_bucket_ms[s_air_hour % 24] holds

static uint32_t airtime_add(uint32_t ms) {
    const uint32_t now_h = lw_uptime_s() / 3600u;
    if (now_h - s_air_hour >= 24u) {
        memset(s_air_bucket_ms, 0, sizeof(s_air_bucket_ms));   // idle > 24 h: all stale
    } else {
        while (s_air_hour < now_h) s_air_bucket_ms[++s_air_hour % 24u] = 0;   // clear skipped hours
    }
    s_air_hour = now_h;
    s_air_bucket_ms[now_h % 24u] += ms;
    uint32_t sum = 0;
    for (uint32_t i = 0; i < 24u; i++) sum += s_air_bucket_ms[i];
    return sum;
}

// Uplinks since the last downlink. Mirrors RadioLib's ADR backoff counter
// (LoRaWAN.cpp adrBackoff: ADRACKReq from 64 uplinks without a downlink, one
// backoff step every 32 after that) closely enough to explain DR changes —
// but RadioLib's copy persists in the NVS session, ours restarts at boot.
#define ADR_ACK_LIMIT 64u   // RADIOLIB_LORAWAN_ADR_ACK_LIMIT_EXP = 6
static uint32_t s_ul_since_dl = 0;

// Previous uplink's radio parameters, to log ADR-driven changes.
static bool    s_prev_valid  = false;
static uint8_t s_prev_dr     = 0;
static int16_t s_prev_power  = 0;
static bool    s_prev_had_dl = false;   // previous exchange received a downlink

// Receive-side-only errors after a real TX: lw_downlink_unusable()
// (lorawan_rxobs.h, host-tested). Counting them as "uplink failed" would
// inflate fail=/last_err= with events that say nothing about the uplink.
// Only used when a TX was actually seen.
static bool downlink_unusable(int16_t st) { return lw_downlink_unusable(st); }

// Classify one sendReceive() outcome under s_status_mux. House rule (memory:
// no-blocking-calls-in-critical-section): time()/ESP_LOG/RadioLib getters
// (s_radio.packetStatus() does a live SPI round-trip) are ALL called
// BEFORE taking the spinlock; only the plain-value stores happen inside it.
//
// V2.8.12: `st` > 0 is the RX window (1/2) a downlink arrived in — counted
// as a downlink even when it carried only MAC commands (dl_len == 0). Until
// V2.8.11 only application payloads were logged, so e.g. a received
// LinkADRReq was invisible (2026-10-18: NbTrans=3 applied with no log line).
// `air_ms` = air time of this uplink incl. NbTrans repeats (0 if not sent).
static void record_uplink_result(int16_t st, size_t dl_len, const LoRaWANEvent_t *ev,
                                 uint32_t air_ms) {
    int64_t now = (int64_t)time(NULL);           // captured before the lock
    const uint32_t up_s = lw_uptime_s();
    float rssi = 0.0f, snr = 0.0f;
    bool have_dl = (st > 0);
    if (have_dl) {
        // V2.8.19: RssiPkt, SignalRssiPkt and SNR from one status read (SPI
        // round-trip — before the lock). The signal estimate follows the SNR
        // on the line: at negative SNR RssiPkt is mostly the noise floor.
        float signal = 0.0f;
        s_radio.packetStatus(&rssi, &signal, &snr);
        // V2.8.19: on a Heltec V4.3 with the LNA on, both readings include
        // the LNA's gain (~24 dB measured), so the line says so at the end.
        const char *lna = "";
#if HAL_HAS_LORA_FEM
        if (lorawan_dl_rssi_includes_lna()) lna = ", LNA on";
#endif
        if (dl_len > 0) {
            ESP_LOGI(TAG, "downlink (RX%d): port %u, %u bytes, FCntDown=%lu, RSSI %.0f dBm SNR %.1f dB, signal %.0f dBm%s",
                     (int)st, (unsigned)ev->fPort, (unsigned)dl_len, (unsigned long)ev->fCnt,
                     (double)rssi, (double)snr, (double)signal, lna);
        } else {
            ESP_LOGI(TAG, "downlink (RX%d): MAC commands only, FCntDown=%lu, RSSI %.0f dBm SNR %.1f dB, signal %.0f dBm%s",
                     (int)st, (unsigned long)ev->fCnt, (double)rssi, (double)snr, (double)signal, lna);
        }
    }

    // Air time counts whenever something went on air (air_ms > 0), even if the
    // exchange then failed; a duty-cycle-blocked or pre-TX failure adds 0.
    const uint32_t air_24h = airtime_add(air_ms);
    // V2.8.12: a receive-side-only error after a real TX is a sent uplink
    // without a usable downlink, not a failed uplink — see downlink_unusable().
    const bool rx_only = (air_ms > 0) && downlink_unusable(st);
    const bool sent_ok = (st >= RADIOLIB_ERR_NONE) || rx_only;
    if (rx_only) {
        ESP_LOGI(TAG, "downlink unusable: %d (%s) — the uplink itself was sent", (int)st, lw_err_text(st));
    }
    if (sent_ok) {
        if (have_dl) {
            s_ul_since_dl = 0;
        } else if (++s_ul_since_dl == ADR_ACK_LIMIT && __atomic_load_n(&s_fixed_dr, __ATOMIC_RELAXED)) {
            // Fixed DR = ADR off: RadioLib sets neither the ADR bit nor
            // ADRACKReq and skips adrBackoff() (LoRaWAN.cpp composeUplink /
            // sendReceive), so nothing changes on the node's side.
            ESP_LOGW(TAG, "no downlink for %u uplinks since boot (fixed data rate, ADR off: "
                     "no ADRACKReq, no backoff)", (unsigned)ADR_ACK_LIMIT);
        } else if (s_ul_since_dl == ADR_ACK_LIMIT) {
            // Worded for THIS boot's count: RadioLib's own counter lives in the
            // NVS session and survives reboots, so after a reboot it may be well
            // past 64 already (2026-10-18: this line came 2 h after RadioLib's
            // ADRACKReq started, with two backoff steps already taken).
            ESP_LOGW(TAG, "no downlink for %u uplinks since boot — the node is asking the "
                     "network for an answer (ADRACKReq; RadioLib's count survives reboots "
                     "and may be higher); ADR backoff (one step every 32 uplinks: TX power "
                     "to max, then DR-1, last the default channels with one transmission "
                     "per frame) continues while the network stays silent",
                     (unsigned)ADR_ACK_LIMIT);
        }
    }

    portENTER_CRITICAL(&s_status_mux);
    s_status.airtime_24h_ms = air_24h;
    portEXIT_CRITICAL(&s_status_mux);

    if (sent_ok) {
        portENTER_CRITICAL(&s_status_mux);
        s_status.uplinks_sent++;
        s_status.last_uplink_at = now;
        s_status.uplinks_since_dl = s_ul_since_dl;
        if (have_dl) {
            s_status.last_dl_rssi = rssi;
            s_status.last_dl_snr  = snr;
            s_status.last_dl_uptime_s = up_s ? up_s : 1;   // 0 is reserved for "none"
        }
        portEXIT_CRITICAL(&s_status_mux);
    } else if (st == RADIOLIB_ERR_UPLINK_UNAVAILABLE || st == RADIOLIB_ERR_NO_CHANNEL_AVAILABLE ||
               st == RADIOLIB_ERR_DWELL_TIME_EXCEEDED) {
        // Regulatory guard: RadioLib itself refused the send before TX (duty
        // cycle, no enabled channel, or a frame over the 400 ms dwell limit),
        // not a radio/network failure — counted as skipped, not as failed.
        // duty_skip= counts refused attempts, not lost frames.
        portENTER_CRITICAL(&s_status_mux);
        s_status.duty_skipped++;
        portEXIT_CRITICAL(&s_status_mux);
        ESP_LOGI(TAG, "frame skipped by the regulatory guard: %d (%s)", (int)st, lw_err_text(st));
    } else if (st == RADIOLIB_ERR_MAC_ANSWERS_PENDING) {
        // Refused before TX: the payload fits the DR, but not together with the
        // queued MAC answers. send_app_frame() recovers (MAC answers alone,
        // then the frame), so neither failed= nor last_err= is touched; logged
        // there. (A payload too long on its own is PACKET_TOO_LONG, -4: nothing
        // recovers that, so it counts as a failure below.)
    } else {
        portENTER_CRITICAL(&s_status_mux);
        s_status.failed++;
        s_status.last_error = st;
        portEXIT_CRITICAL(&s_status_mux);
        ESP_LOGW(TAG, "uplink failed: %d (%s)", (int)st, lw_err_text(st));
    }
}

// Success-path visibility: one INFO line per accepted uplink, with the radio
// parameters RadioLib reports for the frame actually put on air. Added at
// user request during the first live-gateway bench session — the silent
// success path (counters-only, /status) made /log look like LoRaWAN was
// doing nothing at all. `what` carries the per-port payload summary so the
// radio-parameter tail isn't duplicated at both call sites.
//
// fcnt_wire = ev->fCnt: RadioLib 8.0 fills the uplink event BEFORE it
// advances fCntUp for the next frame, so it is the FCnt that went on air.
// (getFCntUp() is not: after a MAC-only uplink RadioLib sent inside the
// exchange, it names that frame's counter.)
//
// The channels and the TX count come from the RX-window observer
// (rxobs_uplink_tx()): one frequency per repeat ("867.9/868.3/868.1 MHz" —
// ev->freq is only the last repeat's, and in Hz), "TX=2/3" = sent / NbTrans
// in force, plus any TX of a MAC-only uplink inside the exchange. The
// "rx windows:" line right after says why fewer repeats went out. `st` is
// sendReceive()'s return: 0 = no downlink, 1/2 = RX window. The RSSI/SNR of a
// received downlink are on the "downlink (RXn)" line logged just before.
static void log_uplink_ok(const char *what, uint32_t fcnt_wire, const LoRaWANEvent_t *ev,
                          int16_t st, uint32_t air_ms) {
    char freqs[OBS_MAX_REP * 7 + 8], tx[48];   // tx worst case "15/15 +255 MAC-only (from FCnt 4294967295)"
    rxobs_uplink_tx(&s_obs, st, fcnt_wire, freqs, sizeof(freqs), tx, sizeof(tx));
    ESP_LOGI(TAG, "uplink OK: %s FCnt=%lu DR%u %s MHz %d dBm TX=%s air=%lu ms | DL: %s",
             what, (unsigned long)fcnt_wire, (unsigned)ev->datarate, freqs, (int)ev->power, tx,
             (unsigned long)air_ms,
             st == 1 ? "RX1" : st == 2 ? "RX2" : st < 0 ? "unusable (see rx windows)" : "none");
}

// Log a DR / TX-power change between consecutive uplinks and its likely
// cause. A change shows up on the uplink AFTER the exchange that caused it:
// a received LinkADRReq (previous exchange had a downlink) or RadioLib's ADR
// backoff (applied at the end of an exchange without one). NbTrans is not
// readable through RadioLib's API, so only DR and power are tracked.
static void adr_track(const LoRaWANEvent_t *ev, int16_t st) {
    if (s_prev_valid && (ev->datarate != s_prev_dr || ev->power != s_prev_power)) {
        ESP_LOGW(TAG, "ADR: DR%u -> DR%u, %d -> %d dBm (%s)",
                 (unsigned)s_prev_dr, (unsigned)ev->datarate, (int)s_prev_power, (int)ev->power,
                 s_prev_had_dl ? "network, after downlink"
                               : "ADR backoff — no downlink heard");
    }
    s_prev_valid  = true;
    s_prev_dr     = ev->datarate;
    s_prev_power  = ev->power;
    s_prev_had_dl = (st > 0);
}

// --- DeviceTimeReq network time sync (design approved 2026-07-27) -------------
//
// Class A nodes can't hear Class B time beacons; the standard Class A
// equivalent is the DeviceTimeReq MAC command (LoRaWAN 1.0.3+), piggybacked
// on a normal uplink and answered by the network in the RX window with
// GPS-epoch time (RadioLib converts to unix UTC, leap seconds included).
// Cadence: first uplink after boot + 24 h refresh — the ANSWER is a network
// downlink, and TTN Fair Use allows only 10 downlinks/day, so per-TX-round
// requests are off the table. No answer this round (networks may serve MAC
// answers lazily) → the stale s_timesync_last_us re-arms the request on the
// next cycle until one lands.

#define TIMESYNC_INTERVAL_S   (24UL * 3600UL)   // 24 h between answers
#define TIMESYNC_STEP_MIN_S   2                 // don't step for less
// Give-up rule (asymmetric-link guard, 2026-07-27): every request the
// network hears makes it schedule a DeviceTimeAns DOWNLINK, whether or not
// the node can hear it. A node with working uplink but dead downlink
// (gateway hears us, we can't hear it) would otherwise re-request every
// cycle — ~576 pointless gateway transmissions/day, each one deafening the
// gateway to every other node while it sends. After this many unanswered
// attempts, fall back to the normal daily cadence instead.
#define TIMESYNC_MAX_ATTEMPTS 5

// Uptime SECONDS, not µs: raw esp_timer µs would force a 64-bit stamp
// (32-bit µs wraps every ~71.6 min — useless against a 24 h interval), but
// second granularity is ample here and fits uint32_t with a 136-year wrap.
// 0 = never: the first uplink can't happen inside the boot's first second
// (join/resume + a full TX interval come first), and even if it somehow
// did, the cost is one extra harmless request. Worker-task-private, so no
// cross-task atomicity concern either way — this is purely width hygiene.
static uint32_t timesync_now_s(void) {
    return (uint32_t)(esp_timer_get_time() / 1000000LL);
}
static uint32_t s_timesync_last_s   = 0;   // uptime s at last DeviceTimeAns (or give-up); 0 = never
static uint32_t s_timesync_attempts = 0;   // consecutive unanswered requests in the current window

// No "stale answer" guard is needed: sendReceive() clears the MAC downlink
// store before every uplink (LoRaWAN.cpp), so an answer read right after the
// port-1 exchange can only belong to that exchange.

// Shared give-up path: stamp the window as if answered so the next request
// waits the full daily interval, WARN once per window (not per cycle).
// Reached from BOTH the no-answer poll and repeated sendMacCommandReq
// queue failures — the latter must count toward give-up too, else a
// persistently full FOpts queue would retry + WARN every cycle forever.
static void timesync_give_up(void) {
    // Wording covers both give-up causes: unanswered requests (downlink
    // dead / asymmetric link) AND requests that never went on air (queue
    // failures) — both paths land here.
    ESP_LOGW(TAG, "DeviceTime sync unsuccessful after %u attempts (unanswered "
             "or not queued) — deferring further requests to the daily window",
             (unsigned)s_timesync_attempts);
    s_timesync_last_s   = timesync_now_s();
    s_timesync_attempts = 0;
}

// Apply one DeviceTimeAns to the system clock. UTC in, settimeofday() UTC —
// local rendering everywhere already goes through localtime()/the configured
// POSIX TZ (ntp_set_timezone() at boot), exactly as with NTP, per the user's
// TZ requirement. Gated on "SNTP has never synced" (ntp_boot_epoch()==0):
// NTP stays the sole clock authority the moment it ever syncs (house
// precedent: GNSS time is display-only). Deliberately NOT gated on
// ntp_time_valid() — that is a clock-plausibility check (past 2026-01-01)
// that turns true the moment WE set the clock, which would kill the daily
// re-sync after the first apply.
//
// RadioLib returns the answer already advanced by the time elapsed since the
// end of the uplink that carried the request (getMacDeviceTimeAns():
// `ms += millis() - tUplinkEnd`), with the fraction in milliseconds, so for
// a frame sent once there is no lag left. With NbTrans > 1, tUplinkEnd is the
// end of the LAST repeat while the network's time refers to the repeat it
// heard, so the clock can be early by up to one repeat period (~6-7 s at
// EU868); the daily re-sync keeps that bounded.
static void timesync_apply(uint32_t unix_s, uint16_t ms) {
    if (ntp_boot_epoch() != 0) {
        // NTP synced while the request was out: NTP stays the clock authority,
        // but how far the two disagree is worth one line.
        struct timeval now;
        gettimeofday(&now, NULL);
        const int64_t diff_ms = ((int64_t)unix_s * 1000 + ms) - ((int64_t)now.tv_sec * 1000 + now.tv_usec / 1000);
        const time_t nt = (time_t)unix_s;
        struct tm ut;
        gmtime_r(&nt, &ut);
        char when[24];
        strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S", &ut);
        ESP_LOGI(TAG, "DeviceTime: network clock %s.%03u UTC, %lld ms %s the NTP clock — not applied (NTP is synced)",
                 when, (unsigned)ms, (long long)(diff_ms < 0 ? -diff_ms : diff_ms),
                 diff_ms < 0 ? "behind" : "ahead of");
        return;
    }
    int64_t delta = (int64_t)unix_s - (int64_t)time(NULL);
    if (delta >= -TIMESYNC_STEP_MIN_S && delta <= TIMESYNC_STEP_MIN_S) {
        ESP_LOGI(TAG, "network time received: clock already within %+lld s — no step",
                 (long long)delta);
        return;
    }
    struct timeval tv;
    tv.tv_sec  = (time_t)unix_s;
    tv.tv_usec = (suseconds_t)((uint32_t)ms * 1000UL);   // RadioLib's fraction is in ms
    settimeofday(&tv, NULL);
    time_t t = (time_t)unix_s;
    struct tm lt;
    localtime_r(&t, &lt);
    char buf[40];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &lt);
    ESP_LOGI(TAG, "network time applied: %s (DeviceTimeAns, step %+lld s)",
             buf, (long long)delta);
}

// --- One uplink exchange (V2.8.12) ---------------------------------------------
//
// sendReceive() plus everything around it that both ports need: the OTA
// suspension handshake, the RX-window observer, result bookkeeping + log
// lines, and the session save.
//
// Session saved after EVERY exchange (A4), not once per cycle: a reset
// between port 1 and port 2 used to rewind FCntUp to the cycle start, so the
// already-sent port-1 counter went out again and the network dropped it as a
// replay (XIAO 2026-10-18: FCnt 91 at 19:25:35 and again at 19:28:16 after an
// OTA-time watchdog reset). Wear: one ~300 B write per frame, and the V2.8.14
// schedule caps the frame rate (at most one port-1 frame per 4 min, port 2
// hourly on Geiger + environment nodes, every cycle on environment-only
// ones) — decades on the wear-levelled NVS
// partition (see nvs_save_blob()).
//
// V2.8.14 duty-cycle backstop: RadioLib's limiter is on (setDutyCycle(true)
// in lorawan_task(); none on US915/AU915/CN470/KR920/IN865). On EU868-type
// plans the limit is the band's 36 s/h (1 %) since V2.8.21: RadioLib's
// session default is 1/128 = ~28 s/h (createSession() applies a synthetic
// DutyCycleReq, MaxDCycle 7, and restores replay it), which
// applyBandDutyCycle() undoes after activation; a limit the network set is
// kept. sendReceive() would REFUSE a frame that comes too early
// (RADIOLIB_ERR_UPLINK_UNAVAILABLE). The SF schedule normally keeps well
// clear of it; the limiter bites at very high NbTrans and for the port-2
// frame right after port 1, which waits ~100 x the port-1 exchange's air
// time (~2.5 min at SF12; ~128 x under a 1/128 network limit). So wait here
// instead of losing the frame — outside exchange_begin(), so an OTA can
// still suspend the worker meanwhile — unless the wait would exceed
// DUTY_WAIT_MAX_MS, then skip it (counted as duty_skipped).
//
// `what` is the payload summary for the "uplink OK" line. Returns
// sendReceive()'s status (<0 error, 0 no downlink, 1/2 downlink window),
// except that a receive-side-only error after a real TX returns 0.
#define DUTY_WAIT_MAX_MS (10UL * 60UL * 1000UL)
// Worker-task-private: did the last uplink_exchange() put a frame on air?
// Read by poll_device_time(), so a skipped frame isn't counted as an
// unanswered DeviceTimeReq.
static bool s_last_exchange_sent = false;

/** @brief One info line when RadioLib's 32-bit millisecond clock (HAL millis())
 *         has wrapped since the last check — every 49.7 days of uptime. RadioLib
 *         8.0 handles the wrap itself; the line lets a reader tie anything odd
 *         at that moment to it. Worker task only (before each exchange and join
 *         attempt). */
static void lw_clock_wrap_note(GeigerNode &node) {
    static uint32_t s_last_ms = 0;
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    const bool wrapped = now_ms < s_last_ms;
    s_last_ms = now_ms;
    if (!wrapped) return;
    ESP_LOGI(TAG, "RadioLib's millisecond clock wrapped (every 49.7 days of uptime) — handled by RadioLib 8.0, "
                  "next uplink allowed in %lu ms", (unsigned long)node.timeUntilUplink());
}
static int16_t uplink_exchange(GeigerNode &node, const uint8_t *data, size_t len, uint8_t port,
                               const char *what, uint8_t *dl, size_t *dl_len) {
    s_last_exchange_sent = false;
    lw_clock_wrap_note(node);
    const RadioLibTime_t wait_ms = node.timeUntilUplink();
    if (wait_ms > DUTY_WAIT_MAX_MS) {
        ESP_LOGW(TAG, "duty cycle: port %u frame skipped, next uplink allowed in %lu s",
                 (unsigned)port, (unsigned long)(wait_ms / 1000));
        portENTER_CRITICAL(&s_status_mux); s_status.duty_skipped++; portEXIT_CRITICAL(&s_status_mux);
        return RADIOLIB_ERR_UPLINK_UNAVAILABLE;
    }
    if (wait_ms > 0) {
        ESP_LOGI(TAG, "duty cycle: waiting %.1f s before the port %u frame",
                 (double)wait_ms / 1000.0, (unsigned)port);
        vTaskDelay(pdMS_TO_TICKS(wait_ms) + 1);
    }
    if (!exchange_begin()) park_for_ota();
    LoRaWANEvent_t ev_up = {}, ev_dn = {};
    *dl_len = 0;
    rxobs_begin(&s_obs, node.nbTransNow());   // NbTrans read BEFORE the exchange changes it
    int16_t st = node.sendReceive(data, len, port, dl, dl_len, false, &ev_up, &ev_dn);
    s_obs.on = false;
    // getLastToA(): RadioLib resets it at the start of each sendReceive() and
    // adds every NbTrans repeat (also before a TX timeout), and a MAC-only
    // uplink it sends inside the exchange keeps the outer air time — so this
    // is the exchange's whole uplink air time. Keyed on "a TX was seen", not
    // on st: a frame that went on air but whose downlink failed to parse
    // (st < 0) still spent the air time.
    const bool sent = (s_obs.tx_us != 0);
    s_last_exchange_sent = sent;
    const uint32_t air_ms = sent ? (uint32_t)node.getLastToA() : 0;
    record_uplink_result(st, *dl_len, &ev_dn, air_ms);
    // Receive-side-only errors after a real TX count as "sent, no usable
    // downlink" (record_uplink_result() / downlink_unusable()). RadioLib fills
    // eventUp after its TX loop on these paths too, so the uplink line is valid.
    const bool rx_only = sent && downlink_unusable(st);
    if (st >= RADIOLIB_ERR_NONE || rx_only) {
        log_uplink_ok(what, ev_up.fCnt, &ev_up, st, air_ms);
        adr_track(&ev_up, rx_only ? 0 : st);
    }
    if (sent) {
        // Also (especially) after a failed exchange: which window saw what is
        // the first question for "uplink failed: <code>".
        // Static, not on the worker's stack (~600 B with NbTrans 3 and frames);
        // this task is the only user.
        static char rx[LW_RXOBS_LINE_MAX];
        rxobs_format(&s_obs, rx, sizeof(rx), st);
        ESP_LOGI(TAG, "rx windows: %s", rx);
    }
    // Whatever RadioLib handed the store callbacks during this exchange
    // (nothing for a frame refused before TX); lw8_flush() keeps the
    // s_forget_pending gate.
    lw8_flush();
    exchange_end();
    // A downlink may have moved the DR (LinkADRReq; with ADR off RadioLib
    // skips the backoff, so that is the only way): undo that in fixed-DR
    // mode, then let the next cycle's interval follow the DR the next
    // uplink will actually use.
    apply_fixed_dr(node);
    set_cycle_sf_noted(sf_of_dr(s_band, node.uplinkDr()), false);
    // Callers treat an rx-only error like "no downlink": the uplink went out,
    // so e.g. the port-2 frame of the same cycle must still be sent.
    return rx_only ? 0 : st;
}

// Poll for the DeviceTimeAns right after the exchange that carried the
// request — the cycle's first frame (port 1, or port 2 on an environment-only
// node), not at end of cycle: the answer's time field is captured at the END
// of the uplink that carried the request and TTN serves it in that uplink's
// RX1, so polling here (a) avoids adding a second exchange's ~3-8 s to the
// apply latency and (b) can't lose the stored answer to the next uplink
// clearing fOptsDown (RadioLib clears it before every uplink, LoRaWAN.cpp
// sendReceive()). The returned time is advanced to "now" by RadioLib itself
// (see timesync_apply(); up to one repeat period early with NbTrans > 1) — this
// feature still only rescues standalone nodes from 1970-era timestamps; it
// is not NTP. Clears *want, so a second call in the same cycle is a no-op.
// If the frame never went on air (duty-cycle skip, dwell refusal), the
// attempt is not counted against TIMESYNC_MAX_ATTEMPTS; the request stays
// queued in RadioLib for the next uplink (not after a dwell refusal of a
// MAC-only frame, whose queue RadioLib has already consumed — the next
// cycle simply queues it again).
static void poll_device_time(GeigerNode &node, bool *want) {
    if (!*want) return;
    *want = false;
    if (!s_last_exchange_sent) {
        if (s_timesync_attempts > 0) s_timesync_attempts--;
        return;
    }
    uint32_t unix_s = 0;
    uint16_t ms     = 0;
    const bool fresh = node.getMacDeviceTimeAns(&unix_s, &ms, true) == RADIOLIB_ERR_NONE && unix_s != 0;
    if (fresh) {
        timesync_apply(unix_s, ms);
        s_timesync_last_s   = timesync_now_s();
        s_timesync_attempts = 0;
    } else if (s_timesync_attempts >= TIMESYNC_MAX_ATTEMPTS) {
        timesync_give_up();
    }
}

// Send one application frame, then poll for the DeviceTimeAns.
//
// Small-DR stall guard: RadioLib refuses a frame before TX with
// RADIOLIB_ERR_MAC_ANSWERS_PENDING (-1124) when the payload fits the DR but
// not together with the queued MAC answers (isValidUplink() ->
// testPayloadLength(), dwell-time aware). At AU915/AS923 DR2 (400 ms dwell:
// 11 B) or US915 DR0 (11 B) a 10 B port-1 frame plus a 2 B LinkADRAns or 3 B
// DevStatusAns no longer fits, and the answers are only cleared by a TX.
// Here: send the answers alone (port 0, RadioLib moves them into
// FRMPayload), poll for a DeviceTimeAns right after that frame (the retry
// would clear it), then send the frame once more. One retry only: if the
// port-0 exchange's downlink queued new answers that don't fit either,
// they go out alone again next cycle.
static int16_t send_app_frame(GeigerNode &node, const uint8_t *data, size_t len, uint8_t port,
                              const char *what, uint8_t *dl, size_t *dl_len, bool *want_time) {
    int16_t st = uplink_exchange(node, data, len, port, what, dl, dl_len);
    if (st == RADIOLIB_ERR_MAC_ANSWERS_PENDING) {
        ESP_LOGW(TAG, "port %u frame (%u B) and %u B of queued MAC answers do not fit together at DR%u — "
                      "sending the MAC answers alone first", (unsigned)port, (unsigned)len,
                 (unsigned)node.getMacUplinkLen(), (unsigned)node.uplinkDr());
        st = uplink_exchange(node, NULL, 0, 0, "MAC answers only (port 0)", dl, dl_len);
        poll_device_time(node, want_time);
        if (st >= RADIOLIB_ERR_NONE) st = uplink_exchange(node, data, len, port, what, dl, dl_len);
    }
    poll_device_time(node, want_time);   // no-op if already polled
    return st;
}

// --- Duty-cycle hold note (V2.8.21) -------------------------------------------
//
// main.c logs "next LoRaWAN uplink" when it QUEUES a round — before that
// round's exchanges run — so its line can only show the schedule. The
// duty-cycle wait those exchanges leave behind is known here, after the
// round's last one: if it holds the next uplink past the next round, the same
// line follows with that later time and the reason.

// esp_timer ms at which the next round starts (lorawan_note_next_round());
// 0 = not known yet. Under s_status_mux (64-bit: no single atomic store).
static uint64_t s_next_round_ms = 0;

void lorawan_note_next_round(uint32_t in_ms) {
    const uint64_t at = (uint64_t)(esp_timer_get_time() / 1000) + in_ms;
    portENTER_CRITICAL(&s_status_mux);
    s_next_round_ms = at;
    portEXIT_CRITICAL(&s_status_mux);
}

/** @brief After a round's last exchange: log "next LoRaWAN uplink at about …
 *         (in N min: duty-cycle wait; upload round in M min, …)" when RadioLib's
 *         limiter will hold the next uplink past the next round. Silent when
 *         the round comes first (main.c's line already said when) or the next
 *         round's time is not known. Worker task only. */
static void log_duty_hold(GeigerNode &node) {
    const RadioLibTime_t wait_ms = node.timeUntilUplink();   // 0 = no wait
    if (wait_ms == 0) return;
    portENTER_CRITICAL(&s_status_mux);
    const uint64_t round_ms = s_next_round_ms;
    portEXIT_CRITICAL(&s_status_mux);
    const uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000);
    if (round_ms <= now_ms) return;   // not known, or the round is already due
    const uint32_t in_s   = (uint32_t)((round_ms - now_ms) / 1000u);
    const uint32_t duty_s = (uint32_t)(wait_ms / 1000u);
    if (duty_s <= in_s) return;       // the round comes first
    int hh = -1, mm = 0, ss = 0;      // < 0: clock not set, no time of day
    if (ntp_time_valid()) {
        const time_t at = time(NULL) + (time_t)duty_s;
        struct tm lt;
        localtime_r(&at, &lt);
        hh = lt.tm_hour; mm = lt.tm_min; ss = lt.tm_sec;
    }
    char line[128];   // lw_next_uplink_line()'s worst case is ~115 chars
    lw_next_uplink_line(line, sizeof(line), in_s, hh, mm, ss, __atomic_load_n(&s_cycle_sf, __ATOMIC_RELAXED),
                        duty_s);
    ESP_LOGI(TAG, "%s", line);
}

// Worker-task-private frame-plan state (V2.8.14): when the last port-2 frame
// went on air (for the hourly cadence next to port 1), and a one-shot flag
// for the "nothing to send" warning.
static bool     s_p2_sent_once   = false;
static uint32_t s_p2_last_s      = 0;
static bool     s_nothing_logged = false;

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
    static GeigerNode node(&s_radio, k_bands[region], subband);
    node.beginOTAA(creds.join_eui, creds.dev_eui, NULL, creds.app_key); // NULL nwkKey = LoRaWAN 1.0.x
    // V2.8.14 duty-cycle backstop (see uplink_exchange()): none on
    // US915/AU915/CN470/KR920/IN865. The enable flag is RAM-only in RadioLib,
    // so set on every boot. The LIMIT set here does not stick: the pre-join
    // session and every restore put RadioLib's 1/128 seed back (EU868 joins
    // run under it); after activation applyBandDutyCycle() restores the
    // band's 1 % unless the network set a limit.
    node.setDutyCycle(true);

    // Restore persisted state through RadioLib's buffer callbacks (see
    // lw8_store_pers()): loadBuffers() reads the persistence buffer (DevNonce /
    // JoinNonce, join replay protection) first, then the session (rejoin-free
    // resume). beginOTAA() above must come first: it clears both and computes
    // the key checksum the persistence buffer is checked against. A rejected
    // restore is self-healing: log, fall through to a clean join. A node with
    // no persistence blob (new board) starts at DevNonce 0, as RadioLib does.
    node.setCallbackStorePersistence(lw8_store_pers);
    node.setCallbackRestorePersistence(lw8_restore_pers);
    node.setCallbackStoreSession(lw8_store_sess);
    node.setCallbackRestoreSession(lw8_restore_sess);
    // True while loadBuffers() has staged a session that the first
    // activateOTAA() will activate without a join (see the join loop).
    bool session_pending = false;
    {
        const int16_t st = node.loadBuffers();
        session_pending = (st == RADIOLIB_ERR_NONE);
        switch (lw_restore_verdict(st, s_lw8_pers_found, s_lw8_sess_called, s_lw8_sess_found,
                                   s_lw8_sess_unfinished)) {
        case LW_RESTORE_OK:
            break;
        case LW_RESTORE_NO_PERS:
            ESP_LOGI(TAG, "no saved DevNonce history (new board, first start after the update to V2.8.21, or "
                          "after a full LoRaWAN reset) — joining, DevNonce starts at 0");
            break;
        case LW_RESTORE_PERS_BAD:
            ESP_LOGW(TAG, "saved DevNonce history not usable: %d (%s) — joining, DevNonce restarts at 0 "
                          "(reset the used DevNonces in TTN)", (int)st, lw_err_text(st));
            break;
        case LW_RESTORE_NO_SESS:
            ESP_LOGI(TAG, "no saved LoRaWAN session — joining");
            break;
        case LW_RESTORE_SESS_BAD:
            ESP_LOGW(TAG, "saved LoRaWAN session not usable: %d (%s) — joining", (int)st, lw_err_text(st));
            break;
        case LW_RESTORE_SESS_UNFINISHED:   // see lw8_restore_sess()
            ESP_LOGW(TAG, "saved LoRaWAN session is from before its first uplink (restart right after the "
                          "join) — joining again");
            break;
        }
    }

    // Join with exponential backoff: 1,2,4,... min, about an hour at most (spec §4;
    // the doubling stops once 3600 s is passed, so the longest wait is 3840 s).
    // (Loop starts at 60 s — the "1,2,4..." is in minutes: 60 s, 120 s, ...)
    //
    // V2.8.12 join data-rate step-down (dynamic band plans only: EU868,
    // AS923, KR920, IN865, EU433). RadioLib alone always joins at the first
    // channel's default DR (EU868: DR3 = SF9) and never steps down, so a node
    // with a weak link retried at SF9 forever (XIAO 2026-10-18: no indoor SF9
    // join reached the gateway). Now: first attempt at that default, then one
    // DR lower per failed attempt down to the floor, then stay there — a good
    // link still joins fast on little air time, a weak one ends at SF12.
    // Floor DR0, or DR2 on dwell-time-limited plans (AS923: 400 ms dwell, an
    // SF11/SF12 join request exceeds it). Fixed plans (US915, AU915, CN470)
    // take the join DR from the channel, so no DR is set for them.
    // activateOTAA() takes no initial DR: the join DR is set with
    // setDatarate() right before each attempt (setDatarate() creates the
    // pre-join session itself). Skipped while a restored session is
    // pending: activateOTAA() then just activates it, without a TX, and its
    // data rate must stay the one the network set.
    //
    // The join DR also becomes the session's DATA DR (createSession applies it
    // as a synthetic LinkADRReq; the JoinAccept doesn't change it), so a node
    // that joined at DR0 sends at SF12 until the network's ADR raises the DR.
    // V2.8.12 accepted that with no air-time limit (known gap); since V2.8.14
    // the uplink interval follows the SF (lorawan_sched.h) and RadioLib's
    // duty-cycle limiter is on as a backstop (uplink_exchange()).
    //
    // V2.8.14 fixed DR (/config lorawan_dr): dynamic plans join at that DR
    // (not below the dwell floor), with no step-down — the user chose the
    // link's DR; apply_fixed_dr() then pins it for the data frames.
    set_state(LORAWAN_ST_JOINING);
    uint32_t backoff_s = 60;
    const LoRaWANBand_t *band = k_bands[region];
    const bool dynamic_band = (band->bandType == RADIOLIB_LORAWAN_BAND_DYNAMIC);
    uint8_t join_dr = band->txFreqs[0].dr;
    uint8_t join_dr_floor = band->dwellTimeUp ? 2 : 0;
    if (join_dr_floor > join_dr) join_dr_floor = join_dr;
    if (dynamic_band && __atomic_load_n(&s_fixed_dr, __ATOMIC_RELAXED)) {
        join_dr = (s_fixed_dr_value < join_dr_floor) ? join_dr_floor : s_fixed_dr_value;
        join_dr_floor = join_dr;   // no step-down below the chosen DR
    }
    const bool join_dr_steps = dynamic_band;   // a DR is set before each attempt
    for (;;) {
        // Duty-cycle backstop for join requests too: a retry 60 s after an
        // SF12 request (~1.5 s on air) would otherwise be refused by RadioLib
        // and wasted as a "failed" attempt that never went on air.
        lw_clock_wrap_note(node);
        const RadioLibTime_t jwait_ms = node.timeUntilUplink();
        if (jwait_ms > 0) vTaskDelay(pdMS_TO_TICKS(jwait_ms) + 1);
        if (join_dr_steps && !session_pending) set_cycle_sf(sf_of_dr(band, join_dr));
        if (!exchange_begin()) park_for_ota();
        // The observer tells join accepts from data frames by own_addr == 0
        // (RadioLib: sessionStatus ACTIVATING); reset it per attempt so a
        // rejoin later in the same boot reads its accept right.
        s_obs.own_addr = 0;
        rxobs_begin(&s_obs, 1);   // a join request is sent once per activateOTAA()
        if (join_dr_steps && !session_pending) {
            const int16_t dst = node.setDatarate(join_dr);
            if (dst != RADIOLIB_ERR_NONE) {
                ESP_LOGW(TAG, "join DR%u not set: %d (%s)", (unsigned)join_dr, (int)dst, lw_err_text(dst));
            }
        }
        int16_t st = node.activateOTAA();
        session_pending = false;   // a pending session is consumed by the first activateOTAA()
        s_obs.on = false;
        const bool sent = (s_obs.tx_us != 0);   // false for a restored session (no TX)
        // RadioLib resets lastToA for every join request, so this is this
        // request's air time and the duty-cycle limiter sees only it.
        const uint32_t join_air_ms = sent ? (uint32_t)node.getLastToA() : 0;
        // The DevNonce RadioLib stored before the request (persistence store
        // callback). lw8_flush() keeps the s_forget_pending gate: a join attempt
        // completing between an operator's "wipe DevNonce history" and the
        // deferred restart must not write the nonces back.
        lw8_flush();
        exchange_end();   // after the NVS write — radio is in standby by now
        portENTER_CRITICAL(&s_status_mux); s_status.join_attempts++; portEXIT_CRITICAL(&s_status_mux);

        // V2.8.12 per-attempt line: what went on air and what each RX window
        // saw, e.g. "join attempt 3 (DR1): SF11 868.300 MHz air=741 ms ->
        // no accept (-1116) | RX1 ... timeout, no preamble | RX2 ...".
        uint32_t attempts;
        portENTER_CRITICAL(&s_status_mux); attempts = s_status.join_attempts; portEXIT_CRITICAL(&s_status_mux);
        int accept_win = 0;   // RX window (1/2) that delivered the accept, 0 = none
        if (sent) {
            const uint32_t air_24h = airtime_add(join_air_ms);
            for (int i = 0; i < s_obs.n_win; i++) {
                if (rxobs_rank(rxobs_final(&s_obs.win[i])) == 6 &&   // clean RX_DONE ...
                    !s_obs.win[i].ignored) {                          // ... handed to RadioLib
                    accept_win = i + 1;
                }
            }
            static char rx[LW_RXOBS_LINE_MAX];   // worker task only, like the uplink line's
            rxobs_format(&s_obs, rx, sizeof(rx), 0);   // one TX: the single-TX form
            char drs[12] = "";   // " (DR255)" worst case for a uint8_t
            if (join_dr_steps) snprintf(drs, sizeof(drs), " (DR%u)", (unsigned)join_dr);
            char res[96];
            if (st == RADIOLIB_LORAWAN_NEW_SESSION) snprintf(res, sizeof(res), "accepted");
            else snprintf(res, sizeof(res), "no accept: %d (%s)", (int)st, lw_err_text(st));
            ESP_LOGI(TAG, "join attempt %lu%s: SF%u %.3f MHz air=%lu ms -> %s | %s",
                     (unsigned long)attempts, drs, (unsigned)s_obs.tx_sf, rxobs_mhz(s_obs.tx_freq_raw),
                     (unsigned long)join_air_ms, res, rx);
            portENTER_CRITICAL(&s_status_mux); s_status.airtime_24h_ms = air_24h; portEXIT_CRITICAL(&s_status_mux);
        }

        if (st == RADIOLIB_LORAWAN_NEW_SESSION || st == RADIOLIB_LORAWAN_SESSION_RESTORED) {
            uint32_t dev_addr = node.getDevAddr();   // RadioLib getter — before the lock
            char via[20] = "";   // " via RX%d" sized for any int, keeps -Wformat-truncation quiet
            if (accept_win) snprintf(via, sizeof(via), " via RX%d", accept_win);
            ESP_LOGI(TAG, "%s%s — DevAddr %08lx, region %s sub-band %u",
                     st == RADIOLIB_LORAWAN_SESSION_RESTORED ? "session resumed" : "joined (OTAA)",
                     via, (unsigned long)dev_addr, lorawan_region_name(region), (unsigned)subband);
            const uint32_t ok_khz = (uint32_t)(rxobs_mhz(s_obs.tx_freq_raw) * 1000.0 + 0.5);
            portENTER_CRITICAL(&s_status_mux);
            s_status.dev_addr = dev_addr;
            s_obs.own_addr = dev_addr;   // worker task, like the observer itself
            // The "boot: LoRaWAN:" banner line (syslog.c) reads these.
            s_status.boot_resumed = (st == RADIOLIB_LORAWAN_SESSION_RESTORED);
            if (sent) {
                s_status.join_ok_dr     = join_dr_steps ? join_dr : 0xFF;
                s_status.join_ok_sf     = s_obs.tx_sf;
                s_status.join_ok_window = (uint8_t)accept_win;
                s_status.join_ok_khz    = ok_khz;
            }
            portEXIT_CRITICAL(&s_status_mux);
            apply_fixed_dr(node);   // ADR off + the fixed DR (no-op in ADR mode)
            lw_erase_v2820_session();   // the old session is dead from here on
            {
                bool by_network = false;
                const uint32_t dc_ms = node.applyBandDutyCycle(&by_network);
                if (dc_ms == 0) {
                    ESP_LOGI(TAG, "duty-cycle limit: none in %s", lorawan_region_name(region));
                } else {
                    ESP_LOGI(TAG, "duty-cycle limit: %.1f s of air time per hour (%.2f %%, %s)", dc_ms / 1000.0,
                             dc_ms / 36000.0, by_network ? "set by the network" : "band default");
                }
            }
            set_cycle_sf_noted(sf_of_dr(band, node.uplinkDr()), true);
            set_state(LORAWAN_ST_JOINED);
            break;
        }
        portENTER_CRITICAL(&s_status_mux);
        s_status.last_error = st;
        s_status.join_last_error = st;   // kept for the boot line; uplinks overwrite last_error
        portEXIT_CRITICAL(&s_status_mux);
        // Step the join DR down only after an attempt that actually went on air
        // (a duty-cycle-blocked attempt says nothing about the link).
        const bool step = join_dr_steps && sent && join_dr > join_dr_floor;
        if (step) join_dr--;
        ESP_LOGW(TAG, "join failed: %d (%s) — retry in %lu s%s", (int)st, lw_err_text(st), (unsigned long)backoff_s,
                 step ? " (next attempt one DR lower)" : "");
        vTaskDelay(pdMS_TO_TICKS(backoff_s * 1000));
        if (backoff_s < 3600) backoff_s *= 2;
    }

    // Uplink loop: one queue slot; freshest-wins (the producer overwrites
    // via xQueueOverwrite in lorawan_transmit(), so this pulls the latest
    // window each time).
    lorawan_snapshot_t snap;
    for (;;) {
        if (xQueueReceive(s_q, &snap, portMAX_DELAY) != pdTRUE) continue;
        // (V2.8.12: s_busy is now raised per radio exchange inside
        // uplink_exchange(), together with the OTA-suspension check.)

        // V2.8.14 frame plan (lorawan_sched.h): port 1 only with a tube;
        // port 2 every cycle on an environment-only node, at most hourly
        // next to port 1. Built first, so a cycle with nothing to send
        // queues no MAC request either.
        const bool send_p1 = snap.tube_valid;
        uint8_t p2[5];
        const bool have_p2 = snap.env_valid &&
                             lw_build_port2(p2, snap.temperature_c, snap.humidity_pct, snap.pressure_pa);
        const bool send_p2 = have_p2 && lw_sched_port2_due(send_p1, s_p2_sent_once, lw_uptime_s(),
                                                           s_p2_last_s);
        if (!send_p1 && !send_p2) {
            if (!s_nothing_logged) {
                ESP_LOGW(TAG, "nothing to send: no Geiger tube enabled and no environment reading "
                              "(logged once until a frame goes out again)");
                s_nothing_logged = true;
            }
            continue;
        }
        s_nothing_logged = false;

        // DeviceTimeReq piggyback — see the timesync block above for the
        // full cadence/authority rationale. Queued BEFORE the cycle's first
        // frame is built so it rides in that uplink's FOpts; duplicate
        // requests are discarded by RadioLib, so re-arming every cycle until
        // an answer lands is safe.
        bool want_time = (ntp_boot_epoch() == 0) &&
                         (s_timesync_last_s == 0 ||
                          (timesync_now_s() - s_timesync_last_s) >= TIMESYNC_INTERVAL_S);
        // V2.8.14: not while the request (1 B) would push the cycle's first
        // frame plus the queued MAC answers over the DR's limit (AU915/AS923
        // DR2, US915 DR0: 11 B) — the answers go first, the request rides a
        // later cycle; no attempt is counted (see send_app_frame()). The +1
        // is conservative after a skipped exchange: a request still queued
        // from it is already in getMacUplinkLen() (at worst one answer lost).
        if (want_time && !node.frameFits(send_p1 ? 10 : 5, (size_t)node.getMacUplinkLen() + 1)) {
            want_time = false;
        }
        if (want_time) {
            int16_t tst = node.sendMacCommandReq(RADIOLIB_LORAWAN_MAC_DEVICE_TIME);
            s_timesync_attempts++;   // queue failures count too — see timesync_give_up()
            if (tst != RADIOLIB_ERR_NONE) {
                ESP_LOGW(TAG, "DeviceTimeReq not queued: %d (%s)", (int)tst, lw_err_text(tst));
                want_time = false;   // nothing went on air — don't poll for an answer
                if (s_timesync_attempts >= TIMESYNC_MAX_ATTEMPTS) timesync_give_up();
            }
        }

        // Downlink scratch. MUST be sized for the largest frame RadioLib can
        // deliver: sendReceive()'s dataDown parameter has NO size argument and
        // RadioLib writes the decrypted payload straight in (no clamp to the
        // caller's buffer), setting *dl_len afterward. A network-scheduled
        // application downlink can be up to RADIOLIB_LORAWAN_MAX_PAYLOAD_SIZE
        // (242 B) at higher data rates; a smaller buffer here would let a (MIC-valid)
        // oversized downlink smash this task's stack. Sized like RadioLib's
        // own sendReceive() wrappers (+1). ~243 B on the 8 KB worker stack is fine.
        uint8_t dl[RADIOLIB_LORAWAN_MAX_PAYLOAD_SIZE + 1]; size_t dl_len = 0;
        char what[64];
        int16_t st = RADIOLIB_ERR_NONE;
        if (send_p1) {
            uint8_t p1[10];
            lw_build_port1(p1, snap.gm_counts, snap.dt_ms, s_version_packed, snap.tube_nbr);
            snprintf(what, sizeof(what), "port 1 (counts=%lu dt=%lu ms)",
                     (unsigned long)snap.gm_counts, (unsigned long)snap.dt_ms);
            st = send_app_frame(node, p1, sizeof(p1), 1, what, dl, &dl_len, &want_time);
        }

        // Port 2 after a port-1 hard error is skipped (radio/HAL trouble, or
        // the duty-cycle backstop refused the cycle). An rx-only error is no
        // hard error — uplink_exchange() returns 0 for it.
        if (send_p2 && st >= RADIOLIB_ERR_NONE) {
            // Log the values DECODED BACK from the wire bytes, not the
            // raw snapshot floats: the V1.9-frozen codec TRUNCATES
            // (temp*10, hum*2, Pa/10 — all toward zero) while a
            // printf %.1f of the raw float ROUNDS, so e.g. T=16.89
            // went on air as 16.8 but logged as 16.9 (user-caught
            // 2026-07-27 comparing /log against TTN's decoded
            // payload). Decoding p2 makes this line match TTN's
            // formatter output digit-for-digit, always.
            int16_t  traw = (int16_t)(((uint16_t)p2[0] << 8) | p2[1]);
            uint16_t praw = (uint16_t)(((uint16_t)p2[3] << 8) | p2[4]);
            snprintf(what, sizeof(what), "port 2 (T=%.1f°C H=%.1f%% P=%.1f hPa)",
                     (double)traw / 10.0, (double)p2[2] / 2.0, (double)praw / 10.0);
            // Result fully handled inside (counters, logs, session save).
            if (send_app_frame(node, p2, sizeof(p2), 2, what, dl, &dl_len, &want_time) >= RADIOLIB_ERR_NONE) {
                s_p2_sent_once = true;
                s_p2_last_s    = lw_uptime_s();
            }
        }
        if (send_p1 || send_p2) log_duty_hold(node);   // after the round's LAST exchange
        // Session persistence: since V2.8.12 saved inside uplink_exchange()
        // after EVERY exchange (A4), behind the same s_forget_pending gate —
        // suppressed once lorawan_forget_session() has run, because the
        // imminent-restart window can exceed the documented ~2 s on WiFi+LoRa
        // nodes (the restart defers on the WiFi TX worker's idle check, i.e.
        // up to a full TLS upload cycle, since V2.8.15 plus the round's wait
        // for a LoRaWAN exchange), and an uplink completing in that window
        // would silently re-save the just-erased session.
        //
        // V2.8.15: if exchange_begin() waited longer than one cycle for the
        // WiFi round, the snapshot sent was the older one and the newer one
        // is already queued — it follows as the next exchange (each frame
        // carries its own dt, so the data stays right; on EU868 the duty
        // cycle may delay or skip it).
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

    // V2.8.14 schedule, set before the worker exists (main task, at boot, so
    // no /config POST can race these reads — same rule as lorawan_enabled).
    // Region clamped like lorawan_task() does (silently here; it logs).
    uint8_t region = g_cfg.lorawan_region;
    if (region >= (sizeof(k_bands) / sizeof(k_bands[0]))) region = 2;
    s_band = k_bands[region];
    s_fixed_dr = lw_sched_fixed_dr(g_cfg.lorawan_dr, &s_fixed_dr_value);
    // First interval: the join DR — the fixed DR on a dynamic plan, else the
    // plan's default (dynamic: first channel's DR; fixed plans: the join span's).
    const bool dynamic_band = (s_band->bandType == RADIOLIB_LORAWAN_BAND_DYNAMIC);
    const uint8_t join_dr = (dynamic_band && s_fixed_dr) ? s_fixed_dr_value
                          : dynamic_band ? s_band->txFreqs[0].dr
                          : s_band->txSpans[0].drJoinRequest;
    set_cycle_sf(sf_of_dr(s_band, join_dr));

    s_q = xQueueCreate(1, sizeof(lorawan_snapshot_t));
    configASSERT(s_q);

    // Same worker-task convention as transmission.c's tx_setup(): pinned to
    // CPU1 so a multi-second join/uplink RadioLib call (blocking SPI polls +
    // RX-window waits) never starves CPU0's idle task / task watchdog.
    // Both LoRaWAN boards (Heltec V4, XIAO + Wio) are ESP32-S3 (dual-core) —
    // always the pinned branch here, unlike tx_setup()'s single-core carve-out.
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
    return !__atomic_load_n(&s_busy, __ATOMIC_SEQ_CST) &&
           (s_q == NULL || uxQueueMessagesWaiting(s_q) == 0);
}

bool lorawan_on_air(void) { return __atomic_load_n(&s_busy, __ATOMIC_SEQ_CST); }

// OTA prep (httpd task). Publishes the suspension, then waits for any
// exchange already on air — join attempt or uplink incl. its RX windows — to
// finish. Deliberately NOT lorawan_is_idle(): a queued snapshot is harmless
// (the worker discards it once suspended), and during a join backoff the
// queue legitimately stays full for minutes. See the OTA-suspension block
// at the top of this file for the handshake and the 2026-10-18 crash.
bool lorawan_suspend(uint32_t timeout_ms) {
    __atomic_store_n(&s_suspended, true, __ATOMIC_SEQ_CST);
    const int64_t t0 = esp_timer_get_time();
    const int64_t deadline = t0 + (int64_t)timeout_ms * 1000;
    while (__atomic_load_n(&s_busy, __ATOMIC_SEQ_CST)) {
        if (esp_timer_get_time() >= deadline) return false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // Say so when the OTA actually had to wait: without this line, the A3 bench
    // test (2026-10-18 21:13) could only be read from timing — flash start 77 ms
    // after the exchange's "uplink OK", with the WiFi TX worker already idle.
    const int64_t waited_ms = (esp_timer_get_time() - t0) / 1000;
    if (waited_ms >= 100) {
        ESP_LOGI(TAG, "OTA: waited %.1f s for the running LoRaWAN exchange — radio quiet now",
                 (double)waited_ms / 1000.0);
    }
    return true;
}

#endif // CONFIG_GEIGER_LORAWAN
