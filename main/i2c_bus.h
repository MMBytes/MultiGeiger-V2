#pragma once

/** @file
 *  @brief V2.3.29: centralised I²C bus lifecycle owner.
 *
 *  Replaces the previous arrangement where env_sensor.c owned the primary
 *  bus and display.c owned the secondary bus. Both responsibilities now
 *  live here. Sensor and display modules become pure consumers — they
 *  receive bus handles from main.c via the accessors below.
 *
 *  Per-board behaviour:
 *    Heltec V2 (+ 4 MB clone): primary bus on PIN_I2C_SDA / PIN_I2C_SCL
 *      (GPIO 4 / 15) with the Vext active-LOW gate driven LOW to power
 *      the OLED + I²C pull-up rail (older modules ignore the drive,
 *      newer modules require it). Secondary bus: not available — no
 *      STEMMA2 hardware on these boards.
 *
 *    FeatherS3-D: primary bus on STEMMA1 (IO8 / IO9) — always-on rail
 *      from LDO1. Secondary bus on STEMMA2 (IO16 SDA / IO15 SCL),
 *      LDO2-gated by IO39 (HIGH = on). Secondary is LAZY: LDO2 stays
 *      off at boot; first call to i2c_bus_get_secondary() drives IO39
 *      HIGH and creates the I²C controller. After all module init,
 *      main.c calls i2c_bus_finalize() — if no consumer marked the
 *      secondary bus as in-use, it gets torn down (LDO2 dropped, ~5–10 mA
 *      saved).
 *
 *    QT Py ESP32-PICO: primary bus on STEMMA QT (IO22 SDA / IO19 SCL).
 *      Secondary bus: not available.
 *
 *    Heltec WiFi LoRa 32 V4-R2: primary bus on the external env-sensor
 *      header (IO48 SDA / IO47 SCL). Secondary bus on the module's fixed
 *      internal OLED bus (IO17 SDA / IO18 SCL) — Vext_Ctrl-gated (GPIO36,
 *      active-LOW MOSFET): V2.6.20 bench finding, the datasheet reading
 *      that Vext only gates the external Ve header was wrong.
 *      i2c_bus_get_secondary() drives the gate LOW before creating the
 *      controller; the rail is never dropped afterwards. Unlike every
 *      other board's secondary bus, this one can only ever host the
 *      onboard OLED — it isn't wired to anything else, so there's no
 *      "no consumer, shed it" case to handle.
 *
 *  Lazy + sheddable secondary lets the multi-page display task and any
 *  future STEMMA2-attached sensor opt into bus 2 without forcing it
 *  always-on for deployments that only use STEMMA1. (Heltec V4-R2 is the
 *  exception — see above.)
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

/** @brief V2.5.19: select the primary-bus pin route BEFORE the first
 *  i2c_bus_get_primary() call. On a board with HAL_HAS_I2C_PINOUT_SWITCH==1
 *  (QT Py ESP32-PICO), `true` routes the primary master bus to the broken-out
 *  SDA/SCL pads (PIN_I2C_*_ALT) instead of the onboard/STEMMA pins. No-op on
 *  boards without the switch — they always use PIN_I2C_SDA / PIN_I2C_SCL.
 *
 *  Must be called before the first i2c_bus_get_primary() (the bus is created
 *  lazily and cached on first use, so a later change won't take effect until
 *  reboot). main.c calls this right after config_load().
 */
void i2c_bus_set_primary_pinout(bool use_pinout);

/** @brief Get the primary I²C bus handle. Lazy init on first call.
 *  Always returns a valid handle on a properly-configured board (any
 *  failure logs an error and returns NULL — only seen if I2C peripheral
 *  init itself fails, which is fatal).
 *
 *  On Heltec, this also drives Vext LOW and waits for the rail to
 *  settle before creating the bus. Subsequent calls return the cached
 *  handle without re-initialising.
 */
i2c_master_bus_handle_t i2c_bus_get_primary(void);

/** @brief Get the secondary I²C bus handle (FeatherS3-D STEMMA2, or the
 *  Heltec WiFi LoRa 32 V4-R2's dedicated OLED bus).
 *
 *  On FeatherS3-D: lazy init on first call — drives IO39 HIGH (LDO2
 *  enable), waits 10 ms for the rail to settle, then creates the
 *  I²C controller on I²C_NUM_1 / IO15 / IO16. Subsequent calls return
 *  the cached handle.
 *
 *  On Heltec WiFi LoRa 32 V4-R2: lazy init on first call — drives
 *  Vext_Ctrl (GPIO36) LOW to power the OLED rail (V2.6.20 — this bus is
 *  NOT always-on, contrary to the original datasheet reading), waits
 *  50 ms for the rail to settle, then creates the I²C controller on
 *  I²C_NUM_1 / IO17 / IO18. Always non-NULL once created; never torn
 *  down (see i2c_bus_finalize()'s log-only branch for this board).
 *
 *  On Heltec V2 / QT Py / any board without a second bus: always returns
 *  NULL. Callers should treat NULL as "no second bus on this board"
 *  and skip secondary-bus probing gracefully.
 *
 *  Calling this on FeatherS3-D enables LDO2 — adds ~5–10 mA continuous draw
 *  (LDO2 quiescent + onboard NeoPixel idle). If no consumer subsequently
 *  calls i2c_bus_secondary_keep_alive(), i2c_bus_finalize() will tear it
 *  back down. On Heltec WiFi LoRa 32 V4-R2 the Vext rail is deliberately
 *  left ON even without a consumer — the bus is dedicated to the onboard
 *  OLED, so a missing consumer means a failed probe (anomalous), not a
 *  power-save opportunity.
 */
i2c_master_bus_handle_t i2c_bus_get_secondary(void);

/** @brief Mark the secondary bus as in-use. Called by any module that
 *  successfully bound a device on the secondary bus. Prevents
 *  i2c_bus_finalize() from disabling the bus.
 *
 *  Idempotent — safe to call multiple times. No-op on boards without a
 *  secondary bus.
 */
void i2c_bus_secondary_keep_alive(void);

/** @brief End-of-init cleanup. If the secondary bus was lazily enabled
 *  but no consumer marked it as in-use, tears the bus controller down
 *  and drops LDO2 (IO39 LOW). Otherwise no-op.
 *
 *  Call once from main.c after ALL sensor and display init has
 *  completed. Safe to call on any board (no-op when there's no
 *  secondary bus).
 */
void i2c_bus_finalize(void);

// --- Per-device helpers -------------------------------------------------
//
// Every driver in main/ was independently reimplementing the same handful
// of register-protocol primitives (write_reg/read_regs, 16-bit LE/BE reads)
// and the same probe→add_device→teardown-on-failure ceremony. Centralised
// here (V2.6.6) so the wire-level details — byte order, 100 ms transaction
// timeout, the exact i2c_device_config_t shape — are written once, since
// VEML7700 (LE) and MAX17048 (BE) sit right next to each other in this
// codebase and getting byte order backwards silently scales every reading
// by a factor of 256 — exactly the kind of bug this consolidation is meant
// to make harder to introduce in the first place.
//
// These are `static inline` so each translation unit gets its own copy with
// no ODR concerns — same pattern as the rest of this project's shared
// headers (e.g. util.h).

/** @brief Write one 8-bit register: [reg, val]. 100 ms transaction timeout. */
static inline esp_err_t i2c_dev_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

/** @brief Read n bytes starting at an 8-bit register address (write-then-read,
 *  single transaction). 100 ms transaction timeout. */
static inline esp_err_t i2c_dev_read_regs(i2c_master_dev_handle_t dev, uint8_t reg,
                                            uint8_t *buf, size_t n) {
    return i2c_master_transmit_receive(dev, &reg, 1, buf, n, 100);
}

/** @brief Read a 16-bit register that is big-endian on the wire (MSB first —
 *  e.g. MAX17048). 100 ms transaction timeout. */
static inline esp_err_t i2c_dev_read_u16_be(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t *out) {
    uint8_t in[2];
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, in, sizeof(in), 100);
    if (err != ESP_OK) return err;
    *out = ((uint16_t)in[0] << 8) | (uint16_t)in[1];
    return ESP_OK;
}

/** @brief Read a 16-bit register that is little-endian on the wire (LSB first
 *  — e.g. VEML7700). 100 ms transaction timeout. */
static inline esp_err_t i2c_dev_read_u16_le(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t *out) {
    uint8_t in[2];
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, in, sizeof(in), 100);
    if (err != ESP_OK) return err;
    *out = (uint16_t)in[0] | ((uint16_t)in[1] << 8);
    return ESP_OK;
}

/** @brief Write a 16-bit register that is little-endian on the wire: [reg,
 *  val_lo, val_hi] — e.g. VEML7700. 100 ms transaction timeout. */
static inline esp_err_t i2c_dev_write_u16_le(i2c_master_dev_handle_t dev, uint8_t reg, uint16_t val) {
    uint8_t buf[3] = { reg, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

/** @brief Bind a device handle at a known-present address: fixed 7-bit
 *  addressing, the given SCL speed, no other options. Covers every driver's
 *  i2c_device_config_t — they only ever varied address and speed. */
static inline esp_err_t i2c_add_device(i2c_master_bus_handle_t bus, uint8_t addr,
                                         uint32_t scl_speed_hz,
                                         i2c_master_dev_handle_t *dev) {
    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = scl_speed_hz,
    };
    return i2c_master_bus_add_device(bus, &devcfg, dev);
}

/** @brief Probe an address, then bind a device to it if something ACKed.
 *  Returns ESP_ERR_NOT_FOUND if the probe times out (nothing at that
 *  address), or the underlying esp_err_t from i2c_add_device() on a bind
 *  failure. For the common single-candidate-address init path; drivers that
 *  probe multiple candidate addresses (e.g. BMP581's 0x46/0x47) still call
 *  i2c_master_probe()/i2c_add_device() directly to avoid double-probing the
 *  address they settle on. */
static inline esp_err_t i2c_probe_and_add(i2c_master_bus_handle_t bus, uint8_t addr,
                                            uint32_t scl_speed_hz, int probe_timeout_ms,
                                            i2c_master_dev_handle_t *dev) {
    if (i2c_master_probe(bus, addr, probe_timeout_ms) != ESP_OK) return ESP_ERR_NOT_FOUND;
    return i2c_add_device(bus, addr, scl_speed_hz, dev);
}

/** @brief Remove a device handle and null it out — the standard driver
 *  init-failure cleanup. Safe to call with *dev already NULL (no-op). */
static inline void i2c_dev_teardown(i2c_master_dev_handle_t *dev) {
    if (dev && *dev) {
        i2c_master_bus_rm_device(*dev);
        *dev = NULL;
    }
}
