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
 *  Lazy + sheddable secondary lets the multi-page display task and any
 *  future STEMMA2-attached sensor opt into bus 2 without forcing it
 *  always-on for deployments that only use STEMMA1.
 */

#include <stdbool.h>
#include "driver/i2c_master.h"

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

/** @brief Get the secondary I²C bus handle (FeatherS3-D STEMMA2 only).
 *
 *  On FeatherS3-D: lazy init on first call — drives IO39 HIGH (LDO2
 *  enable), waits 10 ms for the rail to settle, then creates the
 *  I²C controller on I²C_NUM_1 / IO15 / IO16. Subsequent calls return
 *  the cached handle.
 *
 *  On Heltec / QT Py / any board without a second bus: always returns
 *  NULL. Callers should treat NULL as "no second bus on this board"
 *  and skip secondary-bus probing gracefully.
 *
 *  Calling this enables LDO2 — adds ~5–10 mA continuous draw (LDO2
 *  quiescent + onboard NeoPixel idle on FeatherS3-D). If no consumer
 *  subsequently calls i2c_bus_secondary_keep_alive(), i2c_bus_finalize()
 *  will tear it back down.
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
