/**
 * @file task_prio.h
 * @brief FreeRTOS priorities of every task this firmware creates or configures.
 *
 * V2.8.34: one table instead of a number in each file. The order is by how
 * late a job may run, not by how important it looks:
 *
 *   6  LoRaWAN worker  — receive windows must open within a few ms
 *   2  main task, SGP41, LED / NeoPixel flash, display rotation
 *                      — measurement and short periodic work
 *   1  web server (:443 and :80), MQTT client, upload worker
 *                      — bulk work that can wait (TLS, /log, OTA, uploads)
 *
 * ESP-IDF's own tasks sit far above all of these and are not changed:
 * IPC 24 (runs at the caller's priority per call, CONFIG_ESP_IPC_USES_CALLERS_
 * PRIORITY), WiFi 23 (core 0), esp_timer 22 (core 0), event loop 20, lwIP 18.
 * The GM tube pulses and the HV recharge timer are interrupts, unaffected.
 *
 * Why (2026-11-22, FeatherS3-D, full 4 MB log ring): the web server ran at
 * ESP-IDF's default 5. A 9 s HTTPS GET /log is CPU-bound on TLS encryption,
 * and the main task (then 1, pinned to core 0) got almost no CPU: the sensor
 * reads of a cycle took 6.9–7.5 s instead of 0.08–0.3 s (every full overlap;
 * the same download over plain HTTP: 0.3 s). The SGP41 task then hit an I2C
 * timeout, because the IDF bus lock is a binary semaphore (no priority
 * inheritance) and its wait uses the transfer's own 50–100 ms timeout. With
 * this table (2026-11-23, same 4 MB stream): 0.08–0.09 s, 6 of 6, no I2C
 * error; the download alone is as fast as before (~9.3 s). The LoRaWAN worker
 * (then 1) was exposed the same way in principle; not seen (receive windows
 * opened at +4990 ms under HTTPS load before and after), so 6 is a precaution.
 *
 * Measured side effects: none on HTTPS pages during a LoRaWAN exchange (status
 * page ~0.7 s, handshake ~0.6 s, same as outside it) or on OTA receive time.
 * A /log download that overlaps the upload round makes both ~40 % slower;
 * that sharing was always there (same on V2.8.33), the stalled main task
 * used to start the round 7 s late and so hid it.
 *
 * Rules that keep this table safe:
 * - All tasks that use the shared I2C bus periodically (main, SGP41, display
 *   rotation) share one level, so none can hold the bus while a higher one
 *   times out waiting for it. The web server touches I2C only rarely (fuel
 *   gauge and VEML7700 light sensor on the status page, display contrast on a
 *   /config save). At 1 it can be preempted while holding the bus; accepted,
 *   because those are 1–2 short reads, and a level-2 task that then blocks on
 *   the bus frees the CPU for it.
 * - The LoRaWAN worker busy-waits through a whole exchange (RadioLib's
 *   smartDelayUntil() spins with yield() because no sleep callback is set),
 *   3–8 s per uplink, up to ~25 s at SF12 with repeats. At 6 it owns core 1
 *   for that time. Acceptable: it is pinned to core 1, the only other task
 *   pinned there is the upload worker, and radio_turn_rule.h already keeps
 *   the two apart; every other task can run on core 0. The upload worker
 *   gets no CPU at all during an exchange (see wait_for_lorawan_quiet()).
 * - The main task can only run on core 0, so raising it never takes CPU
 *   away from core 1.
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/** @brief LoRaWAN worker (lorawan.cpp, pinned core 1): above every app task. */
#define TASK_PRIO_LORAWAN   (tskIDLE_PRIORITY + 6)

/** @brief app_main (sensor reads, cycle line, diag; pinned core 0 by Kconfig).
 *  ESP-IDF starts it at ESP_TASK_MAIN_PRIO (= 1) with no Kconfig to change
 *  that, so app_main raises itself as its first statement. */
#define TASK_PRIO_MAIN      (tskIDLE_PRIORITY + 2)

/** @brief SGP41 sampling task (1 s vTaskDelayUntil cadence, I2C). */
#define TASK_PRIO_SGP41     (tskIDLE_PRIORITY + 2)

/** @brief Tube-pulse LED / NeoPixel flash workers (woken from the pulse ISR). */
#define TASK_PRIO_PULSE     (tskIDLE_PRIORITY + 2)

/** @brief Multi-page display rotation task. Same level as main / SGP41 on
 *  purpose: it sends ~140 I2C transfers per page on the shared bus (see the
 *  bus-lock rule above), so it must not sit below them. */
#define TASK_PRIO_DISPLAY   (tskIDLE_PRIORITY + 2)

/** @brief Both esp_http_server instances (:443 or plain, and the :80 one).
 *  ESP-IDF's default is 5; long TLS work (GET /log, OTA) must not outrank
 *  measurement or the radio. OTA prep stops LoRaWAN, MQTT and uploads, so
 *  an OTA still gets a nearly idle device. */
#define TASK_PRIO_HTTPD     (tskIDLE_PRIORITY + 1)

/** @brief esp-mqtt client task (ESP-IDF default 5; a TLS reconnect is
 *  ~1 s of handshake CPU). */
#define TASK_PRIO_MQTT      (tskIDLE_PRIORITY + 1)

/** @brief Upload worker (transmission.c; core 1, unpinned on the C5). */
#define TASK_PRIO_TX        (tskIDLE_PRIORITY + 1)
