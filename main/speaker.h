#pragma once

/** @file
 *  @brief Speaker click and LED blink on each GM pulse, plus boot melody.
 *
 *  LEDC drives the piezo tone on PIN_SPEAKER_P; PIN_SPEAKER_N is toggled
 *  static-HIGH during a tone for push-pull drive (volume=1) or held LOW for
 *  single-ended drive (volume=0, quieter). A 1 ms esp_timer consumes tick
 *  requests latched from the tube ISR, so the hard-real-time path stays
 *  trivial (a single volatile write).
 *
 *  Melody playback (V2.3.21+) replicates the V1 firmware's boot tune.
 *  Melody takes priority over ticks while playing; tick requests during a
 *  melody are silently dropped (the eye/ear can't interleave them sensibly).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief One step in a melody sequence.
 *
 *  freq_mhz: tone frequency in millihertz (e.g. 880994 = 880.994 Hz).
 *            Set to 0 for a silent rest.
 *  volume:   0 = single-ended drive (quieter), 1 = push-pull drive (louder).
 *            Ignored when freq_mhz == 0.
 *  duration_ms: how long this step lasts before advancing to the next.
 *               A step with duration_ms == 0 marks end-of-sequence.
 */
typedef struct {
    uint32_t freq_mhz;
    uint8_t  volume;
    uint16_t duration_ms;
} melody_step_t;

/** @brief Configure LEDC/GPIO, start the audio timer, register the pulse callback.
 *
 *  @param play_sound    Play the V1 boot melody if true (~3.5 s tune).
 *  @param led_tick      Enable onboard-LED blink per pulse.
 *  @param speaker_tick  Enable audible click per pulse.
 */
void speaker_setup(bool play_sound, bool led_tick, bool speaker_tick);

/** @brief Toggle the tick effects at runtime (e.g. after a config save). */
void speaker_set_modes(bool led_tick, bool speaker_tick);

/** @brief Stop the pulse tick for the rest of this boot — OTA teardown (V2.7.5).
 *
 *  Clears the tick modes and any melody, stops the 1 ms audio timer and forces
 *  the LEDC duty and the N pin low. Called from the /update POST teardown
 *  alongside mqtt_stop() / log_ftp_pause() / tube_pcnt_stop(), because an OTA
 *  flash write starves the audio timer and a tick caught mid-flight holds its
 *  tone instead of ending after 4 ms.
 *
 *  STOPS THE VISUAL TICK TOO, not just the piezo. tick_start() drives the
 *  NeoPixel / PIN_LED_BUILTIN flash from the same call site as the tone, so on
 *  a board with a speaker the per-pulse blink stops with it. Boards without a
 *  speaker are unaffected: led.c owns the tick there (its
 *  `!HAL_HAS_SPEAKER && !HAL_HAS_NEOPIXEL` gate) and keeps blinking through
 *  the update.
 *
 *  One-way, like the rest of the OTA teardown — the device reboots on OTA
 *  success; after a FAILED OTA the tick stays off until a manual /reboot,
 *  exactly as MQTT and the FTPS schedule do. There is deliberately no
 *  speaker_resume(): nothing in the firmware needs one, and adding it would
 *  mean re-establishing state this function is free to leave inconsistent.
 *
 *  Counting is untouched: the tube ISR keeps running and keeps latching tick
 *  requests, they just go unread. No-op when HAL_HAS_SPEAKER == 0.
 */
void speaker_suspend(void);

/** @brief Start playing a melody sequence.
 *
 *  @p seq must be a pointer to a `melody_step_t` array terminated by an
 *  entry with duration_ms == 0. The pointer is held by reference (not
 *  copied) — the caller MUST keep the storage alive until playback ends
 *  (typically a `static const` array in BSS, which is the V1 pattern).
 *
 *  Idempotent if a melody is already playing — the new sequence replaces
 *  the old one cleanly.
 *
 *  No-op when HAL_HAS_SPEAKER == 0.
 */
void speaker_play_melody(const melody_step_t *seq);
