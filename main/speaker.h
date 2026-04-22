#pragma once

/** @file
 *  @brief Speaker click and LED blink on each GM pulse.
 *
 *  LEDC drives the piezo tone on PIN_SPEAKER_P; PIN_SPEAKER_N is toggled
 *  static-HIGH during a tick for push-pull drive. A 1 ms esp_timer consumes
 *  tick requests latched from the tube ISR, so the hard-real-time path stays
 *  trivial (a single volatile write).
 */

#include <stdbool.h>

/** @brief Configure LEDC/GPIO, start the audio timer, register the pulse callback.
 *
 *  @param play_sound    Emit a short boot chirp if true.
 *  @param led_tick      Enable onboard-LED blink per pulse.
 *  @param speaker_tick  Enable audible click per pulse.
 */
void speaker_setup(bool play_sound, bool led_tick, bool speaker_tick);

/** @brief Toggle the tick effects at runtime (e.g. after a config save). */
void speaker_set_modes(bool led_tick, bool speaker_tick);
