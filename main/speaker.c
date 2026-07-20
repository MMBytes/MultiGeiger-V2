#include "speaker.h"

#include "hal.h"   // PIN_SPEAKER_P, PIN_SPEAKER_N, PIN_LED_BUILTIN, HAL_HAS_SPEAKER

// Boards without a piezo (HAL_HAS_SPEAKER == 0, e.g. small QT Py form factor
// where pin budget can't accommodate it) get no-op stubs for the public API.
// All the LEDC + esp_timer + ISR plumbing is gated out so the firmware
// doesn't try to drive PIN_SPEAKER_* (which are intentionally undefined on
// those boards) and doesn't allocate hardware resources it'll never use.
#if HAL_HAS_SPEAKER

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tube.h"
#if HAL_HAS_NEOPIXEL
#include "neopixel.h"   // neopixel_notify_pulse — led_tick flash, preferred over the plain LED
#endif

static const char *TAG = "speaker";

// PIN_SPEAKER_P is driven by LEDC (tone generator).
// PIN_SPEAKER_N is held HIGH during a tick for push-pull drive against PIN_SPEAKER_P.
// led_tick visual: boards with HAL_HAS_NEOPIXEL flash the NeoPixel via
// neopixel_notify_pulse() — preferred over the plain PIN_LED_BUILTIN even when
// both exist (V2.6.24, user decision at #5477 bring-up: the RGB pixel is the
// intended pulse indicator on dual-LED Feathers, reversing the original
// "PIN_LED_BUILTIN wins" port decision). Boards without a NeoPixel light
// PIN_LED_BUILTIN instead. Same "intentionally undefined optional pin" idiom
// used for PIN_OLED_RESET.

#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_NUM  LEDC_TIMER_0
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_RES_BITS   LEDC_TIMER_10_BIT
#define LEDC_DUTY_HALF  (1 << 9)         // 50% of 10-bit range

#define TICK_FREQ_HZ    5000
#define TICK_LEN_MS     4
#define AUDIO_TICK_US   1000             // 1 ms audio timer period

static volatile bool s_speaker_tick = false;
static volatile bool s_led_tick     = false;

// Shared between GM ISR and audio timer callback. ISR writes, timer reads.
static volatile uint32_t s_pending_ticks = 0;
static uint32_t s_tick_remaining_ms = 0;   // audio-timer private

// Melody playback state (V2.3.21+). Single writer (speaker_play_melody from
// the main task at boot) → single reader (audio_timer_cb on the esp_timer
// task). volatile pointer write is the publishing barrier — when the cb
// observes s_melody_seq != NULL, the index/remaining state is already
// initialised.
static volatile const melody_step_t *s_melody_seq = NULL;
static uint32_t s_melody_idx = 0;
static uint32_t s_melody_step_remaining_ms = 0;

static esp_timer_handle_t s_audio_timer = NULL;

// Called from tube.c GM ISR. Keep it trivial — just latch a pending tick.
static void IRAM_ATTR on_gm_pulse(void) {
    if (s_speaker_tick || s_led_tick) {
        s_pending_ticks = 1;
    }
}

static void tick_start(void) {
    if (s_speaker_tick) {
        ledc_set_freq(LEDC_SPEED_MODE, LEDC_TIMER_NUM, TICK_FREQ_HZ);
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, LEDC_DUTY_HALF);
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
        gpio_set_level(PIN_SPEAKER_N, 1);
    }
    if (s_led_tick) {
#if HAL_HAS_NEOPIXEL
        neopixel_notify_pulse();   // NeoPixel preferred wherever present (V2.6.24)
#elif defined(PIN_LED_BUILTIN)
        gpio_set_level(PIN_LED_BUILTIN, 1);
#endif
    }
}

static void tick_end(void) {
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
    gpio_set_level(PIN_SPEAKER_N, 0);
#if !HAL_HAS_NEOPIXEL && defined(PIN_LED_BUILTIN)
    // Only the plain-LED tick needs an off-edge here; the NeoPixel worker
    // times its own 40 ms flash and restores the base colour itself.
    gpio_set_level(PIN_LED_BUILTIN,   0);
#endif
}

// Apply one step of a melody sequence — drives LEDC freq + duty + the N-pin
// push-pull level. Called only from audio_timer_cb (esp_timer task) so LEDC
// API is safe.
static void melody_apply_step(const melody_step_t *step) {
    if (step->freq_mhz > 0) {
        ledc_set_freq(LEDC_SPEED_MODE, LEDC_TIMER_NUM, step->freq_mhz / 1000);
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, LEDC_DUTY_HALF);
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
        gpio_set_level(PIN_SPEAKER_N, step->volume >= 1 ? 1 : 0);
    } else {
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
        gpio_set_level(PIN_SPEAKER_N, 0);
    }
}

// Runs at esp_timer task context — LEDC-safe. 1 ms period.
static void audio_timer_cb(void *arg) {
    // Melody takes priority over ticks. The boot melody runs for ~3.5 s
    // before WiFi/HTTP comes up, so dropping any tick during that window
    // costs nothing (the tube ISR only fires after tube_setup() runs in
    // app_main, well before the first cycle's CPM accumulates anyway).
    if (s_melody_seq) {
        if (s_melody_step_remaining_ms == 0) {
            // Time to advance to the next step.
            const melody_step_t *step = (const melody_step_t *)&s_melody_seq[s_melody_idx];
            if (step->duration_ms == 0) {
                // End-of-sequence marker. Silence everything and clear state.
                ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
                ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
                gpio_set_level(PIN_SPEAKER_N, 0);
                s_melody_seq = NULL;
                s_melody_idx = 0;
                s_pending_ticks = 0;   // discard any tick queued during melody
                return;
            }
            melody_apply_step(step);
            s_melody_step_remaining_ms = step->duration_ms;
            s_melody_idx++;
            return;
        }
        s_melody_step_remaining_ms--;
        return;
    }

    if (s_tick_remaining_ms > 0) {
        if (--s_tick_remaining_ms == 0) {
            tick_end();
        }
        return;
    }
    if (s_pending_ticks) {
        s_pending_ticks = 0;
        tick_start();
        s_tick_remaining_ms = TICK_LEN_MS;
    }
}

void speaker_play_melody(const melody_step_t *seq) {
    if (!seq) return;
    // Order matters: index + remaining first, then sequence pointer last.
    // audio_timer_cb gates on s_melody_seq != NULL, so when it observes the
    // pointer, the rest of the state is already valid.
    s_melody_idx               = 0;
    s_melody_step_remaining_ms = 0;
    s_melody_seq               = seq;
}

// V1 boot melody — replicated note-for-note from the original firmware
// (Git_Repository_Geiger/multigeiger/speaker.cpp). V1 used a TONE() macro
// that scaled frequency × 0.75 and duration × 85 ms; we bake those values
// into the literals directly. ~3.5 s total, pattern D-E-F#-G | D-E-D | B-C-B.
//
// volume=1 = push-pull (PIN_N driven HIGH during tone) for max audible level;
// the next-to-last "B" note uses volume=0 = single-ended drive for a quieter
// fade, matching the V1 tone_volume parameter.
static const melody_step_t s_v1_boot_melody[] = {
    {  880994, 1, 170 },   // D
    {       0, 0, 170 },   // rest
    {  988882, 1, 170 },   // E
    {       0, 0, 170 },   // rest
    { 1109983, 1, 170 },   // F# (Fis)
    {       0, 0, 170 },   // rest
    { 1175986, 1, 340 },   // G
    {  880994, 1, 170 },   // D
    {  988882, 1, 170 },   // E
    {  880994, 1, 340 },   // D
    {  740825, 1, 170 },   // B (H in German notation)
    {  784876, 1, 170 },   // C
    {  740825, 1, 340 },   // B
    {  740825, 0, 340 },   // B (single-ended — quieter fade, matches V1 vol=0)
    {       0, 0, 170 },   // rest
    {       0, 0,   0 },   // END marker (duration_ms == 0)
};

void speaker_set_modes(bool led_tick, bool speaker_tick) {
    s_led_tick     = led_tick;
    s_speaker_tick = speaker_tick;
    if (!led_tick && !speaker_tick) {
        s_pending_ticks = 0;
        s_tick_remaining_ms = 0;
        tick_end();
    }
    ESP_LOGI(TAG, "modes: led=%d speaker=%d", led_tick, speaker_tick);
}

void speaker_setup(bool play_sound, bool led_tick, bool speaker_tick) {
    // LED (if this board has one) + speaker N pin as GPIO outputs.
    gpio_config_t out_cfg = {
#ifdef PIN_LED_BUILTIN
        .pin_bit_mask = (1ULL << PIN_LED_BUILTIN) | (1ULL << PIN_SPEAKER_N),
#else
        .pin_bit_mask = (1ULL << PIN_SPEAKER_N),
#endif
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));
#ifdef PIN_LED_BUILTIN
    gpio_set_level(PIN_LED_BUILTIN,   0);
#endif
    gpio_set_level(PIN_SPEAKER_N, 0);

    // LEDC timer — 10-bit resolution, base frequency just a seed (set_freq
    // rewrites it per tick).
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_SPEED_MODE,
        .timer_num       = LEDC_TIMER_NUM,
        .duty_resolution = LEDC_RES_BITS,
        .freq_hz         = 1000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    ledc_channel_config_t ccfg = {
        .gpio_num   = PIN_SPEAKER_P,
        .speed_mode = LEDC_SPEED_MODE,
        .channel    = LEDC_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_NUM,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg));

    // 1 ms audio timer — runs audio_timer_cb on the esp_timer task.
    const esp_timer_create_args_t targs = {
        .callback = audio_timer_cb,
        .name     = "speaker_audio",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_audio_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_audio_timer, AUDIO_TICK_US));

    speaker_set_modes(led_tick, speaker_tick);
    // Claims the single tube-pulse callback slot for this board. On boards
    // that also have HAL_HAS_NEOPIXEL (with or without a PIN_LED_BUILTIN —
    // dual-LED Feathers included), this handler is
    // the ONLY one registered — it drives the NeoPixel directly via
    // neopixel_notify_pulse() in tick_start() rather than letting
    // neopixel_register_pulse_tick() re-register its own callback and
    // silently steal the slot back (which would break this speaker's audio
    // tick). See neopixel_register_pulse_tick()'s doc comment.
    tube_set_pulse_callback(on_gm_pulse);

    // V2.3.21: full V1 boot melody when play_sound is enabled (~3.5 s).
    // Replaces the prior two-click chirp. The melody-playback path in
    // audio_timer_cb walks s_v1_boot_melody at 1 ms resolution; ticks are
    // suppressed for the duration of playback.
    if (play_sound) {
        speaker_play_melody(s_v1_boot_melody);
    }

#if HAL_HAS_NEOPIXEL
    ESP_LOGI(TAG, "speaker setup: P=%d N=%d, led_tick drives NeoPixel (led_tick=%d speaker_tick=%d play=%d)",
             PIN_SPEAKER_P, PIN_SPEAKER_N,
             led_tick, speaker_tick, play_sound);
#elif defined(PIN_LED_BUILTIN)
    ESP_LOGI(TAG, "speaker setup: LED=%d P=%d N=%d (led_tick=%d speaker_tick=%d play=%d)",
             PIN_LED_BUILTIN, PIN_SPEAKER_P, PIN_SPEAKER_N,
             led_tick, speaker_tick, play_sound);
#else
    // Speaker but no visual indicator at all — a legal (if unlikely) future
    // combination; tick_start() already no-ops led_tick for it.
    ESP_LOGI(TAG, "speaker setup: P=%d N=%d, no led_tick indicator (led_tick=%d speaker_tick=%d play=%d)",
             PIN_SPEAKER_P, PIN_SPEAKER_N,
             led_tick, speaker_tick, play_sound);
#endif
}

#else   // HAL_HAS_SPEAKER == 0 → no-op stubs for boards without a piezo.

void speaker_setup(bool play_sound, bool led_tick, bool speaker_tick) {
    (void)play_sound; (void)led_tick; (void)speaker_tick;
}
void speaker_set_modes(bool led_tick, bool speaker_tick) {
    (void)led_tick; (void)speaker_tick;
}
void speaker_play_melody(const melody_step_t *seq) {
    (void)seq;
}

#endif  // HAL_HAS_SPEAKER
