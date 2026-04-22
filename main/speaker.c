#include "speaker.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tube.h"

static const char *TAG = "speaker";

// Heltec Wireless Stick V2 wiring.
#define PIN_SPEAKER_P   12   // driven by LEDC (tone generator)
#define PIN_SPEAKER_N    0   // static HIGH during tick for push-pull drive
#define LED_BUILTIN     25   // onboard LED

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
        gpio_set_level(LED_BUILTIN, 1);
    }
}

static void tick_end(void) {
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
    gpio_set_level(PIN_SPEAKER_N, 0);
    gpio_set_level(LED_BUILTIN,   0);
}

// Runs at esp_timer task context — LEDC-safe. 1 ms period.
static void audio_timer_cb(void *arg) {
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
    // LED + speaker N pin as GPIO outputs.
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << LED_BUILTIN) | (1ULL << PIN_SPEAKER_N),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));
    gpio_set_level(LED_BUILTIN,   0);
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
    tube_set_pulse_callback(on_gm_pulse);

    // Short boot chirp — two quick clicks — only when play_sound is enabled.
    if (play_sound) {
        s_pending_ticks = 1;
        vTaskDelay(pdMS_TO_TICKS(100));
        s_pending_ticks = 1;
    }

    ESP_LOGI(TAG, "speaker setup: LED=%d P=%d N=%d (led_tick=%d speaker_tick=%d play=%d)",
             LED_BUILTIN, PIN_SPEAKER_P, PIN_SPEAKER_N,
             led_tick, speaker_tick, play_sound);
}
