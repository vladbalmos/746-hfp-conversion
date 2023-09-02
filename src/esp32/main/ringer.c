#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "ringer.h"

#define RINGER_FREQ_HZ 20
#define RINGER_ON_TIMEOUT_MS 2000
#define RINGER_OFF_TIMEOUT_MS RINGER_ON_TIMEOUT_MS * 2

static gpio_num_t enable_pin;
static gpio_num_t signal_pin;

static uint8_t last_ringing_state = 0;
static uint8_t ringer_enabled;

static esp_timer_handle_t ringing_timer;

static const ledc_timer_config_t pwm_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0,
    .duty_resolution = LEDC_TIMER_13_BIT,
    .freq_hz = RINGER_FREQ_HZ,
    .clk_cfg = LEDC_AUTO_CLK
};

static ledc_channel_config_t pwm_channel;

static void set_duty_cycle(uint8_t duty_cycle_percentage) {
    uint32_t duty_cycle = ((2 << (LEDC_TIMER_13_BIT - 1)) - 1) * (duty_cycle_percentage / 100.0);
    ESP_LOGI(RG_TAG, "Setting duty cycle to %"PRId32, duty_cycle);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void IRAM_ATTR ringing_timer_callback(void *arg) {
    set_duty_cycle(last_ringing_state ? 0 : 50);
    uint64_t timeout_ms = (last_ringing_state) ? RINGER_OFF_TIMEOUT_MS : RINGER_ON_TIMEOUT_MS;
    last_ringing_state = !last_ringing_state;
    
    ESP_ERROR_CHECK(esp_timer_start_once(ringing_timer, timeout_ms * 1000));
}


void ringer_enable(uint8_t status) {
    uint8_t duty_cycle_percentage = (status) ? 50 : 0;
    ringer_enabled = status;
    
    set_duty_cycle(duty_cycle_percentage);
    
    if (!status) {
        ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
    }
    gpio_set_level(enable_pin, status);
    
    if (status) {
        last_ringing_state = 1;
        ESP_ERROR_CHECK(esp_timer_start_once(ringing_timer, RINGER_ON_TIMEOUT_MS * 1000));
        return;
    }
    
    esp_timer_stop(ringing_timer);
}

uint8_t ringer_get_state() {
    return ringer_enabled;
}

void ringer_init(gpio_num_t en_pin, gpio_num_t sig_pin)
{
    enable_pin = en_pin;
    signal_pin = sig_pin;

    pwm_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    pwm_channel.channel = LEDC_CHANNEL_0;
    pwm_channel.timer_sel = LEDC_TIMER_0;
    pwm_channel.intr_type = LEDC_INTR_DISABLE;
    pwm_channel.gpio_num = signal_pin;
    pwm_channel.duty = 0;
    pwm_channel.hpoint = 0;

    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << enable_pin;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    const esp_timer_create_args_t ringing_timer_args = {
        .callback = &ringing_timer_callback,
        .name = "ringing-timer"
    };
    
    ESP_ERROR_CHECK(esp_timer_create(&ringing_timer_args, &ringing_timer));
}