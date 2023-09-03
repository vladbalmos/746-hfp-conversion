#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define BT_LED_BLINK_INTERVAL_US 125 * 1000

static gpio_num_t led_pin;
static uint8_t last_led_state = 0;
static esp_timer_handle_t blink_timer;

static void blink_timer_callback(void *arg) {
    last_led_state = !last_led_state;
    gpio_set_level(led_pin, last_led_state);
}

void bt_state_led_on() {
    gpio_set_level(led_pin, 1);
    esp_timer_stop(blink_timer);
}

void bt_state_led_off() {
    gpio_set_level(led_pin, 0);
    esp_timer_stop(blink_timer);
}

void bt_state_led_blink() {
    esp_timer_create_args_t timer_args = {
        .callback = &blink_timer_callback,
        .name = "bt-led-blink-timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &blink_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(blink_timer, BT_LED_BLINK_INTERVAL_US));
}

void bt_state_led_init(gpio_num_t pin) {
    led_pin = pin;

    gpio_config_t output_conf = {};
    output_conf.intr_type = GPIO_INTR_DISABLE;
    output_conf.mode = GPIO_MODE_OUTPUT;
    output_conf.pin_bit_mask = 1ULL << led_pin;
    output_conf.pull_up_en = 0;
    output_conf.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&output_conf));
    bt_state_led_off();
}