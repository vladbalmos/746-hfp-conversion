#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "dialer.h"

#define ESP_INTR_FLAG_DEFAULT 0
#define DIALER_PULSE_PIN 16
#define HOOK_SWITCH_PIN 17

static uint32_t triggered = 0;

void on_headset_state_change(uint8_t state) {
    printf("Headset state change: %d\n", state);
}

void on_start_dialing() {
    printf("Started dialing\n");
}

void on_digit(uint8_t digit) {
    printf("Dialed digit: %d\n", digit);
}

void on_end_dialing(const char *number, uint8_t number_length) {
    if (!number_length) {
        return;
    }
    printf("Dialied number: %s. Number length: %d\n", number, number_length);
}

void app_main(void) {
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    
    dialer_init(DIALER_PULSE_PIN, HOOK_SWITCH_PIN, on_headset_state_change, on_start_dialing, on_digit, on_end_dialing);
    dialer_enable(1);
    
    while(1) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
