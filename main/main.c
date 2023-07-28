#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "dialer.h"
#include "ringer.h"

#define TAG "MAIN"

#define ESP_INTR_FLAG_DEFAULT 0
#define DIALER_PULSE_PIN 16
#define HOOK_SWITCH_PIN 17
#define RINGER_SIGNAL_PIN 5
#define RINGER_ENABLE_PIN 18

void on_headset_state_change(uint8_t state) {
    ESP_LOGI(TAG, "Headset state change: %d", state);
    ringer_enable(!state);
}

void on_start_dialing() {
    ESP_LOGI(TAG, "Started dialing");
}

void on_digit(uint8_t digit) {
    ESP_LOGI(TAG, "Dialed digit: %d", digit);
}

void on_end_dialing(const char *number, uint8_t number_length) {
    ESP_LOGI(TAG, "End dialing");
    if (!number_length) {
        return;
    }
    ESP_LOGI(TAG, "Dialed number: %s. Number length: %d", number, number_length);
}

void app_main(void) {
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    
    dialer_init(DIALER_PULSE_PIN, HOOK_SWITCH_PIN, on_headset_state_change, on_start_dialing, on_digit, on_end_dialing);
    dialer_enable(1);
    
    ringer_init(RINGER_ENABLE_PIN, RINGER_SIGNAL_PIN);
    ESP_LOGI(TAG, "Headset state is %d", dialer_get_headset_state());
    
    while(1) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
