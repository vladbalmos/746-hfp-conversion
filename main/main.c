#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "bt.h"
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
    // Initialize phone interface
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    
    dialer_init(DIALER_PULSE_PIN, HOOK_SWITCH_PIN, on_headset_state_change, on_start_dialing, on_digit, on_end_dialing);
    dialer_enable(1);
    
    ringer_init(RINGER_ENABLE_PIN, RINGER_SIGNAL_PIN);
    ESP_LOGI(TAG, "Headset state is %d", dialer_get_headset_state());
    
    // Bluetooth initialization
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    
    esp_err_t err;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((err = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((err = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    bt_init();
    
    while(1) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
