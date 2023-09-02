#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/dac_continuous.h"
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
#define DIALER_PULSE_PIN 15
#define HOOK_POWER_PIN 4
#define HOOK_SWITCH_PIN 2
#define RINGER_SIGNAL_PIN 16
#define RINGER_ENABLE_PIN 17

static uint8_t incoming_call_alert = 0;
static uint8_t call_in_progress = 0;

void on_headset_state_change(uint8_t state) {
    ESP_LOGI(TAG, "Headset state change: %d", state);
    
    if (state && incoming_call_alert) {
        bt_task_send(BT_EV_ANSWER_CALL, NULL, 0);
        ringer_enable(0);
        return;
    }
    
    if (!state && call_in_progress) {
        bt_task_send(BT_EV_CALL_HANGUP, NULL, 0);
        return;
    }
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
    
    if (dialer_get_headset_state()) {
        bt_task_send(BT_EV_DIAL_NUMBER, (void *) number, (size_t) number_length);
        ESP_LOGI(TAG, "Dialing number: %s. Number length: %d", number, number_length);
    }
}

static void main_task_handler(void *arg) {
    // Initialize phone interface
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    
    dialer_init(DIALER_PULSE_PIN, HOOK_POWER_PIN, HOOK_SWITCH_PIN, on_headset_state_change, on_start_dialing, on_digit, on_end_dialing);
    dialer_enable(1);
    
    ringer_init(RINGER_ENABLE_PIN, RINGER_SIGNAL_PIN);
    ESP_LOGI(TAG, "Headset state is %d", dialer_get_headset_state());
    
    // Bluetooth initialization
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

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
    
    QueueHandle_t bt_msg_queue = xQueueCreate(10, sizeof(bt_msg_t));
    assert(bt_msg_queue != NULL);

    bt_init(bt_msg_queue);
    
    bt_msg_t msg;
    
    ESP_LOGI(TAG, "10 ms = %"PRId32" tick", pdMS_TO_TICKS(10));

    while(1) {
        if (xQueueReceive(bt_msg_queue, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (msg.ev) {
            case BT_EV_INCOMING_CALL: {
                if (!dialer_get_headset_state()) {
                    incoming_call_alert = 1;
                    if (!ringer_get_state()) {
                        ringer_enable(1);
                    }
                } else {
                    bt_task_send(BT_EV_CALL_HANGUP, NULL, 0);
                }
                ESP_LOGI(TAG, "Incoming call");
                break;
            }

            case BT_EV_CALL_STATUS_IDLE: {
                if (ringer_get_state()) {
                    ringer_enable(0);
                }
                ESP_LOGI(TAG, "Call canceled");
                incoming_call_alert = 0;
                break;
            }

            case BT_EV_CALL_IN_PROGRESS: {
                incoming_call_alert = 0;
                call_in_progress = 1;
                ESP_LOGI(TAG, "Call in progress");
                break;
            }

            case BT_EV_CALL_HANGUP: {
                call_in_progress = 0;
                ESP_LOGI(TAG, "Call hangup");
                break;
            }
                                   
            default: {
                ESP_LOGW(TAG, "Unhandled message %d", msg.ev);
                break;
            }
        }
    }

}

void app_main(void) {
    BaseType_t r = xTaskCreatePinnedToCore(main_task_handler, "main", 4096, NULL, 5, NULL, 1);
    assert(r == pdTRUE);
    
    while (1) {
        vTaskDelay(1000);
    }
}
