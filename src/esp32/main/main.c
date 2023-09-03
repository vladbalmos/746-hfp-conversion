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
#include "bt_state_led.h"

#define TAG "MAIN"

#define ESP_INTR_FLAG_DEFAULT 0

#define DIALER_PULSE_PIN 15
#define HOOK_POWER_PIN 4
#define HOOK_SWITCH_PIN 2
#define RINGER_SIGNAL_PIN 16
#define RINGER_ENABLE_PIN 17
#define ENABLE_PAIRING_PIN 18
#define BT_STATE_LED_PIN 19

static uint8_t incoming_call_alert = 0;
static uint8_t call_in_progress = 0;
static uint32_t last_irq_time_ms = 0;


static void IRAM_ATTR gpio_isr_enable_pairing_handler(void* arg) {
    QueueHandle_t q = (QueueHandle_t) arg;
    uint32_t now = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    uint32_t diff_ms = now - last_irq_time_ms;
    last_irq_time_ms = now;
    
    if (diff_ms < 100) {
        return;
    }

    bt_msg_t msg = bt_create_msg(BT_EV_PAIR_BTN_ACTIVITY, NULL, 0);
    xQueueSendFromISR(q, &msg, NULL);
}

void on_headset_state_change(uint8_t state) {
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

void on_end_dialing(const char *number, uint8_t number_length) {
    if (!number_length) {
        return;
    }
    
    if (dialer_get_headset_state()) {
        bt_task_send(BT_EV_DIAL_NUMBER, (void *) number, (size_t) number_length);
        ESP_LOGI(TAG, "Dialing number: %s. Number length: %d", number, number_length);
    }
}

static void main_task_handler(void *arg) {
    QueueHandle_t bt_msg_queue = xQueueCreate(10, sizeof(bt_msg_t));
    assert(bt_msg_queue != NULL);

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    
    gpio_config_t input_conf = {};
    input_conf.intr_type = GPIO_INTR_ANYEDGE;
    input_conf.mode = GPIO_MODE_INPUT;
    input_conf.pin_bit_mask = 1ULL << ENABLE_PAIRING_PIN;
    input_conf.pull_up_en = 0;
    input_conf.pull_down_en = 1;
    ESP_ERROR_CHECK(gpio_config(&input_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENABLE_PAIRING_PIN, gpio_isr_enable_pairing_handler, (void *) bt_msg_queue));
    
    // Init bluetooth connection state led
    bt_state_led_init(BT_STATE_LED_PIN);

    // Initialize phone interface
    dialer_init(DIALER_PULSE_PIN, HOOK_POWER_PIN, HOOK_SWITCH_PIN, on_headset_state_change, NULL, NULL, on_end_dialing);
    dialer_enable(1);
    
    ringer_init(RINGER_ENABLE_PIN, RINGER_SIGNAL_PIN);
    
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

            case BT_EV_OUTGOING_CALL: {
                call_in_progress = 1;
                ESP_LOGI(TAG, "Outgoing call");
                break;
            }

            case BT_EV_CALL_STATUS_IDLE: {
                if (ringer_get_state()) {
                    ringer_enable(0);
                }
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
                                    
            case BT_EV_PAIR_BTN_ACTIVITY: {
                uint8_t btn_pressed = gpio_get_level(ENABLE_PAIRING_PIN);
                if (btn_pressed) {
                    bt_task_send(BT_EV_ENABLE_PAIRING, NULL, 0);
                }
                break;
            }
                                          
            case BT_EV_START_PAIRING: {
                bt_state_led_blink();
                ESP_LOGW(TAG, "Start pairing");
                break;
            }

            case BT_EV_CONNECTED: {
                bt_state_led_on();
                ESP_LOGW(TAG, "Connected");
                break;
            }

            case BT_EV_PAIRING_TIMEOUT: {
                bt_state_led_off();
                bt_task_send(BT_EV_STOP_PAIRING, NULL, 0);
                ESP_LOGW(TAG, "Pairing timeout");
                break;
            }
            case BT_EV_DISCONNECTED: {
                bt_state_led_off();
                ESP_LOGW(TAG, "Disconnected");
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
