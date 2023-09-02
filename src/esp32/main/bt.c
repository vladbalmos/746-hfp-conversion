#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_timer.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"
#include "esp_log.h"
#include "audio.h"
#include "bt.h"


#define TAG "BT"
#define BT_QUEUE_ADD_MAX_WAIT_TICKS 10
#define BT_QUEUE_MAX_SIZE 16
#define BT_DEVICE_NAME "GPO 746"

static QueueHandle_t msg_queue = NULL;
static QueueHandle_t audio_ready_queue = NULL;
static QueueHandle_t outgoing_msg_queue = NULL;

static int64_t last_incoming_buffer_us = 0;
static int64_t receive_interval_us = 0;
static int64_t last_outgoing_buffer_us = 0;
static int64_t send_interval_us = 0;
static int64_t audio_ready_interval_us = 0;
static uint8_t audio_enabled = 0;

static void audio_signal_ready_task_handler(void *arg) {
    QueueHandle_t audio_ready_queue = (QueueHandle_t) arg;
    uint8_t _;
    int64_t now;
    int64_t last_received_item_us = 0;


    while (1) {
        if (!audio_enabled) {
            vTaskDelay(1);
            continue;
        }

        if (xQueueReceive(audio_ready_queue, &_, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }

        now = esp_timer_get_time();
        audio_ready_interval_us = now - last_received_item_us;
        last_received_item_us = now;

        if (audio_enabled) {
            esp_hf_client_outgoing_data_ready();
        }
    }
}

bt_msg_t bt_create_msg(bt_event_type_t ev, void *data, size_t data_size) {
    bt_msg_t msg;

    memset(&msg, 0, sizeof(bt_msg_t));
    msg.ev = ev;
    msg.data_size = data_size;
    
    if (!data_size) {
        return msg;
    }

    msg.data = malloc(data_size);
    assert(msg.data != NULL);
    memcpy(msg.data, data, data_size);
    return msg;
}

static uint32_t IRAM_ATTR hf_outgoing_data_callback(uint8_t *p_buf, uint32_t sz) {
    int64_t now = esp_timer_get_time();
    send_interval_us = now - last_outgoing_buffer_us;
    last_outgoing_buffer_us = now;
    
    size_t item_size = 0;
    audio_receive(p_buf, &item_size, sz);
    uint32_t ret = (item_size == sz) ? sz : 0;
    return ret;
}

static void IRAM_ATTR hf_incoming_data_callback(const uint8_t *buf, uint32_t size) {
    int64_t now = esp_timer_get_time();
    receive_interval_us = now - last_incoming_buffer_us;
    last_incoming_buffer_us = now;
    audio_send(buf, size);
}


static void send_outgoing_message(bt_event_type_t ev, void *data, size_t data_size) {
    bt_msg_t msg = bt_create_msg(ev, data, 0);
    xQueueSend(outgoing_msg_queue, &msg, BT_QUEUE_ADD_MAX_WAIT_TICKS / portTICK_PERIOD_MS);
}

static void IRAM_ATTR hf_client_audio_open(esp_hf_client_audio_state_t con_state) {
    sample_rate_t current_sample_rate = audio_get_sample_rate();
    sample_rate_t new_sample_rate = (con_state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) 
                                                   ? SAMPLE_RATE_16KHZ : SAMPLE_RATE_8KHZ;
                                                   
    if (current_sample_rate != new_sample_rate) {
        audio_deinit();
        audio_init(new_sample_rate, audio_ready_queue);
    }
    
    audio_enable(1);
    audio_enabled = 1;
}

static void IRAM_ATTR hf_client_audio_close() {
    audio_enabled = 0;
    audio_enable(0);
}

static void IRAM_ATTR esp_bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {

        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            }
            break;
        }

        case ESP_BT_GAP_PIN_REQ_EVT: {
            if (param->pin_req.min_16_digit) {
                esp_bt_pin_code_t pin_code = {0};
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            } else {
                esp_bt_pin_code_t pin_code;
                pin_code[0] = '1';
                pin_code[1] = '2';
                pin_code[2] = '3';
                pin_code[3] = '4';
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            }
            break;
        }

        case ESP_BT_GAP_CFM_REQ_EVT: {
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        }

        default: {
            break;
        }
    }
}


static void IRAM_ATTR esp_bt_hf_client_callback(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param) {
    switch (event) {
        case ESP_HF_CLIENT_CONNECTION_STATE_EVT: {
            if (param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED) {
                ESP_LOGI(TAG, "Initializing dac");
                audio_init(SAMPLE_RATE_16KHZ, audio_ready_queue);
                // Disable discoverability after first pair
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            } else if (param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "DE_Initializing dac");
                audio_deinit();
                // Re-enable discoverability
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            }
            break;
        }
                                                 
        case ESP_HF_CLIENT_AUDIO_STATE_EVT: {
            if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
                param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC)
            {
                hf_client_audio_open(param->audio_stat.state);
            } else {
                hf_client_audio_close();
            }
            
            break;
        }

        case ESP_HF_CLIENT_CIND_CALL_EVT:
        {
            if (param->call.status == ESP_HF_CALL_STATUS_NO_CALLS) {
                send_outgoing_message(BT_EV_CALL_HANGUP, NULL, 0);
            } else if (param->call.status == ESP_HF_CALL_STATUS_CALL_IN_PROGRESS) {
                send_outgoing_message(BT_EV_CALL_IN_PROGRESS, NULL, 0);
            }
            break;
        }

        case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
        {
            if (param->call_setup.status == ESP_HF_CALL_SETUP_STATUS_INCOMING) {
                send_outgoing_message(BT_EV_INCOMING_CALL, NULL, 0);
            } else if (param->call_setup.status == ESP_HF_CALL_SETUP_STATUS_IDLE) {
                send_outgoing_message(BT_EV_CALL_STATUS_IDLE, NULL, 0);
            }
            break;
        }
        default:
            break;
    }
}

static void bt_init_stack() {
    // Init HFP stack
    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(BT_DEVICE_NAME));
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(esp_bt_gap_callback));
    ESP_ERROR_CHECK(esp_hf_client_register_callback(esp_bt_hf_client_callback));
    ESP_ERROR_CHECK(esp_hf_client_init());
    ESP_ERROR_CHECK(esp_hf_client_register_data_callback(hf_incoming_data_callback, hf_outgoing_data_callback));
    
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code;
    pin_code[0] = '0';
    pin_code[1] = '0';
    pin_code[2] = '0';
    pin_code[3] = '0';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);

    /* set discoverable and connectable mode, wait to be connected */
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    
    audio_init_transport();
}

static void msg_task_handler(void *arg) {
    QueueHandle_t msg_queue = (QueueHandle_t) arg;
    bt_msg_t msg;
    while (1) {
        if (xQueueReceive(msg_queue, &msg, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }
        
        switch (msg.ev) {
            case BT_EV_INITIALIZE: {
                bt_init_stack();
                break;
            }
            case BT_EV_ANSWER_CALL: {
                esp_hf_client_answer_call();
                break;
            }
            case BT_EV_CALL_HANGUP: {
                esp_hf_client_reject_call();
                break;
            }
            case BT_EV_DIAL_NUMBER: {
                const char *number = (char *) msg.data;
                esp_hf_client_dial(number);
            }
                                   
            default: {
                break;
            }
        }
        
        if (msg.data) {
            free(msg.data);
        }
        
        ESP_LOGI(TAG, "Received message %d", msg.ev);
    }
}

void bt_task_send(bt_event_type_t ev, void *data, size_t data_size) {
    bt_msg_t msg = bt_create_msg(ev, data, data_size);
    xQueueSend(msg_queue, &msg, BT_QUEUE_ADD_MAX_WAIT_TICKS / portTICK_PERIOD_MS);
}

static void debug_task_handler(void *arg) {
    while (1) {
        vTaskDelay(200);
        ESP_LOGI(TAG, "Receive interval: %"PRId64" Send interval: %"PRId64" Ready interval: %"PRId64, receive_interval_us, send_interval_us, audio_ready_interval_us);
    }
}

void bt_init(QueueHandle_t _outgoing_msg_queue) {
    BaseType_t r;
    outgoing_msg_queue = _outgoing_msg_queue;
    msg_queue = xQueueCreate(BT_QUEUE_MAX_SIZE, sizeof(bt_msg_t));
    assert(msg_queue != NULL);
    
    audio_ready_queue = xQueueCreate(8, sizeof(uint8_t));
    assert(audio_ready_queue != NULL);
    
    xTaskCreatePinnedToCore(debug_task_handler, "audio_debug", 2048, NULL, 1, NULL, 1);

    r = xTaskCreatePinnedToCore(audio_signal_ready_task_handler, "bt_signal_audio_ready", 4096, audio_ready_queue, 5, NULL, 1);
    assert(r == pdPASS);
    ESP_LOGI(TAG, "Created audio handler task");

    r = xTaskCreatePinnedToCore(msg_task_handler, "bt_msg_handler", 8192, msg_queue, 5, NULL, 1);
    assert(r == pdPASS);
    ESP_LOGI(TAG, "Created bluetooth task");
    
    bt_task_send(BT_EV_INITIALIZE, NULL, 0);
    ESP_LOGD(TAG, "Sent 'initialize' event to task");
}
