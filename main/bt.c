#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"
#include "esp_log.h"
#include "dac_audio.h"
#include "bt.h"

#define BT_TAG "BT"
#define BT_QUEUE_ADD_MAX_WAIT_TICKS 10
#define BT_QUEUE_MAX_SIZE 16
#define BT_DEVICE_NAME "GPO 746"
#define BT_HFP_RINGBUF_SIZE 3600
#define BT_AUDIO_BUF_POOL_SIZE 3
#define BT_AUDIO_MAX_SAMPLES 512

const static char *c_hf_evt_str[] = {
    "CONNECTION_STATE_EVT",              /*!< connection state changed event */
    "AUDIO_STATE_EVT",                   /*!< audio connection state change event */
    "VR_STATE_CHANGE_EVT",                /*!< voice recognition state changed */
    "CALL_IND_EVT",                      /*!< call indication event */
    "CALL_SETUP_IND_EVT",                /*!< call setup indication event */
    "CALL_HELD_IND_EVT",                 /*!< call held indicator event */
    "NETWORK_STATE_EVT",                 /*!< network state change event */
    "SIGNAL_STRENGTH_IND_EVT",           /*!< signal strength indication event */
    "ROAMING_STATUS_IND_EVT",            /*!< roaming status indication event */
    "BATTERY_LEVEL_IND_EVT",             /*!< battery level indication event */
    "CURRENT_OPERATOR_EVT",              /*!< current operator name event */
    "RESP_AND_HOLD_EVT",                 /*!< response and hold event */
    "CLIP_EVT",                          /*!< Calling Line Identification notification event */
    "CALL_WAITING_EVT",                  /*!< call waiting notification */
    "CLCC_EVT",                          /*!< listing current calls event */
    "VOLUME_CONTROL_EVT",                /*!< audio volume control event */
    "AT_RESPONSE",                       /*!< audio volume control event */
    "SUBSCRIBER_INFO_EVT",               /*!< subscriber information event */
    "INBAND_RING_TONE_EVT",              /*!< in-band ring tone settings */
    "LAST_VOICE_TAG_NUMBER_EVT",         /*!< requested number from AG event */
    "RING_IND_EVT",                      /*!< ring indication event */
};


const static char *c_connection_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "slc_connected",
    "disconnecting",
};

const static char *c_audio_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "connected_msbc",
};

static QueueHandle_t bt_msg_queue = NULL;
static RingbufHandle_t m_rb = NULL;
static dac_audio_buffer_pool_t *audio_out_buf_pool = NULL;
static dac_audio_buffer_pool_t *audio_in_buf_pool = NULL;

static uint32_t bt_hf_outgoing_data_callback(uint8_t *p_buf, uint32_t sz) {
    if (!m_rb) {
        return 0;
    }

    size_t item_size = 0;
    uint8_t *data = xRingbufferReceiveUpTo(m_rb, &item_size, 0, sz);
    if (item_size == sz) {
        memcpy(p_buf, data, item_size);
        vRingbufferReturnItem(m_rb, data);
        return sz;
    } else if (0 < item_size) {
        vRingbufferReturnItem(m_rb, data);
        return 0;
    } else {
        // data not enough, do not read
        return 0;
    }
}


static void bt_hf_incoming_data_callback(const uint8_t *buf, uint32_t sz) {
    if (!m_rb) {
        return;
    }
    BaseType_t done = xRingbufferSend(m_rb, (uint8_t *)buf, sz, 0);
    if (!done) {
        ESP_LOGE(BT_TAG, "rb send fail");
    }
    esp_hf_client_outgoing_data_ready();
}

static void bt_hf_client_audio_open() {
    m_rb = xRingbufferCreate(BT_HFP_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
}

static void bt_hf_client_audio_close() {
    if (!m_rb) {
        return;
    }

    vRingbufferDelete(m_rb);
}

static void esp_bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {

        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(BT_TAG, "authentication success: %s", param->auth_cmpl.device_name);
                esp_log_buffer_hex(BT_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            } else {
                ESP_LOGE(BT_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
            }
            break;
        }

        case ESP_BT_GAP_PIN_REQ_EVT: {
            ESP_LOGI(BT_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
            if (param->pin_req.min_16_digit) {
                ESP_LOGI(BT_TAG, "Input pin code: 0000 0000 0000 0000");
                esp_bt_pin_code_t pin_code = {0};
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            } else {
                ESP_LOGI(BT_TAG, "Input pin code: 1234");
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
            ESP_LOGI(BT_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        }

        case ESP_BT_GAP_KEY_NOTIF_EVT: {
            ESP_LOGI(BT_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%"PRIu32, param->key_notif.passkey);
            break;
        }

        case ESP_BT_GAP_KEY_REQ_EVT: {
            ESP_LOGI(BT_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
            break;
        }
                                     
        default: {
            ESP_LOGD(BT_TAG, "Unhandled event: %d", event);
        }
    }
}


static void esp_bt_hf_client_callback(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param) {
    if (event <= ESP_HF_CLIENT_RING_IND_EVT) {
        ESP_LOGI(BT_TAG, "APP HFP event: %s", c_hf_evt_str[event]);
    } else {
        ESP_LOGE(BT_TAG, "APP HFP invalid event %d", event);
    }
    
    switch (event) {
        case ESP_HF_CLIENT_CONNECTION_STATE_EVT: {
            ESP_LOGI(BT_TAG, "--connection state %s, peer feats 0x%"PRIx32", chld_feats 0x%"PRIx32,
                    c_connection_state_str[param->conn_stat.state],
                    param->conn_stat.peer_feat,
                    param->conn_stat.chld_feat);
            break;
        }
                                                 
        case ESP_HF_CLIENT_AUDIO_STATE_EVT: {
            ESP_LOGI(BT_TAG, "--audio state %s",
                    c_audio_state_str[param->audio_stat.state]);
            
            if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
                param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC)
            {
                esp_hf_client_register_data_callback(bt_hf_incoming_data_callback,
                                                    bt_hf_outgoing_data_callback);
                bt_hf_client_audio_open();
            } else {
                bt_hf_client_audio_close();
            }
            
            break;
        }
                                            
        default: {
            ESP_LOGD(BT_TAG, "Unhandled HF event: %d", event);
        }

        
    }
}

static void bt_init_stack() {
    // Init audio part
    audio_out_buf_pool = dac_audio_init_buffer_pool(BT_AUDIO_BUF_POOL_SIZE, BT_AUDIO_MAX_SAMPLES);
    assert(audio_out_buf_pool != NULL);

    audio_in_buf_pool = dac_audio_init_buffer_pool(BT_AUDIO_BUF_POOL_SIZE, BT_AUDIO_MAX_SAMPLES);
    assert(audio_in_buf_pool != NULL);

    dac_audio_init(DAC_SAMPLE_RATE_16KHZ);

    // Init HFP stack
    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(BT_DEVICE_NAME));
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(esp_bt_gap_callback));
    ESP_ERROR_CHECK(esp_hf_client_register_callback(esp_bt_hf_client_callback));
    ESP_ERROR_CHECK(esp_hf_client_init());
    
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code;
    pin_code[0] = '0';
    pin_code[1] = '0';
    pin_code[2] = '0';
    pin_code[3] = '0';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);

    /* set discoverable and connectable mode, wait to be connected */
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
}

static void bt_msg_handler(void *arg) {
    bt_msg_t msg;
    while (1) {
        if (xQueueReceive(bt_msg_queue, &msg, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }
        
        switch (msg.ev) {
            case BT_EV_INITIALIZE: {
                bt_init_stack();
                break;
            }
        }
        
        if (msg.data) {
            free(msg.data);
        }
        
        ESP_LOGI(BT_TAG, "Received message %d", msg.ev);
    }
}

void bt_task_send_msg(bt_msg_t *msg) {
    xQueueSend(bt_msg_queue, msg, BT_QUEUE_ADD_MAX_WAIT_TICKS / portTICK_PERIOD_MS);
}

void bt_task_send(bt_event_type_t ev, void *data, uint32_t data_size) {
    bt_msg_t msg = {
        .ev = ev,
        .data_size = data_size
    };
    memset(&msg, 0, sizeof(bt_msg_t));
    
    if (!data_size) {
        return bt_task_send_msg(&msg);
    }
    
    msg.data = malloc(data_size);
    assert(msg.data != NULL);
    memcpy(msg.data, data, data_size);
}

void bt_init() {
    bt_msg_queue = xQueueCreate(BT_QUEUE_MAX_SIZE, sizeof(bt_msg_t));
    BaseType_t r = xTaskCreate(bt_msg_handler, "bt_msg_handler", 2048, NULL, configMAX_PRIORITIES - 3, NULL);
    assert(r == pdPASS);
    ESP_LOGI(BT_TAG, "Created bluetooth task");
    
    bt_task_send(BT_EV_INITIALIZE, NULL, 0);
    ESP_LOGD(BT_TAG, "Sent 'initialize' event to task");
}