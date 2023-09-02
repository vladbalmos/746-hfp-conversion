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


#define BT_TAG "BT"
#define BT_QUEUE_ADD_MAX_WAIT_TICKS 10
#define BT_QUEUE_MAX_SIZE 16
#define BT_DEVICE_NAME "GPO 746"

static QueueHandle_t bt_msg_queue = NULL;
static QueueHandle_t bt_audio_data_available_queue = NULL;
static QueueHandle_t bt_outgoing_msg_queue = NULL;

static uint64_t rcv_buf_count = 0;
static uint64_t send_buf_count = 0;
static int64_t last_incoming_buffer_us = 0;
static int64_t receive_interval_us = 0;
static int64_t last_outgoing_buffer_us = 0;
static int64_t send_interval_us = 0;
static int64_t audio_ready_interval_us = 0;
static uint8_t audio_enabled = 0;

static uint32_t bt_hf_outgoing_data_callback(uint8_t *p_buf, uint32_t sz);

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

// esp_hf_client_connection_state_t
const char *c_connection_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "slc_connected",
    "disconnecting",
};

// esp_hf_client_audio_state_t
const char *c_audio_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "connected_msbc",
};

/// esp_hf_vr_state_t
const char *c_vr_state_str[] = {
    "disabled",
    "enabled",
};

// esp_hf_service_availability_status_t
const char *c_service_availability_status_str[] = {
    "unavailable",
    "available",
};

// esp_hf_roaming_status_t
const char *c_roaming_status_str[] = {
    "inactive",
    "active",
};

// esp_hf_client_call_state_t
const char *c_call_str[] = {
    "NO call in progress",
    "call in progress",
};

// esp_hf_client_callsetup_t
const char *c_call_setup_str[] = {
    "NONE",
    "INCOMING",
    "OUTGOING_DIALING",
    "OUTGOING_ALERTING"
};

// esp_hf_client_callheld_t
const char *c_call_held_str[] = {
    "NONE held",
    "Held and Active",
    "Held",
};

// esp_hf_response_and_hold_status_t
const char *c_resp_and_hold_str[] = {
    "HELD",
    "HELD ACCEPTED",
    "HELD REJECTED",
};

// esp_hf_client_call_direction_t
const char *c_call_dir_str[] = {
    "outgoing",
    "incoming",
};

// esp_hf_client_call_state_t
const char *c_call_state_str[] = {
    "active",
    "held",
    "dialing",
    "alerting",
    "incoming",
    "waiting",
    "held_by_resp_hold",
};

// esp_hf_current_call_mpty_type_t
const char *c_call_mpty_type_str[] = {
    "single",
    "multi",
};

// esp_hf_volume_control_target_t
const char *c_volume_control_target_str[] = {
    "SPEAKER",
    "MICROPHONE"
};

// esp_hf_at_response_code_t
const char *c_at_response_code_str[] = {
    "OK",
    "ERROR"
    "ERR_NO_CARRIER",
    "ERR_BUSY",
    "ERR_NO_ANSWER",
    "ERR_DELAYED",
    "ERR_BLACKLILSTED",
    "ERR_CME",
};

// esp_hf_subscriber_service_type_t
const char *c_subscriber_service_type_str[] = {
    "unknown",
    "voice",
    "fax",
};

// esp_hf_client_in_band_ring_state_t
const char *c_inband_ring_state_str[] = {
    "NOT provided",
    "Provided",
};

static void bt_signal_audio_ready_handler(void *arg) {
    uint8_t _;
    int64_t now;
    int64_t last_us = 0;


    while (1) {
        if (!audio_enabled) {
            vTaskDelay(1);
            continue;
        }

        if (xQueueReceive(bt_audio_data_available_queue, &_, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }

        now = esp_timer_get_time();
        audio_ready_interval_us = now - last_us;
        last_us = now;

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

static uint32_t IRAM_ATTR bt_hf_outgoing_data_callback(uint8_t *p_buf, uint32_t sz) {
    int64_t now = esp_timer_get_time();
    send_interval_us = now - last_outgoing_buffer_us;
    last_outgoing_buffer_us = now;
    
    size_t item_size = 0;
    audio_receive(p_buf, &item_size, sz);
    uint32_t ret = (item_size == sz) ? sz : 0;
    return ret;
}

static void IRAM_ATTR bt_hf_incoming_data_callback(const uint8_t *buf, uint32_t size) {
    int64_t now = esp_timer_get_time();
    receive_interval_us = now - last_incoming_buffer_us;
    last_incoming_buffer_us = now;
    audio_send(buf, size);
}


static void send_outgoing_message(bt_event_type_t ev, void *data, size_t data_size) {
    bt_msg_t msg = bt_create_msg(ev, data, 0);
    xQueueSend(bt_outgoing_msg_queue, &msg, BT_QUEUE_ADD_MAX_WAIT_TICKS / portTICK_PERIOD_MS);
    ESP_LOGW(BT_TAG, "Sent incoming call %d", ev);
}

static void IRAM_ATTR bt_hf_client_audio_open(esp_hf_client_audio_state_t con_state) {
    sample_rate_t current_sample_rate = audio_get_sample_rate();
    sample_rate_t new_sample_rate = (con_state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) 
                                                   ? SAMPLE_RATE_16KHZ : SAMPLE_RATE_8KHZ;
                                                   
    if (current_sample_rate != new_sample_rate) {
        audio_deinit();
        audio_init(new_sample_rate, bt_audio_data_available_queue);
    }
    
    audio_enable(1);
    audio_enabled = 1;
}

static void IRAM_ATTR bt_hf_client_audio_close() {
    audio_enabled = 0;
    audio_enable(0);
}

static void IRAM_ATTR esp_bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
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
                                     
        case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT: {
            ESP_LOGI(BT_TAG, "ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT Disconnected");
            break;
        }

        case ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT: {
            ESP_LOGI(BT_TAG, "ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT Bond removed");
            break;
        }
                                     
        default: {
            ESP_LOGD(BT_TAG, "Unhandled event: %d", event);
        }
    }
}


static void IRAM_ATTR esp_bt_hf_client_callback(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param) {
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
            
            if (param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED) {
                ESP_LOGI(BT_TAG, "Initializing dac");
                audio_init(SAMPLE_RATE_16KHZ, bt_audio_data_available_queue);
                // Disable discoverability after first pair
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            } else if (param->conn_stat.state == ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED) {
                ESP_LOGI(BT_TAG, "DE_Initializing dac");
                audio_deinit();
                // Re-enable discoverability
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            }
            break;
        }
                                                 
        case ESP_HF_CLIENT_AUDIO_STATE_EVT: {
            ESP_LOGI(BT_TAG, "--audio state %s",
                    c_audio_state_str[param->audio_stat.state]);
            
            if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
                param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC)
            {
                if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
                    ESP_LOGW(BT_TAG, "msbc connection");
                } else {
                    ESP_LOGE(BT_TAG, "cvsd connection");
                }
                bt_hf_client_audio_open(param->audio_stat.state);
            } else {
                bt_hf_client_audio_close();
            }
            
            break;
        }

        case ESP_HF_CLIENT_BVRA_EVT:
        {
            ESP_LOGI(BT_TAG, "--VR state %s",
                    c_vr_state_str[param->bvra.value]);
            break;
        }

        case ESP_HF_CLIENT_CIND_SERVICE_AVAILABILITY_EVT:
        {
            ESP_LOGI(BT_TAG, "--NETWORK STATE %s",
                    c_service_availability_status_str[param->service_availability.status]);
            break;
        }

        case ESP_HF_CLIENT_CIND_ROAMING_STATUS_EVT:
        {
            ESP_LOGI(BT_TAG, "--ROAMING: %s",
                    c_roaming_status_str[param->roaming.status]);
            break;
        }

        case ESP_HF_CLIENT_CIND_SIGNAL_STRENGTH_EVT:
        {
            ESP_LOGI(BT_TAG, "-- signal strength: %d",
                    param->signal_strength.value);
            break;
        }

        case ESP_HF_CLIENT_CIND_BATTERY_LEVEL_EVT:
        {
            ESP_LOGI(BT_TAG, "--battery level %d",
                    param->battery_level.value);
            break;
        }

        case ESP_HF_CLIENT_COPS_CURRENT_OPERATOR_EVT:
        {
            ESP_LOGI(BT_TAG, "--operator name: %s",
                    param->cops.name);
            break;
        }

        case ESP_HF_CLIENT_CIND_CALL_EVT:
        {
            if (param->call.status == ESP_HF_CALL_STATUS_NO_CALLS) {
                send_outgoing_message(BT_EV_CALL_HANGUP, NULL, 0);
            } else if (param->call.status == ESP_HF_CALL_STATUS_CALL_IN_PROGRESS) {
                send_outgoing_message(BT_EV_CALL_IN_PROGRESS, NULL, 0);
            }
            ESP_LOGI(BT_TAG, "--Call indicator %s",
                    c_call_str[param->call.status]);
            break;
        }

        case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
        {
            if (param->call_setup.status == ESP_HF_CALL_SETUP_STATUS_INCOMING) {
                send_outgoing_message(BT_EV_INCOMING_CALL, NULL, 0);
            } else if (param->call_setup.status == ESP_HF_CALL_SETUP_STATUS_IDLE) {
                send_outgoing_message(BT_EV_CALL_STATUS_IDLE, NULL, 0);
            }
            ESP_LOGI(BT_TAG, "--Call setup indicator %s",
                    c_call_setup_str[param->call_setup.status]);
            break;
        }

        case ESP_HF_CLIENT_CIND_CALL_HELD_EVT:
        {
            ESP_LOGI(BT_TAG, "--Call held indicator %s",
                    c_call_held_str[param->call_held.status]);
            break;
        }

        case ESP_HF_CLIENT_BTRH_EVT:
        {
            ESP_LOGI(BT_TAG, "--response and hold %s",
                    c_resp_and_hold_str[param->btrh.status]);
            break;
        }

        case ESP_HF_CLIENT_CLIP_EVT:
        {
            ESP_LOGI(BT_TAG, "--clip number %s",
                    (param->clip.number == NULL) ? "NULL" : (param->clip.number));
            break;
        }

        case ESP_HF_CLIENT_CCWA_EVT:
        {
            ESP_LOGI(BT_TAG, "--call_waiting %s",
                    (param->ccwa.number == NULL) ? "NULL" : (param->ccwa.number));
            break;
        }

        case ESP_HF_CLIENT_CLCC_EVT:
        {
            ESP_LOGI(BT_TAG, "--Current call: idx %d, dir %s, state %s, mpty %s, number %s",
                    param->clcc.idx,
                    c_call_dir_str[param->clcc.dir],
                    c_call_state_str[param->clcc.status],
                    c_call_mpty_type_str[param->clcc.mpty],
                    (param->clcc.number == NULL) ? "NULL" : (param->clcc.number));
            break;
        }

        case ESP_HF_CLIENT_VOLUME_CONTROL_EVT:
        {
            ESP_LOGI(BT_TAG, "--volume_target: %s, volume %d",
                    c_volume_control_target_str[param->volume_control.type],
                    param->volume_control.volume);
            break;
        }

        case ESP_HF_CLIENT_AT_RESPONSE_EVT:
        {
            ESP_LOGI(BT_TAG, "--AT response event, code %d, cme %d",
                    param->at_response.code, param->at_response.cme);
            break;
        }

        case ESP_HF_CLIENT_CNUM_EVT:
        {
            ESP_LOGI(BT_TAG, "--subscriber type %s, number %s",
                    c_subscriber_service_type_str[param->cnum.type],
                    (param->cnum.number == NULL) ? "NULL" : param->cnum.number);
            break;
        }

        case ESP_HF_CLIENT_BSIR_EVT:
        {
            ESP_LOGI(BT_TAG, "--inband ring state %s",
                    c_inband_ring_state_str[param->bsir.state]);
            break;
        }

        case ESP_HF_CLIENT_BINP_EVT:
        {
            ESP_LOGI(BT_TAG, "--last voice tag number: %s",
                    (param->binp.number == NULL) ? "NULL" : param->binp.number);
            break;
        }

        default:
            ESP_LOGE(BT_TAG, "HF_CLIENT EVT: %d", event);
            break;

        
    }
}

static void bt_init_stack() {
    // Init HFP stack
    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(BT_DEVICE_NAME));
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(esp_bt_gap_callback));
    ESP_ERROR_CHECK(esp_hf_client_register_callback(esp_bt_hf_client_callback));
    ESP_ERROR_CHECK(esp_hf_client_init());
    ESP_ERROR_CHECK(esp_hf_client_register_data_callback(bt_hf_incoming_data_callback, bt_hf_outgoing_data_callback));
    
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
        
        ESP_LOGI(BT_TAG, "Received message %d", msg.ev);
    }
}

void bt_task_send(bt_event_type_t ev, void *data, size_t data_size) {
    bt_msg_t msg = bt_create_msg(ev, data, data_size);
    xQueueSend(bt_msg_queue, &msg, BT_QUEUE_ADD_MAX_WAIT_TICKS / portTICK_PERIOD_MS);
}

static void debug(void *arg) {
    while (1) {
        vTaskDelay(200);
        ESP_LOGI(BT_TAG, "Receive interval: %"PRId64" Send interval: %"PRId64" Ready interval: %"PRId64, receive_interval_us, send_interval_us, audio_ready_interval_us);
    }
}

void bt_init(QueueHandle_t outgoing_msg_queue) {
    BaseType_t r;
    bt_outgoing_msg_queue = outgoing_msg_queue;
    bt_msg_queue = xQueueCreate(BT_QUEUE_MAX_SIZE, sizeof(bt_msg_t));
    assert(bt_msg_queue != NULL);
    
    bt_audio_data_available_queue = xQueueCreate(8, sizeof(uint8_t));
    assert(bt_audio_data_available_queue != NULL);
    
    xTaskCreatePinnedToCore(debug, "audio_debug", 2048, NULL, 1, NULL, 1);

    r = xTaskCreatePinnedToCore(bt_signal_audio_ready_handler, "bt_signal_audio_rd", 4096, NULL, 5, NULL, 1);
    assert(r == pdPASS);
    ESP_LOGI(BT_TAG, "Created audio handler task");

    r = xTaskCreatePinnedToCore(bt_msg_handler, "bt_msg_handler", 8192, NULL, 5, NULL, 1);
    assert(r == pdPASS);
    ESP_LOGI(BT_TAG, "Created bluetooth task");
    
    bt_task_send(BT_EV_INITIALIZE, NULL, 0);
    ESP_LOGD(BT_TAG, "Sent 'initialize' event to task");
}
