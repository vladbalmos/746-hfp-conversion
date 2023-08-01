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
#include "dac_audio.h"
#include "bt.h"
#include "dialer.h"
#include "ringer.h"

#define TAG "MAIN"

#define ESP_INTR_FLAG_DEFAULT 0
#define DIALER_PULSE_PIN 16
#define HOOK_SWITCH_PIN 17
#define RINGER_SIGNAL_PIN 5
#define RINGER_ENABLE_PIN 18

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
        bt_task_send(BT_EV_DIAL_NUMBER, number, (size_t) number_length);
        ESP_LOGI(TAG, "Dialing number: %s. Number length: %d", number, number_length);
    }
}

#define PI 3.14159265
#define SINE_WAVE_FREQ_HZ 400
#define SINE_MAX_VAL (2 << 15) - 1
#define SAMPLE_RATE 16000

// uint16_t utils_generate_sine_wave(uint16_t frequency, int16_t *buffer, uint sample_rate, int16_t max_sine_value) {
//     const uint16_t samples = sample_rate / frequency;
    
//     for (uint16_t i = 0; i < samples; i++) {
//         double angle = (double)i / samples * 2.0 * PI;
//         double sine_val = sin(angle);
//         buffer[i] = round(sine_val * max_sine_value);
//     }
//     return samples;
// }

static bool IRAM_ATTR  dac_on_convert_done_callback(dac_continuous_handle_t handle, const dac_event_data_t *event, void *user_data)
{
    QueueHandle_t que = (QueueHandle_t)user_data;
    BaseType_t need_awoke;
    /* When the queue is full, drop the oldest item */
    if (xQueueIsQueueFullFromISR(que)) {
        dac_event_data_t dummy;
        xQueueReceiveFromISR(que, &dummy, &need_awoke);
    }
    /* Send the event from callback */
    xQueueSendFromISR(que, event, &need_awoke);
    return need_awoke;
}

uint16_t utils_generate_sine_wave(uint16_t frequency, uint16_t *buffer, uint16_t sample_rate) {
    const uint16_t samples = sample_rate / frequency;
    
    for (uint16_t i = 0; i < samples; i++) {
        double angle = (double)i / (double)samples * 2.0 * PI;
        double sine_val = sin(angle);
        int val = round((sine_val + 1.0) / 2.0 * 65535);
        if (val < 0) {
            val = 0;
        }
        buffer[i] = (uint16_t) val;
    }
    return samples;
}

static uint8_t *buff = NULL;
static uint8_t big_buff[240];

void app_main(void) {
    // Initialize phone interface
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    
    dialer_init(DIALER_PULSE_PIN, HOOK_SWITCH_PIN, on_headset_state_change, on_start_dialing, on_digit, on_end_dialing);
    dialer_enable(1);
    
    ringer_init(RINGER_ENABLE_PIN, RINGER_SIGNAL_PIN);
    ESP_LOGI(TAG, "Headset state is %d", dialer_get_headset_state());

    // int max_samples = SAMPLE_RATE / SINE_WAVE_FREQ_HZ + 1;
    // int buffer_size = max_samples * sizeof(int16_t);
    // buff = malloc(buffer_size);
    // assert(buff != NULL);
    // uint16_t samples = utils_generate_sine_wave(SINE_WAVE_FREQ_HZ, (uint16_t *)buff, SAMPLE_RATE);


    
    // printf("Total samples: %d\n", samples);
    // printf("Samples 16bit: \n");
    // uint16_t *samples_buf = (uint16_t *) buff;
    // uint8_t sine_buf[samples];

    // for (int i = 0; i < samples; i++) {
    //     sine_buf[i] = (uint8_t) (samples_buf[i] >> 8);
    //     printf("% 5d ", samples_buf[i]);
        
    //     if (i && i % 15 == 0) {
    //         printf("\n");
    //     }
    // }
    // printf("\n");

    // for (int i = 0; i < 7; i++) {
    //     for (int j = 0; j < 40; j++) {
    //         int index = (40 * i) + j;
    //         // printf("%d\n", index);
    //         big_buff[index] = sine_buf[j];
    //     }
    // }

    // printf("Samples 8bit: \n");
    // for (int i = 0; i < 224; i++) {
    //     printf("% 5d ", big_buff[i]);
        
    //     if (i && i % 15 == 0) {
    //         printf("\n");
    //     }
    // }
    // printf("\n");

    
    // dac_continuous_handle_t dac_handle;
    // dac_continuous_config_t dac_cfg = {
    //     .chan_mask = DAC_CHANNEL_MASK_CH0,
    //     .desc_num = 4,
    //     .buf_size = 240,
    //     .freq_hz = SAMPLE_RATE,
    //     .clk_src = DAC_DIGI_CLK_SRC_APLL,
    // };
    // ESP_ERROR_CHECK(dac_continuous_new_channels(&dac_cfg, &dac_handle));
    
    // QueueHandle_t q = xQueueCreate(10, sizeof(dac_event_data_t));
    
    // dac_event_callbacks_t cbs = {
    //     .on_convert_done = dac_on_convert_done_callback,
    //     .on_stop = NULL,
    // };
    // ESP_ERROR_CHECK(dac_continuous_register_event_callback(dac_handle, &cbs, q));
    // ESP_ERROR_CHECK(dac_continuous_enable(dac_handle));
    // ESP_ERROR_CHECK(dac_continuous_start_async_writing(dac_handle));
    
    // while (1) {
    //     dac_event_data_t evt_data;
    //     size_t bytes_written = 0;

    //     while (bytes_written < 240) {
    //         if (xQueueReceive(q, &evt_data, portMAX_DELAY) != pdTRUE) {
    //             break;
    //         }
    //         size_t loaded_bytes = 0;
    //         ESP_ERROR_CHECK(
    //             dac_continuous_write_asynchronously(dac_handle, evt_data.buf, evt_data.buf_size,
    //                                                 big_buff + bytes_written, samples - bytes_written,
    //                                                 &loaded_bytes)
    //         );
    //         bytes_written += loaded_bytes;
    //         // ESP_LOGI(TAG, "%d %d", evt_data.buf_size, samples);
    //     }
    //     vTaskDelay(0);
    // }
    
    
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
    
    QueueHandle_t bt_msg_queue = xQueueCreate(10, sizeof(bt_msg_t));
    assert(bt_msg_queue != NULL);

    bt_init(bt_msg_queue);
    
    bt_msg_t msg;

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
