#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "audio.h"

#define TAG "Audio"

#define I2C_PORT 0
#define I2C_SLAVE_ADDRESS 32

#define AUDIO_SDA_PIN GPIO_NUM_26
#define AUDIO_SCL_PIN GPIO_NUM_14

#define AUDIO_16KHZ_SAMPLES_SIZE 240
#define AUDIO_8KHZ_SAMPLES_SIZE 120

#define CMD_AUDIO_ENABLE 1
#define CMD_AUDIO_TRANSMIT 2
#define CMD_AUDIO_RECEIVE 6
#define CMD_AUDIO_DISABLE 7

static sample_rate_t audio_sample_rate;
static QueueHandle_t audio_ready_queue = NULL;

static QueueHandle_t cmd_queue = NULL;

static uint8_t *audio_in_buf = NULL; // buffer holding ADC data
static uint8_t *audio_out_buf = NULL; // buffer holding ADC data
static uint8_t *audio_out_resample_buf = NULL; // buffer holding ADC data
static RingbufHandle_t audio_in_rb = NULL;
static RingbufHandle_t audio_out_rb = NULL;

uint8_t *i2c_tx_buffer = NULL;
size_t i2c_tx_buffer_size = 0;

static uint8_t initialized = 0;
static uint8_t enabled = 0;

static uint8_t signal_flag1 = 1;
static uint8_t cmd = 0;

static inline size_t audio_get_buffer_size() {
    if (audio_sample_rate == SAMPLE_RATE_16KHZ) {
        return AUDIO_16KHZ_SAMPLES_SIZE;
    }
    if (audio_sample_rate == SAMPLE_RATE_8KHZ) {
        return AUDIO_8KHZ_SAMPLES_SIZE;
    }
    return 0;
}

static inline size_t audio_get_rb_size() {
    return 4 * audio_get_buffer_size();
}


static void IRAM_ATTR audio_data_available_isr(void* arg) {
    QueueHandle_t q = (QueueHandle_t) arg;
    xQueueSendFromISR(q, &signal_flag1, NULL);
}

static void cmd_task_handler(void *arg) {
    QueueHandle_t q = (QueueHandle_t) arg;
    size_t buf_size = audio_get_buffer_size();
    size_t received = 0;
    uint16_t i2c_cmd = 0;
    uint16_t i2c_cmd_reply = 0;
    uint8_t *tmp_buf = NULL;

    while (1) {
        if (xQueueReceive(q, &cmd, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }

        buf_size = audio_get_buffer_size();
        ESP_LOGW(TAG, "Command is: %d", cmd);
        
        if (cmd == CMD_AUDIO_ENABLE) {
            ESP_LOGW(TAG, "Setting sample rate to: %d", audio_sample_rate);
            i2c_cmd = (CMD_AUDIO_ENABLE << 8) | audio_sample_rate;
            ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_PORT, I2C_SLAVE_ADDRESS, (uint8_t *) &i2c_cmd, 2, (uint8_t *) &i2c_cmd_reply, 2, portMAX_DELAY));
            ESP_LOGI(TAG, "Configured audio device: %d", i2c_cmd_reply);
            assert(i2c_cmd_reply == 1);
            i2c_cmd_reply = 0;
            continue;
        }

        if (cmd == CMD_AUDIO_DISABLE) {
            ESP_LOGW(TAG, "Disabling audio");
            i2c_cmd = CMD_AUDIO_DISABLE << 8;
            ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_PORT, I2C_SLAVE_ADDRESS, (uint8_t *) &i2c_cmd, 2, (uint8_t *) &i2c_cmd_reply, 2, portMAX_DELAY));
            assert(i2c_cmd_reply == 1);
            i2c_cmd_reply = 0;
            continue;
        }
        
        // use write_read to receive adc data

        // if (cmd == CMD_AUDIO_TRANSMIT) {
        //     // Write audio data to DAC
        //     tmp_buf = xRingbufferReceiveUpTo(audio_out_rb, &received, 0, buf_size);
        //     if (tmp_buf == NULL) {
        //         goto receive;
        //     }

        //     if (received && (received != buf_size)) {
        //         received = 0;
        //         vRingbufferReturnItem(audio_out_rb, tmp_buf);
        //         goto receive;
        //     }
            
        //     memcpy(audio_out_buf, tmp_buf, received);
        //     received = 0;
        //     vRingbufferReturnItem(audio_out_rb, tmp_buf);

                // TODO: write command and data in the same transaction
        //     i2c_cmd = CMD_AUDIO_TRANSMIT << 8;
        //     ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, I2C_SLAVE_ADDRESS, (uint8_t *) &i2c_cmd, 2, portMAX_DELAY));
        //     i2c_master_write_to_device(I2C_PORT, I2C_SLAVE_ADDRESS, audio_out_buf, buf_size, portMAX_DELAY);
            
        // }

// receive:
            
//         i2c_cmd = CMD_AUDIO_RECEIVE << 8;
//         ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, I2C_SLAVE_ADDRESS, (uint8_t *) &i2c_cmd, 2, portMAX_DELAY));
//         if (i2c_master_read_from_device(I2C_PORT, I2C_SLAVE_ADDRESS, audio_in_buf, buf_size, portMAX_DELAY) == ESP_OK) {
//             if (xRingbufferSend(audio_in_rb, audio_in_buf, buf_size, 0) == pdTRUE) {
//                 xQueueSend(audio_ready_queue, &signal_flag1, 0);
//             }
//         }
    }
}

void audio_receive(uint8_t *dst, size_t *received, size_t requested_size) {
    if (!initialized || !enabled) {
        *received = 0;
        return;
    }
    
    uint8_t *data = xRingbufferReceiveUpTo(audio_in_rb, received, 0, requested_size);
    
    if (*received == requested_size) {
        memcpy(dst, data, *received);
        vRingbufferReturnItem(audio_in_rb, data);
        return;
    }
    
    if (*received) {
        vRingbufferReturnItem(audio_in_rb, data);
    }
    
    return;
}

void audio_send(const uint8_t *buf, size_t size) {
    if (!initialized || !enabled) {
        return;
    }
    
    int16_t *src = (int16_t *) buf;
    uint16_t *dst = (uint16_t *) audio_out_resample_buf;
    
    uint8_t sample_count = size / 2;
    
    for (int i = 0; i < sample_count; i++) {
        dst[i] = ((int16_t)(src[i] * 0.3) - INT16_MIN) >> 4;
    }
    
    xRingbufferSend(audio_out_rb, audio_out_resample_buf, size, 0);
    memset(audio_out_resample_buf, '\0', size);
    xQueueSend(cmd_queue, CMD_AUDIO_TRANSMIT, 0);
}

sample_rate_t audio_get_sample_rate() {
    return audio_sample_rate;
}

void audio_enable(uint8_t status) {
    if (!initialized && status) {
        return;
    }

    if (status == enabled) {
        return;
    }
    
    uint8_t cmd;
    
    if (status) {
        ESP_LOGW(TAG, "Enabling audio");
        cmd = CMD_AUDIO_ENABLE;
        xQueueSend(cmd_queue, &cmd, portMAX_DELAY);
        enabled = 1;
        return;
    }

    ESP_LOGW(TAG, "Disabling audio");
    cmd = CMD_AUDIO_DISABLE;
    xQueueSend(cmd_queue, &cmd, portMAX_DELAY);
    enabled = 0;
}

void audio_init_transport() {
    ESP_LOGI(TAG, "Initializing audio I2C transport");

    cmd_queue = xQueueCreate(8, sizeof(uint8_t));
    assert(cmd_queue != NULL);

    BaseType_t r = xTaskCreatePinnedToCore(cmd_task_handler, "i2c_data_task", 4048, cmd_queue, configMAX_PRIORITIES - 3, NULL, 1);
    assert(r == pdPASS);

    i2c_port_t i2c_port = I2C_PORT;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = AUDIO_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = AUDIO_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1 * 1000 * 1000,
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, conf.mode, 0, 0, 0));
    ESP_ERROR_CHECK(i2c_set_timeout(I2C_PORT, 1600)); // PICO is slower, increase timeout
}

void audio_init(sample_rate_t sample_rate, QueueHandle_t audio_ready_q) {
    if (initialized) {
        return;
    }
    assert(sample_rate != SAMPLE_RATE_NONE);
    audio_sample_rate = sample_rate;
    audio_ready_queue = audio_ready_q;

    size_t audio_buf_size = audio_get_buffer_size();
    uint16_t sample_rate_hz = (sample_rate == SAMPLE_RATE_16KHZ) ? 16000 : 8000;
    ESP_LOGI(TAG, "Initializing Audio. Sample rate: %d. Buffer size: %d", sample_rate_hz, audio_buf_size);

    audio_in_buf = malloc(audio_buf_size);
    assert(audio_in_buf != NULL);
    memset(audio_in_buf, '\0', audio_buf_size);

    audio_out_buf = malloc(audio_buf_size);
    assert(audio_out_buf != NULL);
    memset(audio_out_buf, '\0', audio_buf_size);

    audio_out_resample_buf = malloc(audio_buf_size);
    assert(audio_out_resample_buf != NULL);
    memset(audio_out_resample_buf, '\0', audio_buf_size);
    
    size_t audio_rb_size = audio_get_rb_size();
    
    audio_in_rb = xRingbufferCreate(audio_rb_size, RINGBUF_TYPE_BYTEBUF);
    assert(audio_in_rb != NULL);

    audio_out_rb = xRingbufferCreate(audio_rb_size, RINGBUF_TYPE_BYTEBUF);
    assert(audio_out_rb != NULL);
    
    i2c_tx_buffer_size = I2C_LINK_RECOMMENDED_SIZE(audio_buf_size / 2);
    i2c_tx_buffer = malloc(i2c_tx_buffer_size);
    assert(i2c_tx_buffer);

    initialized = 1;
}

void audio_deinit() {
    if (!initialized) {
        return;
    }
    
    audio_enable(0);
    
    audio_sample_rate = SAMPLE_RATE_NONE;

    vRingbufferDelete(audio_in_rb);
    audio_in_rb = NULL;

    vRingbufferDelete(audio_out_rb);
    audio_out_rb = NULL;
    
    free(audio_in_buf);
    audio_in_buf = NULL;

    free(audio_out_buf);
    audio_out_buf = NULL;

    free(audio_out_resample_buf);
    audio_out_resample_buf = NULL;

    free(i2c_tx_buffer);
    i2c_tx_buffer = NULL;
    i2c_tx_buffer_size = 0;

    initialized = 0;
    audio_ready_queue = NULL;

    ESP_LOGI(TAG, "De-initialized audio");
}
