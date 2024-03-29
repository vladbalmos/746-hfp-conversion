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
#define AUDIO_SCL_PIN GPIO_NUM_27

#define AUDIO_16KHZ_SAMPLES_SIZE 240
#define AUDIO_8KHZ_SAMPLES_SIZE 120

#define AUDIO_HFP_MAX_VOLUME 14.0

#define AUDIO_POLL_ADC_INTERVAL_US 3000
#define AUDIO_BUFFERS_COUNT 8

#define CMD_AUDIO_ENABLE 1 // enable audio
#define CMD_AUDIO_TRANSMIT 2 // transmit PCM samples command
#define CMD_AUDIO_POLL 5 // poll Pico for more audio samples
#define CMD_AUDIO_RECEIVE 6 // receive PCM samples command
#define CMD_AUDIO_DISABLE 7 // disable audio

static sample_rate_t audio_sample_rate; // configured sample rate
static QueueHandle_t audio_ready_queue = NULL;

static QueueHandle_t cmd_queue = NULL;

static uint8_t *audio_in_buf = NULL; // buffer holding ADC data
static uint8_t *audio_out_buf = NULL; // buffer holding DAC data
static uint8_t *audio_out_resample_buf = NULL; // buffer holding ADC data
static RingbufHandle_t audio_in_rb = NULL;
static RingbufHandle_t audio_out_rb = NULL;
static float volume = 1.0; // max volume

static esp_timer_handle_t poll_adc_timer;

static uint8_t initialized = 0;
static uint8_t enabled = 0;

static uint64_t buffered_samples_count = 0;
static uint8_t cmd = 0;

static inline size_t get_buffer_size() {
    if (audio_sample_rate == SAMPLE_RATE_16KHZ) {
        return AUDIO_16KHZ_SAMPLES_SIZE;
    }
    if (audio_sample_rate == SAMPLE_RATE_8KHZ) {
        return AUDIO_8KHZ_SAMPLES_SIZE;
    }
    return 0;
}

static inline size_t get_rb_size() {
    return AUDIO_BUFFERS_COUNT * get_buffer_size();
}


static void IRAM_ATTR poll_adc_timer_callback(void *arg) {
    QueueHandle_t q = (QueueHandle_t) arg;
    uint8_t cmd = CMD_AUDIO_POLL;

    xQueueSendFromISR(q, &cmd, NULL);
}

static inline void drain_ringbuffer(RingbufHandle_t rb, size_t max_size) {
    if (rb == NULL) {
        return;
    }

    size_t received = 0;
    do {
        uint8_t *data = xRingbufferReceive(rb, &received, 0);
        if (received) {
            vRingbufferReturnItem(rb, data);
            received = 0;
        }
    } while (received != 0);
}

static void drain_ringbuffers() {
    size_t rb_size = get_rb_size();
    drain_ringbuffer(audio_in_rb, rb_size);
    drain_ringbuffer(audio_out_rb, rb_size);
}

static void cmd_task_handler(void *arg) {
    QueueHandle_t q = (QueueHandle_t) arg;
    size_t buf_size = get_buffer_size();
    size_t received = 0;
    uint16_t i2c_cmd = 0;
    uint16_t i2c_cmd_reply = 0;
    uint8_t *pcm_out = NULL;

    while (1) {
        if (xQueueReceive(q, &cmd, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (cmd == CMD_AUDIO_ENABLE) {
            i2c_cmd = (CMD_AUDIO_ENABLE << 8) | audio_sample_rate;
            ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_PORT, I2C_SLAVE_ADDRESS, (uint8_t *) &i2c_cmd, 2, (uint8_t *) &i2c_cmd_reply, 2, portMAX_DELAY));
            assert(i2c_cmd_reply == 1);
            i2c_cmd_reply = 0;
            continue;
        }

        if (cmd == CMD_AUDIO_DISABLE) {
            i2c_cmd = CMD_AUDIO_DISABLE << 8;
            ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_PORT, I2C_SLAVE_ADDRESS, (uint8_t *) &i2c_cmd, 2, (uint8_t *) &i2c_cmd_reply, 2, portMAX_DELAY));
            assert(i2c_cmd_reply == 1);
            i2c_cmd_reply = 0;
            
            drain_ringbuffers();
            continue;
        }
        
        if (cmd == CMD_AUDIO_TRANSMIT) {
            pcm_out = xRingbufferReceiveUpTo(audio_out_rb, &received, 0, buf_size);
            if (pcm_out == NULL) {
                continue;
            }

            if (received && (received != buf_size)) {
                received = 0;
                vRingbufferReturnItem(audio_out_rb, pcm_out);
                continue;
            }

            i2c_cmd = CMD_AUDIO_TRANSMIT << 8;
            memcpy(audio_out_buf, &i2c_cmd, sizeof(i2c_cmd));
            memcpy(audio_out_buf + sizeof(i2c_cmd), pcm_out, received);
            received = 0;
            vRingbufferReturnItem(audio_out_rb, pcm_out);

            i2c_master_write_to_device(I2C_PORT, I2C_SLAVE_ADDRESS, audio_out_buf, buf_size + sizeof(i2c_cmd), portMAX_DELAY);
            continue;
        }

        if (cmd == CMD_AUDIO_POLL && enabled) {
            i2c_cmd = CMD_AUDIO_POLL << 8;
            if (i2c_master_write_read_device(I2C_PORT, I2C_SLAVE_ADDRESS, (uint8_t *) &i2c_cmd, 2, (uint8_t *) &i2c_cmd_reply, 2, portMAX_DELAY) != ESP_OK) {
                continue;
            }

            if (!i2c_cmd_reply) {
                continue;
            }
            i2c_cmd_reply = 0;

            buf_size = get_buffer_size();
            i2c_cmd = CMD_AUDIO_RECEIVE << 8;

            if (i2c_master_write_read_device(I2C_PORT, I2C_SLAVE_ADDRESS, (uint8_t *) &i2c_cmd, 2, audio_in_buf, buf_size, portMAX_DELAY) == ESP_OK) {
                if (xRingbufferSend(audio_in_rb, audio_in_buf, buf_size, 0) == pdTRUE) {
                    xQueueSend(audio_ready_queue, &cmd, 0);
                }
                memset(audio_in_buf, '\0', buf_size);
            }
        }
            
    }
}

void IRAM_ATTR audio_receive(uint8_t *dst, size_t *received, size_t requested_size) {
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

void IRAM_ATTR audio_send(const uint8_t *buf, size_t size) {
    if (!initialized || !enabled) {
        return;
    }
    
    static uint8_t send_threshold = AUDIO_BUFFERS_COUNT - 3;
    uint8_t cmd = CMD_AUDIO_TRANSMIT;
    int16_t *src = (int16_t *) buf;
    uint16_t *dst = (uint16_t *) audio_out_resample_buf;
    
    uint8_t sample_count = size / 2;
    
    for (int i = 0; i < sample_count; i++) {
        dst[i] = ((int16_t)(src[i] * volume) - INT16_MIN) >> 4;
    }
    
    xRingbufferSend(audio_out_rb, audio_out_resample_buf, size, 0);
    memset(audio_out_resample_buf, '\0', size);
    
    if (buffered_samples_count++ >= send_threshold) {
        xQueueSend(cmd_queue, &cmd, 0);
    }
}

void IRAM_ATTR audio_set_volume(uint8_t vol) {
    volume = vol / AUDIO_HFP_MAX_VOLUME;
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
        esp_timer_start_periodic(poll_adc_timer, AUDIO_POLL_ADC_INTERVAL_US);
        enabled = 1;
        return;
    }

    ESP_LOGW(TAG, "Disabling audio");
    esp_timer_stop(poll_adc_timer);
    cmd = CMD_AUDIO_DISABLE;
    xQueueSend(cmd_queue, &cmd, portMAX_DELAY);
    enabled = 0;
    buffered_samples_count = 0;
}

void audio_init_transport() {
    ESP_LOGI(TAG, "Initializing audio I2C transport");

    cmd_queue = xQueueCreate(8, sizeof(uint8_t));
    assert(cmd_queue != NULL);

    BaseType_t r = xTaskCreatePinnedToCore(cmd_task_handler, "cmd_data_task", 4048, cmd_queue, configMAX_PRIORITIES - 3, NULL, 1);
    assert(r == pdPASS);

    esp_timer_create_args_t poll_timer_args = {
        .callback = &poll_adc_timer_callback,
        .arg = cmd_queue,
        .name = "poll-adc-timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&poll_timer_args, &poll_adc_timer));

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
    ESP_ERROR_CHECK(i2c_set_timeout(I2C_PORT, 10 * 800)); // PICO is slower, increase timeout
}

void audio_init(sample_rate_t sample_rate, QueueHandle_t audio_ready_q) {
    if (initialized) {
        return;
    }
    assert(sample_rate != SAMPLE_RATE_NONE);
    audio_sample_rate = sample_rate;
    audio_ready_queue = audio_ready_q;

    size_t audio_buf_size = get_buffer_size();
    uint16_t sample_rate_hz = (sample_rate == SAMPLE_RATE_16KHZ) ? 16000 : 8000;
    ESP_LOGI(TAG, "Initializing Audio. Sample rate: %d. Buffer size: %d", sample_rate_hz, audio_buf_size);

    audio_in_buf = malloc(audio_buf_size);
    assert(audio_in_buf != NULL);
    memset(audio_in_buf, '\0', audio_buf_size);

    // [transmit command: 2bytes + audio data: {audio_buf_size} bytes]
    audio_out_buf = malloc(audio_buf_size + sizeof(uint16_t));
    assert(audio_out_buf != NULL);
    memset(audio_out_buf, '\0', audio_buf_size);

    audio_out_resample_buf = malloc(audio_buf_size);
    assert(audio_out_resample_buf != NULL);
    memset(audio_out_resample_buf, '\0', audio_buf_size);
    
    size_t audio_rb_size = get_rb_size();
    
    audio_in_rb = xRingbufferCreate(audio_rb_size, RINGBUF_TYPE_BYTEBUF);
    assert(audio_in_rb != NULL);

    audio_out_rb = xRingbufferCreate(audio_rb_size, RINGBUF_TYPE_BYTEBUF);
    assert(audio_out_rb != NULL);
    
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

    initialized = 0;
    audio_ready_queue = NULL;

    ESP_LOGI(TAG, "De-initialized audio");
}
