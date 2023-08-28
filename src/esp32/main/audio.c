#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "audio.h"

#define TAG "Audio"

#define I2C_PORT 0
#define I2C_SLAVE_ADDRESS 32

#define AUDIO_ENABLE_PIN GPIO_NUM_13
#define AUDIO_DATA_READY_PIN GPIO_NUM_25

#define AUDIO_SDA_PIN GPIO_NUM_26
#define AUDIO_SCL_PIN GPIO_NUM_14

#define AUDIO_16KHZ_SAMPLES_SIZE 240
#define AUDIO_8KHZ_SAMPLES_SIZE 120


static sample_rate_t audio_sample_rate;
static QueueHandle_t audio_ready_queue = NULL;

static uint8_t *audio_in_buf = NULL; // buffer holding ADC data
static uint8_t *audio_out_buf = NULL; // buffer holding ADC data
static uint8_t *audio_out_resample_buf = NULL; // buffer holding ADC data
static RingbufHandle_t audio_in_rb = NULL;
static RingbufHandle_t audio_out_rb = NULL;

uint8_t *i2c_tx_buffer = NULL;
size_t i2c_tx_buffer_size = 0;

static uint8_t initialized = 0;
static uint8_t configured = 0;
static uint8_t enabled = 0;

static uint8_t signal_flag1 = 1;
static uint8_t signal_flag2 = 1;

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

static void audio_i2c_data_task_handler(void *arg) {
    QueueHandle_t q = (QueueHandle_t) arg;
    size_t buf_size = audio_get_buffer_size();
    size_t received = 0;
    uint8_t *tmp_buf = NULL;
    i2c_cmd_handle_t i2c_cmd_handle;
    esp_err_t err = ESP_OK;

    while (1) {
        if (!enabled) {
            received = 0;
            vTaskDelay(1);
            continue;
        }

        if (xQueueReceive(q, &signal_flag2, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }
        buf_size = audio_get_buffer_size();
        
        if (!configured) {
            ESP_LOGW(TAG, "Setting sample rate to: %d", audio_sample_rate);
            ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, I2C_SLAVE_ADDRESS, (uint8_t *) &audio_sample_rate, 1, portMAX_DELAY));
            configured = 1;
            ESP_LOGI(TAG, "Configured audio device");
            continue;
        }
        
        // Get audio data from ADC
        if (i2c_master_read_from_device(I2C_PORT, I2C_SLAVE_ADDRESS, audio_in_buf, buf_size, portMAX_DELAY) == ESP_OK) {
            if (xRingbufferSend(audio_in_rb, audio_in_buf, buf_size, 0) == pdTRUE) {
                xQueueSend(audio_ready_queue, &signal_flag1, 0);
            }
        }
        
        // Write audio data to DAC
        tmp_buf = xRingbufferReceiveUpTo(audio_out_rb, &received, 0, buf_size);
        if (tmp_buf == NULL) {
            continue;
        }

        if (received && (received != buf_size)) {
            received = 0;
            vRingbufferReturnItem(audio_out_rb, tmp_buf);
            continue;
        }
        
        memcpy(audio_out_buf, tmp_buf, received);
        received = 0;
        vRingbufferReturnItem(audio_out_rb, tmp_buf);
        i2c_master_write_to_device(I2C_PORT, 32, audio_out_buf, buf_size, portMAX_DELAY);
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
    
    if (status) {
        ESP_LOGW(TAG, "Enabling audio");
        enabled = 1;
        gpio_set_level(AUDIO_ENABLE_PIN, 1);
        return;
    }

    ESP_LOGW(TAG, "Disabling audio");
    gpio_set_level(AUDIO_ENABLE_PIN, 0);
    i2c_reset_tx_fifo(I2C_PORT);
    i2c_reset_rx_fifo(I2C_PORT);
    enabled = 0;
    configured = 0;
}

void audio_init_transport() {
    ESP_LOGI(TAG, "Initializing audio I2C transport");

    QueueHandle_t audio_read_notif_queue = xQueueCreate(8, sizeof(uint8_t));
    assert(audio_read_notif_queue != NULL);

    BaseType_t r = xTaskCreatePinnedToCore(audio_i2c_data_task_handler, "i2c_data_task", 4048, audio_read_notif_queue, configMAX_PRIORITIES - 3, NULL, 1);
    assert(r == pdPASS);

    // Configure enable pin
    gpio_config_t output_conf = {};
    output_conf.intr_type = GPIO_INTR_DISABLE;
    output_conf.mode = GPIO_MODE_OUTPUT;
    output_conf.pin_bit_mask = 1ULL << AUDIO_ENABLE_PIN;
    ESP_ERROR_CHECK(gpio_config(&output_conf));

    // Configure data ready pin
    gpio_config_t input_conf = {};
    input_conf.intr_type = GPIO_INTR_POSEDGE;
    input_conf.mode = GPIO_MODE_INPUT;
    input_conf.pin_bit_mask = 1ULL << AUDIO_DATA_READY_PIN;
    input_conf.pull_up_en = 0;
    input_conf.pull_down_en = 1;
    ESP_ERROR_CHECK(gpio_config(&input_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(AUDIO_DATA_READY_PIN, audio_data_available_isr, audio_read_notif_queue));
    
    i2c_port_t i2c_port = I2C_PORT;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = AUDIO_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = AUDIO_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1 * 1000 * 1000,
        // .master.clk_speed = 100000,
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, conf.mode, 0, 0, 0));
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

    configured = 0;
    initialized = 0;
    audio_ready_queue = NULL;

    ESP_LOGI(TAG, "De-initialized audio");
}
