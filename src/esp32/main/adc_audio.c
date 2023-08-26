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
#include "adc_audio.h"
#include "dac_audio.h"

#define AD_TAG "AD"

#define I2C_PORT 0

#define ADC_ENABLE_PIN GPIO_NUM_13
#define ADC_DATA_READY_PIN GPIO_NUM_25
#define ADC_MOSI_PIN GPIO_NUM_26
#define ADC_MISO_PIN GPIO_NUM_27
#define ADC_CLK_PIN GPIO_NUM_12
#define ADC_CS_PIN GPIO_NUM_14
#define ADC_SPI_QUEUE_SIZE 4 * 120
#define ADC_SPI_CLOCK_SPEED_HZ 2500000

#define ADC_SDA_PIN GPIO_NUM_26
#define ADC_SCL_PIN GPIO_NUM_14

#define ADC_16KHZ_SAMPLES_NUM 240
#define ADC_8KHZ_SAMPLES_NUM 120


static QueueHandle_t audio_ready_queue = NULL;
static uint8_t adc_configured = 0;

static DRAM_ATTR uint8_t *audio_in_buf = NULL; // buffer holding ADC data
static DRAM_ATTR int16_t *samples_buf = NULL;
static sample_rate_t adc_sample_rate;
static DRAM_ATTR RingbufHandle_t audio_in_rb = NULL;
static uint8_t initialized = 0;
static uint8_t enabled = 0;

static int64_t transmission_interval_us = 0;
static int64_t last_transmission_us = 0;
static int64_t now_us = 0;
static int64_t start_us = 0;
static int64_t duration_us = 0;

static uint8_t signal_flag1 = 1;
static uint8_t signal_flag2 = 1;

static inline size_t adc_audio_get_buffer_size();

static void IRAM_ATTR adc_data_available_isr(void* arg) {
    QueueHandle_t q = (QueueHandle_t) arg;

    now_us = esp_timer_get_time();
    transmission_interval_us = now_us - last_transmission_us;
    last_transmission_us = now_us;
    xQueueSendFromISR(q, &signal_flag1, NULL);
}

static void adc_read_i2c_data_task_handler(void *arg) {
    QueueHandle_t q = (QueueHandle_t) arg;
    int16_t notified = 0;
    size_t buf_size = adc_audio_get_buffer_size();

    while (1) {
        if (xQueueReceive(q, &signal_flag2, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }
        buf_size = adc_audio_get_buffer_size();
        
        if (!adc_configured) {
            ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_PORT, 32, (uint8_t *) &adc_sample_rate, 1, portMAX_DELAY));
            adc_configured = 1;
            ESP_LOGI(AD_TAG, "Configured ADC");
            continue;
        }
        
        // start_us = esp_timer_get_time();
        
        if (i2c_master_read_from_device(I2C_PORT, 32, audio_in_buf, buf_size, portMAX_DELAY) != ESP_OK) {
            continue;
        }

        if (xRingbufferSend(audio_in_rb, audio_in_buf, buf_size, 0) == pdTRUE) {
            xQueueSend(audio_ready_queue, &signal_flag1, 0);
        }

        // now_us = esp_timer_get_time();
        // duration_us = now_us - start_us;
        // ESP_LOGW(AD_TAG, "Duration %lld", duration_us);
    }
}

static inline size_t adc_audio_get_buffer_size() {
    if (adc_sample_rate == SAMPLE_RATE_16KHZ) {
        return ADC_16KHZ_SAMPLES_NUM;
    }
    if (adc_sample_rate == SAMPLE_RATE_8KHZ) {
        return ADC_8KHZ_SAMPLES_NUM;
    }
    return 0;
}

static inline size_t adc_audio_get_rb_size() {
    return 4 * adc_audio_get_buffer_size();
}

void adc_audio_receive(uint8_t *dst, size_t *received, size_t requested_size) {
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

void adc_audio_enable(uint8_t status) {
    if (!initialized && status) {
        return;
    }

    if (status == enabled) {
        return;
    }
    
    if (status) {
        ESP_LOGW(AD_TAG, "Enabling ADC");
        gpio_set_level(ADC_ENABLE_PIN, 1);
        enabled = 1;
        return;
    }

    ESP_LOGW(AD_TAG, "Disabling ADC");
    gpio_set_level(ADC_ENABLE_PIN, 0);
    enabled = 0;
}

void adc_audio_init_transport() {
    ESP_LOGI(AD_TAG, "Initializing ADC I2C transport");

    QueueHandle_t adc_read_notif_queue = xQueueCreate(8, sizeof(uint8_t));
    assert(adc_read_notif_queue != NULL);

    BaseType_t r = xTaskCreatePinnedToCore(adc_read_i2c_data_task_handler, "i2c_data_task", 4092, adc_read_notif_queue, 5, NULL, 1);
    assert(r == pdPASS);

    // Configure enable pin
    gpio_config_t output_conf = {};
    output_conf.intr_type = GPIO_INTR_DISABLE;
    output_conf.mode = GPIO_MODE_OUTPUT;
    output_conf.pin_bit_mask = 1ULL << ADC_ENABLE_PIN;
    ESP_ERROR_CHECK(gpio_config(&output_conf));

    // Configure data ready pin
    gpio_config_t input_conf = {};
    input_conf.intr_type = GPIO_INTR_POSEDGE;
    input_conf.mode = GPIO_MODE_INPUT;
    input_conf.pin_bit_mask = 1ULL << ADC_DATA_READY_PIN;
    input_conf.pull_up_en = 0;
    input_conf.pull_down_en = 1;
    ESP_ERROR_CHECK(gpio_config(&input_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ADC_DATA_READY_PIN, adc_data_available_isr, adc_read_notif_queue));
    
    i2c_port_t i2c_port = I2C_PORT;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = ADC_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = ADC_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1 * 1000 * 1000,
        // .master.clk_speed = 100000,
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_port, conf.mode, 0, 0, 0));
}

void adc_audio_init(sample_rate_t sample_rate, QueueHandle_t audio_ready_q) {
    if (initialized) {
        return;
    }
    assert(sample_rate != SAMPLE_RATE_NONE);
    adc_sample_rate = sample_rate;
    audio_ready_queue = audio_ready_q;

    size_t adc_buf_size = adc_audio_get_buffer_size();
    uint16_t sample_rate_hz = (sample_rate == SAMPLE_RATE_16KHZ) ? 16000 : 8000;
    ESP_LOGI(AD_TAG, "Initializing ADC. Sample rate: %d. Buffer size: %d", sample_rate_hz, adc_buf_size);

    audio_in_buf = malloc(adc_buf_size);
    assert(audio_in_buf != NULL);
    memset(audio_in_buf, '\0', adc_buf_size);
    
    audio_in_rb = xRingbufferCreate(adc_audio_get_rb_size(), RINGBUF_TYPE_BYTEBUF);
    assert(audio_in_rb != NULL);
    initialized = 1;
}

void adc_audio_deinit() {
    if (!initialized) {
        return;
    }
    
    adc_audio_enable(0);
    
    adc_sample_rate = SAMPLE_RATE_NONE;

    vRingbufferDelete(audio_in_rb);
    audio_in_rb = NULL;
    
    free(audio_in_buf);
    audio_in_buf = NULL;

    adc_configured = 0;
    initialized = 0;
    audio_ready_queue = NULL;

    ESP_LOGI(AD_TAG, "De-initialized ADC");
}
