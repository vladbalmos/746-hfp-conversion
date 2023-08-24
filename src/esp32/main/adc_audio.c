#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "adc_audio.h"

#define AD_TAG "AD"

#define ADC_ENABLE_PIN GPIO_NUM_13
#define ADC_DATA_READY_PIN GPIO_NUM_25
#define ADC_MOSI_PIN GPIO_NUM_26
#define ADC_MISO_PIN GPIO_NUM_27
#define ADC_CLK_PIN GPIO_NUM_12
#define ADC_CS_PIN GPIO_NUM_14
#define ADC_SPI_QUEUE_SIZE 4 * 120
#define ADC_SPI_CLOCK_SPEED_HZ 2500000

#define ADC_16KHZ_SAMPLES_NUM 240
#define ADC_8KHZ_SAMPLES_NUM 120

static spi_device_handle_t spi;
static spi_bus_config_t spi_bus_cfg;
static spi_device_interface_config_t spi_dev_cfg;
static spi_transaction_t adc_config_spi_tx;
static spi_transaction_t adc_data_tx[ADC_SPI_QUEUE_SIZE];

static QueueHandle_t audio_ready_queue = NULL;
static uint8_t adc_configured = 0;


static uint8_t *audio_in_buf = NULL; // buffer holding ADC data
static sample_rate_t adc_sample_rate;
static size_t adc_buf_sample_count = 0;
static RingbufHandle_t audio_in_rb = NULL;
static uint8_t initialized = 0;
static uint8_t enabled = 0;

static int64_t transmission_interval_us = 0;
static int64_t last_transmission_us = 0;
static int64_t now_us = 0;
static int64_t start_us = 0;
static int64_t duration_us = 0;

static uint8_t signal_flag1 = 1;
static uint8_t signal_flag2 = 1;

static void IRAM_ATTR adc_data_available_isr(void* arg) {
    QueueHandle_t q = (QueueHandle_t) arg;

    now_us = esp_timer_get_time();
    transmission_interval_us = now_us - last_transmission_us;
    last_transmission_us = now_us;
    xQueueSendFromISR(q, &signal_flag1, NULL);
}

static void adc_read_spi_data_task_handler(void *arg) {
    QueueHandle_t q = (QueueHandle_t) arg;
    
    while (1) {
        if (xQueueReceive(q, &signal_flag2, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }
        
        if (!adc_configured) {
            uint16_t data = SPI_SWAP_DATA_TX((uint16_t) adc_sample_rate, 16);
        
            adc_config_spi_tx.user = NULL;
            adc_config_spi_tx.rx_buffer = NULL;
            adc_config_spi_tx.tx_buffer = &data;
            adc_config_spi_tx.length = 16;
            ESP_ERROR_CHECK(spi_device_transmit(spi, &adc_config_spi_tx));
            adc_configured = 1;

            ESP_LOGI(AD_TAG, "Configured ADC");
            continue;
        }
        
        start_us = esp_timer_get_time();
        int16_t *buf = (int16_t *) audio_in_buf;
        for (uint8_t i = 0; i < adc_buf_sample_count; i++) {
            ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &adc_data_tx[i]));
            int16_t sample = SPI_SWAP_DATA_RX(*(uint32_t *)adc_data_tx[i].rx_data, 16);
            buf[i] = sample;
            
        }
        now_us = esp_timer_get_time();
        duration_us = now_us - start_us;
        xRingbufferSend(audio_in_rb, audio_in_buf, adc_buf_sample_count * sizeof(int16_t), 0);
        
        xQueueSend(audio_ready_queue, &signal_flag2, 0);
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
    ESP_LOGI(AD_TAG, "Initializing ADC SPI transport");

    QueueHandle_t adc_read_notif_queue = xQueueCreate(8, sizeof(uint8_t));
    assert(adc_read_notif_queue != NULL);

    BaseType_t r = xTaskCreatePinnedToCore(adc_read_spi_data_task_handler, "spi_data_task", 4092, adc_read_notif_queue, configMAX_PRIORITIES - 5, NULL, 1);
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

    // Setup bus
    spi_bus_cfg.miso_io_num = ADC_MISO_PIN;
    spi_bus_cfg.mosi_io_num = ADC_MOSI_PIN;
    spi_bus_cfg.sclk_io_num = ADC_CLK_PIN;
    spi_bus_cfg.quadwp_io_num = -1;
    spi_bus_cfg.quadhd_io_num = -1;
    spi_bus_cfg.max_transfer_sz = 2;

    // Setup device
    spi_dev_cfg.clock_speed_hz = ADC_SPI_CLOCK_SPEED_HZ;
    spi_dev_cfg.mode = 0;
    spi_dev_cfg.command_bits = 0;
    spi_dev_cfg.address_bits = 0;
    spi_dev_cfg.dummy_bits = 0;
    spi_dev_cfg.spics_io_num = ADC_CS_PIN;
    spi_dev_cfg.queue_size = ADC_SPI_QUEUE_SIZE;

    // ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &spi_bus_cfg, 0));
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &spi_dev_cfg, &spi));
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));

    int freq_khz;
    size_t max_tx_length;

    ESP_ERROR_CHECK(spi_device_get_actual_freq(spi, &freq_khz));
    ESP_LOGI(AD_TAG, "Actual SPI transfer frequency %dKHZ", freq_khz);

    ESP_ERROR_CHECK(spi_bus_get_max_transaction_len(VSPI_HOST, &max_tx_length));
    ESP_LOGI(AD_TAG, "Max SPI transaction length: %d bytes", max_tx_length);

    // Prepare transactions
    memset(&adc_config_spi_tx, 0, sizeof(spi_transaction_t));
    for (int i = 0; i < ADC_SPI_QUEUE_SIZE; i++) {
        memset(&adc_data_tx[i], 0, sizeof(spi_transaction_t));
        adc_data_tx[i].user = NULL;
        adc_data_tx[i].rx_buffer = NULL;
        adc_data_tx[i].tx_buffer = NULL;
        adc_data_tx[i].length = 16;
        adc_data_tx[i].flags = SPI_TRANS_USE_RXDATA;
    }
}

void adc_audio_init(sample_rate_t sample_rate, QueueHandle_t audio_ready_q) {
    if (initialized) {
        return;
    }
    assert(sample_rate != SAMPLE_RATE_NONE);
    adc_sample_rate = sample_rate;
    audio_ready_queue = audio_ready_q;

    size_t adc_buf_size = adc_audio_get_buffer_size();
    adc_buf_sample_count = adc_buf_size / 2;
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
