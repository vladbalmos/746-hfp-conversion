#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "sample_rate.h"
#include "dac_audio.h"

#define TAG "TEST"

#define ADC_SAMPLES_COUNT 120
#define ADC_ENABLE_PIN GPIO_NUM_13
#define ADC_DATA_READY_PIN GPIO_NUM_25
#define ADC_MOSI_PIN GPIO_NUM_26
#define ADC_MISO_PIN GPIO_NUM_27
#define ADC_CLK_PIN GPIO_NUM_12
#define ADC_CS_PIN GPIO_NUM_14
#define ADC_SPI_QUEUE_SIZE ADC_SAMPLES_COUNT

static IRAM_ATTR QueueHandle_t buf_ready_queue;
static spi_device_handle_t spi;
static spi_bus_config_t spi_bus_cfg;
static spi_device_interface_config_t spi_dev_cfg;
static spi_transaction_t adc_config_spi_tx;
static spi_transaction_t adc_data_tx[ADC_SPI_QUEUE_SIZE];
static int16_t buf[ADC_SAMPLES_COUNT];
static int16_t dac_buf[3][ADC_SAMPLES_COUNT] = {0};
static uint8_t dac_buf_index = 0;
static uint8_t dac_streaming_started = 0;
static uint8_t adc_configured = 0;
static uint64_t spi_read_tx_counter = 0;
static uint64_t spi_cb_counter = 0;
static int64_t start;
static int64_t duration;
static int64_t transmission_interval;
static int64_t last_transmission = 0;

static void IRAM_ATTR data_read_isr(void* arg) {
    QueueHandle_t q = (QueueHandle_t) arg;
    uint8_t dummy = 1;

    int64_t now = esp_timer_get_time();
    transmission_interval = now - last_transmission;
    last_transmission = now;
    xQueueSendFromISR(q, &dummy, NULL);
}

static void IRAM_ATTR on_spi_data_isr(spi_transaction_t *tx) {
    uint8_t sample_index = (uint8_t) tx->user;
    int16_t sample = SPI_SWAP_DATA_RX(*(uint32_t *)tx->rx_data, 16);
    uint8_t dummy = 1;
    
    buf[sample_index] = sample;
    
    if (sample_index == (ADC_SAMPLES_COUNT - 1)) {
        uint64_t now = esp_timer_get_time();
        duration = now - start;
        xQueueSendFromISR(buf_ready_queue, &dummy, NULL);
    }
}

static void read_spi_data_task_handler(void *arg) {
    QueueHandle_t q = (QueueHandle_t) arg;
    uint8_t signal = 0;
    
    while (1) {
        if (xQueueReceive(q, &signal, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }
        
        if (!adc_configured) {
            uint16_t data = SPI_SWAP_DATA_TX((uint16_t) SAMPLE_RATE_16KHZ, 16);
        
            adc_config_spi_tx.user = NULL;
            adc_config_spi_tx.rx_buffer = NULL;
            adc_config_spi_tx.tx_buffer = &data;
            adc_config_spi_tx.length = 16;
            ESP_ERROR_CHECK(spi_device_transmit(spi, &adc_config_spi_tx));
            adc_configured = 1;

            ESP_LOGI(TAG, "Configured ADC");
            continue;
        }
        
        // if (spi_read_tx_counter++ % 500 == 0) {
        //     ESP_LOGW(TAG, "Receiving data");
        // }
        start = esp_timer_get_time();
        for (uint8_t i = 0; i < ADC_SAMPLES_COUNT; i++) {
            // adc_data_tx[i].user = (void *) i;
            // ESP_ERROR_CHECK(spi_device_queue_trans(spi, &adc_data_tx[i], portMAX_DELAY));
            ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &adc_data_tx[i]));
            int16_t sample = SPI_SWAP_DATA_RX(*(uint32_t *)adc_data_tx[i].rx_data, 16);
            buf[i] = sample;
            
        }
        uint64_t now = esp_timer_get_time();
        duration = now - start;
        xQueueSend(buf_ready_queue, &signal, 0);
    }
}

static void consume_audio_dac(void *arg) {
    QueueHandle_t q = (QueueHandle_t) arg;
    uint8_t buf_index = 0;
    uint8_t *buf = NULL;
    
    while (1) {
        if (xQueueReceive(q, &buf_index, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }
        buf = (uint8_t *) &dac_buf[buf_index];
        dac_audio_send(buf, ADC_SAMPLES_COUNT * 2);
    }
}

void app_main(void) {
    buf_ready_queue = xQueueCreate(8, sizeof(uint8_t));
    assert(buf_ready_queue != NULL);

    QueueHandle_t data_ready_queue = xQueueCreate(8, sizeof(uint8_t));
    assert(data_ready_queue != NULL);
    
    BaseType_t r = xTaskCreatePinnedToCore(read_spi_data_task_handler, "spi_data_task", 4092, data_ready_queue, 15, NULL, 0);
    assert(r == pdPASS);

    QueueHandle_t audio_queue = xQueueCreate(8, sizeof(uint8_t));
    assert(audio_queue != NULL);

    r = xTaskCreatePinnedToCore(consume_audio_dac, "consume_audio_dac", 4092, audio_queue, 10, NULL, 1);
    assert(r == pdPASS);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    
    // Enable DAC
    dac_audio_init(SAMPLE_RATE_16KHZ);;
    dac_audio_enable(1);

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
    ESP_ERROR_CHECK(gpio_isr_handler_add(ADC_DATA_READY_PIN, data_read_isr, data_ready_queue));
    
    memset(buf, 0, ADC_SAMPLES_COUNT * sizeof(int16_t));

    // SPI config
    
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

    // Setup bus
    spi_bus_cfg.miso_io_num = ADC_MISO_PIN;
    spi_bus_cfg.mosi_io_num = ADC_MOSI_PIN;
    spi_bus_cfg.sclk_io_num = ADC_CLK_PIN;
    spi_bus_cfg.quadwp_io_num = -1;
    spi_bus_cfg.quadhd_io_num = -1;
    spi_bus_cfg.max_transfer_sz = 2;

    // Setup device
    spi_dev_cfg.clock_speed_hz = 2500000;
    spi_dev_cfg.mode = 0;
    spi_dev_cfg.command_bits = 0;
    spi_dev_cfg.address_bits = 0;
    spi_dev_cfg.dummy_bits = 0;
    spi_dev_cfg.spics_io_num = ADC_CS_PIN;
    spi_dev_cfg.queue_size = ADC_SPI_QUEUE_SIZE;
    // spi_dev_cfg.post_cb = on_spi_data_isr;
 
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &spi_dev_cfg, &spi));
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));

    int freq_khz;
    size_t max_tx_length;

    ESP_ERROR_CHECK(spi_device_get_actual_freq(spi, &freq_khz));
    ESP_LOGI(TAG, "Actual SPI transfer frequency %dKHZ", freq_khz);

    ESP_ERROR_CHECK(spi_bus_get_max_transaction_len(VSPI_HOST, &max_tx_length));
    ESP_LOGI(TAG, "Max SPI transaction length: %d bytes", max_tx_length);
    

    ESP_LOGI(TAG, "Enabling ADC");
    gpio_set_level(ADC_ENABLE_PIN, 1);

    uint8_t dummy = 0;
    uint64_t print_counter = 0;

    while (1) {
        if (xQueueReceive(buf_ready_queue, &dummy, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }

        memcpy(&dac_buf[dac_buf_index++], buf, ADC_SAMPLES_COUNT * 2);

        if (dac_buf_index >= 3) {
            for (int i = 0; i < dac_buf_index; i++) {
                xQueueSend(audio_queue, &i, portMAX_DELAY);
            }
            dac_buf_index = 0;
        }

        
        if (print_counter++ % 75 == 0) {
            ESP_LOGW(TAG, "Duration is %"PRId64, duration);
            ESP_LOGW(TAG, "Transmission interval %"PRId64, transmission_interval);
            // for (int i = 0; i < ADC_SAMPLES_COUNT; i++) {
            //     printf("%d ", buf[i]);
                
            //     if (i && i % 16 == 0) {
            //         printf("\n");
            //     }
            // }
            // printf("\n");
        }
        
    }
}
