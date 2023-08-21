#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sample_rate.h"

#define TAG "TEST"
#define ENABLE_PIN GPIO_NUM_13

#define ADC_MOSI_PIN 26
#define ADC_MISO_PIN 27
#define ADC_CLK_PIN 12
#define ADC_CS_PIN 14
#define ADC_SPI_QUEUE_SIZE 120

#define MAX 5

static spi_device_handle_t spi;
static spi_bus_config_t spi_bus_cfg;
static spi_device_interface_config_t spi_dev_cfg;
static spi_transaction_t spi_transaction;

void app_main(void) {

    gpio_config_t output_conf = {};
    output_conf.intr_type = GPIO_INTR_DISABLE;
    output_conf.mode = GPIO_MODE_OUTPUT;
    output_conf.pin_bit_mask = 1ULL << ENABLE_PIN;
    ESP_ERROR_CHECK(gpio_config(&output_conf));
    
    // SPI config
    memset(&spi_transaction, 0, sizeof(spi_transaction_t));

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
    spi_dev_cfg.input_delay_ns = 100;
    
    // ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &spi_bus_cfg, 0));
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &spi_dev_cfg, &spi));
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));

    int freq_khz;
    size_t max_tx_length;

    ESP_ERROR_CHECK(spi_device_get_actual_freq(spi, &freq_khz));
    ESP_LOGI(TAG, "Actual SPI transfer frequency %dKHZ", freq_khz);

    ESP_ERROR_CHECK(spi_bus_get_max_transaction_len(HSPI_HOST, &max_tx_length));
    ESP_LOGI(TAG, "Max SPI transaction length: %d bytes", max_tx_length);
    
    gpio_set_level(ENABLE_PIN, 1);
    uint16_t data = SPI_SWAP_DATA_TX((uint16_t) SAMPLE_RATE_16KHZ, 16);
    
    spi_transaction.user = NULL;
    spi_transaction.rx_buffer = NULL;
    spi_transaction.tx_buffer = &data;
    spi_transaction.length = 16;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &spi_transaction));



    // vTaskDelay(1);

    // static int16_t buf[MAX];
    // memset(buf, 0, MAX * 2);
    // static spi_transaction_t txs[MAX];
    // spi_transaction_t *tx;
    
    // for (int i = 0; i < MAX; i++) {
    //     tx = &txs[i];
    //     memset(tx, 0, sizeof(spi_transaction_t));
    //     tx->user = NULL;
    //     tx->rx_buffer = &buf[i];
    //     tx->tx_buffer = NULL;
    //     tx->length = 16;
        
    //     // ESP_ERROR_CHECK(spi_device_queue_trans(spi, tx, portMAX_DELAY));
    //     ESP_ERROR_CHECK(spi_device_polling_transmit(spi, tx));
    // }
    
    // vTaskDelay(2);

    // spi_transaction_t *r;

    // for (int i = 0; i < MAX; i++) {
    //     ESP_ERROR_CHECK(spi_device_get_trans_result(spi, &r, portMAX_DELAY));
    // }
    
    // for (int i = 0; i < MAX; i++) {
    //     int16_t pcm_sample = SPI_SWAP_DATA_RX(buf[i], 16);
    //     printf("%d ", pcm_sample);
    //     if (i && i % 8 == 0) {
    //         printf("\n");
    //     }
    // }
    printf("\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
