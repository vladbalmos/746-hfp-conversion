#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "sample_rate.h"
#include "sine_8000_666.h"

#define TAG "MAIN"
#define DA_TAG "MAIN"
#define MISO_PIN 36
#define MOSI_PIN 23
#define CLK_PIN 18
#define CS_PIN 5
#define SPI_QUEUE_SIZE 60

#define BUF_SIZE 120

static int64_t last_incoming_buffer_us = 0;

static uint64_t debug_counter = 0;
static uint64_t sent_buf_counter = 0;
static uint64_t send_fail_counter = 0;
static uint64_t not_enabled_counter = 0;
static uint64_t too_many_spi_tx_counter = 0;
static uint64_t no_samples_counter = 0;
static uint64_t enabled_counter = 0;
static uint64_t not_enough_samples_counter = 0;
static uint64_t failed_to_queue_spi_tx_counter = 0;

static spi_device_handle_t spi;
static spi_bus_config_t spi_bus_cfg;
static spi_device_interface_config_t spi_dev_cfg;
static uint8_t spi_tx_index = 0;
static spi_transaction_t spi_transactions[SPI_QUEUE_SIZE];

static void IRAM_ATTR post_spi_tx_callback(spi_transaction_t *tx) {
    spi_tx_index--;

    // int64_t now = esp_timer_get_time();
    // int64_t interval = now - last_incoming_buffer_us;
    // last_incoming_buffer_us = now;
    // if (sent_buf_counter++ % 100 == 0) {
    //     ESP_DRAM_LOGI(DA_TAG, "Send interval %"PRId64, interval);
    // }
}

static void send_spi_tx_blocking(const uint8_t *buf, size_t buf_size) {
    spi_transaction_t *tx = &spi_transactions[spi_tx_index];
    memset(tx, 0, sizeof(spi_transaction_t));
    
    tx->user = NULL;
    tx->tx_buffer = buf;
    tx->rx_buffer = NULL;
    tx->length = buf_size * 8;
    tx->flags = 0;
    
    esp_err_t result = spi_device_transmit(spi, tx);
    if (result != ESP_OK) {
        if (failed_to_queue_spi_tx_counter++ % 100 == 0) {
            ESP_LOGE(DA_TAG, "Failed to queue spi transaction: %d", result);
        }
        return;
    }
}

static void send_small_spi_tx(const uint8_t *buf, size_t buf_size) {
    for (int i = 0; i < 60; i++) {
        spi_transaction_t *tx = &spi_transactions[i];
        memset(tx, 0, sizeof(spi_transaction_t));
        
        tx->user = NULL;
        tx->tx_buffer = buf;
        tx->rx_buffer = NULL;
        tx->length = 16;
        
        buf += 2;
        
        esp_err_t result = spi_device_queue_trans(spi, tx, portMAX_DELAY);
        // esp_err_t result = spi_device_transmit(spi, tx);
        if (result != ESP_OK) {
            if (failed_to_queue_spi_tx_counter++ % 100 == 0) {
                ESP_LOGE(DA_TAG, "Failed to queue spi transaction: %d", result);
            }
            return;
        }
        spi_tx_index++;
    }
    
    spi_transaction_t *rtrans;
    for (int i = 0; i < 60; i++) {
        spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
    }
}

static void send_spi_tx(const uint8_t *buf, size_t buf_size) {
    spi_transaction_t *tx = &spi_transactions[spi_tx_index];
    memset(tx, 0, sizeof(spi_transaction_t));
    
    tx->user = NULL;
    tx->tx_buffer = buf;
    tx->rx_buffer = NULL;
    tx->length = buf_size * 8;
    tx->flags = 0;
    // ESP_LOGW(DA_TAG, "%d", spi_tx_index);
    
    // if (debug_counter++ % 1000 == 0) {
    //     uint16_t *x = (uint16_t *) buf;
    //     for (int i = 0; i < (buf_size / 2); i++) {
    //         printf("%d ", x[i]);
    //         if (i && i % 8 == 0) {
    //             printf("\n");
    //         }
    //     }
    
    //     printf("\n=================================================\n");
    // }
    
    esp_err_t result = spi_device_queue_trans(spi, tx, portMAX_DELAY);
    if (result != ESP_OK) {
        if (failed_to_queue_spi_tx_counter++ % 100 == 0) {
            ESP_LOGE(DA_TAG, "Failed to queue spi transaction: %d", result);
        }
        return;
    }
    spi_tx_index++;
}

void app_main(void) {
    // Setup buffers
    // for (uint8_t i = 0; i < SPI_QUEUE_SIZE; i++) {
    //     audio_out_buf[i] = malloc(SPI_QUEUE_SIZE * BUF_SIZE);
    //     assert(audio_out_buf[i] != NULL);
    // }

    // Setup bus
    spi_bus_cfg.miso_io_num = -1; // we're not interested in reading
    spi_bus_cfg.mosi_io_num = MOSI_PIN;
    spi_bus_cfg.sclk_io_num = CLK_PIN;
    spi_bus_cfg.quadwp_io_num = -1;
    spi_bus_cfg.quadhd_io_num = -1;
    spi_bus_cfg.max_transfer_sz = BUF_SIZE;

    // Setup device
    spi_dev_cfg.clock_speed_hz = 256000;
    spi_dev_cfg.mode = 0;
    spi_dev_cfg.command_bits = 0;
    spi_dev_cfg.address_bits = 0;
    spi_dev_cfg.dummy_bits = 0;
    spi_dev_cfg.spics_io_num = CS_PIN;
    spi_dev_cfg.queue_size = SPI_QUEUE_SIZE;
    spi_dev_cfg.post_cb = &post_spi_tx_callback;

    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &spi_dev_cfg, &spi));
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));

    int freq_khz;
    size_t max_tx_length;

    ESP_ERROR_CHECK(spi_device_get_actual_freq(spi, &freq_khz));
    ESP_LOGI(DA_TAG, "Actual SPI transfer frequency %dKHZ", freq_khz);

    ESP_ERROR_CHECK(spi_bus_get_max_transaction_len(HSPI_HOST, &max_tx_length));
    ESP_LOGI(DA_TAG, "Max SPI transaction length: %d bytes", max_tx_length);
    
    
    uint16_t dac_samples[60];
    
    for (int i = 0; i < 60; i++) {
        // dac_samples[i] = ((sinewave_8000[i] - INT16_MIN) >> 6) << 2;
        dac_samples[i] = SPI_SWAP_DATA_TX((sinewave_8000[i] - INT16_MIN) >> 4, 16);
    }

    uint8_t *out_buf = NULL;
    // uint16_t out_buf[3] = {4092, 2048, 0};
    uint8_t counter = 0;
    
    while (1) {
        if (spi_tx_index == (SPI_QUEUE_SIZE - 1)) {
            // vTaskDelay(pdMS_TO_TICKS(500));
            // vTaskDelay(10);
            // vTaskDelay(1);
            continue;
        }
        
        if (counter >= 120) {
            counter = 0;
        }
        
        
        // out_buf = audio_out_buf[spi_tx_index];

        // memcpy(out_buf, dac_samples, BUF_SIZE);
        // send_spi_tx(out_buf, BUF_SIZE);
        
        send_small_spi_tx(dac_samples, BUF_SIZE);
    }
}
