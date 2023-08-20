#define INCLUDE_vTaskSuspend 1

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sine_16000_666.h"
#include "sine_8000_666.h"
#include "dac_audio.h"

#define DA_TAG "DA"
#define DAC_AUDIO_16KHZ_BUFFER_SIZE 240
#define DAC_AUDIO_8KHZ_BUFFER_SIZE 120
#define DAC_SAMPLE_BYTE_COUNT 2
#define DAC_SAMPLE_BIT_SIZE DAC_SAMPLE_BYTE_COUNT * 8

#define DAC_MOSI_PIN 23
#define DAC_CLK_PIN 18
#define DAC_CS_PIN 5
#define DAC_SPI_QUEUE_SIZE 4

static gptimer_handle_t spi_transfer_timer = NULL;
static QueueHandle_t spi_data_queue = NULL;
static TaskHandle_t audio_consumer_task;
static sample_rate_t dac_sample_rate;
static uint16_t sample_rate_hz;
static uint8_t initialized = 0;
static uint8_t enabled = 0;
static int64_t last_incoming_buffer_us = 0;

static uint64_t debug_counter = 0;
static uint64_t sent_buf_counter = 0;
static uint64_t send_fail_counter = 0;
static uint64_t not_enabled_counter = 0;
static uint64_t too_many_spi_tx_counter = 0;
static uint64_t no_samples_counter = 0;
static uint64_t enabled_counter = 0;
static uint64_t not_enough_samples_counter = 0;
static uint64_t failed_to_send_spi_tx_counter = 0;

static spi_device_handle_t spi;
static spi_bus_config_t spi_bus_cfg;
static spi_device_interface_config_t spi_dev_cfg;
static spi_transaction_t spi_transaction;

/**
 * Return the buffer size based on current sample rate
 */
static inline size_t dac_audio_get_buffer_size() {
    if (dac_sample_rate == SAMPLE_RATE_16KHZ) {
        return DAC_AUDIO_16KHZ_BUFFER_SIZE;
    }

    if (dac_sample_rate == SAMPLE_RATE_8KHZ) {
        return DAC_AUDIO_8KHZ_BUFFER_SIZE;
    }
    
    return 0;
}

static void consume_audio_task_handler(void *arg) {
    QueueHandle_t spi_data_queue = (QueueHandle_t) arg;

    uint16_t sample;

    while (1) {
        vTaskSuspend(NULL);
        if (xQueueReceive(spi_data_queue, &sample, 0) != pdTRUE) {
            sample = 0;
        }

        spi_transaction.tx_buffer = &sample;
        esp_err_t result = spi_device_polling_transmit(spi, &spi_transaction);
        if (result != ESP_OK) {
            if (failed_to_send_spi_tx_counter++ % 100 == 0) {
                ESP_LOGE(DA_TAG, "Failed to send spi transaction: %d", result);
            }
        }
    }
}


static bool IRAM_ATTR wakeup_spi_transfer_task(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    TaskHandle_t spi_transfer_task = (TaskHandle_t) user_data;
    xTaskResumeFromISR(spi_transfer_task);
    return true;
}

sample_rate_t dac_audio_get_sample_rate() {
    return dac_sample_rate;
}

void dac_audio_send(const uint8_t *buf, size_t size) {
    if (!enabled) {
        return;
    }
    size_t samples_to_write = size / 2;
    
    // int16_t *src = (dac_sample_rate == SAMPLE_RATE_16KHZ) ? sinewave_16000 : sinewave_8000;
    int16_t *src = (int16_t *) buf;
    uint16_t sample;
    
    // TLC5615 (10bit DAC) expects the data to be formatted as follows:
    // 4 upper dummy bits, 10 data bits, 2 extra (don't care) sub-LSB bits
    for (int i = 0; i < samples_to_write; i++) {
        sample = SPI_SWAP_DATA_TX((src[i] - INT16_MIN) >> 4, DAC_SAMPLE_BIT_SIZE);
        xQueueSend(spi_data_queue, &sample, 0);
    }
}

void dac_audio_enable(uint8_t status) {
    if (!initialized && status) {
        return;
    }

    if (status == enabled) {
        return;
    }

    if (status) {
        ESP_LOGW(DA_TAG, "Enabling DAC");
        ESP_ERROR_CHECK(gptimer_set_raw_count(spi_transfer_timer, 0));
        ESP_ERROR_CHECK(gptimer_start(spi_transfer_timer));
        enabled = 1;
        return;
    }

    ESP_LOGW(DA_TAG, "Disabling dac");
    ESP_ERROR_CHECK(gptimer_stop(spi_transfer_timer));
    enabled = 0;
}

void dac_audio_init(sample_rate_t sample_rate) {
    if (initialized) {
        return;
    }
    
    memset(&spi_transaction, 0, sizeof(spi_transaction_t));
    
    // Pre-fill non-volatile tx fields
    spi_transaction.user = NULL;
    spi_transaction.rx_buffer = NULL;
    spi_transaction.length = DAC_SAMPLE_BIT_SIZE;

    assert(sample_rate != SAMPLE_RATE_NONE);
    dac_sample_rate = sample_rate;

    size_t buf_size = dac_audio_get_buffer_size();
    sample_rate_hz = (sample_rate == SAMPLE_RATE_16KHZ) ? 16000 : 8000;
    ESP_LOGI(DA_TAG, "Initializing DAC. Sample rate: %d. Buffer size: %d", sample_rate_hz, buf_size);

    spi_data_queue = xQueueCreate(DAC_SPI_QUEUE_SIZE * buf_size, sizeof(uint16_t));
    assert(spi_data_queue != NULL);
    
    BaseType_t r = xTaskCreatePinnedToCore(consume_audio_task_handler, "consume_audio", 2048, spi_data_queue, 15, &audio_consumer_task, 1);
    assert(r == pdPASS);

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000 // 1MHZ
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &spi_transfer_timer));
        
    gptimer_event_callbacks_t cbs = {
        .on_alarm = wakeup_spi_transfer_task
    };
    
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(spi_transfer_timer, &cbs, audio_consumer_task));
    ESP_ERROR_CHECK(gptimer_enable(spi_transfer_timer));
    
    uint64_t alarm_period_us = 1000000 / sample_rate_hz;
    gptimer_alarm_config_t spi_alarm_config = {
        .reload_count = 0,
        .alarm_count = alarm_period_us,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(spi_transfer_timer, &spi_alarm_config));
    ESP_LOGI(DA_TAG, "Created gptimer with alarm period (us) of: %"PRId64, alarm_period_us);

    // Setup bus
    spi_bus_cfg.miso_io_num = -1; // we're not interested in reading
    spi_bus_cfg.mosi_io_num = DAC_MOSI_PIN;
    spi_bus_cfg.sclk_io_num = DAC_CLK_PIN;
    spi_bus_cfg.quadwp_io_num = -1;
    spi_bus_cfg.quadhd_io_num = -1;

    // Setup device
    spi_dev_cfg.clock_speed_hz = (dac_sample_rate == SAMPLE_RATE_16KHZ) ? 8000000 : 2000000;
    spi_dev_cfg.mode = 0;
    spi_dev_cfg.command_bits = 0;
    spi_dev_cfg.address_bits = 0;
    spi_dev_cfg.dummy_bits = 0;
    spi_dev_cfg.spics_io_num = DAC_CS_PIN;
    spi_dev_cfg.queue_size = DAC_SPI_QUEUE_SIZE;
    
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &spi_bus_cfg, 0));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &spi_dev_cfg, &spi));
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));

    int freq_khz;
    size_t max_tx_length;

    ESP_ERROR_CHECK(spi_device_get_actual_freq(spi, &freq_khz));
    ESP_LOGI(DA_TAG, "Actual SPI transfer frequency %dKHZ", freq_khz);

    ESP_ERROR_CHECK(spi_bus_get_max_transaction_len(HSPI_HOST, &max_tx_length));
    ESP_LOGI(DA_TAG, "Max SPI transaction length: %d bytes", max_tx_length);

    initialized = 1;
}

void dac_audio_deinit() {
    if (!initialized) {
        return;
    }

    ESP_LOGW(DA_TAG, "Unintializing dac");
    dac_audio_enable(0);
    ESP_LOGW(DA_TAG, "Releasing channels");
    dac_sample_rate = SAMPLE_RATE_NONE;
    
    spi_device_release_bus(spi);
    ESP_ERROR_CHECK(spi_bus_remove_device(spi));
    ESP_ERROR_CHECK(spi_bus_free(HSPI_HOST));

    ESP_LOGW(DA_TAG, "Deleting task");
    vTaskDelete(audio_consumer_task);

    vQueueDelete(spi_data_queue);

    initialized = 0;
}