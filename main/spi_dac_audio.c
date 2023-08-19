#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sine_16000_666.h"
#include "sine_8000_666.h"
#include "spi_dac_audio.h"

#define DA_TAG "DA"
#define DAC_AUDIO_16KHZ_BUFFER_SIZE 240
#define DAC_AUDIO_8KHZ_BUFFER_SIZE 120
#define DAC_SAMPLE_BYTE_COUNT 2
#define DAC_SAMPLE_BIT_SIZE DAC_SAMPLE_BYTE_COUNT * 8
#define DAC_SPI_CLOCK_SPEED_HZ 2000000

#define DAC_MOSI_PIN 23
#define DAC_CLK_PIN 18
#define DAC_CS_PIN 5
#define DAC_SPI_QUEUE_SIZE 4

typedef struct {
    RingbufHandle_t audio_rb;
    QueueHandle_t spi_data_queue;
} consumer_task_input_t;

static DMA_ATTR uint8_t *audio_out_buf[DAC_SPI_QUEUE_SIZE] = {NULL, NULL, NULL, NULL}; // buffer sent to DAC for output
static uint8_t *tmp_audio_buf = NULL; // buffer holding 8 bit down converted mSBC frames
static uint8_t *audio_data = NULL; // data fetched from ring buffer
static size_t tmp_audio_buf_size = 0;
static esp_timer_handle_t spi_transfer_timer = NULL;
static QueueHandle_t spi_data_queue = NULL;
static TaskHandle_t audio_consumer_task;
static RingbufHandle_t audio_out_rb = NULL;
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

static inline size_t dac_audio_get_rb_size() {
    return 8 * dac_audio_get_buffer_size();
}

static void spi_transfer_callback(void *arg) {
    QueueHandle_t queue = (QueueHandle_t) arg;
    uint16_t sample;

    if (xQueueReceive(queue, &sample, 0) != pdTRUE) {
        return;
    }
    spi_transaction.tx_buffer = &sample;
    // esp_err_t result = spi_device_polling_transmit(spi, &spi_transaction);
    esp_err_t result = spi_device_queue_trans(spi, &spi_transaction, 0);
    if (result != ESP_OK) {
        if (failed_to_send_spi_tx_counter++ % 100 == 0) {
            ESP_LOGE(DA_TAG, "Failed to send spi transaction: %d", result);
        }
        return;
    }
}

static void queue_spi_data(QueueHandle_t queue, const uint8_t *buf, size_t buf_size) {
    uint16_t max = buf_size / DAC_SAMPLE_BYTE_COUNT;
    uint16_t *src = (uint16_t *) buf;

    for (int i = 0; i < max; i++) {
        xQueueSend(queue, &src[i], portMAX_DELAY);
    }
    
}

static void consume_audio_task_handler(void *arg) {
    // RingbufHandle_t audio_out_rb = (RingbufHandle_t) arg;
    consumer_task_input_t *input = (consumer_task_input_t *) arg;
    RingbufHandle_t audio_out_rb = input->audio_rb;
    QueueHandle_t spi_data_queue = input->spi_data_queue;

    size_t buffered_samples_size;
    size_t dac_buf_size = dac_audio_get_buffer_size();
    size_t min_buffered_samples_size = dac_buf_size;
    size_t received_bytes = 0;
    uint8_t out_buf_index = 0;
    uint8_t *out_buf = NULL;

    ESP_LOGW(DA_TAG, "DAC audio buffer size is: %d", dac_buf_size);
    while (1) {
        if (!enabled) {
            if (not_enabled_counter++ % 250 == 0) {
                ESP_LOGI(DA_TAG, "Not enabled");
            }
            vTaskDelay(1);
            continue;
        }

        if (out_buf_index >= DAC_SPI_QUEUE_SIZE) {
            out_buf_index = 0;
        } 
        out_buf = audio_out_buf[out_buf_index];

        buffered_samples_size = dac_audio_get_rb_size() - xRingbufferGetCurFreeSize(audio_out_rb);
        
        if (buffered_samples_size < min_buffered_samples_size) {
            if (not_enough_samples_counter++ % 250 == 0) {
                ESP_LOGW(DA_TAG, "Not enough samples. Streaming silence. Expected: %d. Received %d", min_buffered_samples_size, buffered_samples_size);
            }
            goto stream_silence;
        }

        audio_data = xRingbufferReceiveUpTo(audio_out_rb, &received_bytes, 0, dac_buf_size);
        if (!received_bytes) {
            if (no_samples_counter++ % 250 == 0) {
                ESP_LOGW(DA_TAG, "No bytes received from ring buffer. Streaming silence");
            }
            goto stream_silence;
        }
        out_buf_index++;
        

        memcpy(out_buf, audio_data, received_bytes);
        vRingbufferReturnItem(audio_out_rb, audio_data);

        audio_data = NULL;
        if (received_bytes < dac_buf_size) {
            // If we got less than what we asked for, fill the buffer with silence
            memset(out_buf + received_bytes, 0, dac_buf_size - received_bytes);
        }
        
        queue_spi_data(spi_data_queue, out_buf, dac_buf_size);
        
        if (enabled_counter++ % 250 == 0) {
            ESP_LOGW(DA_TAG, "Streaming data");
        }
        received_bytes = 0;
        continue;
        
stream_silence:
        memset(out_buf, 0, dac_buf_size);
        queue_spi_data(spi_data_queue, out_buf, dac_buf_size);
        continue;
    }
}

sample_rate_t dac_audio_get_sample_rate() {
    return dac_sample_rate;
}

void dac_audio_send(const uint8_t *buf, size_t size) {
    if (!enabled) {
        return;
    }
    size_t samples_to_write = size / 2;
    size_t bytes_written = 0;
    
    if (samples_to_write > tmp_audio_buf_size) {
        ESP_LOGW(DA_TAG, "Audio buffer to small. Max value is %d, received: %d", tmp_audio_buf_size, samples_to_write);
        samples_to_write = tmp_audio_buf_size;
    }
    
    // int16_t *src = (dac_sample_rate == SAMPLE_RATE_16KHZ) ? sinewave_16000 : sinewave_8000;
    int16_t *src = (int16_t *) buf;
    uint16_t *dst = (uint16_t *) tmp_audio_buf;
    

    // TLC5615 (10bit DAC) expects the data to be formatted as follows:
    // 4 upper dummy bits, 10 data bits, 2 extra (don't care) sub-LSB bits
    for (int i = 0; i < samples_to_write; i++) {
        dst[i] = SPI_SWAP_DATA_TX((src[i] - INT16_MIN) >> 4, DAC_SAMPLE_BIT_SIZE);
        bytes_written += DAC_SAMPLE_BYTE_COUNT;
    }

    // if (debug_counter++ % 1000 == 0) {
    //     printf("\nOriginal\n");
    //     for (int i = 0; i < samples_to_write; i++) {
    //         printf("%d ", src[i]);
    //         if (i && i % 8 == 0) {
    //             printf("\n");
    //         }
    //     }
    
    //     printf("\n=================================================\n");

    //     printf("\nConverted\n");
    //     for (int i = 0; i < samples_to_write; i++) {
    //         printf("%d ", dst[i]);
    //         if (i && i % 8 == 0) {
    //             printf("\n");
    //         }
    //     }
    
    //     printf("\n=================================================\n");
    // }

    
    BaseType_t r = xRingbufferSend(audio_out_rb, tmp_audio_buf, bytes_written, 0);
    if (r != pdTRUE) {
        if (send_fail_counter++ % 250 == 0) {
            ESP_LOGW(DA_TAG, "Failed to write audio data to ring buffer %d %d", xRingbufferGetCurFreeSize(audio_out_rb), bytes_written);
        }
    }
    memset(tmp_audio_buf, 0, tmp_audio_buf_size);
}

void dac_audio_enable(uint8_t status) {
    if (!initialized && status) {
        return;
    }

    if (status == enabled) {
        return;
    }

    memset(tmp_audio_buf, 0, tmp_audio_buf_size);
    if (status) {
        ESP_LOGW(DA_TAG, "Enabling DAC");
        enabled = 1;
        uint64_t period = (uint64_t) 1000000 / (uint64_t) sample_rate_hz;
        ESP_ERROR_CHECK(esp_timer_start_periodic(spi_transfer_timer, period));
        return;
    }

    enabled = 0;
    ESP_ERROR_CHECK(esp_timer_stop(spi_transfer_timer));
    ESP_LOGW(DA_TAG, "Disabling dac");
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

    // Setup buffers
    for (uint8_t i = 0; i < DAC_SPI_QUEUE_SIZE; i++) {
        audio_out_buf[i] = malloc(DAC_SPI_QUEUE_SIZE * buf_size);
        assert(audio_out_buf[i] != NULL);
    }

    tmp_audio_buf_size = 2 * buf_size;
    tmp_audio_buf = malloc(tmp_audio_buf_size);
    assert(tmp_audio_buf != NULL);

    audio_out_rb = xRingbufferCreate(dac_audio_get_rb_size(), RINGBUF_TYPE_BYTEBUF);
    assert(audio_out_rb != NULL);

    spi_data_queue = xQueueCreate(DAC_SPI_QUEUE_SIZE * buf_size, sizeof(uint16_t));
    assert(spi_data_queue != NULL);
    
    consumer_task_input_t input = {
        .audio_rb = audio_out_rb,
        .spi_data_queue = spi_data_queue
    };

    BaseType_t r = xTaskCreatePinnedToCore(consume_audio_task_handler, "consume_audio", 2048, &input, 15, &audio_consumer_task, 1);
    assert(r == pdPASS);
    
    const esp_timer_create_args_t transfer_timer_args = {
        .callback = &spi_transfer_callback,
        .arg = spi_data_queue,
        .name = 'spi-transfer'
    };
    ESP_ERROR_CHECK(esp_timer_create(&transfer_timer_args, &spi_transfer_timer));

    // Setup bus
    spi_bus_cfg.miso_io_num = -1; // we're not interested in reading
    spi_bus_cfg.mosi_io_num = DAC_MOSI_PIN;
    spi_bus_cfg.sclk_io_num = DAC_CLK_PIN;
    spi_bus_cfg.quadwp_io_num = -1;
    spi_bus_cfg.quadhd_io_num = -1;
    // spi_bus_cfg.max_transfer_sz = buf_size;

    // Setup device
    spi_dev_cfg.clock_speed_hz = DAC_SPI_CLOCK_SPEED_HZ;
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
    
    ESP_ERROR_CHECK(esp_timer_delete(spi_transfer_timer));
    
    spi_device_release_bus(spi);
    ESP_ERROR_CHECK(spi_bus_remove_device(spi));
    ESP_ERROR_CHECK(spi_bus_free(HSPI_HOST));

    ESP_LOGW(DA_TAG, "Deleting task");
    vTaskDelete(audio_consumer_task);
    
    vRingbufferDelete(audio_out_rb);
    audio_out_rb = NULL;
    
    vQueueDelete(spi_data_queue);

    ESP_LOGW(DA_TAG, "Freeing audio buf");
    for (uint8_t i = 0; i < DAC_SPI_QUEUE_SIZE; i++) {
        free(audio_out_buf[i]);
        audio_out_buf[i] = NULL;
    }
    ESP_LOGW(DA_TAG, "Freeing tmp audio buf");
    free(tmp_audio_buf);
    tmp_audio_buf = NULL;
    tmp_audio_buf_size = 0;
    initialized = 0;
}