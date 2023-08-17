#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "spi_dac_audio.h"

#define DA_TAG "DA"
#define DAC_AUDIO_16KHZ_BUFFER_SIZE 240
#define DAC_AUDIO_8KHZ_BUFFER_SIZE 120

#define MISO_PIN 36
#define MOSI_PIN 23
#define CLK_PIN 18
#define CS_PIN 5
#define SPI_QUEUE_SIZE 4

static uint8_t *audio_out_buf = NULL; // buffer sent to DAC for output
static uint8_t *tmp_audio_buf = NULL; // buffer holding 8 bit down converted mSBC frames
static uint8_t *audio_data = NULL; // data fetched from ring buffer
static size_t tmp_audio_buf_size = 0;
static TaskHandle_t audio_consumer_task;
static RingbufHandle_t audio_out_rb = NULL;
static sample_rate_t dac_sample_rate;
static uint8_t initialized = 0;
static uint8_t enabled = 0;
static uint8_t spi_tx_index = 0;
static int64_t last_incoming_buffer_us = 0;

static uint64_t sent_buf_counter = 0;
static uint64_t send_fail_counter = 0;
static uint64_t not_enabled_counter = 0;
static uint64_t no_samples_counter = 0;
static uint64_t enabled_counter = 0;
static uint64_t not_enough_samples_counter = 0;

static spi_device_handle_t spi;
static spi_bus_config_t spi_bus_cfg;
static spi_device_interface_config_t spi_dev_cfg;
static spi_transaction_t spi_transactions[SPI_QUEUE_SIZE];

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
    return 7 * dac_audio_get_buffer_size();
}


static void consume_audio_task_handler(void *arg) {
    RingbufHandle_t audio_out_rb = (RingbufHandle_t) arg;

    size_t buffered_samples_size;
    size_t dac_buf_size = dac_audio_get_buffer_size();
    size_t min_buffered_samples_size = dac_buf_size;
    size_t received_bytes = 0;

    ESP_LOGW(DA_TAG, "DAC audio buffer size is: %d", dac_buf_size);
    while (1) {
        if (!enabled) {
            if (not_enabled_counter++ % 250 == 0) {
                ESP_LOGW(DA_TAG, "Not enabled");
            }
            vTaskDelay(1);
            continue;
        }

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

        memcpy(audio_out_buf, audio_data, received_bytes);
        vRingbufferReturnItem(audio_out_rb, audio_data);

        audio_data = NULL;
        if (received_bytes < dac_buf_size) {
            // If we got less than what we asked for, fill the buffer with silence
            memset(audio_out_buf + received_bytes, 0, dac_buf_size - received_bytes);
        }
        
        if (enabled_counter++ % 250 == 0) {
            ESP_LOGW(DA_TAG, "Streaming data");
        }
        received_bytes = 0;
        // dac_continuous_write(dac_handle, audio_out_buf, dac_buf_size, NULL, DAC_WRITE_TIMEOUT_MS); // must finish in at most 16ms
        continue;
        
stream_silence:
        memset(audio_out_buf, 0, dac_buf_size / 2);
        // dac_continuous_write(dac_handle, audio_out_buf, dac_buf_size / 2, NULL, DAC_WRITE_TIMEOUT_MS); // must finish in at most 8ms
        continue;
    }
}

static void IRAM_ATTR post_spi_tx_callback(spi_transaction_t *tx) {
    spi_tx_index--;

    int64_t now = esp_timer_get_time();
    int64_t interval = now - last_incoming_buffer_us;
    last_incoming_buffer_us = now;
    if (sent_buf_counter++ % 100 == 0) {
        ESP_DRAM_LOGI(DA_TAG, "Send interval %"PRId64, interval);
    }
}

// void dac_audio_send_simple(uint8_t b1, uint8_t b2) {
//     spi_transaction_t *tx = &spi_transactions[spi_tx_index];
//     memset(tx, 0, sizeof(spi_transaction_t));

//     tx->tx_buffer = NULL;
//     tx->rx_buffer = NULL;
//     tx->tx_data[0] = b1;
//     tx->tx_data[1] = b2;
//     tx->length = 16;
//     tx->flags = SPI_TRANS_USE_TXDATA;

//     esp_err_t result = spi_device_transmit(spi, tx);
//     if (result == ESP_OK) {
//         // ESP_LOGI(DA_TAG, "Sent buffer");
//     }
// }

// void dac_audio_send(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buf) {
//     if (spi_tx_index == (SPI_QUEUE_SIZE - 1)) {
//         ESP_LOGE(DA_TAG, "Too many queued SPI transactions");
//         goto free_buffer;
//     }
    
//     spi_transaction_t *tx = &spi_transactions[spi_tx_index];
//     memset(tx, 0, sizeof(spi_transaction_t));
    
//     // Prepare scheduling audio buffer release after transmit
//     free_buf_msgs[spi_tx_index].pool = pool;
//     free_buf_msgs[spi_tx_index].buf = buf;

//     tx->user = &free_buf_msgs[spi_tx_index];
//     tx->tx_buffer = buf->bytes;
//     tx->rx_buffer = NULL;
//     tx->length = buf->size * 8;
//     tx->flags = 0;
    
//     spi_tx_index++;
    
//     esp_err_t result = spi_device_queue_trans(spi, tx, portMAX_DELAY);

//     if (result == ESP_OK) {
//         // ESP_LOGI(DA_TAG, "Sent buffer");
//         return;
//     }
    
//     ESP_LOGE(DA_TAG, "Unable to queue spi transaction. Error: %d", result);
//     // Reset the tx index counter
//     spi_tx_index--;

//     ESP_LOGE(DA_TAG, "Error: %d", result);
    
//     free_buffer:
//         dac_audio_schedule_used_buf_release(pool, buf, 0);
// }

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
    
    int16_t *src = (int16_t *) buf;
    uint8_t *dst = tmp_audio_buf;

    for (size_t i = 0; i < samples_to_write; i++) {
        float dither = ((float)rand() / (float)RAND_MAX) - 0.5f;

        float val = ((src[i] - INT16_MIN) >> 8) + dither;
        if (val < 0) {
            val = 0.0;
        } else if (val > 255) {
            val = 255.0;
        }
        *dst = (uint8_t) val;
        dst++;
        bytes_written++;
    }
    
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

    // memset(tmp_audio_buf, 0, tmp_audio_buf_size);

    if (status) {
        ESP_LOGW(DA_TAG, "Enabling DAC");
        // ESP_ERROR_CHECK(dac_continuous_enable(dac_handle));
        enabled = 1;
        return;
    }

    enabled = 0;
    ESP_LOGW(DA_TAG, "Disabling dac");
    // ESP_ERROR_CHECK(dac_continuous_disable(dac_handle));
}

void dac_audio_init(sample_rate_t sample_rate) {
    if (initialized) {
        return;
    }

    assert(sample_rate != SAMPLE_RATE_NONE);
    dac_sample_rate = sample_rate;

    size_t buf_size = dac_audio_get_buffer_size();
    uint16_t sample_rate_hz = (sample_rate == SAMPLE_RATE_16KHZ) ? 16000 : 8000;
    ESP_LOGI(DA_TAG, "Initializing DAC. Sample rate: %d. Buffer size: %d", sample_rate_hz, buf_size);

    // Setup buffers
    audio_out_buf = malloc(buf_size);
    assert(audio_out_buf != NULL);

    tmp_audio_buf_size = 2 * buf_size;
    tmp_audio_buf = malloc(tmp_audio_buf_size);
    assert(tmp_audio_buf != NULL);

    audio_out_rb = xRingbufferCreate(dac_audio_get_rb_size(), RINGBUF_TYPE_BYTEBUF);
    assert(audio_out_rb != NULL);

    BaseType_t r = xTaskCreatePinnedToCore(consume_audio_task_handler, "consume_audio", 2048, audio_out_rb, 15, &audio_consumer_task, 1);
    assert(r == pdPASS);

    // Setup bus
    spi_bus_cfg.miso_io_num = MISO_PIN; // we're not interested in reading
    spi_bus_cfg.mosi_io_num = MOSI_PIN;
    spi_bus_cfg.sclk_io_num = CLK_PIN;
    spi_bus_cfg.quadwp_io_num = -1;
    spi_bus_cfg.quadhd_io_num = -1;
    spi_bus_cfg.max_transfer_sz = buf_size;

    // Setup device
    spi_dev_cfg.clock_speed_hz = sample_rate_hz;
    spi_dev_cfg.mode = 0;
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

    vRingbufferDelete(audio_out_rb);
    audio_out_rb = NULL;

    ESP_LOGW(DA_TAG, "Freeing audio buf");
    free(audio_out_buf);
    audio_out_buf = NULL;
    ESP_LOGW(DA_TAG, "Freeing tmp audio buf");
    free(tmp_audio_buf);
    tmp_audio_buf = NULL;
    tmp_audio_buf_size = 0;
    initialized = 0;
}