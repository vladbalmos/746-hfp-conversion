#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
// #include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/dac_continuous.h"
#include "esp_log.h"
#include "dac_audio.h"

#define DA_TAG "DA"
#define DAC_DESCRIPTORS_NUM 8
#define DAC_WRITE_TIMEOUT_MS 20

#define DAC_AUDIO_16KHZ_BUFFER_SIZE 240
#define DAC_AUDIO_8KHZ_BUFFER_SIZE 120

static uint8_t *audio_out_buf = NULL; // buffer sent to DAC for output
static uint8_t *tmp_audio_buf = NULL; // buffer holding 8 bit down converted mSBC frames
static uint8_t *audio_data = NULL; // data fetched from ring buffer
static size_t tmp_audio_buf_size = 0;
static TaskHandle_t audio_consumer_task;
static dac_continuous_handle_t dac_handle;
static dac_continuous_config_t dac_config;
static RingbufHandle_t audio_out_rb = NULL;
static sample_rate_t dac_sample_rate;
static uint8_t initialized = 0;
static uint8_t enabled = 0;

static uint64_t send_fail_counter = 0;
static uint64_t not_enabled_counter = 0;
static uint64_t no_samples_counter = 0;
static uint64_t enabled_counter = 0;
static uint64_t not_enough_samples_counter = 0;

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
        dac_continuous_write(dac_handle, audio_out_buf, dac_buf_size, NULL, DAC_WRITE_TIMEOUT_MS); // must finish in at most 16ms
        continue;
        
stream_silence:
        memset(audio_out_buf, 0, dac_buf_size / 2);
        dac_continuous_write(dac_handle, audio_out_buf, dac_buf_size / 2, NULL, DAC_WRITE_TIMEOUT_MS); // must finish in at most 8ms
        continue;
    }
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

    memset(tmp_audio_buf, 0, tmp_audio_buf_size);

    if (status) {
        ESP_LOGW(DA_TAG, "Enabling DAC");
        ESP_ERROR_CHECK(dac_continuous_enable(dac_handle));
        enabled = 1;
        return;
    }

    enabled = 0;
    ESP_LOGW(DA_TAG, "Disabling dac");
    ESP_ERROR_CHECK(dac_continuous_disable(dac_handle));
}

sample_rate_t dac_audio_get_sample_rate() {
    return dac_sample_rate;
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
    
    audio_out_buf = malloc(buf_size);
    assert(audio_out_buf != NULL);

    tmp_audio_buf_size = 2 * buf_size;
    tmp_audio_buf = malloc(tmp_audio_buf_size);
    assert(tmp_audio_buf != NULL);
    
    // Init dac
    dac_config.chan_mask = DAC_CHANNEL_MASK_CH0;
    dac_config.desc_num = DAC_DESCRIPTORS_NUM;
    dac_config.buf_size = buf_size;
    dac_config.freq_hz = sample_rate_hz;
    dac_config.clk_src = DAC_DIGI_CLK_SRC_APLL;
    ESP_ERROR_CHECK(dac_continuous_new_channels(&dac_config, &dac_handle));

    audio_out_rb = xRingbufferCreate(dac_audio_get_rb_size(), RINGBUF_TYPE_BYTEBUF);
    assert(audio_out_rb != NULL);

    BaseType_t r = xTaskCreatePinnedToCore(consume_audio_task_handler, "consume_audio", 2048, audio_out_rb, 15, &audio_consumer_task, 1);
    assert(r == pdPASS);
    
    initialized = 1;
}

void dac_audio_deinit() {
    if (!initialized) {
        return;
    }

    ESP_LOGW(DA_TAG, "Unintializing dac");
    dac_audio_enable(0);
    ESP_LOGW(DA_TAG, "Releasing channels");
    ESP_ERROR_CHECK(dac_continuous_del_channels(dac_handle));
    dac_sample_rate = SAMPLE_RATE_NONE;

    ESP_LOGW(DA_TAG, "Deleting task");
    vTaskDelete(audio_consumer_task);

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
