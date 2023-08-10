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
static dac_audio_sample_rate_t dac_sample_rate;
static uint8_t initialized = 0;
static uint8_t enabled = 0;
uint64_t send_fail_counter = 0;
uint64_t not_enabled_counter = 0;
uint64_t no_samples_counter = 0;
uint64_t enabled_counter = 0;
uint64_t not_enough_samples_counter = 0;
/**
 * Return the buffer size based on current sample rate
 */
static inline size_t dac_audio_get_buffer_size() {
    if (dac_sample_rate == DAC_SAMPLE_RATE_16KHZ) {
        return DAC_AUDIO_16KHZ_BUFFER_SIZE;
    }

    if (dac_sample_rate == DAC_SAMPLE_RATE_8KHZ) {
        return DAC_AUDIO_8KHZ_BUFFER_SIZE;
    }
    
    return 0;
}

static inline size_t dac_audio_get_rb_size() {
    return 7 * dac_audio_get_buffer_size();
}

static void consume_buffers_task_handler(void *arg) {
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

// static dac_audio_buffer_t *init_audio_buffer(size_t buffer_size) {
//     uint8_t *bytes = malloc(buffer_size);
//     dac_audio_buffer_t *audio_buffer = malloc(sizeof(dac_audio_buffer_t));
    
//     if (bytes == NULL) {
//         return NULL;
//     }

//     if (audio_buffer == NULL) {
//         return NULL;
//     }
    
//     audio_buffer->bytes = bytes;
//     audio_buffer->size = buffer_size;
//     audio_buffer->next = NULL;
//     return audio_buffer;
// }

// uint8_t dac_audio_remaining_free_buffer_slots(dac_audio_buffer_pool_t *pool) {
//     if (pool == NULL) {
//         return 0;
//     }
    
//     return pool->size - pool->available_buffers_count;
// }

// uint8_t dac_audio_enqueue_free_buffer(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buf) {
//     if (!dac_audio_remaining_free_buffer_slots(pool)) {
//         return 0;
//     }

//     memset(buf->bytes, 0, buf->size);
//     buf->next = NULL;

//     pool->available_buffers_count++;

//     if (pool->free_buffers_queue_head == NULL) {
//         pool->free_buffers_queue_head = buf;
//         pool->free_buffers_queue_tail = buf;
//         return 1;
//     }
    
//     pool->free_buffers_queue_tail->next = buf;
//     pool->free_buffers_queue_tail = buf;
//     return 1;
// }

// uint8_t dac_audio_enqueue_free_buffer_safe(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buf, TickType_t wait) {
//     if (!xSemaphoreTake(pool->free_buff_sem, wait)) {
//         ESP_LOGE(DA_TAG, "Free buffer pool is in use (enqueue)");
//         return 0;
//     }
    
//     uint8_t ret = dac_audio_enqueue_free_buffer(pool, buf);
//     xSemaphoreGive(pool->free_buff_sem);
//     return ret;
// }

// dac_audio_buffer_t *dac_audio_take_free_buffer(dac_audio_buffer_pool_t *pool) {
//     if (pool->free_buffers_queue_head == NULL) {
//         return NULL;
//     }
    
//     pool->available_buffers_count--;
    
//     dac_audio_buffer_t *buf = pool->free_buffers_queue_head;
    
//     if (buf == pool->free_buffers_queue_tail) {
//         pool->free_buffers_queue_head = NULL;
//         pool->free_buffers_queue_tail = NULL;
//     } else {
//         pool->free_buffers_queue_head = buf->next;
//     }

//     return buf;
// }

// dac_audio_buffer_t *dac_audio_take_free_buffer_safe(dac_audio_buffer_pool_t *pool, TickType_t wait) {
//     if (!xSemaphoreTake(pool->free_buff_sem, wait)) {
//         ESP_LOGE(DA_TAG, "Free buffer pool is in use (take)");
//         return NULL;
//     }

//     dac_audio_buffer_t *audio_buf = dac_audio_take_free_buffer(pool);
//     xSemaphoreGive(pool->free_buff_sem);
//     return audio_buf;
// }

// uint8_t dac_audio_enqueue_ready_buffer(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buf) {
//     if (!dac_audio_remaining_ready_buffer_slots(pool)) {
//         return 0;
//     }
//     buf->next = NULL;

//     pool->ready_buffers_count++;

//     if (pool->ready_buffers_queue_head == NULL) {
//         pool->ready_buffers_queue_head = buf;
//         pool->ready_buffers_queue_tail = buf;
//         return 1;
//     }
    
//     pool->ready_buffers_queue_tail->next = buf;
//     pool->ready_buffers_queue_tail = buf;
//     return 1;
// }

// uint8_t dac_audio_enqueue_ready_buffer_safe(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buf, TickType_t wait) {
//     if (!xSemaphoreTake(pool->ready_buff_sem, wait)) {
//         ESP_LOGE(DA_TAG, "Ready buffer pool is in use (enqueue)");
//         return 0;
//     }
    
//     uint8_t ret = dac_audio_enqueue_ready_buffer(pool, buf);
//     xSemaphoreGive(pool->ready_buff_sem);
//     return ret;
// }

// dac_audio_buffer_t *dac_audio_take_ready_buffer(dac_audio_buffer_pool_t *pool) {
//     if (pool->ready_buffers_queue_head == NULL) {
//         return NULL;
//     }
    
//     pool->ready_buffers_count--;
    
//     dac_audio_buffer_t *buf = pool->ready_buffers_queue_head;
    
//     if (buf == pool->ready_buffers_queue_tail) {
//         pool->ready_buffers_queue_head = NULL;
//         pool->ready_buffers_queue_tail = NULL;
//     } else {
//         pool->ready_buffers_queue_head = buf->next;
//     }

//     return buf;
// }

// dac_audio_buffer_t *dac_audio_take_ready_buffer_safe(dac_audio_buffer_pool_t *pool, TickType_t wait) {
//     if (!xSemaphoreTake(pool->ready_buff_sem, wait)) {
//         ESP_LOGE(DA_TAG, "Ready buffer pool is in use (take)");
//         return NULL;
//     }

//     dac_audio_buffer_t *buf = dac_audio_take_ready_buffer(pool);
//     xSemaphoreGive(pool->ready_buff_sem);
//     return buf;
// }

// uint8_t dac_audio_remaining_ready_buffer_slots(dac_audio_buffer_pool_t *pool) {
//     if (pool == NULL) {
//         return 0;
//     }
    
//     return pool->size - pool->ready_buffers_count;
// }

// void dac_audio_reset_buffer_pool(dac_audio_buffer_pool_t *pool) {
//     while (pool->ready_buffers_count) {
//         assert(dac_audio_enqueue_free_buffer(pool, dac_audio_take_ready_buffer(pool)) == 1);
//     }
//     ESP_LOGI(DA_TAG, "Buffer pool reset\n");
//     ESP_LOGI(DA_TAG, "Available buffers count %d", pool->available_buffers_count);
//     ESP_LOGI(DA_TAG, "Ready buffers count %d", pool->ready_buffers_count);
//     ESP_LOGI(DA_TAG, "Remaining free buffer slots %d", dac_audio_remaining_free_buffer_slots(pool));
//     ESP_LOGI(DA_TAG, "Remaining ready buffer slots %d", dac_audio_remaining_ready_buffer_slots(pool));
// }

// dac_audio_buffer_pool_t *dac_audio_init_buffer_pool(uint8_t pool_size, size_t buffer_size) {
//     dac_audio_buffer_pool_t *buffer_pool = malloc(sizeof(dac_audio_buffer_pool_t));
    

//     if (buffer_pool == NULL) {
//         return NULL;
//     }
//     buffer_pool->free_buffers_queue_head = NULL;
//     buffer_pool->free_buffers_queue_tail = NULL;
//     buffer_pool->free_buff_sem = xSemaphoreCreateBinary();
//     xSemaphoreGive(buffer_pool->free_buff_sem);

//     buffer_pool->ready_buffers_queue_head = NULL;
//     buffer_pool->ready_buffers_queue_tail = NULL;
//     buffer_pool->ready_buff_sem = xSemaphoreCreateBinary();
//     xSemaphoreGive(buffer_pool->ready_buff_sem);

//     buffer_pool->ready_buffers_count = 0;
//     buffer_pool->available_buffers_count = 0;
//     buffer_pool->size = pool_size;
//     buffer_pool->buffer_size = buffer_size;
    
    
//     for (uint8_t i = 0; i < pool_size; i++) {
//         dac_audio_buffer_t *buffer = init_audio_buffer(buffer_size);
//         if (buffer == NULL) {
//             return NULL;
//         }
//         assert(dac_audio_enqueue_free_buffer(buffer_pool, buffer) == 1);
//     }
//     ESP_LOGI(DA_TAG, "Initialized DAC audio buffer pool of %d buffers using %d max sample count", pool_size, buffer_size);
//     ESP_LOGI(DA_TAG, "Buffer pool reset\n");
//     ESP_LOGI(DA_TAG, "Available buffers count %d", buffer_pool->available_buffers_count);
//     ESP_LOGI(DA_TAG, "Ready buffers count %d", buffer_pool->ready_buffers_count);
//     ESP_LOGI(DA_TAG, "Remaining free buffer slots %d", dac_audio_remaining_free_buffer_slots(buffer_pool));
//     ESP_LOGI(DA_TAG, "Remaining ready buffer slots %d", dac_audio_remaining_ready_buffer_slots(buffer_pool));
    
//     return buffer_pool;
// }

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

dac_audio_sample_rate_t dac_audio_get_sample_rate() {
    return dac_sample_rate;
}

void dac_audio_init(dac_audio_sample_rate_t sample_rate) {
    if (initialized) {
        return;
    }

    assert(sample_rate != DAC_SAMPLE_RATE_NONE);
    dac_sample_rate = sample_rate;

    size_t buf_size = dac_audio_get_buffer_size();
    uint16_t sample_rate_hz = (sample_rate == DAC_SAMPLE_RATE_16KHZ) ? 16000 : 8000;
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

    BaseType_t r = xTaskCreate(consume_buffers_task_handler, "consume_buf", 4096, audio_out_rb, 15, &audio_consumer_task);
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
    dac_sample_rate = DAC_SAMPLE_RATE_NONE;

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
