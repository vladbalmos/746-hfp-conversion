#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/dac_continuous.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "dac_audio.h"

#define DA_TAG "DA"
#define DAC_DESCRIPTORS_NUM 8

#define DAC_PCM_BLOCK_DURATION_US 7500
#define DAC_WBS_PCM_SAMPLING_RATE_KHZ  16
#define DAC_BYTES_PER_SAMPLE  2
#define DAC_WBS_PCM_INPUT_DATA_SIZE (DAC_WBS_PCM_SAMPLING_RATE_KHZ * DAC_PCM_BLOCK_DURATION_US / 1000 * DAC_BYTES_PER_SAMPLE) // 240
#define DAC_AUDIO_BUFFER_SIZE DAC_WBS_PCM_INPUT_DATA_SIZE
#define DAC_AUDIO_RING_BUF_SIZE 7 * DAC_WBS_PCM_INPUT_DATA_SIZE // buffer 50ms
#define DAC_AUDIO_TMP_BUF_SIZE 2 * DAC_WBS_PCM_INPUT_DATA_SIZE

static uint8_t audio_out_buf[DAC_AUDIO_BUFFER_SIZE]; // buffer sent to DAC for output
static uint8_t tmp_audio_buf[DAC_AUDIO_TMP_BUF_SIZE]; // buffer holding 8 bit down converted mSBC frames
static TaskHandle_t audio_consumer_task;
static dac_continuous_handle_t dac_handle;
static dac_continuous_config_t dac_config;
static RingbufHandle_t audio_out_rb = NULL;

static void consume_buffers_task_handler(void *arg) {
    RingbufHandle_t rb = audio_out_rb;
    uint8_t *audio_data = NULL;
    size_t buffered_samples;
    // size_t samples_to_stream = DAC_WBS_PCM_INPUT_DATA_SIZE / 2;
    size_t samples_to_stream = 240;
    size_t min_buffered_samples = 2 * samples_to_stream;
    size_t received_samples = 0;

    while (1) {
        if (audio_out_rb == NULL) {
            vTaskDelay(1);
            continue;
        }

        buffered_samples = DAC_AUDIO_RING_BUF_SIZE - xRingbufferGetCurFreeSize(rb);
        
        if (buffered_samples < min_buffered_samples) {
            // ESP_LOGW(DA_TAG, "Not enough buffered samples");
            goto stream_silence;
        }

        audio_data = xRingbufferReceiveUpTo(rb, &received_samples, 0, DAC_AUDIO_BUFFER_SIZE);
        if (!received_samples) {
            // ESP_LOGW(DA_TAG, "No samples buffered");
            goto stream_silence;
        }

        memcpy(audio_out_buf, audio_data, received_samples);
        vRingbufferReturnItem(rb, audio_data);
        if (received_samples < DAC_AUDIO_BUFFER_SIZE) {
            // If we got less than what we asked for, fill the buffer with silence
            memset(audio_out_buf + received_samples, 0, DAC_AUDIO_BUFFER_SIZE - received_samples);
        }
        received_samples = 0;
        dac_continuous_write(dac_handle, audio_out_buf, DAC_AUDIO_BUFFER_SIZE, NULL, -1); // must finish in at most 16ms
        continue;
        
stream_silence:
        memset(audio_out_buf, 0, DAC_AUDIO_BUFFER_SIZE / 2);
        dac_continuous_write(dac_handle, audio_out_buf, DAC_AUDIO_BUFFER_SIZE / 2, NULL, -1); // must finish in at most 8ms
        continue;
    }
}

static dac_audio_buffer_t *init_audio_buffer(size_t buffer_size) {
    uint8_t *bytes = malloc(buffer_size);
    dac_audio_buffer_t *audio_buffer = malloc(sizeof(dac_audio_buffer_t));
    
    if (bytes == NULL) {
        return NULL;
    }

    if (audio_buffer == NULL) {
        return NULL;
    }
    
    audio_buffer->bytes = bytes;
    audio_buffer->size = buffer_size;
    audio_buffer->next = NULL;
    return audio_buffer;
}

uint8_t dac_audio_remaining_free_buffer_slots(dac_audio_buffer_pool_t *pool) {
    if (pool == NULL) {
        return 0;
    }
    
    return pool->size - pool->available_buffers_count;
}

uint8_t dac_audio_enqueue_free_buffer(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buf) {
    if (!dac_audio_remaining_free_buffer_slots(pool)) {
        return 0;
    }

    memset(buf->bytes, 0, buf->size);
    buf->next = NULL;

    pool->available_buffers_count++;

    if (pool->free_buffers_queue_head == NULL) {
        pool->free_buffers_queue_head = buf;
        pool->free_buffers_queue_tail = buf;
        return 1;
    }
    
    pool->free_buffers_queue_tail->next = buf;
    pool->free_buffers_queue_tail = buf;
    return 1;
}

uint8_t dac_audio_enqueue_free_buffer_safe(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buf, TickType_t wait) {
    if (!xSemaphoreTake(pool->free_buff_sem, wait)) {
        ESP_LOGE(DA_TAG, "Free buffer pool is in use (enqueue)");
        return 0;
    }
    
    uint8_t ret = dac_audio_enqueue_free_buffer(pool, buf);
    xSemaphoreGive(pool->free_buff_sem);
    return ret;
}

dac_audio_buffer_t *dac_audio_take_free_buffer(dac_audio_buffer_pool_t *pool) {
    if (pool->free_buffers_queue_head == NULL) {
        return NULL;
    }
    
    pool->available_buffers_count--;
    
    dac_audio_buffer_t *buf = pool->free_buffers_queue_head;
    
    if (buf == pool->free_buffers_queue_tail) {
        pool->free_buffers_queue_head = NULL;
        pool->free_buffers_queue_tail = NULL;
    } else {
        pool->free_buffers_queue_head = buf->next;
    }

    return buf;
}

dac_audio_buffer_t *dac_audio_take_free_buffer_safe(dac_audio_buffer_pool_t *pool, TickType_t wait) {
    if (!xSemaphoreTake(pool->free_buff_sem, wait)) {
        ESP_LOGE(DA_TAG, "Free buffer pool is in use (take)");
        return NULL;
    }

    dac_audio_buffer_t *audio_buf = dac_audio_take_free_buffer(pool);
    xSemaphoreGive(pool->free_buff_sem);
    return audio_buf;
}

uint8_t dac_audio_enqueue_ready_buffer(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buf) {
    if (!dac_audio_remaining_ready_buffer_slots(pool)) {
        return 0;
    }
    buf->next = NULL;

    pool->ready_buffers_count++;

    if (pool->ready_buffers_queue_head == NULL) {
        pool->ready_buffers_queue_head = buf;
        pool->ready_buffers_queue_tail = buf;
        return 1;
    }
    
    pool->ready_buffers_queue_tail->next = buf;
    pool->ready_buffers_queue_tail = buf;
    return 1;
}

uint8_t dac_audio_enqueue_ready_buffer_safe(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buf, TickType_t wait) {
    if (!xSemaphoreTake(pool->ready_buff_sem, wait)) {
        ESP_LOGE(DA_TAG, "Ready buffer pool is in use (enqueue)");
        return 0;
    }
    
    uint8_t ret = dac_audio_enqueue_ready_buffer(pool, buf);
    xSemaphoreGive(pool->ready_buff_sem);
    return ret;
}

dac_audio_buffer_t *dac_audio_take_ready_buffer(dac_audio_buffer_pool_t *pool) {
    if (pool->ready_buffers_queue_head == NULL) {
        return NULL;
    }
    
    pool->ready_buffers_count--;
    
    dac_audio_buffer_t *buf = pool->ready_buffers_queue_head;
    
    if (buf == pool->ready_buffers_queue_tail) {
        pool->ready_buffers_queue_head = NULL;
        pool->ready_buffers_queue_tail = NULL;
    } else {
        pool->ready_buffers_queue_head = buf->next;
    }

    return buf;
}

dac_audio_buffer_t *dac_audio_take_ready_buffer_safe(dac_audio_buffer_pool_t *pool, TickType_t wait) {
    if (!xSemaphoreTake(pool->ready_buff_sem, wait)) {
        ESP_LOGE(DA_TAG, "Ready buffer pool is in use (take)");
        return NULL;
    }

    dac_audio_buffer_t *buf = dac_audio_take_ready_buffer(pool);
    xSemaphoreGive(pool->ready_buff_sem);
    return buf;
}

uint8_t dac_audio_remaining_ready_buffer_slots(dac_audio_buffer_pool_t *pool) {
    if (pool == NULL) {
        return 0;
    }
    
    return pool->size - pool->ready_buffers_count;
}

void dac_audio_reset_buffer_pool(dac_audio_buffer_pool_t *pool) {
    while (pool->ready_buffers_count) {
        assert(dac_audio_enqueue_free_buffer(pool, dac_audio_take_ready_buffer(pool)) == 1);
    }
    ESP_LOGI(DA_TAG, "Buffer pool reset\n");
    ESP_LOGI(DA_TAG, "Available buffers count %d", pool->available_buffers_count);
    ESP_LOGI(DA_TAG, "Ready buffers count %d", pool->ready_buffers_count);
    ESP_LOGI(DA_TAG, "Remaining free buffer slots %d", dac_audio_remaining_free_buffer_slots(pool));
    ESP_LOGI(DA_TAG, "Remaining ready buffer slots %d", dac_audio_remaining_ready_buffer_slots(pool));
}

dac_audio_buffer_pool_t *dac_audio_init_buffer_pool(uint8_t pool_size, size_t buffer_size) {
    dac_audio_buffer_pool_t *buffer_pool = malloc(sizeof(dac_audio_buffer_pool_t));
    

    if (buffer_pool == NULL) {
        return NULL;
    }
    buffer_pool->free_buffers_queue_head = NULL;
    buffer_pool->free_buffers_queue_tail = NULL;
    buffer_pool->free_buff_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(buffer_pool->free_buff_sem);

    buffer_pool->ready_buffers_queue_head = NULL;
    buffer_pool->ready_buffers_queue_tail = NULL;
    buffer_pool->ready_buff_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(buffer_pool->ready_buff_sem);

    buffer_pool->ready_buffers_count = 0;
    buffer_pool->available_buffers_count = 0;
    buffer_pool->size = pool_size;
    buffer_pool->buffer_size = buffer_size;
    
    
    for (uint8_t i = 0; i < pool_size; i++) {
        dac_audio_buffer_t *buffer = init_audio_buffer(buffer_size);
        if (buffer == NULL) {
            return NULL;
        }
        assert(dac_audio_enqueue_free_buffer(buffer_pool, buffer) == 1);
    }
    ESP_LOGI(DA_TAG, "Initialized DAC audio buffer pool of %d buffers using %d max sample count", pool_size, buffer_size);
    ESP_LOGI(DA_TAG, "Buffer pool reset\n");
    ESP_LOGI(DA_TAG, "Available buffers count %d", buffer_pool->available_buffers_count);
    ESP_LOGI(DA_TAG, "Ready buffers count %d", buffer_pool->ready_buffers_count);
    ESP_LOGI(DA_TAG, "Remaining free buffer slots %d", dac_audio_remaining_free_buffer_slots(buffer_pool));
    ESP_LOGI(DA_TAG, "Remaining ready buffer slots %d", dac_audio_remaining_ready_buffer_slots(buffer_pool));
    
    return buffer_pool;
}

void dac_audio_send(const uint8_t *buf, size_t size) {
    size_t samples_to_write = size / 2;
    size_t bytes_written = 0;
    
    if (samples_to_write > DAC_AUDIO_TMP_BUF_SIZE) {
        samples_to_write = DAC_AUDIO_TMP_BUF_SIZE;
        ESP_LOGW(DA_TAG, "Audio buffer to small. Max value is %d, received: %d", DAC_AUDIO_TMP_BUF_SIZE, samples_to_write);
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
        ESP_LOGW(DA_TAG, "Failed to write audio data to ring buffer");
    }
    memset(tmp_audio_buf, 0, DAC_AUDIO_TMP_BUF_SIZE);
}

void dac_audio_enable(uint8_t status) {
    memset(tmp_audio_buf, 0, DAC_AUDIO_TMP_BUF_SIZE);
    if (status) {
        audio_out_rb = xRingbufferCreate(DAC_AUDIO_RING_BUF_SIZE, RINGBUF_TYPE_BYTEBUF);
        assert(audio_out_rb != NULL);

        ESP_ERROR_CHECK(dac_continuous_enable(dac_handle));
        vTaskResume(audio_consumer_task);
        return;
    }
    vTaskSuspend(audio_consumer_task);
    ESP_ERROR_CHECK(dac_continuous_disable(dac_handle));
    vRingbufferDelete(audio_out_rb);
    audio_out_rb = NULL;
}

void dac_audio_init(dac_audio_sample_rate_t sample_rate) {
    BaseType_t r = xTaskCreatePinnedToCore(consume_buffers_task_handler, "consume_buf", 4096, NULL, 15, &audio_consumer_task, 1);
    assert(r == pdPASS);
    vTaskSuspend(audio_consumer_task);
    
    // Init dac
    dac_config.chan_mask = DAC_CHANNEL_MASK_CH0;
    dac_config.desc_num = DAC_DESCRIPTORS_NUM;
    dac_config.buf_size = DAC_AUDIO_BUFFER_SIZE;
    dac_config.freq_hz = (sample_rate == DAC_SAMPLE_RATE_16KHZ) ? 16000 : 44100;
    dac_config.clk_src = DAC_DIGI_CLK_SRC_APLL;
    ESP_ERROR_CHECK(dac_continuous_new_channels(&dac_config, &dac_handle));
}
