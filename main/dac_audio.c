#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/dac_continuous.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "dac_audio.h"

#define DA_TAG "DA"
#define DAC_DESCRIPTORS_NUM 8

static dac_continuous_handle_t dac_handle;
static dac_continuous_config_t dac_config;
static TaskHandle_t audio_consumer_task;

static QueueHandle_t dac_events_queue = NULL;
static QueueHandle_t buffers_to_free_queue = NULL;

static void free_buffers_task_handler(void *arg) {
    dac_audio_free_buf_msg_t msg;
    
    while (1) {
        if (!xQueueReceive(buffers_to_free_queue, &msg, portMAX_DELAY)) {
            continue;
        }
        
        dac_audio_enqueue_free_buffer_safe(msg.pool, msg.buf, portMAX_DELAY);
        // ESP_LOGW(DA_TAG, "Releasing buffer");
    }
}

static void consume_buffers_task_handler(void *arg) {
    dac_audio_buffer_pool_t *pool = (dac_audio_buffer_pool_t *) arg;
    while (1) {
        dac_audio_buffer_t *audio_buf = dac_audio_take_ready_buffer_safe(pool, 0);
        if (audio_buf == NULL) {
            // ESP_LOGI(DA_TAG, "No audio buffer present");
            // vTaskDelay(1);
            continue;
        }
        
        dac_event_data_t evt_data;
        size_t bytes_written = 0;

        
        while (bytes_written < audio_buf->size) {
            if (xQueueReceive(dac_events_queue, &evt_data, 0) != pdTRUE) {
                break;
            }
            size_t loaded_bytes = 0;
            ESP_ERROR_CHECK(
                dac_continuous_write_asynchronously(dac_handle, evt_data.buf, evt_data.buf_size,
                                                    audio_buf->bytes + bytes_written, audio_buf->size - bytes_written,
                                                    &loaded_bytes)
            );
            bytes_written += loaded_bytes;
        }

        // ESP_LOGI(DA_TAG, "Wrote data");
        dac_audio_schedule_used_buf_release(pool, audio_buf, 0);
    }
}

static bool IRAM_ATTR dac_on_conversion_done_callback(dac_continuous_handle_t handle, const dac_event_data_t *event, void *user_data) {
    QueueHandle_t que = (QueueHandle_t)user_data;
    BaseType_t need_awoke;

    /* When the queue is full, drop the oldest item */
    if (xQueueIsQueueFullFromISR(que)) {
        dac_event_data_t dummy;
        xQueueReceiveFromISR(que, &dummy, &need_awoke);
    }

    /* Send the event from callback */
    xQueueSendFromISR(que, event, &need_awoke);
    return need_awoke;
}

static dac_audio_buffer_t *init_audio_buffer(size_t buffer_size) {
    // uint8_t *bytes = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
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

void dac_audio_schedule_used_buf_release(dac_audio_buffer_pool_t *return_pool, dac_audio_buffer_t *buf, uint8_t fromISR) {
    dac_audio_free_buf_msg_t msg = {
        .pool = return_pool,
        .buf = buf
    };
    
    if (fromISR) {
        if ((xQueueSendFromISR(buffers_to_free_queue, &msg, NULL) != pdTRUE)) {
            ESP_DRAM_LOGE(DA_TAG, "Unable to schedule buffer release (ISR)");
        }
    } else {
        if ((xQueueSend(buffers_to_free_queue, &msg, 10 / portTICK_PERIOD_MS)) != pdTRUE) {
            ESP_LOGE(DA_TAG, "Unable to schedule buffer release (non-ISR)");
        }
    }
}

void dac_audio_send(uint8_t *buf, size_t size) {
    dac_continuous_write(dac_handle, buf, size, NULL, -1);
}

void dac_audio_init(dac_audio_sample_rate_t sample_rate, dac_audio_buffer_pool_t *buffer_pool, size_t buffer_size) {
    // Create task to free used buffers
    buffers_to_free_queue = xQueueCreate(8, sizeof(dac_audio_free_buf_msg_t));
    assert(buffers_to_free_queue != NULL);
    
    BaseType_t r = xTaskCreate(free_buffers_task_handler, "free_buf", 2048, NULL, 10, NULL);
    assert(r == pdPASS);

    // Create task & queue to consume audio buffers
    dac_events_queue = xQueueCreate(8, sizeof(dac_event_data_t));
    assert(dac_events_queue != NULL);

    r = xTaskCreatePinnedToCore(consume_buffers_task_handler, "consume_buf", 4096, buffer_pool, 15, &audio_consumer_task, 1);
    assert(r == pdPASS);
    vTaskSuspend(audio_consumer_task);
    
    // Init dac
    dac_config.chan_mask = DAC_CHANNEL_MASK_CH0;
    dac_config.desc_num = DAC_DESCRIPTORS_NUM;
    dac_config.buf_size = buffer_size;
    // dac_config.buf_size = 240;
    dac_config.freq_hz = (sample_rate == DAC_SAMPLE_RATE_16KHZ) ? 16000 : 44100;
    dac_config.clk_src = DAC_DIGI_CLK_SRC_APLL;
    ESP_ERROR_CHECK(dac_continuous_new_channels(&dac_config, &dac_handle));
    
    dac_event_callbacks_t dac_callbacks = {
        .on_convert_done = dac_on_conversion_done_callback,
        .on_stop = NULL,
    };

    ESP_ERROR_CHECK(dac_continuous_register_event_callback(dac_handle, &dac_callbacks, dac_events_queue));
}

void dac_audio_enable(uint8_t status) {
    if (status) {
        vTaskResume(audio_consumer_task);
        ESP_ERROR_CHECK(dac_continuous_enable(dac_handle));
        ESP_ERROR_CHECK(dac_continuous_start_async_writing(dac_handle));
        return;
    }
    vTaskSuspend(audio_consumer_task);
    ESP_ERROR_CHECK(dac_continuous_stop_async_writing(dac_handle));
    ESP_ERROR_CHECK(dac_continuous_disable(dac_handle));
}