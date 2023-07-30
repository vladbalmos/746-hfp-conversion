#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "dac_audio.h"

#define DA_TAG "DA"

#define MISO_PIN 12
#define MOSI_PIN 13
#define CLK_PIN 14
#define CS_PIN 15
#define SPI_QUEUE_SIZE 16
// #define SPI_QUEUE_SIZE 2

static spi_device_handle_t spi;
static spi_bus_config_t spi_bus_cfg;
static spi_device_interface_config_t spi_dev_cfg;
static spi_transaction_t spi_transactions[SPI_QUEUE_SIZE];
static dac_audio_free_buf_msg_t free_buf_msgs[SPI_QUEUE_SIZE];
static uint8_t spi_tx_index = 0;
static QueueHandle_t buffers_to_free_queue = NULL;
static int64_t last_incoming_buffer_us = 0;
static int sent_buf_count = 0;

// static uint8_t bytes[4] = {250, 2048, 4095, 1024};

static void free_buffers_task_handler(void *arg) {
    dac_audio_free_buf_msg_t msg;
    
    while (1) {
        if (!xQueueReceive(buffers_to_free_queue, &msg, portMAX_DELAY)) {
            continue;
        }
        
        dac_audio_enqueue_free_buffer_safe(msg.pool, msg.buf, portMAX_DELAY);
    }
}

static dac_audio_buffer_t *init_audio_buffer(uint16_t buffer_size) {
    uint8_t *bytes = heap_caps_malloc(buffer_size * sizeof(uint16_t), MALLOC_CAP_DMA);
    dac_audio_buffer_t *audio_buffer = malloc(sizeof(dac_audio_buffer_t));
    
    if (bytes == NULL) {
        return NULL;
    }

    if (audio_buffer == NULL) {
        return NULL;
    }
    
    audio_buffer->bytes = bytes;
    audio_buffer->samples = buffer_size;
    audio_buffer->size = buffer_size * sizeof(uint16_t);
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

static void IRAM_ATTR post_spi_tx_callback(spi_transaction_t *tx) {
    spi_tx_index--;

    int64_t now = esp_timer_get_time();
    int64_t interval = now - last_incoming_buffer_us;
    last_incoming_buffer_us = now;
    if (sent_buf_count++ % 100 == 0) {
        ESP_DRAM_LOGI(DA_TAG, "Send interval %"PRId64, interval);
    }
    // ESP_DRAM_LOGI(DA_TAG, "Finished sending buffer %"PRId64, interval);
    
    dac_audio_free_buf_msg_t *msg = (dac_audio_free_buf_msg_t *) tx->user;
    dac_audio_schedule_used_buf_release(msg->pool, msg->buf, 1);
}

void dac_audio_send_simple(uint8_t b1, uint8_t b2) {
    spi_transaction_t *tx = &spi_transactions[spi_tx_index];
    memset(tx, 0, sizeof(spi_transaction_t));

    tx->tx_buffer = NULL;
    tx->rx_buffer = NULL;
    tx->tx_data[0] = b1;
    tx->tx_data[1] = b2;
    tx->length = 16;
    tx->flags = SPI_TRANS_USE_TXDATA;

    esp_err_t result = spi_device_transmit(spi, tx);
    if (result == ESP_OK) {
        // ESP_LOGI(DA_TAG, "Sent buffer");
    }
}

void dac_audio_send(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buf) {
    if (spi_tx_index == (SPI_QUEUE_SIZE - 1)) {
        ESP_LOGE(DA_TAG, "Too many queued SPI transactions");
        goto free_buffer;
    }
    
    spi_transaction_t *tx = &spi_transactions[spi_tx_index];
    memset(tx, 0, sizeof(spi_transaction_t));
    
    // Prepare scheduling audio buffer release after transmit
    free_buf_msgs[spi_tx_index].pool = pool;
    free_buf_msgs[spi_tx_index].buf = buf;

    tx->user = &free_buf_msgs[spi_tx_index];
    // tx->tx_buffer = buf->bytes;
    tx->tx_data[0] = buf->bytes[1];
    tx->tx_data[1] = buf->bytes[0];
    tx->tx_data[2] = buf->bytes[3];
    tx->tx_data[3] = buf->bytes[2];
    ESP_LOGI(DA_TAG, "%d %d %d %d", buf->bytes[0], buf->bytes[1], buf->bytes[2], buf->bytes[3]);
    tx->rx_buffer = NULL;
    tx->length = 32;
    // tx->length = buf->size * 8;
    // tx->flags = 0;
    tx->flags = SPI_TRANS_USE_TXDATA;
    
    esp_err_t result = spi_device_transmit(spi, tx);

    spi_tx_index++;
    
    // esp_err_t result = spi_device_queue_trans(spi, tx, portMAX_DELAY);
    // spi_device_get_trans_result(spi, &tx, portMAX_DELAY);
    if (result == ESP_OK) {
        // ESP_LOGI(DA_TAG, "Sent buffer");
        return;
    }
    
    ESP_LOGE(DA_TAG, "Unable to queue spi transaction. Error: %d", result);
    // Reset the tx index counter
    spi_tx_index--;

    ESP_LOGE(DA_TAG, "Error: %d", result);
    
    free_buffer:
        dac_audio_schedule_used_buf_release(pool, buf, 0);
}

void dac_audio_reset_buffer_pool(dac_audio_buffer_pool_t *pool) {
    while (pool->ready_buffers_count) {
        assert(dac_audio_enqueue_free_buffer(pool, dac_audio_take_ready_buffer(pool)) == 1);
    }
    ESP_LOGD(DA_TAG, "Buffer pool reset\n");
    ESP_LOGD(DA_TAG, "Available buffers count %d", pool->available_buffers_count);
    ESP_LOGD(DA_TAG, "Read buffers count %d", pool->ready_buffers_count);
    ESP_LOGD(DA_TAG, "Remaining free buffer slots %d", dac_audio_remaining_free_buffer_slots(pool));
    ESP_LOGD(DA_TAG, "Remaining ready buffer slots %d", dac_audio_remaining_ready_buffer_slots(pool));
}

dac_audio_buffer_pool_t *dac_audio_init_buffer_pool(uint8_t pool_size, uint16_t buffer_size) {
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

void dac_audio_init(dac_audio_sample_rate_t sample_rate) {
    // Create task to free used buffers
    buffers_to_free_queue = xQueueCreate(8, sizeof(dac_audio_free_buf_msg_t));
    assert(buffers_to_free_queue != NULL);
    
    BaseType_t r = xTaskCreate(free_buffers_task_handler, "free_buf", 2048, NULL, 10, NULL);
    assert(r == pdPASS);

    // Setup bus
    spi_bus_cfg.miso_io_num = MISO_PIN; // we're not interested in reading
    spi_bus_cfg.mosi_io_num = MOSI_PIN;
    spi_bus_cfg.sclk_io_num = CLK_PIN;
    spi_bus_cfg.quadwp_io_num = -1;
    spi_bus_cfg.quadhd_io_num = -1;
    spi_bus_cfg.max_transfer_sz = 4096;
    
    // Setup device
    // spi_dev_cfg.clock_speed_hz = sizeof(uint16_t) * 8 * ((sample_rate == DAC_SAMPLE_RATE_16KHZ) ? 16 : 44) * 1000;
    spi_dev_cfg.clock_speed_hz = 500 * 1000;
    spi_dev_cfg.mode = 0;
    spi_dev_cfg.spics_io_num = CS_PIN;
    spi_dev_cfg.queue_size = SPI_QUEUE_SIZE;
    // spi_dev_cfg.flags = SPI_DEVICE_NO_RETURN_RESULT;
    spi_dev_cfg.post_cb = &post_spi_tx_callback;
    
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &spi_dev_cfg, &spi));
    // ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));

    int freq_khz;
    size_t max_tx_length;

    ESP_ERROR_CHECK(spi_device_get_actual_freq(spi, &freq_khz));
    ESP_LOGI(DA_TAG, "Actual SPI transfer frequency %dKHZ", freq_khz);

    ESP_ERROR_CHECK(spi_bus_get_max_transaction_len(HSPI_HOST, &max_tx_length));
    ESP_LOGI(DA_TAG, "Max SPI transaction length: %d bytes", max_tx_length);
}