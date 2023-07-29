#include <inttypes.h>
#include <string.h>
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "dac_audio.h"

#define DA_TAG "DA"

#define MISO_PIN 12
#define MOSI_PIN 13
#define CLK_PIN 14
#define CS_PIN 15

static spi_device_handle_t spi;
static spi_bus_config_t spi_bus_cfg;
static spi_device_interface_config_t spi_dev_cfg;

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

uint8_t dac_audio_remaining_ready_buffer_slots(dac_audio_buffer_pool_t *pool) {
    if (pool == NULL) {
        return 0;
    }
    
    return pool->size - pool->ready_buffers_count;
}

void dac_audio_reset_buffer_pool(dac_audio_buffer_pool_t *pool) {
    while (pool->ready_buffers_count) {
        dac_audio_enqueue_free_buffer(pool, dac_audio_take_ready_buffer(pool));
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
        dac_audio_enqueue_free_buffer(buffer_pool, buffer);
    }
    ESP_LOGI(DA_TAG, "Initialized DAC audio buffer pool of %d buffers using %d max sample count", pool_size, buffer_size);
    
    return buffer_pool;
}

void dac_audio_init(dac_audio_sample_rate_t sample_rate) {
    // Setup bus
    spi_bus_cfg.miso_io_num = MISO_PIN; // we're not interested in reading
    spi_bus_cfg.mosi_io_num = MOSI_PIN;
    spi_bus_cfg.sclk_io_num = CLK_PIN;
    spi_bus_cfg.quadwp_io_num = -1;
    spi_bus_cfg.quadhd_io_num = -1;
    spi_bus_cfg.max_transfer_sz = 16 * 320 * 2 + 8;
    
    // Setup device
    spi_dev_cfg.clock_speed_hz = ((sample_rate == DAC_SAMPLE_RATE_16KHZ) ? 16 : 44) * 1000;
    spi_dev_cfg.mode = 0;
    spi_dev_cfg.spics_io_num = CS_PIN;
    spi_dev_cfg.queue_size = 8;
    
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &spi_dev_cfg, &spi));
    
    int freq_khz;
    size_t max_tx_length;

    ESP_ERROR_CHECK(spi_device_get_actual_freq(spi, &freq_khz));
    ESP_LOGI(DA_TAG, "Actual SPI transfer frequency %dKHZ", freq_khz);

    ESP_ERROR_CHECK(spi_bus_get_max_transaction_len(HSPI_HOST, &max_tx_length));
    ESP_LOGI(DA_TAG, "Max SPI transaction length: %d bytes", max_tx_length);
}