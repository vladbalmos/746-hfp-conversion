#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct dac_audio_buffer {
    uint8_t *bytes;
    uint16_t size;
    uint16_t samples;
    struct dac_audio_buffer *next;
    
} dac_audio_buffer_t;

typedef struct dac_audio_buffer_pool {
    dac_audio_buffer_t *free_buffers_queue_head;
    dac_audio_buffer_t *free_buffers_queue_tail;
    SemaphoreHandle_t free_buff_sem;

    dac_audio_buffer_t *ready_buffers_queue_head;
    dac_audio_buffer_t *ready_buffers_queue_tail;
    SemaphoreHandle_t ready_buff_sem;
    
    uint8_t size;
    uint8_t available_buffers_count;
    uint8_t ready_buffers_count;
    uint16_t buffer_size;
} dac_audio_buffer_pool_t;

typedef struct dac_audio_free_buf_msg {
    dac_audio_buffer_pool_t *pool;
    dac_audio_buffer_t *buf;
} dac_audio_free_buf_msg_t;

typedef enum {
    DAC_SAMPLE_RATE_16KHZ,
    DAC_SAMPLE_RATE_44KHZ,
} dac_audio_sample_rate_t;


void dac_audio_init(dac_audio_sample_rate_t sample_rate);

dac_audio_buffer_pool_t *dac_audio_init_buffer_pool(uint8_t pool_size, uint16_t buffer_size);
void dac_audio_reset_buffer_pool(dac_audio_buffer_pool_t *pool);

void dac_audio_send(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buf);

dac_audio_buffer_t *dac_audio_take_free_buffer(dac_audio_buffer_pool_t *pool);
dac_audio_buffer_t *dac_audio_take_free_buffer_safe(dac_audio_buffer_pool_t *pool, TickType_t wait);
dac_audio_buffer_t *dac_audio_take_ready_buffer(dac_audio_buffer_pool_t *pool);
dac_audio_buffer_t *dac_audio_take_ready_buffer_safe(dac_audio_buffer_pool_t *pool, TickType_t wait);

void dac_audio_schedule_used_buf_release(dac_audio_buffer_pool_t *return_pool, dac_audio_buffer_t *buf, uint8_t fromISR);

uint8_t dac_audio_remaining_free_buffer_slots(dac_audio_buffer_pool_t *pool);
uint8_t dac_audio_remaining_ready_buffer_slots(dac_audio_buffer_pool_t *pool);
uint8_t dac_audio_enqueue_ready_buffer(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buffer);
uint8_t dac_audio_enqueue_ready_buffer_safe(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buffer, TickType_t wait);
uint8_t dac_audio_enqueue_free_buffer(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buffer);
uint8_t dac_audio_enqueue_free_buffer_safe(dac_audio_buffer_pool_t *pool, dac_audio_buffer_t *buffer, TickType_t wait);

void dac_audio_send_simple(uint8_t b1, uint8_t b2);