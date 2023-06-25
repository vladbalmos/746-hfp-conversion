#include "pico/stdlib.h"

typedef struct dac_audio_buffer {
    uint8_t *bytes;
    uint16_t size;
    struct dac_audio_buffer *next;
    
} dac_audio_buffer_t;

typedef struct dac_audio_buffer_pool {
    dac_audio_buffer_t *free_buffers_queue_head;
    dac_audio_buffer_t *free_buffers_queue_tail;

    dac_audio_buffer_t *ready_buffers_queue_head;
    dac_audio_buffer_t *ready_buffers_queue_tail;
    
    uint8_t size;
    uint8_t available_buffers_count;
    uint8_t ready_buffers_count;
    uint16_t buffer_size;
} dac_audio_buffer_pool_t;

typedef enum {
    DAC_SAMPLE_RATE_16KHZ,
    DAC_SAMPLE_RATE_44KHZ,
} dac_audio_sample_rate_t;

void dac_audio_init(spi_inst_t *spi_port, uint8_t mosi, uint8_t clk, uint8_t cs, dac_audio_sample_rate_t sample_rate);
void dac_audio_start_streaming();
void dac_audio_stop_streaming();

dac_audio_buffer_pool_t *dac_audio_init_buffer_pool(uint8_t pool_size, uint16_t buffer_size);
dac_audio_buffer_t *dac_audio_take_free_buffer();
dac_audio_buffer_t *dac_audio_take_ready_buffer();
uint16_t dac_audio_get_sample_rate();
uint8_t dac_audio_remaining_free_buffer_slots();
uint8_t dac_audio_remaining_ready_buffer_slots();
uint8_t dac_audio_enqueue_ready_buffer(dac_audio_buffer_t *buffer);
uint8_t dac_audio_enqueue_free_buffer(dac_audio_buffer_t *buffer);