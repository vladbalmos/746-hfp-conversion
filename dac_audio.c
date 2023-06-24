#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "malloc.h"
#include "dac_audio.h"

#define POLL_MS 5

#define SPI_BAUD_RATE_HZ 10 * 1000 * 1000

static uint dma_timer_0 = 0;
static spi_inst_t *port;
static uint8_t mosi_pin;
static uint8_t clk_pin;
static uint8_t cs_pin;

static dac_audio_buffer_t *current_buffer = NULL;
static int dma_data_chan;
static dma_channel_config dma_data_cfg;

static dac_audio_buffer_pool_t *buffer_pool = NULL;
static uint8_t transfer_in_progress = 0;

static void dma_handler() {
    dac_audio_enqueue_free_buffer(current_buffer);
    // Clear the interrupt request.
    dma_hw->ints0 = 1u << dma_data_chan;
    
    dac_audio_buffer_t *buf = dac_audio_take_ready_buffer();
    if (buf != NULL) {
        current_buffer = buf;
        dma_channel_set_read_addr(dma_data_chan, &buf->bytes[0], true);
        return;
    }
    
    transfer_in_progress = 0;
}

int64_t stream_buffers(alarm_id_t id, void *user_data) {
    if (!transfer_in_progress) {
        transfer_in_progress = 1;
        dac_audio_buffer_t *buf = dac_audio_take_ready_buffer();

        if (buf != NULL)
        {
            current_buffer = buf;
            dma_channel_set_read_addr(dma_data_chan, &buf->bytes[0], true);
        }
    }
    add_alarm_in_ms(POLL_MS, stream_buffers, NULL, false);
    return 0;
}

static dac_audio_buffer_t *init_audio_buffer(uint16_t buffer_size) {
    uint8_t *bytes = malloc(buffer_size * sizeof(uint16_t));
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

void dac_audio_init(spi_inst_t *spi_port, uint8_t mosi, uint8_t clk, uint8_t cs) {
    port = spi_port;

    // Init spi port and baud rate
    spi_init(port, SPI_BAUD_RATE_HZ);
    spi_set_format(port, 16, 0, 0, 0);
    
    mosi_pin = mosi;
    clk_pin = clk;
    cs_pin = cs;

    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(clk_pin, GPIO_FUNC_SPI);
    gpio_set_function(cs_pin, GPIO_FUNC_SPI);

    dma_data_chan = dma_claim_unused_channel(true);

    // Configure data channel
    dma_data_cfg = dma_channel_get_default_config(dma_data_chan);

    channel_config_set_transfer_data_size(&dma_data_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_data_cfg, true);
    channel_config_set_write_increment(&dma_data_cfg, false);
    // SPI write at ~44.1khz
    dma_timer_set_fraction(dma_timer_0, 0x0017, 0xffff);
    channel_config_set_dreq(&dma_data_cfg, DREQ_DMA_TIMER0);
    
    printf("DMA transfer count: %d\n", buffer_pool->buffer_size);
    
    dma_channel_configure(
        dma_data_chan,
        &dma_data_cfg,
        &spi_get_hw(port)->dr,
        NULL,
        buffer_pool->buffer_size,
        false
    );

    dma_channel_set_irq0_enabled(dma_data_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

void dac_audio_start_streaming() {
    if (buffer_pool == NULL) {
        return;
    }
    
    add_alarm_in_ms(POLL_MS, stream_buffers, NULL, false);
}

dac_audio_buffer_pool_t *dac_audio_init_buffer_pool(uint8_t pool_size, uint16_t buffer_size) {
    buffer_pool = malloc(sizeof(dac_audio_buffer_pool_t));
    buffer_pool->free_buffers_queue_head = NULL;
    buffer_pool->free_buffers_queue_tail = NULL;
    buffer_pool->ready_buffers_queue_head = NULL;
    buffer_pool->ready_buffers_queue_tail = NULL;

    buffer_pool->ready_buffers_count = 0;
    buffer_pool->available_buffers_count = 0;
    buffer_pool->size = pool_size;
    buffer_pool->buffer_size = buffer_size;
    
    
    for (uint8_t i = 0; i < pool_size; i++) {
        dac_audio_buffer_t *buffer = init_audio_buffer(buffer_size);
        if (buffer == NULL) {
            return NULL;
        }
        dac_audio_enqueue_free_buffer(buffer);
    }
    printf("Buffer size is %d\n", buffer_size);
    
    return buffer_pool;
}

uint8_t dac_audio_remaining_free_buffer_slots() {
    if (buffer_pool == NULL) {
        return 0;
    }
    
    return buffer_pool->size - buffer_pool->available_buffers_count;
}

uint8_t dac_audio_remaining_ready_buffer_slots() {
    if (buffer_pool == NULL) {
        return 0;
    }
    
    return buffer_pool->size - buffer_pool->ready_buffers_count;
}


uint8_t dac_audio_enqueue_free_buffer(dac_audio_buffer_t *buf) {
    if (!dac_audio_remaining_free_buffer_slots()) {
        return 0;
    }

    memset(buf->bytes, 0, buf->size);
    buf->next = NULL;

    buffer_pool->available_buffers_count++;

    if (buffer_pool->free_buffers_queue_head == NULL) {
        buffer_pool->free_buffers_queue_head = buf;
        buffer_pool->free_buffers_queue_tail = buf;
        return 1;
    }
    
    buffer_pool->free_buffers_queue_tail->next = buf;
    buffer_pool->free_buffers_queue_tail = buf;
    return 1;
}

uint8_t dac_audio_enqueue_ready_buffer(dac_audio_buffer_t *buf) {
    if (!dac_audio_remaining_ready_buffer_slots()) {
        return 0;
    }
    buf->next = NULL;

    buffer_pool->ready_buffers_count++;

    if (buffer_pool->ready_buffers_queue_head == NULL) {
        buffer_pool->ready_buffers_queue_head = buf;
        buffer_pool->ready_buffers_queue_tail = buf;
        return 1;
    }
    
    buffer_pool->ready_buffers_queue_tail->next = buf;
    buffer_pool->ready_buffers_queue_tail = buf;
    return 1;
}

dac_audio_buffer_t *dac_audio_take_free_buffer() {
    if (buffer_pool->free_buffers_queue_head == NULL) {
        return NULL;
    }
    
    buffer_pool->available_buffers_count--;
    
    dac_audio_buffer_t *buf = buffer_pool->free_buffers_queue_head;
    
    if (buf == buffer_pool->free_buffers_queue_tail) {
        buffer_pool->free_buffers_queue_head = NULL;
        buffer_pool->free_buffers_queue_tail = NULL;
    } else {
        buffer_pool->free_buffers_queue_head = buf->next;
    }

    return buf;
}

dac_audio_buffer_t *dac_audio_take_ready_buffer() {
    if (buffer_pool->ready_buffers_queue_head == NULL) {
        return NULL;
    }
    
    buffer_pool->ready_buffers_count--;
    
    dac_audio_buffer_t *buf = buffer_pool->ready_buffers_queue_head;
    
    if (buf == buffer_pool->ready_buffers_queue_tail) {
        buffer_pool->ready_buffers_queue_head = NULL;
        buffer_pool->ready_buffers_queue_tail = NULL;
    } else {
        buffer_pool->ready_buffers_queue_head = buf->next;
    }

    return buf;
}