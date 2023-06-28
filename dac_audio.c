#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "malloc.h"
#include "dac_audio.h"
#include "debug.h"

#define POLL_MS 5
#define SPI_BAUD_RATE_HZ 5 * 1000 * 1000

static spi_inst_t *initialized_spi_port = NULL;

static uint8_t mosi_pin;
static uint8_t clk_pin;
static uint8_t cs_pin;

static uint32_t cpu_clock_freq_hz;


static uint dma_timer_0 = 0;
static int dma_data_chan;
static dma_channel_config dma_data_cfg;
static uint16_t dreq_timer_fractions[2][3] = {
    {8, 63000, 16000},
    {23, 65193, 44100}
};

static dac_audio_buffer_pool_t *buffer_pool = NULL;
static dac_audio_buffer_t *current_buffer = NULL;

static uint8_t transfer_in_progress = 0;

static uint16_t current_sample_rate;
static alarm_id_t stream_alarm;

static uint8_t is_streaming = 0;

static inline void ensure_initialization() {
    if (initialized_spi_port == NULL) {
        panic("DAC not initialized!\n");
    }
}

static inline void dac_single_write(uint16_t val) {
    uint16_t data[1] = { val };
    spi_write16_blocking(initialized_spi_port, data, 1);
}

static inline void dac_enable(uint8_t status) {
    gpio_put(cs_pin, !status);
}

static inline void transfer_ready_buffer() {
    dac_audio_buffer_t *buf = dac_audio_take_ready_buffer();
    if (buf != NULL) {
        current_buffer = buf;
        dma_channel_set_read_addr(dma_data_chan, &buf->bytes[0], true);
        return;
    }
    transfer_in_progress = 0;
}

static void dma_handler() {
    // Free the previous used buffer
    dac_audio_enqueue_free_buffer(current_buffer);
    dma_hw->ints0 = 1u << dma_data_chan;

    if (!is_streaming) {
        transfer_in_progress = 0;
        return;
    }
    
    transfer_ready_buffer();
}

int64_t stream_buffers(alarm_id_t id, void *user_data) {
    if (!is_streaming) {
        return 0;
    }

    if (!transfer_in_progress) {
        transfer_in_progress = 1;
        transfer_ready_buffer();
    }
    stream_alarm = add_alarm_in_ms(POLL_MS, stream_buffers, NULL, false);
    return 0;
}

static dac_audio_buffer_t *init_audio_buffer(uint16_t buffer_size) {
    int8_t *bytes = malloc(buffer_size * sizeof(uint16_t));
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

inline uint16_t dac_audio_get_sample_rate() {
    return current_sample_rate;
}

void dac_audio_init(spi_inst_t *spi_port, uint8_t mosi, uint8_t clk, uint8_t cs, dac_audio_sample_rate_t sample_rate) {
    cpu_clock_freq_hz = clock_get_hz(clk_sys);
    uint16_t dreq_timer_numerator = dreq_timer_fractions[sample_rate][0];
    uint16_t dreq_timer_denominator = dreq_timer_fractions[sample_rate][1];
    current_sample_rate = dreq_timer_fractions[sample_rate][2];
    float actual_sample_rate = cpu_clock_freq_hz * dreq_timer_numerator / dreq_timer_denominator;
    
    initialized_spi_port = spi_port;
    
    DEBUG("Initializing DAC for %d wanted sample rate using DMA timer fractional parts %d/%d\n", current_sample_rate, dreq_timer_numerator, dreq_timer_denominator);
    DEBUG("Actual computed sample rate: %f\n", actual_sample_rate);
    DEBUG("DAC DMA transfer count: %d\n", buffer_pool->buffer_size);

    // Init spi port and baud rate
    spi_init(spi_port, SPI_BAUD_RATE_HZ);
    spi_set_format(spi_port, 16, 0, 0, 0);
    
    mosi_pin = mosi;
    clk_pin = clk;
    cs_pin = cs;

    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(clk_pin, GPIO_FUNC_SPI);
    gpio_set_function(cs_pin, GPIO_FUNC_SPI);
    
    // Silence please!
    dac_enable(0);

    dma_data_chan = dma_claim_unused_channel(true);

    // Configure data channel
    dma_data_cfg = dma_channel_get_default_config(dma_data_chan);

    channel_config_set_transfer_data_size(&dma_data_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_data_cfg, true);
    channel_config_set_write_increment(&dma_data_cfg, false);

    // Set SPI write rate (must clossly match the audio sample rate)
    dma_timer_claim(dma_timer_0);
    dma_timer_set_fraction(dma_timer_0, dreq_timer_numerator, dreq_timer_denominator);
    channel_config_set_dreq(&dma_data_cfg, DREQ_DMA_TIMER0);
    channel_config_set_ring(&dma_data_cfg, false, 0);
    
    dma_channel_configure(
        dma_data_chan,
        &dma_data_cfg,
        &spi_get_hw(spi_port)->dr,
        NULL,
        buffer_pool->buffer_size,
        false
    );

    dma_channel_set_irq0_enabled(dma_data_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

void dac_audio_start_streaming() {
    ensure_initialization();
    if (buffer_pool == NULL) {
        return;
    }
    
    dac_enable(1);
    
    is_streaming = 1;
    stream_buffers(0, NULL);
}

void dac_audio_stop_streaming() {
    DEBUG("Stopping streaming... Current state is %d\n", is_streaming);
    if (!is_streaming) {
        return;
    }
    
    is_streaming = 0;
    cancel_alarm(stream_alarm);

    DEBUG("Available buffers count %d\n", buffer_pool->available_buffers_count);
    DEBUG("Read buffers count %d\n", buffer_pool->ready_buffers_count);
    DEBUG("Remaining free buffer slots%d\n", dac_audio_remaining_free_buffer_slots());
    DEBUG("Remaining ready buffer slots%d\n", dac_audio_remaining_ready_buffer_slots());
    DEBUG("Waiting for dma to finish. Current state is: %d\n", dma_channel_is_busy(dma_data_chan));

    dma_channel_wait_for_finish_blocking(dma_data_chan);

    DEBUG("Resetting buffers\n");
    dac_audio_reset_buffer_pool();
    DEBUG("Disabling DAC\n");
    dac_enable(0);
    DEBUG("Streaming stoped!\n");
}

dac_audio_buffer_pool_t *dac_audio_init_buffer_pool(uint8_t pool_size, uint16_t buffer_size) {
    buffer_pool = malloc(sizeof(dac_audio_buffer_pool_t));
    
    if (buffer_pool == NULL) {
        return NULL;
    }
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
    DEBUG("Initialized DAC audio buffer pool of %d buffers using %d max sample count\n", pool_size, buffer_size);
    
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

void dac_audio_reset_buffer_pool() {
    while (buffer_pool->ready_buffers_count) {
        dac_audio_enqueue_free_buffer(dac_audio_take_ready_buffer());
    }
    DEBUG("Buffer pool reset\n");
    DEBUG("Available buffers count %d\n", buffer_pool->available_buffers_count);
    DEBUG("Read buffers count %d\n", buffer_pool->ready_buffers_count);
    DEBUG("Remaining free buffer slots %d\n", dac_audio_remaining_free_buffer_slots());
    DEBUG("Remaining ready buffer slots %d\n", dac_audio_remaining_ready_buffer_slots());
}