#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "sample_rate.h"
#include "sine_16000_666.h"
#include "sine_8000_666.h"
#include "adc.h"
#include "debug.h"

// SPI Slave device
// Reading and writing to SPI is done after pulling `data_ready_pin` HIGH
// Once spi transfer completes, the `data_ready_pin` is pulled LOW
// 
// Data flow:
// 1. Read sampling rate via SPI during initialization
// 2. Configure ADC to write 120 samples via `audio_adc_dma_chan` to buffer
// 3. `audio_adc_dma_chan` ISR gets triggered and starts `spi_dma_chan` transfer
//    to stream buffer to SPI master. Goto step 2.

#define SPI_BAUDRATE_HZ 2500000
#define ADC_SPI_RX_PIN 12
#define ADC_SPI_TX_PIN 11
#define ADC_SPI_SCK_PIN 10
#define ADC_SPI_CS_PIN 13

#define ADC_CLOCK_SPEED_HZ 48 * 1000 * 1000
#define ADC_MAX_BUFFERS 8
#define ADC_SIGNAL_BIAS 2048
#define ADC_AUDIO_SINEWAVE_DMA_TIMER 0


static uint8_t data_ready_pin;

// SPI related
static int audio_adc_dma_chan;
static int spi_dma_chan;

// ADC & PCM related
static sample_rate_t adc_sample_rate;
static int16_t *adc_samples_buf[ADC_MAX_BUFFERS] = {0};
static int8_t adc_samples_buf_index = -1;
static size_t adc_buffer_samples_count = 0;
static int16_t *sinewave_buf = NULL;
static uint8_t sinewave_enabled = 0;

#ifdef DEBUG_MODE
static uint64_t sent_adc_counter = 0;
static uint64_t sent_spi_counter = 0;
static absolute_time_t adc_start_sampling_us;
static int64_t adc_sampling_duration_us = 0;
static absolute_time_t spi_start_transfer_us;
static int64_t spi_transfer_duration_us = 0;
#endif

static void spi_dma_isr();

static int spi_read(uint16_t *dst, size_t len) {
    size_t read_count = 0;

    gpio_put(data_ready_pin, 1);
    read_count = spi_read16_blocking(spi1, 0, dst, len);
    gpio_put(data_ready_pin, 0);
    
    return read_count;
}

static int spi_write(const uint16_t *src, size_t len) {
    size_t write_count = 0;

    gpio_put(data_ready_pin, 1);
    write_count = spi_write16_blocking(spi1, src, len);
    gpio_put(data_ready_pin, 0);
    
    return write_count;
}

static void audio_adc_dma_isr() {
#ifdef DEBUG_MODE
    absolute_time_t now;

    if (sent_adc_counter) {
        now = get_absolute_time();
        adc_sampling_duration_us = absolute_time_diff_us(adc_start_sampling_us, now);
        adc_start_sampling_us = now;
    }
#endif
    
    dma_hw->ints0 = 1u << audio_adc_dma_chan;


    int32_t sample;
    if (adc_samples_buf_index != -1) {
        if (!sinewave_enabled) {
            for (int i = 0; i < adc_buffer_samples_count; i++) {
                sample = (adc_samples_buf[adc_samples_buf_index][i] - ADC_SIGNAL_BIAS) * 16;
                
                if (sample > INT16_MAX) {
                    sample = INT16_MAX;
                } else if (sample < INT16_MIN) {
                    sample = INT16_MIN;
                }
                adc_samples_buf[adc_samples_buf_index][i] = (int16_t) sample;
            }
        }

#ifdef DEBUG_MODE
        spi_start_transfer_us = now;

        if (sent_adc_counter++ % 250 == 0 && adc_samples_buf_index > -1) {
            DEBUG("ADC Data transfered. Sampling duration: %lld us. SPI transfer duration: %lld\n", adc_sampling_duration_us, spi_transfer_duration_us);
            for (int i = 0; i < adc_buffer_samples_count; i++) {
                DEBUG("%d ", adc_samples_buf[adc_samples_buf_index][i]);
                if (i && i % 8 == 0) {
                    DEBUG("\n");
                }
            }
            DEBUG("\n===================\n");
        }
#endif
        gpio_put(data_ready_pin, 1);
        dma_channel_set_read_addr(spi_dma_chan, adc_samples_buf[adc_samples_buf_index], true);
    }

    adc_samples_buf_index++;
    if (adc_samples_buf_index >= ADC_MAX_BUFFERS) {
        adc_samples_buf_index = 0;
    }
    if (sinewave_enabled) {
        dma_channel_set_read_addr(audio_adc_dma_chan, sinewave_buf, false);
    }
    dma_channel_set_write_addr(audio_adc_dma_chan, adc_samples_buf[adc_samples_buf_index], true);
}

static void spi_dma_isr() {
    gpio_put(data_ready_pin, 0);
    dma_hw->ints1 = 1u << spi_dma_chan;
    
#ifdef DEBUG_MODE
    absolute_time_t now = get_absolute_time();
    spi_transfer_duration_us = absolute_time_diff_us(spi_start_transfer_us, now);

    if (sent_spi_counter++ % 500 == 0) {
        DEBUG("SPI data transfered\n");
    }
#endif
}

void adc_transport_initialize(uint8_t gpio_ready_pin) {
    data_ready_pin = gpio_ready_pin;

    uint actual_baud_rate = spi_init(spi1, SPI_BAUDRATE_HZ);
    spi_set_slave(spi1, true);
    spi_set_format(spi1, 16, 0, 0, SPI_MSB_FIRST);
    DEBUG("Initialized SPI with baudrate: %d\n", actual_baud_rate);
    
    gpio_set_function(ADC_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADC_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADC_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADC_SPI_CS_PIN, GPIO_FUNC_SPI);
}

static void init_audio_sine_dma() {
    sinewave_enabled = 1;
    // Configure sinewave transfer DMA channel
    audio_adc_dma_chan = dma_claim_unused_channel(true);
    assert(audio_adc_dma_chan != -1);
    dma_timer_claim(ADC_AUDIO_SINEWAVE_DMA_TIMER);
    
    dma_channel_config cfg = dma_channel_get_default_config(audio_adc_dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, dma_get_timer_dreq(ADC_AUDIO_SINEWAVE_DMA_TIMER));

    uint16_t dma_timer_num;
    uint16_t dma_timer_denom;

    if (adc_sample_rate == SAMPLE_RATE_16KHZ) {
        dma_timer_num = 8;
        dma_timer_denom = 62500;
    } else if (adc_sample_rate == SAMPLE_RATE_8KHZ) {
        dma_timer_num = 4;
        dma_timer_denom = 62500;
    }
 
    uint32_t cpu_clock_freq_hz = clock_get_hz(clk_sys);
    float actual_sample_rate = cpu_clock_freq_hz * dma_timer_num / dma_timer_denom;
    DEBUG("Transfering data at %f hz %d %d %d %d\n", actual_sample_rate, cpu_clock_freq_hz, dma_timer_num, dma_timer_denom, adc_sample_rate);
    dma_timer_set_fraction(ADC_AUDIO_SINEWAVE_DMA_TIMER, dma_timer_num, dma_timer_denom);
    
    dma_channel_configure(
        audio_adc_dma_chan,
        &cfg,
        NULL,
        NULL,
        adc_buffer_samples_count,
        false
    );

    dma_channel_set_irq0_enabled(audio_adc_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, audio_adc_dma_isr);
    irq_set_enabled(DMA_IRQ_0, true);
}

static void init_audio_adc_dma() {
    adc_init();

    // Configure ADC DMA channel for sampling the audio signal
    audio_adc_dma_chan = dma_claim_unused_channel(true);
    assert(audio_adc_dma_chan != -1);
    
    dma_channel_config cfg = dma_channel_get_default_config(audio_adc_dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_ADC);

    uint16_t clock_div;
    if (adc_sample_rate == SAMPLE_RATE_16KHZ) {
        clock_div = 16 * 1000;
    } else if (adc_sample_rate == SAMPLE_RATE_8KHZ) {
        clock_div = 8 * 1000;
    }
    float adc_clock_div = ADC_CLOCK_SPEED_HZ / clock_div;
    DEBUG("Transfering data at %f hz. Requested sample rate: %d\n", adc_clock_div, adc_sample_rate);

    adc_gpio_init(26);
    adc_select_input(0);
    adc_fifo_setup(
        true,
        true,
        1,
        false,
        false
    );
    adc_set_clkdiv(adc_clock_div);
    
    dma_channel_configure(
        audio_adc_dma_chan,
        &cfg,
        NULL,
        &adc_hw->fifo,
        adc_buffer_samples_count,
        false
    );

    dma_channel_set_irq0_enabled(audio_adc_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, audio_adc_dma_isr);
    irq_set_enabled(DMA_IRQ_0, true);
}

static void init_spi_dma() {
    // Configure SPI DMA channel
    spi_dma_chan = dma_claim_unused_channel(true);
    assert(audio_adc_dma_chan != -1);
    
    dma_channel_config cfg;
    cfg = dma_channel_get_default_config(spi_dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, spi_get_dreq(spi1, true));

    dma_channel_configure(
        spi_dma_chan,
        &cfg,
        &spi_get_hw(spi1)->dr,
        adc_samples_buf[0],
        120,
        false
    );

    dma_channel_set_irq1_enabled(spi_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_1, spi_dma_isr);
    irq_set_enabled(DMA_IRQ_1, true);
    
}

void adc_initialize() {
    DEBUG("Initializing ADC\n");
    uint16_t sample_rate = 0;
    size_t read_count = 0;

    // Read sampling rate from master
    read_count = spi_read(&sample_rate, 1);

    DEBUG("Sample rate raw value: %d\n", sample_rate);
    assert(read_count == 1);
    assert(sample_rate == SAMPLE_RATE_16KHZ || sample_rate == SAMPLE_RATE_8KHZ);
    
    if (sample_rate == SAMPLE_RATE_8KHZ) {
        adc_buffer_samples_count = 60;
        sinewave_buf = sinewave_8000;
        DEBUG("Sample rate: 8khz\n");
    }
    if (sample_rate == SAMPLE_RATE_16KHZ) {
        adc_buffer_samples_count = 120;
        sinewave_buf = sinewave_16000;
        DEBUG("Sample rate: 16khz\n");
    }
    
    DEBUG("ADC buffer samples count: %d\n", adc_buffer_samples_count);
    
    adc_sample_rate = sample_rate;
    for (int i = 0; i < ADC_MAX_BUFFERS; i++) {
        adc_samples_buf[i] = malloc(adc_buffer_samples_count * sizeof(uint16_t));
        if (adc_samples_buf[i] == NULL) {
            panic("Unable to allocate memory for ADC buffer\n");
        }
        assert(adc_samples_buf[i] != NULL);
        memset(adc_samples_buf[i], 0, adc_buffer_samples_count * sizeof(uint16_t));
    }

    init_audio_adc_dma();
    // init_audio_sine_dma();
    init_spi_dma();
    
    
#ifdef DEBUG_MODE
    adc_start_sampling_us = get_absolute_time();
#endif

    audio_adc_dma_isr();
    adc_run(true);
}

void adc_deinit() {
    DEBUG("De-initializing ADC\n");
    
    if (!sinewave_enabled) {
        adc_fifo_drain();
    }
    
    dma_channel_abort(audio_adc_dma_chan);
    dma_channel_abort(spi_dma_chan);

    // de-init adc dma
    irq_set_enabled(DMA_IRQ_0, false);
    irq_remove_handler(DMA_IRQ_0, audio_adc_dma_isr);
    dma_channel_set_irq0_enabled(audio_adc_dma_chan, false);
    if (dma_timer_is_claimed(ADC_AUDIO_SINEWAVE_DMA_TIMER)) {
        dma_timer_unclaim(ADC_AUDIO_SINEWAVE_DMA_TIMER);
    }
    dma_channel_unclaim(audio_adc_dma_chan);
    
    // de-init spi dma
    irq_set_enabled(DMA_IRQ_1, false);
    irq_remove_handler(DMA_IRQ_1, spi_dma_isr);
    dma_channel_set_irq0_enabled(spi_dma_chan, false);
    dma_channel_unclaim(spi_dma_chan);
    
    for (int i = 0; i < ADC_MAX_BUFFERS; i++) {
        free(adc_samples_buf[i]);
        adc_samples_buf[i] = NULL;
    }
    sinewave_enabled = 0;

}