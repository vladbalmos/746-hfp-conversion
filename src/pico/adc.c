#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"
#include "pico/i2c_slave.h"
#include "hardware/dma.h"
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
// 2. Configure ADC to write samples via `audio_adc_dma_chan` to buffer
// 3. `audio_adc_dma_chan` ISR gets triggered and starts `spi_dma_chan` transfer
//    to stream buffer to SPI master. Goto step 2.

#define SPI_BAUDRATE_HZ 2500000
#define ADC_SPI_RX_PIN 12
#define ADC_SPI_TX_PIN 11
#define ADC_SPI_SCK_PIN 10
#define ADC_SPI_CS_PIN 13

#define I2C_BAUDRATE 1000000
// #define I2C_BAUDRATE 100000
#define ADC_SDA_PIN 12
#define ADC_SCL_PIN 13

#define ADC_CLOCK_SPEED_HZ 48 * 1000 * 1000
#define ADC_MAX_BUFFERS 8
#define ADC_SIGNAL_BIAS 2048
#define ADC_AUDIO_SINEWAVE_DMA_TIMER 0


static uint8_t data_ready_pin;

// SPI related
static int audio_adc_dma_chan;
static int spi_dma_chan;
static uint8_t streaming_enabled = 0;

// ADC & PCM related
static sample_rate_t adc_sample_rate;
static int16_t *adc_samples_buf[ADC_MAX_BUFFERS] = {0};
static uint8_t *i2c_data_buf = NULL;
static int8_t adc_samples_buf_index = -1;
static size_t adc_buffer_samples_count = 0;
static int16_t *sinewave_buf = NULL;
static uint8_t sinewave_enabled = 0;
static queue_t samples_ready_q;
static queue_t i2c_msg_q;

#ifdef DEBUG_MODE
static uint64_t sent_adc_counter = 0;
static uint64_t sent_spi_counter = 0;
static absolute_time_t spi_last_sent_sample_us;
static absolute_time_t adc_start_sampling_us;
static int64_t adc_sampling_duration_us = 0;
#endif

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
        if (sent_adc_counter++ % 125 == 0 && adc_samples_buf_index > -1) {
            DEBUG("ADC Data transfered. Sampling duration: %lld us.\n", adc_sampling_duration_us);
        }
#endif
        queue_try_add(&samples_ready_q, &adc_samples_buf[adc_samples_buf_index]);
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

void adc_transfer_samples() {
    uint8_t *buf = NULL;
    if (!queue_try_remove(&samples_ready_q, &buf)) {
        return __wfe();
    }
    
    gpio_put(data_ready_pin, 1);
    memcpy(i2c_data_buf, buf, adc_buffer_samples_count * sizeof(uint16_t));
    gpio_put(data_ready_pin, 0);
    // i2c_write_raw_blocking(i2c0, buf, adc_buffer_samples_count * 2);
}


// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    uint8_t data;
    switch (event) {
        case I2C_SLAVE_RECEIVE: {
            data = i2c_read_byte_raw(i2c);
            queue_try_add(&i2c_msg_q, &data);
            break;
        }
        case I2C_SLAVE_REQUEST: // master is requesting data
            i2c_write_raw_blocking(i2c0, i2c_data_buf, adc_buffer_samples_count * sizeof(uint16_t));
            break;
        case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
            // context.mem_address_written = false;
            break;
        default:
            break;
    }
}

void adc_transport_initialize(uint8_t gpio_ready_pin) {
    data_ready_pin = gpio_ready_pin;

    gpio_init(ADC_SDA_PIN);
    gpio_set_function(ADC_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ADC_SDA_PIN);

    gpio_init(ADC_SCL_PIN);
    gpio_set_function(ADC_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(ADC_SCL_PIN);
    
    i2c_init(i2c0, I2C_BAUDRATE);
    i2c_slave_init(i2c0, 32, &i2c_slave_handler);
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

#ifdef DEBUG_MODE
    float actual_sample_rate = cpu_clock_freq_hz * dma_timer_num / dma_timer_denom;
    DEBUG("Transfering data at %f hz %d %d %d %d\n", actual_sample_rate, cpu_clock_freq_hz, dma_timer_num, dma_timer_denom, adc_sample_rate);
#endif

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
    DEBUG("Transfering data at %f hz. Requested sample rate: %d\n", ADC_CLOCK_SPEED_HZ / adc_clock_div, adc_sample_rate);

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

void adc_initialize() {
    DEBUG("Initializing ADC\n");
    queue_init(&i2c_msg_q, sizeof(uint8_t), 1);

    uint16_t sample_rate = 0;

    gpio_put(data_ready_pin, 1);
    sleep_ms(1);
    gpio_put(data_ready_pin, 0);
    
    queue_remove_blocking(&i2c_msg_q, &sample_rate);

    DEBUG("Sample rate raw value: %d\n", sample_rate);
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
    
    i2c_data_buf = malloc(adc_buffer_samples_count * sizeof(uint16_t));
    assert(i2c_data_buf != NULL);
    
    queue_init(&samples_ready_q, sizeof(uint8_t *), 8);

    // init_audio_adc_dma();
    init_audio_sine_dma();
    
#ifdef DEBUG_MODE
    adc_start_sampling_us = get_absolute_time();
#endif

    audio_adc_dma_isr();
    // adc_run(true);
    streaming_enabled = 1;
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
    
    for (int i = 0; i < ADC_MAX_BUFFERS; i++) {
        free(adc_samples_buf[i]);
        adc_samples_buf[i] = NULL;
    }
    sinewave_enabled = 0;
    streaming_enabled = 0;
}