#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"
#include "pico/i2c_slave.h"
#include "sample_rate.h"
#include "sine_16000_666.h"
#include "sine_8000_666.h"
#include "audio.h"
#include "debug.h"

// I2C Slave device (address: 32)
// ----------------
// Data flow:
// 1. Read sampling rate via I2C during initialization
// 2. Based on received sample rate, determine appropriate buffer sizes & samples count
// 3. Configure ADC to continuous write samples via `audio_adc_dma_chan` to buffer
// 4. `audio_adc_dma_chan` ISR gets triggered after each sample set and adds samples buffer to transmission queue
// 5. Main loop polls transmission queue and notifies I2C master via `data_ready_pin` pulling it high
//    when samples are available
// 6. Master reads available PCM samples
// 7. Master sends playback PCM samples for DAC. Samples are added to the appropriate DAC buffer
// 8. Audio samples for playback are sent to DAC through SPI using `audio_dac_dma_chan`
// 9. GOTO step 5

// I2C
#define I2C_BAUDRATE 1000000
#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 13
#define I2C_ADDRESS 32

// SPI
#define SPI_BAUDRATE I2C_BAUDRATE
#define SPI_DAC_CS_PIN 17
#define SPI_DAC_SCLK_PIN 18
#define SPI_DAC_TX_PIN 19

#define ADC_CLOCK_SPEED_HZ 48 * 1000 * 1000
#define MAX_BUFFERS 8
#define ADC_SIGNAL_BIAS 2048
#define AUDIO_SINEWAVE_DMA_TIMER 0


static uint8_t data_ready_pin;

static int audio_adc_dma_chan;
static int audio_dac_dma_chan;

static sample_rate_t audio_sample_rate;
static int16_t *adc_samples_buf[MAX_BUFFERS] = {0};
static uint8_t *i2c_data_out_buf = NULL;
static int8_t adc_samples_buf_index = -1;

static int16_t *dac_samples_buf[MAX_BUFFERS] = {0};
static uint8_t dac_samples_buf_index = 0;
static int8_t dac_current_buf_index_streaming = -1;
static int8_t dac_streaming = 0;

static size_t buffer_samples_count = 0;
static size_t buffer_size = 0;
static int16_t *sinewave_buf = NULL;
static uint8_t sinewave_enabled = 0;
static queue_t samples_ready_q;
static queue_t i2c_msg_q;
static uint8_t configured = 0;

#ifdef DEBUG_MODE
static uint64_t sent_adc_counter = 0;
static absolute_time_t adc_start_sampling_us;
static int64_t adc_sampling_duration_us = 0;

static uint64_t sent_dac_counter = 0;
static absolute_time_t dac_start_sampling_us;
static int64_t dac_sampling_duration_us = 0;

#endif


static inline size_t i2c_write_blocking_timeout_us(uint8_t *src, size_t len, int64_t timeout_us) {
    absolute_time_t now = get_absolute_time();
    absolute_time_t start = now;
    int64_t elapsed;

    for (size_t i = 0; i < len; ++i) {
        while (!i2c_get_write_available(i2c0)) {
            now = get_absolute_time();
            elapsed = absolute_time_diff_us(start, now);
            if (elapsed >= timeout_us) {
                return 0;
            }
        }

        i2c_get_hw(i2c0)->data_cmd = *src++;
    }
    return len;
}

static inline size_t i2c_read_blocking_timeout_us(uint8_t *dst, size_t len, int64_t timeout_us) {
    absolute_time_t now = get_absolute_time();
    absolute_time_t start = now;
    int64_t elapsed;

    for (size_t i = 0; i < len; ++i) {
        while (!i2c_get_read_available(i2c0)) {
            now = get_absolute_time();
            elapsed = absolute_time_diff_us(start, now);
            if (elapsed >= timeout_us) {
                return 0;
            }
        }
        *dst++ = (uint8_t)i2c_get_hw(i2c0)->data_cmd;
    }
    
    return len;
}

static void audio_dac_dma_isr() {
#ifdef DEBUG_MODE
    absolute_time_t now;

    if (sent_dac_counter) {
        now = get_absolute_time();
        dac_sampling_duration_us = absolute_time_diff_us(dac_start_sampling_us, now);
        dac_start_sampling_us = now;
    }
#endif
    dma_hw->ints1 = 1u << audio_dac_dma_chan;
    
    dac_current_buf_index_streaming++;
    if (dac_current_buf_index_streaming >= MAX_BUFFERS) {
        dac_current_buf_index_streaming = 0;
    }

#ifdef DEBUG_MODE
        if (sent_dac_counter++ % 500 == 0 && dac_current_buf_index_streaming > -1) {
            DEBUG("DAC Data transfered. Sampling duration: %lld us.\n", dac_sampling_duration_us);
        }
#endif
    // dma_channel_set_write_addr(audio_dac_dma_chan, dac_samples_buf[dac_current_buf_index_streaming], true);
    dma_channel_set_read_addr(audio_dac_dma_chan, dac_samples_buf[dac_current_buf_index_streaming], true);
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
            for (int i = 0; i < buffer_samples_count; i++) {
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
        if (sent_adc_counter++ % 500 == 0 && adc_samples_buf_index > -1) {
            DEBUG("ADC Data transfered. Sampling duration: %lld us.\n", adc_sampling_duration_us);
        }
#endif
        queue_try_add(&samples_ready_q, &adc_samples_buf[adc_samples_buf_index]);
    }

    adc_samples_buf_index++;
    if (adc_samples_buf_index >= MAX_BUFFERS) {
        adc_samples_buf_index = 0;
    }
    if (sinewave_enabled) {
        dma_channel_set_read_addr(audio_adc_dma_chan, sinewave_buf, false);
    }
    dma_channel_set_write_addr(audio_adc_dma_chan, adc_samples_buf[adc_samples_buf_index], true);
}

void audio_transfer_samples() {
    uint8_t *buf = NULL;
    if (!queue_try_remove(&samples_ready_q, &buf)) {
        return __wfe();
    }
    
    gpio_put(data_ready_pin, 1);
    memcpy(i2c_data_out_buf, buf, buffer_size);
    gpio_put(data_ready_pin, 0);
}


// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    uint8_t data;
    switch (event) {
        case I2C_SLAVE_RECEIVE: {
            if (!configured) {
                // Read the sample rate
                data = i2c_read_byte_raw(i2c);
                queue_try_add(&i2c_msg_q, &data);
                break;
            }
            
            // Audio samples for DAC
            i2c_read_blocking_timeout_us((uint8_t *) dac_samples_buf[dac_samples_buf_index++], buffer_size, 5000);
            if (dac_samples_buf_index >= MAX_BUFFERS) {
                dac_samples_buf_index = 0;
            }
            
            // Wait for at least two buffers to be received, then start
            // dma streaming to DAC
            if (!dac_streaming && dac_samples_buf_index == 2) {
                dac_streaming = 1;
                audio_dac_dma_isr();
            }
            break;
        }
        case I2C_SLAVE_REQUEST: // master is requesting data
            i2c_write_raw_blocking(i2c0, i2c_data_out_buf, buffer_size);
            break;
        case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
            // context.mem_address_written = false;
            break;
        default:
            break;
    }
}

void audio_transport_init(uint8_t gpio_ready_pin) {
    DEBUG("Initializing audio transport\n");
    data_ready_pin = gpio_ready_pin;

    // I2C
    gpio_init(I2C_SDA_PIN);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);

    gpio_init(I2C_SCL_PIN);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL_PIN);
    
    i2c_init(i2c0, I2C_BAUDRATE);
    i2c_slave_init(i2c0, I2C_ADDRESS, &i2c_slave_handler);
    DEBUG("Initialized I2C\n");

    // SPI
    spi_init(spi0, SPI_BAUDRATE);
    spi_set_format(spi0, 16, 0, 0, SPI_MSB_FIRST);
    
    gpio_set_function(SPI_DAC_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_DAC_SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_DAC_CS_PIN, GPIO_FUNC_SPI);

    queue_init(&i2c_msg_q, sizeof(uint8_t), 1);

    // Silence please
    gpio_put(SPI_DAC_CS_PIN, 1);
    DEBUG("Initialized SPI\n");
}

static void init_audio_sine_dma() {
    sinewave_enabled = 1;
    // Configure sinewave transfer DMA channel
    audio_adc_dma_chan = dma_claim_unused_channel(true);
    assert(audio_adc_dma_chan != -1);
    dma_timer_claim(AUDIO_SINEWAVE_DMA_TIMER);
    
    dma_channel_config cfg = dma_channel_get_default_config(audio_adc_dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, dma_get_timer_dreq(AUDIO_SINEWAVE_DMA_TIMER));

    uint16_t dma_timer_num;
    uint16_t dma_timer_denom;

    if (audio_sample_rate == SAMPLE_RATE_16KHZ) {
        dma_timer_num = 8;
        dma_timer_denom = 62500;
    } else if (audio_sample_rate == SAMPLE_RATE_8KHZ) {
        dma_timer_num = 4;
        dma_timer_denom = 62500;
    }
 
    uint32_t cpu_clock_freq_hz = clock_get_hz(clk_sys);

#ifdef DEBUG_MODE
    float actual_sample_rate = cpu_clock_freq_hz * dma_timer_num / dma_timer_denom;
    DEBUG("Transfering data at %f hz %d %d %d %d\n", actual_sample_rate, cpu_clock_freq_hz, dma_timer_num, dma_timer_denom, audio_sample_rate);
#endif

    dma_timer_set_fraction(AUDIO_SINEWAVE_DMA_TIMER, dma_timer_num, dma_timer_denom);
    
    dma_channel_configure(
        audio_adc_dma_chan,
        &cfg,
        NULL,
        NULL,
        buffer_samples_count,
        false
    );

    dma_channel_set_irq0_enabled(audio_adc_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, audio_adc_dma_isr);
    irq_set_enabled(DMA_IRQ_0, true);

    DEBUG("Initialized sine wave\n");
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
    if (audio_sample_rate == SAMPLE_RATE_16KHZ) {
        clock_div = 16 * 1000;
    } else if (audio_sample_rate == SAMPLE_RATE_8KHZ) {
        clock_div = 8 * 1000;
    }
    float adc_clock_div = ADC_CLOCK_SPEED_HZ / clock_div;
    DEBUG("Transfering data at %f hz. Requested sample rate: %d\n", ADC_CLOCK_SPEED_HZ / adc_clock_div, audio_sample_rate);

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
        buffer_samples_count,
        false
    );

    dma_channel_set_irq0_enabled(audio_adc_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, audio_adc_dma_isr);
    irq_set_enabled(DMA_IRQ_0, true);
    DEBUG("Initialized ADC\n");
}

static void init_audio_dac_dma() {
    audio_dac_dma_chan = dma_claim_unused_channel(true);

    dma_channel_config cfg = dma_channel_get_default_config(audio_dac_dma_chan);

    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);

    if (sinewave_enabled) {
        channel_config_set_dreq(&cfg, dma_get_timer_dreq(AUDIO_SINEWAVE_DMA_TIMER));
    } else {
        channel_config_set_dreq(&cfg, DREQ_ADC);
    }
    
    dma_channel_configure(
        audio_dac_dma_chan,
        &cfg,
        &spi_get_hw(spi0)->dr,
        NULL,
        buffer_samples_count,
        false
    );

    dma_channel_set_irq1_enabled(audio_dac_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_1, audio_dac_dma_isr);
    irq_set_enabled(DMA_IRQ_1, true);

    gpio_put(SPI_DAC_CS_PIN, 0);
    DEBUG("Initialized DAC\n");
}

void audio_init() {
    DEBUG("Initializing ADC\n");

    uint16_t sample_rate = 0;

    gpio_put(data_ready_pin, 1);
    gpio_put(data_ready_pin, 0);
    sleep_ms(5);
    
    queue_remove_blocking(&i2c_msg_q, &sample_rate);

    DEBUG("Sample rate raw value: %d\n", sample_rate);
    assert(sample_rate == SAMPLE_RATE_16KHZ || sample_rate == SAMPLE_RATE_8KHZ);
    
    if (sample_rate == SAMPLE_RATE_8KHZ) {
        buffer_samples_count = 60;
        sinewave_buf = sinewave_8000;
        DEBUG("Sample rate: 8khz\n");
    }
    if (sample_rate == SAMPLE_RATE_16KHZ) {
        buffer_samples_count = 120;
        sinewave_buf = sinewave_16000;
        DEBUG("Sample rate: 16khz\n");
    }
    
    buffer_size = buffer_samples_count * sizeof(uint16_t);
    DEBUG("ADC buffer samples count: %d\n", buffer_samples_count);
    
    audio_sample_rate = sample_rate;

    for (int i = 0; i < MAX_BUFFERS; i++) {
        // ADC buffers
        adc_samples_buf[i] = malloc(buffer_size);
        if (adc_samples_buf[i] == NULL) {
            panic("Unable to allocate memory for ADC buffer\n");
        }
        assert(adc_samples_buf[i] != NULL);
        memset(adc_samples_buf[i], 0, buffer_size);

        // DAC buffers
        dac_samples_buf[i] = malloc(buffer_size);
        if (dac_samples_buf[i] == NULL) {
            panic("Unable to allocate memory for ADC buffer\n");
        }
        assert(dac_samples_buf[i] != NULL);
        memset(dac_samples_buf[i], 0, buffer_size);
    }
    
    i2c_data_out_buf = malloc(buffer_size);
    assert(i2c_data_out_buf != NULL);

    queue_init(&samples_ready_q, sizeof(uint8_t *), 8);

    init_audio_adc_dma();
    // init_audio_sine_dma();
    init_audio_dac_dma();
    
#ifdef DEBUG_MODE
    adc_start_sampling_us = get_absolute_time();
#endif

    configured = 1;
    audio_adc_dma_isr();
    adc_run(true);
}

void audio_deinit() {
    DEBUG("De-initializing audio\n");
    
    // De-init ADC DMA
    dma_channel_set_irq0_enabled(audio_adc_dma_chan, false);
    irq_set_enabled(DMA_IRQ_0, false);
    irq_remove_handler(DMA_IRQ_0, audio_adc_dma_isr);
    dma_channel_abort(audio_adc_dma_chan);
    if (!sinewave_enabled) {
        adc_run(false);
        adc_fifo_drain();
        DEBUG("Drained ADC fifo\n");
    }
    dma_channel_unclaim(audio_adc_dma_chan);
    DEBUG("Disabled ADC DMA\n");

    // De-init DAC DMA
    dma_channel_set_irq0_enabled(audio_dac_dma_chan, false);
    irq_set_enabled(DMA_IRQ_1, false);
    irq_remove_handler(DMA_IRQ_1, audio_dac_dma_isr);
    dma_channel_abort(audio_dac_dma_chan);
    dma_channel_unclaim(audio_dac_dma_chan);
    DEBUG("Disabled DAC DMA\n");

    if (dma_timer_is_claimed(AUDIO_SINEWAVE_DMA_TIMER)) {
        dma_timer_unclaim(AUDIO_SINEWAVE_DMA_TIMER);
        DEBUG("Unclaimed sinewave DMA timer\n");
    }
    
    // Free buffers
    queue_free(&i2c_msg_q);
    queue_free(&samples_ready_q);

    for (int i = 0; i < MAX_BUFFERS; i++) {
        free(adc_samples_buf[i]);
        adc_samples_buf[i] = NULL;

        free(dac_samples_buf[i]);
        dac_samples_buf[i] = NULL;
    }
    
    free(i2c_data_out_buf);
    i2c_data_out_buf = NULL;
    
    sinewave_buf = NULL;
    DEBUG("Freed buffers\n");

    adc_samples_buf_index = -1;
    dac_samples_buf_index = 0;
    dac_current_buf_index_streaming = -1;

    buffer_samples_count = 0;
    buffer_size = 0;

    audio_sample_rate = SAMPLE_RATE_NONE;
    dac_streaming = 0;
    sinewave_enabled = 0;
    configured = 0;
    DEBUG("Reset counters and flags\n");

    gpio_put(data_ready_pin, 0);
    gpio_put(SPI_DAC_CS_PIN, 1);
    DEBUG("Reset signal pins\n");
    
    // Drain I2C fifo
    size_t bytes_to_discard = i2c_get_read_available(i2c0);
    for (int i = 0; i < bytes_to_discard; i++) {
        i2c_read_byte_raw(i2c0);
    }
    DEBUG("Discarded %d bytes of I2C data\n", bytes_to_discard);
}
