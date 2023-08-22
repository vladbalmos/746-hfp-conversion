#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "sample_rate.h"
#include "sine_16000_666.h"
#include "sine_8000_666.h"
#include "adc.h"

// SPI Slave device
// Reading and writing to SPI is done after pulling `data_ready_pin` HIGH
// Once spi transfer completes, the `data_ready_pin` is pulled LOW
// 
// Data flow:
// 1. Read sampling rate via SPI during initialization
// 2. Configure ADC to write 120 samples via `adc_dma_chan` to buffer
// 3. `adc_dma_chan` ISR gets triggered and configures `spi_dma_chan`
//    to stream buffer to SPI master
// 4. `spi_dma_chan` ISR gets triggered which executes step 2.

#define SPI_BAUDRATE_HZ 2500000
#define ADC_SPI_RX_PIN 12
#define ADC_SPI_TX_PIN 11
#define ADC_SPI_SCK_PIN 10
#define ADC_SPI_CS_PIN 13
#define ADC_DMA_TIMER 0

static uint8_t data_ready_pin;
static int adc_dma_chan;
static int spi_dma_chan;
static sample_rate_t adc_sample_rate;
static uint64_t sent_adc_counter = 0;
static uint64_t sent_spi_counter = 0;
static int16_t *adc_samples = NULL;
int16_t *sinewave_buf = NULL;
static absolute_time_t start;
static int64_t duration = 0;

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

void print_binary(unsigned int number) {
    if (number > 1)
    {
        print_binary(number / 2);
    }
    printf("%d", number % 2);
}

void adc_dma_isr() {
    duration = absolute_time_diff_us(start, get_absolute_time());
    
    dma_hw->ints0 = 1u << adc_dma_chan;
    if (sent_adc_counter++ % 500 == 0) {
        printf("ADC Data transfered: %lld\n", duration);
        for (int i = 0; i < 120; i++) {
            printf("%d ", adc_samples[i]);
            if (i && i % 8 == 0) {
                printf("\n");
            }
        }
        printf("\n");
    }
    gpio_put(data_ready_pin, 1);
    dma_channel_set_read_addr(spi_dma_chan, adc_samples, true);
}

void spi_dma_isr() {
    gpio_put(data_ready_pin, 0);
    dma_hw->ints1 = 1u << spi_dma_chan;
    if (sent_spi_counter++ % 500 == 0) {
        printf("SPI data transfered\n");
    }
    start = get_absolute_time();
    dma_channel_set_read_addr(adc_dma_chan, sinewave_buf, false);
    dma_channel_set_write_addr(adc_dma_chan, adc_samples, true);
}

void adc_stop_streaming() {
}

void adc_transport_initialize(uint8_t gpio_ready_pin) {
    data_ready_pin = gpio_ready_pin;

    uint actual_baud_rate = spi_init(spi1, SPI_BAUDRATE_HZ);
    spi_set_slave(spi1, true);
    spi_set_format(spi1, 16, 0, 0, SPI_MSB_FIRST);
    printf("Initialized SPI with baudrate: %d\n", actual_baud_rate);
    
    gpio_set_function(ADC_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADC_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADC_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADC_SPI_CS_PIN, GPIO_FUNC_SPI);
}

void adc_initialize() {
    printf("Initializing ADC\n");
    uint16_t sample_rate = 0;
    size_t read_count = 0;
    size_t adc_samples_buf_size = 0;

    // Read sampling rate from master
    read_count = spi_read(&sample_rate, 1);

    printf("Sample rate raw value: %d\n", sample_rate);
    assert(read_count == 1);
    assert(sample_rate == SAMPLE_RATE_16KHZ || sample_rate == SAMPLE_RATE_8KHZ);
    
    if (sample_rate == SAMPLE_RATE_8KHZ) {
        adc_samples_buf_size = 60;
        sinewave_buf = sinewave_8000;
        printf("Sample rate: 8khz\n");
    }
    if (sample_rate == SAMPLE_RATE_16KHZ) {
        adc_samples_buf_size = 120;
        sinewave_buf = sinewave_16000;
        printf("Sample rate: 16khz\n");
    }
    
    adc_sample_rate = sample_rate;
    adc_samples = malloc(adc_samples_buf_size * sizeof(uint16_t));
    assert(adc_samples != NULL);
    memset(adc_samples, 0, adc_samples_buf_size * sizeof(uint16_t));
    
    // Configure ADC DMA channel
    adc_dma_chan = dma_claim_unused_channel(true);
    assert(adc_dma_chan != -1);
    dma_timer_claim(ADC_DMA_TIMER);
    
    dma_channel_config cfg = dma_channel_get_default_config(adc_dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, dma_get_timer_dreq(ADC_DMA_TIMER));

    // TODO: replace this with DREQ_DMA once ADC is actually configured
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
    printf("Transfering data at %f hz %d %d %d %d\n", actual_sample_rate, cpu_clock_freq_hz, dma_timer_num, dma_timer_denom, adc_sample_rate);
    dma_timer_set_fraction(ADC_DMA_TIMER, dma_timer_num, dma_timer_denom);
    
    dma_channel_configure(
        adc_dma_chan,
        &cfg,
        adc_samples,
        sinewave_buf,
        120,
        false
    );

    dma_channel_set_irq0_enabled(adc_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, adc_dma_isr);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // Configure SPI DMA channel
    spi_dma_chan = dma_claim_unused_channel(true);
    assert(adc_dma_chan != -1);
    
    memset(&cfg, 0, sizeof(dma_channel_config));
    cfg = dma_channel_get_default_config(spi_dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, spi_get_dreq(spi1, true));

    dma_channel_configure(
        spi_dma_chan,
        &cfg,
        &spi_get_hw(spi1)->dr,
        adc_samples,
        120,
        false
    );

    dma_channel_set_irq1_enabled(spi_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_1, spi_dma_isr);
    irq_set_enabled(DMA_IRQ_1, true);
    
    spi_dma_isr();
}

void adc_deinit() {
    printf("De-initializing ADC\n");

    dma_channel_abort(adc_dma_chan);
    dma_channel_abort(spi_dma_chan);

    // de-init adc dma
    irq_set_enabled(DMA_IRQ_0, false);
    irq_remove_handler(DMA_IRQ_0, adc_dma_isr);
    dma_channel_set_irq0_enabled(adc_dma_chan, false);
    dma_timer_unclaim(ADC_DMA_TIMER);
    dma_channel_unclaim(adc_dma_chan);
    
    // de-init spi dma
    irq_set_enabled(DMA_IRQ_1, false);
    irq_remove_handler(DMA_IRQ_1, spi_dma_isr);
    dma_channel_set_irq0_enabled(spi_dma_chan, false);
    dma_channel_unclaim(spi_dma_chan);
    
    free(adc_samples);
    adc_samples = NULL;
}