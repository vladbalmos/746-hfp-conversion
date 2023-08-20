#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "sample_rate.h"
#include "sine_16000_666.h"
#include "adc.h"

#define SPI_BAUDRATE_HZ 2500000
#define ADC_SPI_RX_PIN 12
#define ADC_SPI_TX_PIN 11
#define ADC_SPI_SCK_PIN 10
#define ADC_SPI_CS_PIN 13

static int dma_chan;
static uint64_t sent_counter = 0;

void dma_handler() {
    dma_hw->ints0 = 1u << dma_chan;
    if (sent_counter++ % 100 == 0) {
        printf("Data transfered\n");
    }
    dma_channel_set_write_addr(dma_chan, sinewave_16000, true);
}

void adc_transport_initialize() {
    uint actual_baud_rate = spi_init(spi1, SPI_BAUDRATE_HZ);
    spi_set_format(spi1, 16, 0, 0, SPI_MSB_FIRST);
    printf("Initialized SPI with baudrate: %d\n", actual_baud_rate);
    
    spi_set_slave(spi1, true);
    gpio_set_function(ADC_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADC_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADC_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADC_SPI_CS_PIN, GPIO_FUNC_SPI);
}

void print_binary(unsigned int number) {
    if (number > 1)
    {
        print_binary(number / 2);
    }
    printf("%d", number % 2);
}

static void start_streaming(sample_rate_t sample_rate) {
    dma_chan = dma_claim_unused_channel(true);
    assert(dma_chan != -1);
    
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    dma_timer_claim(0);
    
    uint16_t dma_timer_num;
    uint16_t dma_timer_denom;

    if (sample_rate == SAMPLE_RATE_16KHZ) {
        dma_timer_num = 8;
        dma_timer_denom = 62500;
    } else if (sample_rate == SAMPLE_RATE_8KHZ) {
        dma_timer_num = 4;
        dma_timer_denom = 62500;
    }

    uint32_t cpu_clock_freq_hz = clock_get_hz(clk_sys);
    float actual_sample_rate = cpu_clock_freq_hz * dma_timer_num / dma_timer_denom;
    printf("Transfering data at %f hz\n", actual_sample_rate);
    
    dma_channel_configure(
        dma_chan,
        &cfg,
        &spi_get_hw(spi1)->dr,
        sinewave_16000,
        120,
        true
    );

    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

static void stop_streaming() {
    irq_set_enabled(DMA_IRQ_0, false);
    irq_remove_handler(DMA_IRQ_0, dma_handler);
    dma_channel_set_irq0_enabled(dma_chan, false);
    dma_channel_abort(dma_chan);
    dma_timer_unclaim(0);
    dma_channel_unclaim(dma_chan);
}

void adc_initialize() {
    printf("Initializing ADC\n");
    
    uint16_t sample_rate;
    size_t read_bytes = spi_read16_blocking(spi1, 0, &sample_rate, 1);
    assert(read_bytes == 1);
    assert(sample_rate != SAMPLE_RATE_NONE);
    
    if (sample_rate == SAMPLE_RATE_8KHZ) {
        printf("Sample rate: 8khz\n");
    }
    if (sample_rate == SAMPLE_RATE_16KHZ) {
        printf("Sample rate: 16khz\n");
    }
    
    start_streaming(sample_rate);
}

void adc_deinit() {
    printf("De-initializing ADC\n");
    stop_streaming();
}