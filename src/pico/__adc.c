#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "debug.h"

#define MICRO_SLEEP_MS 1000
// 48mhz / 16khz - 1 = 2999 - audio range of 0 - 8000hz
#define ADC_SAMPLING_RATE 48 * 1000 * 1000 / 16000 - 1
// #define CONV_FACTOR_12b 3.3f / (1 << 12)
// #define CONV_FACTOR_10b 2 * 2.52f / (1 << 10)

#define MAX_SAMPLES 8

#define SPI_PORT spi0
#define SCLK 2
#define MOSI 3
#define CS 5

uint16_t samples_12b[MAX_SAMPLES];

int dma_chan;

void print_binary(uint16_t n) {
    for (int i = 15; i >= 0; i--) {
        uint bit = (n >> i) & 1;
        printf("%u", bit);
    }
    
    printf("\n");
}

void dma_handler() {
    dma_hw->ints0 = 1u << dma_chan;
    dma_channel_set_write_addr(dma_chan, samples_12b, false);
}

int main() {
    stdio_init_all();

    // Init Bluetooth chipset
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }
    

    // Init SPI
    spi_init(SPI_PORT, 500000);
    spi_set_format(SPI_PORT, 16, 0, 0, SPI_MSB_FIRST);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SCLK, GPIO_FUNC_SPI);
    
    // Init Chip select
    gpio_init(CS);
    gpio_set_dir(CS, GPIO_OUT);
    gpio_put(CS, 1);
    

    // Make sure GPIO is high-impedance, no pullups etc
    adc_init();

    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);
    adc_fifo_setup(
        true,
        true,
        1,
        false,
        false
    );
    adc_set_clkdiv(ADC_SAMPLING_RATE);
    
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(dma_chan, &cfg,
        samples_12b,
        &adc_hw->fifo,
        MAX_SAMPLES,
        true
    );
        
    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);
        
    printf("Capturing\n");
    adc_run(true);
    
    while (true) {
        __wfi();
        absolute_time_t last_sent = nil_time;
        absolute_time_t now;
        
        uint i = 0;
        
        while (i < MAX_SAMPLES) {
            now = get_absolute_time();
            uint32_t diff = absolute_time_diff_us(last_sent, now);
            if (diff >= 62) {
                uint16_t sample_12b[1] = {samples_12b[i]};
                
                gpio_put(CS, 0);
                spi_write16_blocking(SPI_PORT, sample_12b, 1);
                gpio_put(CS, 1);
                i++;
                last_sent = now;
            }
            sleep_us(5);
        }
            
        dma_channel_start(dma_chan);
    }
}