#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "adc.h"

#define SPI_BAUDRATE_HZ 2500000
// #define SPI_BAUDRATE_HZ 100000
#define ADC_SPI_RX_PIN 12
#define ADC_SPI_TX_PIN 11
#define ADC_SPI_SCK_PIN 10
#define ADC_SPI_CS_PIN 13

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

void adc_initialize() {
    printf("Initializing ADC\n");
    
    uint16_t sample_rate;
    size_t read_bytes = spi_read16_blocking(spi1, 0, &sample_rate, 1);
    assert(read_bytes == sizeof(uint16_t));
    assert(sample_rate != SAMPLE_RATE_NONE);
    
    if (sample_rate == SAMPLE_RATE_8KHZ) {
        printf("Sample rate: 8khz\n");
    }
    if (sample_rate == SAMPLE_RATE_16KHZ) {
        printf("Sample rate: 16khz\n");
    }
    
    uint
}

void adc_deinit() {
    printf("De-initializing ADC\n");
}