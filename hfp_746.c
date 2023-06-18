#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "debug.h"

#define MICRO_SLEEP_MS 1000
#define BIT_RATE 10

#define MAX_SAMPLES 255
#define PI 3.14159265
#define SAMPLE_RATE 44100

#define SPI_PORT spi0
#define SCLK 2
#define MOSI 3
#define CS 5

int16_t sine_wave[MAX_SAMPLES];
uint16_t biased_sine_wave[MAX_SAMPLES];
const uint16_t MAX_SINE_VALUE = ((1 << 10) / 2) - 1;

void print_binary(uint16_t n) {
    for (int i = 15; i >= 0; i--) {
        uint bit = (n >> i) & 1;
        printf("%u", bit);
    }
    
    printf("\n");
}

double voltage(int16_t sample, uint16_t bitrate, float ref_voltage) {
    double conv_factor = ref_voltage / (1 << BIT_RATE);
    
    double voltage = sample * conv_factor;
    return voltage;
}

double voltage_3_3(int16_t sample, uint16_t bitrate) {
    return voltage(sample, bitrate, 3.3f);
}

double voltage_5(int16_t sample, uint16_t bitrate) {
    return voltage(sample, bitrate, 5.0f);
}

// Generate sine wave at frequency
uint16_t generate_sine_wave(uint16_t frequency, int16_t *buffer) {
    const uint16_t samples = SAMPLE_RATE / frequency;
    
    for (uint16_t i = 0; i < samples; i++) {
        double angle = (double)i / samples * 2.0 * PI;
        double sine_val = sin(angle);
        buffer[i] = round(sine_val * MAX_SINE_VALUE);
    }
    
    
    return samples;
}

int main() {
    stdio_init_all();

    // Init Bluetooth chipset
    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }
    

    // Init SPI
    spi_init(SPI_PORT, 1000000);
    spi_set_format(SPI_PORT, 16, 0, 0, SPI_MSB_FIRST);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SCLK, GPIO_FUNC_SPI);
    
    // Init Chip select
    gpio_init(CS);
    gpio_set_dir(CS, GPIO_OUT);
    gpio_put(CS, 1);
    
    uint16_t freq = 1000;
    
    uint16_t samples_num = generate_sine_wave(freq, sine_wave);
    printf("Generated %d samples for %dhz@%dkhz. Max sine value: %d\n", samples_num, freq, SAMPLE_RATE / 1000, MAX_SINE_VALUE - 1);
    
    for (uint16_t i = 0; i < samples_num; i++) {
        int16_t sample = sine_wave[i];
        biased_sine_wave[i] = sample + MAX_SINE_VALUE;
        printf("%d |", sample, biased_sine_wave[i] << 2);
    }
    
    printf("\n");
 
    while (true) {
        for (uint i = 0; i < samples_num; i++) {
            uint16_t spi_samples[1] = {biased_sine_wave[i] << 2}; // 10 bit conversion
            
            gpio_put(CS, 0);
            spi_write16_blocking(SPI_PORT, spi_samples, 1);
            gpio_put(CS, 1);
            sleep_us(4.6);
        }
    }
}