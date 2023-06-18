#include <stdio.h>
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "hardware/adc.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "bt.h"
#include "dac.h"
#include "utils.h"
#include "debug.h"

#define MICRO_SLEEP_MS 1000
#define BIT_RATE 10

#define MAX_SAMPLES 1024
#define SAMPLE_RATE_HZ 44100

#define SCLK 2
#define MOSI 3
#define CS 5

int16_t sine_wave_buffer[MAX_SAMPLES];
uint16_t dac_data[MAX_SAMPLES];
const uint16_t MAX_SINE_VALUE = ((1 << BIT_RATE) / 2) - 1;

int main() {
    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Failed to initialise cyw43_arch\n");
        return -1;
    }
    
    // Init DAC
    dac_init(spi0, MOSI, SCLK, CS);
    
    // Init Bluetooth
    bt_init();

    uint16_t freq = 1000;
    uint16_t samples_num = utils_generate_sine_wave(freq, sine_wave_buffer, SAMPLE_RATE_HZ, MAX_SINE_VALUE);
    utils_sine_wave_for_tlc5615(sine_wave_buffer, dac_data, samples_num);

    printf("Generated %d samples for %dhz@%dkhz. Max sine value: %d\n", samples_num, freq, SAMPLE_RATE_HZ / 1000, MAX_SINE_VALUE);
    
    // dac_setup_streaming(dac_data, samples_num);
    // dac_start_streaming();

    printf("Streaming data\n");
    
    while (true) {
        __wfe();
        // for (uint i = 0; i < samples_num; i++) {
        //     dac_single_write(dac_data[i]);
        //     sleep_us(4.6);
        // }
    }
}