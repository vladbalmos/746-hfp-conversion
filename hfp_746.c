#include <stdio.h>
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "hardware/adc.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "bt_a2dp_sink.h"
#include "dac_audio.h"
#include "utils.h"
#include "debug.h"

#define MICRO_SLEEP_MS 1000
#define BIT_RATE 10

#define MAX_SAMPLES 1024
// #define SAMPLE_RATE_HZ 44100
#define SAMPLE_RATE_HZ 16000

#define SCLK 2
#define MOSI 3
#define CS 5

int16_t sine_wave_buffer[MAX_SAMPLES];
uint16_t samples_num = 0;
uint16_t dac_data[MAX_SAMPLES];
const uint16_t MAX_SINE_VALUE = ((1 << BIT_RATE) / 2) - 1;
dac_audio_buffer_pool_t *pool = NULL;


int main() {
    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Failed to initialise cyw43_arch\n");
        return -1;
    }
    

    pool = dac_audio_init_buffer_pool(3, 512);

    // Init DAC
    dac_audio_init(spi0, MOSI, SCLK, CS, DAC_SAMPLE_RATE_16KHZ);
    
    // Init Bluetooth
    // bt_init();

    uint16_t freq = 440;
    uint16_t samples_num = utils_generate_sine_wave(freq, sine_wave_buffer, SAMPLE_RATE_HZ, MAX_SINE_VALUE);

    printf("Generated %d samples for %dhz@%dkhz. Max sine value: %d\n", samples_num, freq, SAMPLE_RATE_HZ / 1000, MAX_SINE_VALUE);
    
    dac_audio_start_streaming();
    printf("Streaming data\n");
    
    uint16_t sine_wave_index = 0;
    uint16_t sine_read_samples = 0;
    
    while (true) {
        dac_audio_buffer_t *buf = dac_audio_take_free_buffer();

        if (buf == NULL) {
            __wfe();
            continue;
        }

        uint16_t buffer_length = buf->size / sizeof(uint16_t);
        uint16_t buffer_used = 0;
        uint16_t *buffer = (uint16_t *) buf->bytes;
        uint16_t buffer_free_length = buffer_length - buffer_used;
        
        while (buffer_free_length > 0) {
            uint16_t max = (samples_num < buffer_free_length) ? samples_num : buffer_free_length;

            if (sine_wave_index >= samples_num) {
                sine_wave_index = 0;
            }

            sine_read_samples = utils_sine_wave_for_tlc5615(sine_wave_buffer, buffer + buffer_used, samples_num, max, sine_wave_index);
            sine_wave_index += sine_read_samples;
            
            buffer_used += sine_read_samples;
            buffer_free_length = buffer_length - buffer_used;
        }
        
        dac_audio_enqueue_ready_buffer(buf);
    }
}