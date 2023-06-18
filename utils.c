#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "utils.h"

#define PI 3.14159265

void utils_print_binary(uint16_t n) {
    for (int i = 15; i >= 0; i--) {
        uint bit = (n >> i) & 1;
        printf("%u", bit);
    }
    
    printf("\n");
}

double utils_voltage(int16_t sample, uint16_t bitrate, float ref_voltage) {
    double conv_factor = ref_voltage / (1 << bitrate);
    
    double voltage = sample * conv_factor;
    return voltage;
}

double utils_voltage_3_3(int16_t sample, uint16_t bitrate) {
    return utils_voltage(sample, bitrate, 3.3f);
}

double utils_voltage_5(int16_t sample, uint16_t bitrate) {
    return utils_voltage(sample, bitrate, 5.0f);
}

uint16_t utils_generate_sine_wave(uint16_t frequency, int16_t *buffer, uint sample_rate, uint16_t max_sine_value) {
    const uint16_t samples = sample_rate / frequency;
    
    for (uint16_t i = 0; i < samples; i++) {
        double angle = (double)i / samples * 2.0 * PI;
        double sine_val = sin(angle);
        buffer[i] = round(sine_val * max_sine_value);
    }
    return samples;
}

void utils_bias_sine_wave(int16_t *src_buffer, uint16_t *dst_buffer, uint16_t samples_num, uint16_t bias) {
    for (uint16_t i = 0; i < samples_num; i++) {
        dst_buffer[i] = src_buffer[i] + bias;
    }
}

void utils_sine_wave_for_tlc5615(int16_t *src_buffer, uint16_t *dst_buffer, uint16_t samples_num) {
    utils_bias_sine_wave(src_buffer, dst_buffer, samples_num, ((1 << 10) / 2) - 1);
    for (uint16_t i = 0; i < samples_num; i++) {
        dst_buffer[i] = dst_buffer[i] << 2; // shift by 2 bits to make it compatible with TLC5615
    }
}