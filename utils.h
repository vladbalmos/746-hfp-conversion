#include "pico/stdlib.h"

void utils_print_binary(uint16_t n);

double utils_voltage(int16_t sample, uint16_t bitrate, float ref_voltage);

double utils_voltage_3_3(int16_t sample, uint16_t bitrate);

double utils_voltage_5(int16_t sample, uint16_t bitrate);
/**
 * @brief Generate 16bit sine wave
 */
uint16_t utils_generate_sine_wave(uint16_t frequency, int16_t *buffer, uint sample_rate, uint16_t max_sine_value);

uint16_t utils_bias_audio(int16_t *src_buffer, uint16_t *dst_buffer, uint16_t src_buffer_length, uint16_t samples_num, uint16_t bias, uint16_t start_at);

uint16_t utils_prepare_audio_for_tlc5615(int16_t *src_buffer, uint16_t *dst_buffer, uint16_t src_buffer_length, uint16_t samples_num, uint16_t start_at);