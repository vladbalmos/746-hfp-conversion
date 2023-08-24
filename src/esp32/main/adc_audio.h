#include <inttypes.h>
#include "freertos/queue.h"
#include "sample_rate.h"

void adc_audio_init_transport();
void adc_audio_init(sample_rate_t sample_rate, QueueHandle_t audio_ready_queue);
void adc_audio_enable(uint8_t status);
void adc_audio_receive(uint8_t *dst, size_t *received, size_t requested_size);
void adc_audio_deinit();