#include <inttypes.h>
#include "freertos/queue.h"
#include "sample_rate.h"

void audio_init_transport();
void audio_init(sample_rate_t sample_rate, QueueHandle_t audio_ready_queue);
void audio_enable(uint8_t status);
void audio_receive(uint8_t *dst, size_t *received, size_t requested_size);
void audio_send(const uint8_t *buf, size_t size);
sample_rate_t audio_get_sample_rate();
void audio_deinit();