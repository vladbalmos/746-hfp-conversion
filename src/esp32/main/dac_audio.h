#include <inttypes.h>
#include "freertos/semphr.h"
#include "sample_rate.h"

SemaphoreHandle_t dac_get_sem();
void dac_audio_init(sample_rate_t sample_rate);
void dac_audio_deinit();
void dac_audio_enable(uint8_t status);
void dac_audio_send(const uint8_t *buf, size_t size);
sample_rate_t dac_audio_get_sample_rate();