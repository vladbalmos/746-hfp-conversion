#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    ADC_SAMPLE_RATE_NONE,
    ADC_SAMPLE_RATE_8KHZ,
    ADC_SAMPLE_RATE_16KHZ,
} adc_audio_sample_rate_t;

void adc_audio_init(adc_audio_sample_rate_t sample_rate, QueueHandle_t data_ready_queue);
void adc_audio_enable(uint8_t status);
void adc_audio_receive(uint8_t *buf, size_t size);
void adc_audio_deinit();