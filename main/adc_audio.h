#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sample_rate.h"
#include "freertos/ringbuf.h"

typedef struct {
    QueueHandle_t notif_queue;
    RingbufHandle_t rb;
} audio_provider_task_input_t;

void adc_audio_init(sample_rate_t sample_rate, QueueHandle_t data_ready_queue);
void adc_audio_enable(uint8_t status);
void adc_audio_receive(uint8_t *buf, size_t size);
void adc_audio_deinit();