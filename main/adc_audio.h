#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sample_rate.h"
#include "freertos/ringbuf.h"

typedef struct {
    QueueHandle_t adc_read_notif_queue;
    QueueHandle_t audio_available_notif_queue;
    RingbufHandle_t rb;
} audio_provider_task_input_t;

void adc_audio_init(sample_rate_t sample_rate, QueueHandle_t data_ready_queue);
void adc_audio_enable(uint8_t status);
void adc_audio_receive(uint8_t *dst, size_t *received, size_t requested_size);
void adc_audio_deinit();