#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "adc_audio.h"
#include "dac_audio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "sample_rate.h"

#define TAG "TEST"

static sample_rate_t sample_rate;
static uint8_t buf[512] = {0};

static void consume_audio_dac(void *arg) {
    QueueHandle_t q = (QueueHandle_t) arg;
    uint8_t flag = 0;
    size_t request_bytes = (sample_rate == SAMPLE_RATE_16KHZ) ? 240 : 120;
    size_t received = 0;
    
    while (1) {
        if (xQueueReceive(q, &flag, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }
        
        adc_audio_receive(buf, &received, request_bytes);
        if (received != request_bytes) {
            continue;
        }
        dac_audio_send(buf, request_bytes);
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    QueueHandle_t audio_queue = xQueueCreate(8, sizeof(uint8_t));
    assert(audio_queue != NULL);
    
    sample_rate = SAMPLE_RATE_8KHZ;

    BaseType_t r = xTaskCreatePinnedToCore(consume_audio_dac, "consume_audio_dac", 4092, audio_queue, 10, NULL, 1);
    assert(r == pdPASS);

    adc_audio_init_transport();

    // Enable ADC
    adc_audio_init(sample_rate, audio_queue);
    adc_audio_enable(1);

    // Enable DAC
    dac_audio_init(sample_rate);
    dac_audio_enable(1);

    while (1) {
        vTaskDelay(portMAX_DELAY);
    }

}
