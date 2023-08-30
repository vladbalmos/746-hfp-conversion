#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "audio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "sample_rate.h"

#define TAG "TEST"

static sample_rate_t sample_rate;
static uint8_t buf[240] = {0};

static void consume_audio(void *arg) {
    QueueHandle_t q = (QueueHandle_t) arg;
    uint8_t flag = 0;
    size_t request_bytes = (sample_rate == SAMPLE_RATE_16KHZ) ? 240 : 120;
    size_t received = 0;
    
    while (1) {
        if (xQueueReceive(q, &flag, (TickType_t)portMAX_DELAY) != pdTRUE) {
            continue;
        }
        
        received = 0;
        audio_receive(buf, &received, request_bytes);
        if (received != request_bytes) {
            continue;
        }
        audio_send(buf, request_bytes);
    }
}

void app_main(void) {
    int audio_enable_state = 1;
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    QueueHandle_t audio_queue = xQueueCreate(8, sizeof(uint8_t));
    assert(audio_queue != NULL);
    
    sample_rate = SAMPLE_RATE_16KHZ;
    // sample_rate = SAMPLE_RATE_8KHZ;

    BaseType_t r = xTaskCreatePinnedToCore(consume_audio, "consume_audio", 4092, audio_queue, 5, NULL, 1);
    assert(r == pdPASS);

    audio_init_transport();

    // Enable ADC/DAC
    audio_init(sample_rate, audio_queue);
    audio_enable(audio_enable_state);
    
    int64_t now = esp_timer_get_time();
    int64_t last_changed = esp_timer_get_time();
    int64_t diff = 0;

    while (1) {
        vTaskDelay(10);
        now = esp_timer_get_time();
        
        diff = (now - last_changed) / 1000;
        
        if (diff >= 2000) {
            ESP_LOGI(TAG, "Changing state from %d to %d", audio_enable_state, !audio_enable_state);
            audio_enable_state = !audio_enable_state;
            last_changed = now;
            audio_enable(audio_enable_state);
        }
    }

}
