#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "adc_audio.h"

#define AD_TAG "AD"

#define ADC_READ_TIMEOUT_MS 20
#define ADC_MIDPOINT (2 << 10) - 1
#define ADC_SAMPLE_SIZE 2

#define ADC_MAX_STORE_BUF_SIZE 1024
#define ADC_16KHZ_SAMPLES_NUM 240
#define ADC_8KHZ_SAMPLES_NUM 120

static uint8_t *audio_in_buf = NULL; // buffer holding ADC data
static sample_rate_t adc_sample_rate;
static gptimer_handle_t adc_read_timer = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL;
static QueueHandle_t adc_read_notif_queue = NULL;
static RingbufHandle_t audio_in_rb = NULL;
static TaskHandle_t audio_provider_task;
static audio_provider_task_input_t audio_provider_task_input;
static esp_timer_handle_t spi_transfer_timer;
static uint8_t initialized = 0;
static uint8_t enabled = 0;
static int64_t last_incoming_sample_us = 0;
static uint8_t dummy1 = 0;
static uint8_t dummy2 = 0;

static uint64_t adc_not_enabled_counter = 0;
static uint64_t adc_timeout_counter = 0;
static uint64_t adc_invalid_counter = 0;
static uint64_t adc_read_ok_counter = 0;
static uint64_t adc_stats_counter = 0;

static inline size_t adc_audio_get_buffer_size() {
    if (adc_sample_rate == SAMPLE_RATE_16KHZ) {
        return ADC_16KHZ_SAMPLES_NUM;
    }
    if (adc_sample_rate == SAMPLE_RATE_8KHZ) {
        return ADC_8KHZ_SAMPLES_NUM;
    }
    return 0;
}

static inline uint64_t adc_read_get_period() {
    assert(adc_sample_rate != SAMPLE_RATE_NONE);
    
    if (adc_sample_rate == SAMPLE_RATE_16KHZ) {
        return 62;
    }
    
    return 125;
}


static inline size_t adc_audio_get_rb_size() {
    return 4 * adc_audio_get_buffer_size();
}

static void provide_audio_task_handler(void *arg) {
    audio_provider_task_input_t *task_input = (audio_provider_task_input_t *) arg;
    RingbufHandle_t audio_in_rb = task_input->rb;
    QueueHandle_t notif_queue = task_input->adc_read_notif_queue;
    QueueHandle_t audio_available_queue = task_input->audio_available_notif_queue;
    esp_err_t adc_read_result;
    
    int raw_sample;
    int16_t pcm_sample;
    size_t buf_size = adc_audio_get_buffer_size();
    uint16_t read_bytes = 0;

    uint16_t stats_period = 1000000 / adc_read_get_period();
    
    while (1) {
        if (!enabled) {
            if (adc_not_enabled_counter++ % stats_period == 0) {
                ESP_LOGW(AD_TAG, "Not enabled");
            }
            vTaskDelay(1);
            continue;
        }
        
        if (xQueueReceive(notif_queue, &dummy2, (TickType_t)portMAX_DELAY) != pdTRUE) {
            // if (adc_not_enabled_counter++ % stats_period == 0) {
                ESP_LOGW(AD_TAG, "Not receiving");
            // }
            continue;
        }


        int64_t start_read = esp_timer_get_time();
        adc_read_result = adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &raw_sample);
        
        if (adc_read_result != ESP_OK) {
            if (adc_read_result == ESP_ERR_TIMEOUT) {
                // if (adc_timeout_counter++ % stats_period == 0) {
                    ESP_LOGW(AD_TAG, "Read timeout");
                // }
            } else if (adc_read_result == ESP_ERR_INVALID_ARG) {
                // if (adc_invalid_counter++ % stats_period == 0) {
                    ESP_LOGW(AD_TAG, "Read invalid");
                // }
            }
            continue;
        }
        
        pcm_sample = raw_sample - ADC_MIDPOINT;
        audio_in_buf[read_bytes] = pcm_sample;
        
        if (read_bytes >= buf_size) {
            int64_t now = esp_timer_get_time();
            int64_t interval = now - last_incoming_sample_us;
            last_incoming_sample_us = now;
        
            xRingbufferSend(audio_in_rb, audio_in_buf, read_bytes, 0);
            xQueueSend(audio_available_queue, &dummy1, 0);
            int64_t read_duration = esp_timer_get_time() - start_read;

            if (adc_stats_counter++ % 250 == 0) {
                ESP_LOGI(AD_TAG, "Last sample (us) %"PRId64" ADC read duration: %"PRId64". Read bytes: %d. Buf size: %d", interval, read_duration, read_bytes, buf_size);
            }
            read_bytes = 0;
        } else {
            read_bytes += ADC_SAMPLE_SIZE;
        }

        // if (adc_read_ok_counter++ % 5000 == 0) {
        //     ESP_LOGI(AD_TAG, "Read ok. Raw data %d %d", raw_sample, pcm_sample);
        // }
    }
}

static bool IRAM_ATTR signal_adc_read_alarm_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    QueueHandle_t notif_queue = (QueueHandle_t *) user_data;
    xQueueSendFromISR(notif_queue, &dummy1, NULL);
    return true;
}


void adc_audio_receive(uint8_t *dst, size_t *received, size_t requested_size) {
    if (!initialized || !enabled) {
        *received = 0;
        return;
    }
    
    uint8_t *data = xRingbufferReceiveUpTo(audio_in_rb, received, 0, requested_size);
    
    if (*received == requested_size) {
        memcpy(dst, data, *received);
        vRingbufferReturnItem(audio_in_rb, data);
        return;
    }
    
    if (*received) {
        vRingbufferReturnItem(audio_in_rb, data);
    }
    
    return;
}

void adc_audio_enable(uint8_t status) {
    if (!initialized && status) {
        return;
    }

    if (status == enabled) {
        return;
    }
    
    if (status) {
        ESP_LOGW(AD_TAG, "Enabling ADC");
        ESP_ERROR_CHECK(gptimer_set_raw_count(adc_read_timer, 0));
        ESP_ERROR_CHECK(gptimer_start(adc_read_timer));
        enabled = 1;
        return;
    }

    ESP_LOGW(AD_TAG, "Disabling ADC");
    ESP_ERROR_CHECK(gptimer_stop(adc_read_timer));
    enabled = 0;
}

void adc_audio_init(sample_rate_t sample_rate, QueueHandle_t data_ready_queue) {
    if (initialized) {
        return;
    }
    assert(sample_rate != SAMPLE_RATE_NONE);
    adc_sample_rate = sample_rate;
    
    size_t adc_buf_size = adc_audio_get_buffer_size();
    uint16_t sample_rate_hz = (sample_rate == SAMPLE_RATE_16KHZ) ? 16000 : 8000;
    ESP_LOGI(AD_TAG, "Initializing ADC. Sample rate: %d. Buffer size: %d", sample_rate_hz, adc_buf_size);

    audio_in_buf = malloc(adc_buf_size);
    assert(audio_in_buf != NULL);
    memset(audio_in_buf, '\0', adc_buf_size);
    
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_config, &adc_handle));
    
    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &channel_config));
    
    audio_in_rb = xRingbufferCreate(adc_audio_get_rb_size(), RINGBUF_TYPE_BYTEBUF);
    assert(audio_in_rb != NULL);
    
    adc_read_notif_queue = xQueueCreate(4, sizeof(uint8_t));
    audio_provider_task_input.adc_read_notif_queue = adc_read_notif_queue;
    audio_provider_task_input.audio_available_notif_queue = data_ready_queue;
    audio_provider_task_input.rb = audio_in_rb;

    BaseType_t r = xTaskCreatePinnedToCore(provide_audio_task_handler, "provide_audio", 4096, &audio_provider_task_input, 25, &audio_provider_task, 1);
    assert(r == pdPASS);
    
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000 // 1MHZ
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &adc_read_timer));
        
    gptimer_event_callbacks_t cbs = {
        .on_alarm = signal_adc_read_alarm_callback
    };
    
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(adc_read_timer, &cbs, adc_read_notif_queue));
    ESP_ERROR_CHECK(gptimer_enable(adc_read_timer));
    
    gptimer_alarm_config_t adc_read_alarm_config = {
        .reload_count = 0,
        .alarm_count = adc_read_get_period(),
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(adc_read_timer, &adc_read_alarm_config));

    initialized = 1;
}

void adc_audio_deinit() {
    if (!initialized) {
        return;
    }
    
    adc_audio_enable(0);
    
    adc_sample_rate = SAMPLE_RATE_NONE;
    
    gptimer_disable(adc_read_timer);
    gptimer_del_timer(adc_read_timer);
    
    vTaskDelete(audio_provider_task);
    
    vQueueDelete(adc_read_notif_queue);

    vRingbufferDelete(audio_in_rb);
    audio_in_rb = NULL;
    
    audio_provider_task_input.adc_read_notif_queue = NULL;
    audio_provider_task_input.audio_available_notif_queue = NULL;
    audio_provider_task_input.rb = NULL;

    free(audio_in_buf);
    audio_in_buf = NULL;

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle));
    initialized = 0;

    ESP_LOGI(AD_TAG, "De-initialized ADC");
}
