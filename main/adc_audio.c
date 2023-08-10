#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "adc_audio.h"

#define AD_TAG "AD"

#define ADC_MAX_STORE_BUF_SIZE 1024
#define ADC_16KHZ_SAMPLES_NUM 240
#define ADC_8KHZ_SAMPLES_NUM 120

static adc_audio_sample_rate_t adc_sample_rate;
static adc_continuous_handle_t adc_handle = NULL;
static RingbufHandle_t audio_in_rb = NULL;
static uint8_t initialized = 0;
static uint8_t enabled = 0;

static inline size_t adc_audio_get_conv_frame_size() {
    if (adc_sample_rate == ADC_SAMPLE_RATE_16KHZ) {
        return SOC_ADC_DIGI_DATA_BYTES_PER_CONV * ADC_16KHZ_SAMPLES_NUM;
    }
    if (adc_sample_rate == ADC_SAMPLE_RATE_8KHZ) {
        return SOC_ADC_DIGI_DATA_BYTES_PER_CONV * ADC_8KHZ_SAMPLES_NUM;
    }
    return 0;
}

static inline size_t adc_audio_get_rb_size() {
    return 4 * adc_audio_get_conv_frame_size();
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
        ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
        enabled = 1;
        return;
    }

    ESP_LOGW(AD_TAG, "Disabling ADC");
    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
    enabled = 0;
}

void adc_audio_init(adc_audio_sample_rate_t sample_rate, QueueHandle_t data_ready_queue) {
    if (initialized) {
        return;
    }
    assert(sample_rate != ADC_SAMPLE_RATE_NONE);
    adc_sample_rate = sample_rate;
    
    size_t buf_size = 256;
    uint16_t sample_rate_hz = (sample_rate == ADC_SAMPLE_RATE_16KHZ) ? 16000 : 8000;
    ESP_LOGI(AD_TAG, "Initializing ADC. Sample rate: %d. Buffer size: %d", sample_rate_hz, buf_size);

    size_t conv_frame_size = adc_audio_get_conv_frame_size();

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4 * conv_frame_size,
        .conv_frame_size = conv_frame_size,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));
    
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = sample_rate_hz,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1
    };
    
    adc_digi_pattern_config_t adc_pattern[1] = {0};
    dig_cfg.pattern_num = 1;
    adc_pattern[0].atten = ADC_ATTEN_DB_0;
    adc_pattern[0].channel = ADC_CHANNEL_0 & 0x7;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));

    audio_in_rb = xRingbufferCreate(adc_audio_get_rb_size(), RINGBUF_TYPE_BYTEBUF);
    assert(audio_in_rb != NULL);

    initialized = 1;
}

void adc_audio_deinit() {
    if (!initialized) {
        return;
    }
    
    adc_audio_enable(0);

    adc_sample_rate = ADC_SAMPLE_RATE_NONE;

    vRingbufferDelete(audio_in_rb);
    audio_in_rb = NULL;

    ESP_ERROR_CHECK(adc_continuous_deinit(adc_handle));
    initialized = 0;
    ESP_LOGI(AD_TAG, "De-initialized ADC");
}