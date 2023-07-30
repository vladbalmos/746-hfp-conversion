#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "dac_audio.h"
#include "bt.h"
#include "dialer.h"
#include "ringer.h"

#define TAG "MAIN"

#define ESP_INTR_FLAG_DEFAULT 0
#define DIALER_PULSE_PIN 16
#define HOOK_SWITCH_PIN 17
#define RINGER_SIGNAL_PIN 5
#define RINGER_ENABLE_PIN 18

void on_headset_state_change(uint8_t state) {
    ESP_LOGI(TAG, "Headset state change: %d", state);
    ringer_enable(!state);
}

void on_start_dialing() {
    ESP_LOGI(TAG, "Started dialing");
}

void on_digit(uint8_t digit) {
    ESP_LOGI(TAG, "Dialed digit: %d", digit);
}

void on_end_dialing(const char *number, uint8_t number_length) {
    ESP_LOGI(TAG, "End dialing");
    if (!number_length) {
        return;
    }
    ESP_LOGI(TAG, "Dialed number: %s. Number length: %d", number, number_length);
}

#define PI 3.14159265
#define SINE_WAVE_FREQ_HZ 500
#define SINE_MAX_VAL (2 << 15) - 1
#define SAMPLE_RATE 16000

// uint16_t utils_generate_sine_wave(uint16_t frequency, int16_t *buffer, uint sample_rate, int16_t max_sine_value) {
//     const uint16_t samples = sample_rate / frequency;
    
//     for (uint16_t i = 0; i < samples; i++) {
//         double angle = (double)i / samples * 2.0 * PI;
//         double sine_val = sin(angle);
//         buffer[i] = round(sine_val * max_sine_value);
//     }
//     return samples;
// }

uint16_t utils_generate_sine_wave(uint16_t frequency, uint16_t *buffer, uint16_t sample_rate) {
    const uint16_t samples = sample_rate / frequency;
    
    for (uint16_t i = 0; i < samples; i++) {
        double angle = (double)i / (double)samples * 2.0 * PI;
        double sine_val = sin(angle);
        int val = round((sine_val + 1.0) / 2.0 * 4092);
        if (val < 0) {
            val = 0;
        }
        buffer[i] = (uint16_t) val;
    }
    return samples;
}

static uint8_t *buff = NULL;

void app_main(void) {
    // Initialize phone interface
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    
    dialer_init(DIALER_PULSE_PIN, HOOK_SWITCH_PIN, on_headset_state_change, on_start_dialing, on_digit, on_end_dialing);
    dialer_enable(1);
    
    ringer_init(RINGER_ENABLE_PIN, RINGER_SIGNAL_PIN);
    ESP_LOGI(TAG, "Headset state is %d", dialer_get_headset_state());

    int max_samples = SAMPLE_RATE / SINE_WAVE_FREQ_HZ + 1;
    buff = malloc(max_samples * sizeof(int16_t));
    assert(buff != NULL);
    uint16_t samples = utils_generate_sine_wave(SINE_WAVE_FREQ_HZ, (uint16_t *)buff, SAMPLE_RATE);

    
    printf("Total samples: %d\n", samples);
    uint16_t *samples_buf = (uint16_t *) buff;
    for (int i = 0; i < samples; i++) {
        printf("% 5d ", samples_buf[i]);
        
        if (i && i % 15 == 0) {
            printf("\n");
        }
    }
    printf("\n");
    
    dac_audio_buffer_pool_t *pool = dac_audio_init_buffer_pool(3, samples);
    dac_audio_init(DAC_SAMPLE_RATE_16KHZ);
    
    while (1) {
        dac_audio_buffer_t *buf = dac_audio_take_free_buffer_safe(pool, portMAX_DELAY);
        if (buf == NULL) {
            // ESP_LOGE(TAG, "Free buffer not available");
            continue;
        }

        uint16_t *dst = (uint16_t *) buf->bytes;
        for (int i = 0; i < buf->samples; i++) {
            uint16_t val = buff[i];
            *dst = ((val & 0xff) << 8) | ((val >> 8) & 0xff);
            dst++;
        }
        
        dac_audio_send(pool, buf);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    
    // Bluetooth initialization
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK( ret );

    // ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    
    // esp_err_t err;
    // esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    // if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
    //     ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
    //     return;
    // }

    // if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
    //     ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
    //     return;
    // }

    // if ((err = esp_bluedroid_init()) != ESP_OK) {
    //     ESP_LOGE(TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
    //     return;
    // }

    // if ((err = esp_bluedroid_enable()) != ESP_OK) {
    //     ESP_LOGE(TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
    //     return;
    // }
    // bt_init();
    
    // while(1) {
    //     vTaskDelay(2000 / portTICK_PERIOD_MS);
    // }
}
