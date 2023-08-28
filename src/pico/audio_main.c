#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "audio.h"
#include "debug.h"

#define LED_PIN PICO_DEFAULT_LED_PIN
#define LED_TOGGLE_TIMEOUT_MS 250
#define DATA_READY_PIN 8 // Toggled HIGH to notify master of PCM samples available for transmission

static bool led_state = true;

static alarm_id_t led_toggle_alarm = 0;

static int64_t toggle_led_alarm_callback(alarm_id_t id, void *user_data) {
    led_state = !led_state;
    gpio_put(LED_PIN, led_state);
    led_toggle_alarm = add_alarm_in_ms(LED_TOGGLE_TIMEOUT_MS, toggle_led_alarm_callback, NULL, true);
    return 0;
}

// Implements an audio I2C slave device at address 32
// --------------------------------------------------
// Starts ADC sampling once a sample rate has been received from master
// and notifies master when samples are available for transmission
// 
// During each transmission cycle, fetch any available PCM samples from master and
// forward them to external DAC through SPI
int main() {
    stdio_init_all();

    gpio_init(DATA_READY_PIN);
    gpio_set_dir(DATA_READY_PIN, GPIO_OUT);
    gpio_put(DATA_READY_PIN, 0);
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, led_state);
    
    led_toggle_alarm = add_alarm_in_ms(LED_TOGGLE_TIMEOUT_MS, toggle_led_alarm_callback, NULL, true);
    
    // Initialize I2C & SPI
    audio_transport_init(DATA_READY_PIN);
    audio_init();

    while (1) {
        // Check for any ADC samples in queue, and notify i2c master in case of availablity
        audio_transfer_samples();
    }
}
