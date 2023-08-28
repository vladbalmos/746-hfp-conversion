#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "audio.h"
#include "debug.h"

#define LED_PIN PICO_DEFAULT_LED_PIN
#define LED_TOGGLE_TIMEOUT_MS 250
#define ENABLE_PIN 9 // HIGH when we should transmit/receive PCM data, LOW otherwise
#define DATA_READY_PIN 8 // Toggled HIGH to notify master of PCM samples available for transmission

static bool enabled = false;
static bool initialized = false;
static bool led_state = true;

static alarm_id_t led_toggle_alarm = 0;

// Detect ENABLE_PIN changes
void gpio_callback(uint gpio, uint32_t event_mask) {
    if (event_mask == (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) {
        DEBUG("Enable pin both edges\n");
        return;
    }
    
    
    if (event_mask == GPIO_IRQ_EDGE_RISE) {
        DEBUG("Enable 1\n");
        enabled = true;
        return;
    }

    if (event_mask == GPIO_IRQ_EDGE_FALL) {
        DEBUG("Enable 0\n");
        enabled = false;
    }
}


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
    
    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ENABLE_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    gpio_init(DATA_READY_PIN);
    gpio_set_dir(DATA_READY_PIN, GPIO_OUT);
    gpio_put(DATA_READY_PIN, 0);
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, led_state);
    
    led_toggle_alarm = add_alarm_in_ms(LED_TOGGLE_TIMEOUT_MS, toggle_led_alarm_callback, NULL, true);
    
    // Initialize I2C & SPI
    audio_transport_init(DATA_READY_PIN);

    while (1) {
        if (enabled && !initialized) {
            audio_init();
            initialized = true;
        }
        
        if (!enabled && initialized) {
            initialized = false;
            audio_deinit();
        }
        
        if (initialized) {
            // Check for any ADC samples in queue, and notify i2c master in case of availablity
            audio_transfer_samples();
            continue;
        }
        
        __wfe();
    }
}
