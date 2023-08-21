#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/sync.h"

#define LED_PIN PICO_DEFAULT_LED_PIN
#define LED_TOGGLE_TIMEOUT_MS 250
#define ENABLE_PIN 9

static bool adc_enabled = false;
static bool adc_initialized = false;
static bool led_state = true;

static alarm_id_t led_toggle_alarm = 0;

void gpio_callback(uint gpio, uint32_t event_mask) {
    // printf("In here\n");
    if (event_mask == (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) {
        return;
    }
    
    if (event_mask == GPIO_IRQ_EDGE_RISE) {
        adc_enabled = true;
        return;
    }

    if (event_mask == GPIO_IRQ_EDGE_FALL) {
        adc_enabled = false;
    }
}


static int64_t toggle_led_alarm_callback(alarm_id_t id, void *user_data) {
    led_state = !led_state;
    gpio_put(LED_PIN, led_state);
    led_toggle_alarm = add_alarm_in_ms(LED_TOGGLE_TIMEOUT_MS, toggle_led_alarm_callback, NULL, true);
    return 0;
}

int main() {
    stdio_init_all();
    
    gpio_init(ENABLE_PIN);
    gpio_set_dir(ENABLE_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ENABLE_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, led_state);
    
    
    led_toggle_alarm = add_alarm_in_ms(LED_TOGGLE_TIMEOUT_MS, toggle_led_alarm_callback, NULL, true);
    
    adc_transport_initialize();


    while (1) {
        __wfi();
        // printf("Enabled: %d. Initialized: %d. State: %d\n", adc_enabled, adc_initialized, gpio_get(ENABLE_PIN));
        
        if (adc_enabled && !adc_initialized) {
            adc_initialize();
            adc_initialized = true;
            adc_start_streaming();
            continue;
        }
        
        if (!adc_enabled && adc_initialized) {
            adc_initialized = false;
            adc_deinit();
            continue;
        }
    }
}