#include <stdio.h>
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "hardware/adc.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "bt_a2dp_sink.h"
#include "dac_audio.h"
#include "ringer.h"
#include "dialer.h"
#include "utils.h"
#include "debug.h"

#define BIT_RATE 10
#define MAX_SAMPLES 1024

#define SCLK 2
#define MOSI 3
#define CS 5

#define DAC_BUFFER_POOL_SIZE 3
#define DAC_BUFFER_MAX_SAMPLES 512

#define RING_SIGNAL_PIN 17
#define ENABLE_RINGER_PIN 16
#define ENABLE_RINGER_BTN_PIN 15

#define DIALER_PULSE_PIN 12
#define HOOK_SWITCH_PIN 13

int16_t sine_wave_buffer[MAX_SAMPLES];
uint16_t samples_num = 0;
const uint16_t MAX_SINE_VALUE = ((1 << BIT_RATE) / 2) - 1;
dac_audio_buffer_pool_t *pool = NULL;

void on_headset_state_change(uint8_t state) {
    DEBUG("Headset state change: %d\n", state);
}

void on_start_dialing() {
    DEBUG("Started dialing\n");
}

void on_digit(uint8_t digit) {
    DEBUG("Dialed digit: %d\n", digit);
}

void on_end_dialing(const char *number, uint8_t number_length) {
    DEBUG("Dialied number: %s. Number length: %d\n", number, number_length);
}

void default_irq_callback(uint pin, uint32_t event_mask) {
    dialer_gpio_irq_handler((uint8_t) pin, event_mask);
}

int main() {
    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Failed to initialise cyw43_arch\n");
        return -1;
    }
    DEBUG("Initialized\n");
    
    // Dialer
    // dialer_init(DIALER_PULSE_PIN, HOOK_SWITCH_PIN, on_headset_state_change, on_start_dialing, on_digit, on_end_dialing);
    // dialer_enable(1);
    
    // gpio_set_irq_callback(default_irq_callback);
    // irq_set_enabled(IO_IRQ_BANK0, 1);

    // while (true) {
    //     sleep_ms(1000);
    // }

    
    // Ringer
    // gpio_init(ENABLE_RINGER_BTN_PIN);
    // gpio_set_dir(ENABLE_RINGER_BTN_PIN, GPIO_IN);

    // ringer_init(ENABLE_RINGER_PIN, RING_SIGNAL_PIN);

    // uint8_t ringer_enabled = 0;
    // while (true) {
    //     if (gpio_get(ENABLE_RINGER_BTN_PIN)) {
    //         if (!ringer_enabled) {
    //             sleep_ms(250);
    //             ringer_enabled = 1;
    //             ringer_enable(ringer_enabled);
    //             DEBUG("Ringer is enabled\n");
    //             continue;
    //         }
            
    //         ringer_enabled = 0;
    //         ringer_enable(ringer_enabled);
    //         sleep_ms(250);
    //         DEBUG("Ringer is disabled\n");
    //         continue;
    //     }
        
        
    //     sleep_ms(25);
    // }
    
    // Bluetooth
    bt_init();
    while (true) {
        __wfi();
    }
    return 0;
}
