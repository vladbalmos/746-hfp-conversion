#include <string.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "dialer.h"
#include "debug.h"

#define MAX_DIALED_NUMBER_LENGTH 33
#define SIGNAL_DIGIT_DIALED_TIMEOUT_MS 250

static uint8_t dialer_pin;
static uint8_t dialer_enabled = 0;
static char dialed_number[MAX_DIALED_NUMBER_LENGTH];
static int8_t dialed_digit = -1;
static uint8_t dialed_digits_counter = 0;
static uint8_t dialing_started = 0;

static alarm_id_t signal_digit_dialed_alarm = 0;

static dialer_start_callback *dialer_on_start = NULL;
static dialer_dialed_digit_callback *dialer_on_digit = NULL;
static dialer_end_callback *dialer_on_end = NULL;

static absolute_time_t last_irq_event_time;

static inline void reset_dialed_number() {
    dialed_digits_counter = 0;
    dialed_digit = -1;
    dialing_started = 0;
    memset(dialed_number, '\0', MAX_DIALED_NUMBER_LENGTH);
}

static int64_t signal_digit_dialed_alarm_handler(alarm_id_t id, void *user_data) {
    if (dialed_digit >= 10) {
        dialed_digit = 0;
    }
    // if (dialer_on_digit != NULL) {
    //     dialer_on_digit(dialed_digit);
    // }
    DEBUG("Dialed digit is: %d\n", dialed_digit);
    dialed_number[dialed_digits_counter++] = dialed_digit;
    
    DEBUG("Dialed number is: %s\n", &dialed_number[0]);

    dialed_digit = -1;
    return 0;
}

void dialer_init(uint8_t pin,
                 dialer_start_callback *start_callback,
                 dialer_dialed_digit_callback *digit_callback,
                 dialer_end_callback *end_callback
) {
    last_irq_event_time = nil_time;
    reset_dialed_number();
    dialer_pin = pin;

    gpio_init(dialer_pin);
    gpio_set_dir(dialer_pin, GPIO_IN);
    
    dialer_on_start = start_callback;
    dialer_on_digit = digit_callback;
    dialer_on_end = end_callback;
}

void dialer_enable(uint8_t status) {
    dialer_enabled = status;
    
    gpio_set_irq_enabled(dialer_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, status);
    
    if (!status){
        reset_dialed_number();
    }
}

void dialer_gpio_irq_handler(uint8_t pin, uint32_t event_mask) {
    if (pin != dialer_pin) {
        return;
    }

    if (event_mask == (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) {
        return;
    }
    
    absolute_time_t now = get_absolute_time();
    int64_t diff_ms = absolute_time_diff_us(last_irq_event_time, now) / 1000;
    last_irq_event_time = now;
    
    if (diff_ms < 20) {
        // De-bounce
        return;
    }

    if (event_mask == GPIO_IRQ_EDGE_RISE && !dialing_started) {
        dialing_started = 1;
    }
    
    if (event_mask == GPIO_IRQ_EDGE_RISE && dialing_started) {
        if (signal_digit_dialed_alarm) {
            cancel_alarm(signal_digit_dialed_alarm);
        }
        
        if (diff_ms >= 200 && dialed_digit > -1) {
            signal_digit_dialed_alarm_handler(0, NULL);
        }
    }
    
    if (event_mask == GPIO_IRQ_EDGE_FALL && dialing_started) {
        dialed_digit++;
        if (signal_digit_dialed_alarm) {
            cancel_alarm(signal_digit_dialed_alarm);
        }
        signal_digit_dialed_alarm = add_alarm_in_ms(SIGNAL_DIGIT_DIALED_TIMEOUT_MS, signal_digit_dialed_alarm_handler, NULL, false);
    }
    
    DEBUG("Rise: %d Fall: %d. Diff (us): %lld\n", event_mask == GPIO_IRQ_EDGE_RISE, event_mask == GPIO_IRQ_EDGE_FALL, diff_ms);
}