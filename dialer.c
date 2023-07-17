#include <string.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "dialer.h"
#include "debug.h"

#define MAX_DIALED_NUMBER_LENGTH 33
#define SIGNAL_DIGIT_DIALED_TIMEOUT_MS 250
#define SIGNAL_END_DIALING_TIMEOUT_MS 2000

static uint8_t dialer_pin;
static uint8_t hook_switch_pin;
static uint8_t dialer_enabled = 0;
static char dialed_number[MAX_DIALED_NUMBER_LENGTH];
static int8_t dialed_digit = -1;
static uint8_t dialed_digits_counter = 0;
static uint8_t dialing_started = 0;
static uint8_t headset_state = 0;

static alarm_id_t signal_digit_dialed_alarm = 0;
static alarm_id_t signal_end_dialing_alarm = 0;

static dialer_headset_state_callback_t dialer_on_headset_state_change = NULL;
static dialer_start_callback_t dialer_on_start = NULL;
static dialer_dialed_digit_callback_t dialer_on_digit = NULL;
static dialer_end_callback_t dialer_on_end = NULL;

static absolute_time_t last_irq_event_time;

static inline void reset_dialed_number() {
    dialed_digits_counter = 0;
    dialed_digit = -1;
    dialing_started = 0;
    memset(dialed_number, '\0', MAX_DIALED_NUMBER_LENGTH);
}

static inline void cancel_all_alarms() {
    if (signal_digit_dialed_alarm) {
        cancel_alarm(signal_digit_dialed_alarm);
    }

    if (signal_end_dialing_alarm) {
        cancel_alarm((signal_end_dialing_alarm));
    }
}

static inline void reset_dialer_state() {
    cancel_all_alarms();
    reset_dialed_number();
    
    if (dialer_on_end != NULL) {
        dialer_on_end(NULL, 0);
    }
}

static int64_t signal_end_dialing_alarm_handler(alarm_id_t id, void *user_data) {
    if (dialer_on_end != NULL) {
        dialer_on_end(dialed_number, dialed_digits_counter);
    }
    
    reset_dialed_number();
    return 0;
}

static int64_t signal_digit_dialed_alarm_handler(alarm_id_t id, void *user_data) {
    if (dialed_digit >= 10) {
        dialed_digit = 0;
    }
    
    if (dialed_digits_counter >= (MAX_DIALED_NUMBER_LENGTH - 1)) {
        // Don't go past maximum number length
        if (signal_end_dialing_alarm) {
            cancel_alarm((signal_end_dialing_alarm));
            // signal end of dialing
            signal_end_dialing_alarm_handler(0, NULL);
        }
        return 0;
    }
    if (dialer_on_digit != NULL) {
        dialer_on_digit(dialed_digit);
    }
    dialed_number[dialed_digits_counter++] = dialed_digit + '0';
    dialed_digit = -1;
    signal_end_dialing_alarm = add_alarm_in_ms(SIGNAL_END_DIALING_TIMEOUT_MS, signal_end_dialing_alarm_handler, NULL, false);
    return 0;
}

void dialer_init(uint8_t pin,
                 uint8_t hsw_pin,
                 dialer_headset_state_callback_t on_headset_state_change_callback,
                 dialer_start_callback_t start_callback,
                 dialer_dialed_digit_callback_t digit_callback,
                 dialer_end_callback_t end_callback
) {
    last_irq_event_time = nil_time;
    reset_dialed_number();
    dialer_pin = pin;
    hook_switch_pin = hsw_pin;
    
    gpio_init(dialer_pin);
    gpio_set_dir(dialer_pin, GPIO_IN);

    gpio_init(hook_switch_pin);
    gpio_set_dir(hook_switch_pin, GPIO_IN);
    
    dialer_on_headset_state_change = on_headset_state_change_callback;
    dialer_on_start = start_callback;
    dialer_on_digit = digit_callback;
    dialer_on_end = end_callback;
    headset_state = gpio_get(hook_switch_pin);
}

void dialer_enable(uint8_t status) {
    dialer_enabled = status;
    
    gpio_set_irq_enabled(dialer_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, status);
    gpio_set_irq_enabled(hook_switch_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, status);
    
    if (!status){
        reset_dialed_number();
    }
}

void dialer_gpio_irq_handler(uint8_t pin, uint32_t event_mask) {
    if (pin != dialer_pin && pin != hook_switch_pin) {
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
    
    if (pin == hook_switch_pin) {
        headset_state = (event_mask == GPIO_IRQ_EDGE_RISE) ? 1 : 0;
        if (dialer_on_headset_state_change != NULL) {
            dialer_on_headset_state_change(headset_state);
        }
        if (!headset_state && dialed_digits_counter) {
            reset_dialer_state();
        }
        return;
    }

    if (event_mask == GPIO_IRQ_EDGE_RISE && !dialing_started) {
        dialing_started = 1;
        if (dialer_on_start != NULL) {
            dialer_on_start();
        }
    }
    
    if (event_mask == GPIO_IRQ_EDGE_RISE && dialing_started) {
        cancel_all_alarms();
        if (diff_ms >= 200 && dialed_digit > -1) {
            signal_digit_dialed_alarm_handler(0, NULL);
        }
    }
    
    if (event_mask == GPIO_IRQ_EDGE_FALL && dialing_started) {
        dialed_digit++;
        cancel_all_alarms();
        signal_digit_dialed_alarm = add_alarm_in_ms(SIGNAL_DIGIT_DIALED_TIMEOUT_MS, signal_digit_dialed_alarm_handler, NULL, false);
    }
    // DEBUG("Rise: %d Fall: %d. Diff (us): %lld\n", event_mask == GPIO_IRQ_EDGE_RISE, event_mask == GPIO_IRQ_EDGE_FALL, diff_ms);
}