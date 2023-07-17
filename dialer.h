#include "pico/stdlib.h"

// called every time the headset is taken/placed on/off hook
typedef void (*dialer_headset_state_callback_t)(uint8_t state);
// called when dialing is first started and headset is off hook
typedef void (*dialer_start_callback_t)();
// called after a digit is dialed and heaset is off hook
typedef void (*dialer_dialed_digit_callback_t)(uint8_t digit);
// called after a whole number has been dialed with the actual number as char* and the number length
// dialed_number is NULL and number_length is 0 (zero) if the headset is returned ON HOOK after at least a digit has been dialed
typedef void (*dialer_end_callback_t)(const char *dialed_number, uint8_t number_length);


void dialer_init(uint8_t dialer_pin,
                 uint8_t hook_switch_pin,
                 dialer_headset_state_callback_t headset_state_callback,
                 dialer_start_callback_t start_callback,
                 dialer_dialed_digit_callback_t digit_callback,
                 dialer_end_callback_t end_callback);
void dialer_gpio_irq_handler(uint8_t pin, uint32_t event_mask);
void dialer_enable(uint8_t status);