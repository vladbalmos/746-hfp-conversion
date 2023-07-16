#include "pico/stdlib.h"

typedef void (*dialer_start_callback)();
typedef void (*dialer_dialed_digit_callback)(uint8_t digit);
typedef void (*dialer_end_callback)(const char *dialed_number, uint8_t number_length);


void dialer_init(uint8_t dialer_pin,
                 dialer_start_callback *start_callback,
                 dialer_dialed_digit_callback *digit_callback,
                 dialer_end_callback *end_callback);
void dialer_gpio_irq_handler(uint8_t pin, uint32_t event_mask);
void dialer_enable(uint8_t status);