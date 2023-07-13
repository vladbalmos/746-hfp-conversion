#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "ringer.h"

static uint8_t ringer_enable_pin;
static uint8_t ringer_signal_pin;
static uint pwm_slice;
static uint pwm_chan;

void ringer_init(uint8_t enable_pin, uint8_t signal_pin) {
    ringer_enable_pin = enable_pin;
    ringer_signal_pin = signal_pin;
    
    gpio_init(ringer_enable_pin);
    gpio_set_dir(ringer_enable_pin, GPIO_OUT);
    gpio_put(ringer_enable_pin, 0);
    
    gpio_set_function(ringer_signal_pin, GPIO_FUNC_PWM);
    
    pwm_slice = pwm_gpio_to_slice_num(ringer_signal_pin);
    pwm_chan = pwm_gpio_to_channel(ringer_signal_pin);
    pwm_set_clkdiv(pwm_slice, 95.367431641);
    pwm_set_wrap(pwm_slice, 65534);
    pwm_set_chan_level(pwm_slice, pwm_chan, 65534 / 2);
    
    pwm_set_enabled(pwm_slice, false);
}

void ringer_enable(uint8_t status) {
    pwm_set_enabled(pwm_slice, status);
    gpio_put(ringer_enable_pin, status);
}