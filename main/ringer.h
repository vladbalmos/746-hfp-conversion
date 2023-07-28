#include <inttypes.h>
#include "driver/gpio.h"

#define RG_TAG "RINGER"

void ringer_init(gpio_num_t enable_pin, gpio_num_t signal_pin);
void ringer_enable(uint8_t status);