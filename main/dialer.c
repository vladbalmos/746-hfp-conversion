#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "dialer.h"

#define MAX_DIALED_NUMBER_LENGTH 33
#define SIGNAL_DIGIT_DIALED_TIMEOUT_MS 250
#define SIGNAL_END_DIALING_TIMEOUT_MS 2000

static QueueHandle_t irq_event_queue = NULL;
static gpio_num_t dialer_pin;
static gpio_num_t hook_switch_pin;
static uint8_t dialer_enabled = 0;
static char dialed_number[MAX_DIALED_NUMBER_LENGTH];
static int8_t dialed_digit = -1;
static uint8_t dialed_digits_counter = 0;
static uint8_t dialing_started = 0;
static uint8_t headset_state = 0;

static esp_timer_handle_t signal_digit_dialed_timer;
static esp_timer_handle_t signal_end_dialing_timer;

static dialer_headset_state_callback_t dialer_on_headset_state_change = NULL;
static dialer_start_callback_t dialer_on_start = NULL;
static dialer_dialed_digit_callback_t dialer_on_digit = NULL;
static dialer_end_callback_t dialer_on_end = NULL;

static int64_t last_irq_event_time;

static inline void reset_dialed_number() {
    dialed_digits_counter = 0;
    dialed_digit = -1;
    dialing_started = 0;
    memset(dialed_number, '\0', MAX_DIALED_NUMBER_LENGTH);
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t pin = (uint32_t) arg;
    xQueueSendFromISR(irq_event_queue, &pin, NULL);
}

static inline void cancel_all_alarms() {
    esp_timer_stop(signal_digit_dialed_timer);
    esp_timer_stop(signal_end_dialing_timer);
}

static inline void reset_dialer_state() {
    cancel_all_alarms();
    reset_dialed_number();
    
    if (dialer_on_end != NULL) {
        dialer_on_end(NULL, 0);
    }
}

static void signal_end_dialing_timer_callback(void *arg) {
    if (dialer_on_end != NULL) {
        dialer_on_end(dialed_number, dialed_digits_counter);
    }
    
    reset_dialed_number();
}

static void signal_digit_dialed_timer_callback(void *arg) {
    if (dialed_digit >= 10) {
        dialed_digit = 0;
    }
    
    if (dialed_digits_counter >= (MAX_DIALED_NUMBER_LENGTH - 1)) {
        // Don't go past maximum number length
        esp_timer_stop(signal_end_dialing_timer);
        signal_end_dialing_timer_callback(NULL);
        return;
    }
    if (dialer_on_digit != NULL) {
        dialer_on_digit(dialed_digit);
    }
    dialed_number[dialed_digits_counter++] = dialed_digit + '0';
    dialed_digit = -1;
    ESP_ERROR_CHECK(esp_timer_start_once(signal_end_dialing_timer, SIGNAL_END_DIALING_TIMEOUT_MS * 1000));
    return;
}

static void process_irq_events(void *arg) {
    uint32_t pin;
    int64_t now;
    int64_t diff_ms;
    uint8_t pin_level;
    
    while (1) {
        uint8_t received = 0;
        if (xQueueReceive(irq_event_queue, &pin, portMAX_DELAY)) {
            pin_level = gpio_get_level(pin);
            received = true;
        }
        if (!received) {
            continue;
        }
        
        now = esp_timer_get_time();
        diff_ms = (now - last_irq_event_time) / 1000;
        last_irq_event_time = now;
        
        if (diff_ms < 20) {
            // De-bounce
            continue;
        }
        
        if (pin == hook_switch_pin) {
            headset_state = pin_level;
            if (dialer_on_headset_state_change != NULL) {
                dialer_on_headset_state_change(headset_state);
            }
            if (!headset_state && dialed_digits_counter) {
                reset_dialer_state();
            }
            continue;
        }
        
        if (!headset_state) {
            // Dialing not allowed unless the headset is off hook
            continue;
        }
        
        if (pin_level && !dialing_started) {
            dialing_started = 1;

            if (dialer_on_start != NULL) {
                dialer_on_start();
            }
            continue;
        }
        
        if (pin_level && dialing_started) {
            cancel_all_alarms();
            if (diff_ms >= 200 && dialed_digit > -1) {
                signal_digit_dialed_timer_callback(NULL);
            }
            continue;
        }
        
        if (!pin_level && dialing_started) {
            dialed_digit++;
            cancel_all_alarms();
            ESP_ERROR_CHECK(esp_timer_start_once(signal_digit_dialed_timer, SIGNAL_DIGIT_DIALED_TIMEOUT_MS * 1000));
        }
    }
}

void dialer_init(gpio_num_t pin,
                 gpio_num_t hsw_pin,
                 dialer_headset_state_callback_t on_headset_state_change_callback,
                 dialer_start_callback_t start_callback,
                 dialer_dialed_digit_callback_t digit_callback,
                 dialer_end_callback_t end_callback
) {
    last_irq_event_time = esp_timer_get_time();
    reset_dialed_number();
    dialer_pin = pin;
    hook_switch_pin = hsw_pin;
    
    const esp_timer_create_args_t digit_dialed_timer_args = {
        .callback = &signal_digit_dialed_timer_callback,
        .arg = (void *) signal_digit_dialed_timer,
        .name = "digit-dialed"
    };
    const esp_timer_create_args_t end_dialing_timer_args = {
        .callback = &signal_end_dialing_timer_callback,
        .arg = (void *) signal_end_dialing_timer,
        .name = "digit-dialed"
    };
    
    ESP_ERROR_CHECK(esp_timer_create(&digit_dialed_timer_args, &signal_digit_dialed_timer));
    ESP_ERROR_CHECK(esp_timer_create(&end_dialing_timer_args, &signal_end_dialing_timer));

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << dialer_pin) | (1ULL << hook_switch_pin);
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_isr_handler_add(dialer_pin, gpio_isr_handler, (void *) dialer_pin));
    ESP_ERROR_CHECK(gpio_isr_handler_add(hook_switch_pin, gpio_isr_handler, (void *)hook_switch_pin));
    
    irq_event_queue = xQueueCreate(32, sizeof(uint32_t));
    
    dialer_on_headset_state_change = on_headset_state_change_callback;
    dialer_on_start = start_callback;
    dialer_on_digit = digit_callback;
    dialer_on_end = end_callback;
    headset_state = gpio_get_level(hook_switch_pin);
    
    BaseType_t result =  xTaskCreate(process_irq_events, "dial_proc_irq_ev", 2048, NULL, 10, NULL);
    assert(result == pdPASS);
}

uint8_t dialer_get_headset_state() {
    return headset_state;
}

void dialer_enable(uint8_t status) {
    dialer_enabled = status;
    
    if (!status){
        reset_dialed_number();
    }
}