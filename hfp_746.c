#include <stdio.h>
#include "pico/stdio.h"
#include "pico/stdlib.h"

#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "dac_audio.h"
#include "ringer.h"
#include "dialer.h"
#include "utils.h"
#include "debug.h"

#define DAC_BUFFER_POOL_SIZE 3
#define DAC_BUFFER_MAX_SAMPLES 512

#define ADC_SAMPLING_RATE 48 * 1000 * 1000 / 16000 - 1

#define BIT_RATE 10

#define SCLK 2
#define MOSI 3
#define CS 5

#define RING_SIGNAL_PIN 17
#define ENABLE_RINGER_PIN 16
#define ENABLE_RINGER_BTN_PIN 15

#define DIALER_PULSE_PIN 12
#define HOOK_SWITCH_PIN 13

int dma_chan;
static spin_lock_t *sl;
dac_audio_buffer_pool_t *pool = NULL;
static int16_t pcm_data[DAC_BUFFER_MAX_SAMPLES];

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

void dma_handler() {
    dma_hw->ints1 = 1u << dma_chan;
    uint32_t save_irq = spin_lock_blocking(sl);
    dac_audio_buffer_t *buff = dac_audio_take_free_buffer();
    spin_unlock(sl, save_irq);
    if (buff == NULL) {
        return;
    }
    uint16_t *b = (uint16_t *) buff->bytes;
    int counter = 0;
    for (int i = 0; i < pool->buffer_size; i++) {
        // uint16_t sample = ((pcm_data[i] + 32768) / 64) << 2;
        uint16_t sample = pcm_data[i] >> 2;
        // printf("%d ", sample);
        // if (counter++ >= 7) {
        //     counter = 0;
        //     printf("\n");
        // }
        *b = sample;
        b++;
    }
    // printf("=======================================\n");
    memset(pcm_data, 0, sizeof(pcm_data));
    dma_channel_set_write_addr(dma_chan, pcm_data, true);

    save_irq = spin_lock_blocking(sl);
    dac_audio_enqueue_ready_buffer(buff);
    spin_unlock(sl, save_irq);
}

int main() {
    stdio_init_all();

    pool = dac_audio_init_buffer_pool(DAC_BUFFER_POOL_SIZE, DAC_BUFFER_MAX_SAMPLES);

    if (pool == NULL) {
        panic("Unable to allocate memory for DAC buffer pool!\n");
    }
    dac_audio_init(spi0, MOSI, SCLK, CS, DAC_SAMPLE_RATE_16KHZ);

    adc_init();

    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);
    adc_fifo_setup(
        true,
        true,
        1,
        false,
        false
    );
    adc_set_clkdiv(ADC_SAMPLING_RATE);

    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_ADC);

    adc_run(true);
    dac_audio_start_streaming();
    dma_channel_configure(dma_chan, &cfg,
        pcm_data,
        &adc_hw->fifo,
        DAC_BUFFER_MAX_SAMPLES,
        true
    );
        
    DEBUG("Initialized\n");

    dma_channel_set_irq1_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_1, dma_handler);
    irq_set_enabled(DMA_IRQ_1, true);
        
    printf("Capturing\n");
    
    while (1) {
        __wfe();
        // if (dac_audio_remaining_free_buffer_slots()) {
        //     dma_handler();
        // } else {
        //     DEBUG("No \n");
        // }
    }
    
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
    return 0;
}
