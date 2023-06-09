/**
 * Based on the btstack_audio_pico.c from:
 *   https://github.com/raspberrypi/pico-examples/blob/master/pico_w/bt/btstack_audio_pico.c
 */
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
#include "btstack.h"
#include "dac_audio.h"
#include "bt_audio.h"
#include "debug.h"

#define SCLK 2
#define MOSI 3
#define CS 5

#define DAC_BUFFER_POOL_SIZE 3
#define DAC_BUFFER_MAX_SAMPLES 512

#define DRIVER_POLL_INTERVAL_MS 5

static uint8_t btstack_audio_pico_sink_active;

static void (*playback_callback)(int16_t * buffer, uint16_t num_samples);

static btstack_timer_source_t  driver_timer_sink;

static dac_audio_buffer_pool_t *pool = NULL;

// Holds stereo PCM data
static int16_t pcm_data[DAC_BUFFER_MAX_SAMPLES * 2];

static spin_lock_t *sl;

static void bt_audio_fill_buffers(void){
    while (true){
        uint32_t save_irq = spin_lock_blocking(sl);
        dac_audio_buffer_t *audio_buffer = dac_audio_take_free_buffer();
        spin_unlock(sl, save_irq);

        if (audio_buffer == NULL){
            break;
        }

        (*playback_callback)(pcm_data, pool->buffer_size);
        uint16_t *b = (uint16_t *) audio_buffer->bytes;

        // Copy the left channel data
        for (int i = 0; i < pool->buffer_size * 2; i += 2) {
            uint16_t sample = ((pcm_data[i] + 32768) / 64) << 2;
            *b = sample;
            b++;
        }
        
        memset(pcm_data, 0, sizeof(pcm_data));

        save_irq = spin_lock_blocking(sl);
        dac_audio_enqueue_ready_buffer(audio_buffer);
        spin_unlock(sl, save_irq);
    }
}

static void driver_timer_handler_sink(btstack_timer_source_t * ts){
    bt_audio_fill_buffers();
    btstack_run_loop_set_timer(ts, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(ts);
}

static int bt_audio_sink_init(
    uint8_t channels,
    uint32_t samplerate, 
    void (*playback)(int16_t * buffer, uint16_t num_samples)
){
    playback_callback  = playback;

    pool = dac_audio_init_buffer_pool(DAC_BUFFER_POOL_SIZE, DAC_BUFFER_MAX_SAMPLES);

    if (pool == NULL) {
        panic("Unable to allocate memory for DAC buffer pool!\n");
    }
    dac_audio_init(spi0, MOSI, SCLK, CS, DAC_SAMPLE_RATE_44KHZ);
    
    sl = spin_lock_init(spin_lock_claim_unused(true));

    return 0;
}

static void bt_audio_sink_set_volume(uint8_t volume){
}

static void bt_audio_sink_start_stream(void){
    bt_audio_fill_buffers();

    btstack_run_loop_set_timer_handler(&driver_timer_sink, &driver_timer_handler_sink);
    btstack_run_loop_set_timer(&driver_timer_sink, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(&driver_timer_sink);

    btstack_audio_pico_sink_active = 1;

    dac_audio_start_streaming();
}

static void bt_audio_sink_stop_stream(void) {
    btstack_run_loop_remove_timer(&driver_timer_sink);
    dac_audio_stop_streaming();
    btstack_audio_pico_sink_active = 0;
}

static void bt_audio_sink_close(void){
    if (btstack_audio_pico_sink_active){
        bt_audio_sink_stop_stream();
    }
}

static const btstack_audio_sink_t bt_audio_sink = {
    .init = &bt_audio_sink_init,
    .set_volume = &bt_audio_sink_set_volume,
    .start_stream = &bt_audio_sink_start_stream,
    .stop_stream = &bt_audio_sink_stop_stream,
    .close = &bt_audio_sink_close,
};

const btstack_audio_sink_t * bt_audio_sink_get_instance(void) {
    return &bt_audio_sink;
}
