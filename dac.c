#include <stdio.h>
#include "pico/stdio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"

#define SPI_BAUD_RATE_HZ 10 * 1000 * 1000

#define _DMA_TIMER_0 0

static spi_inst_t *port;
static uint8_t mosi_pin;
static uint8_t clk_pin;
static uint8_t cs_pin;

static uint16_t *dma_buffer;
static uint16_t dma_buffer_length;

static int data_chan_dma;
static int ctrl_chan_dma;

static uint16_t *ctrl_dma_read_address;

static void dma_handler() {
    // Clear the interrupt request.
    dma_hw->ints0 = 1u << ctrl_chan_dma;
}

void dac_init(spi_inst_t *spi_port, uint8_t mosi, uint8_t clk, uint8_t cs) {
    port = spi_port;

    // Init spi port and baud rate
    spi_init(port, SPI_BAUD_RATE_HZ);
    spi_set_format(port, 16, 0, 0, 0);
    
    mosi_pin = mosi;
    clk_pin = clk;
    cs_pin = cs;

    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(clk_pin, GPIO_FUNC_SPI);
    gpio_set_function(cs_pin, GPIO_FUNC_SPI);
}

void dac_setup_streaming(uint16_t *buffer, uint16_t buffer_length) {
    dma_buffer = buffer;
    dma_buffer_length = buffer_length;
    
    ctrl_dma_read_address = &dma_buffer[0];
    
    data_chan_dma = dma_claim_unused_channel(true);
    ctrl_chan_dma = dma_claim_unused_channel(true);
    
    // Configure data channel
    dma_channel_config data_cfg = dma_channel_get_default_config(data_chan_dma);

    channel_config_set_transfer_data_size(&data_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&data_cfg, true);
    channel_config_set_write_increment(&data_cfg, false);
    // SPI write at ~44.1khz
    dma_timer_set_fraction(_DMA_TIMER_0, 0x0017, 0xffff);
    channel_config_set_dreq(&data_cfg, DREQ_DMA_TIMER0);
    channel_config_set_chain_to(&data_cfg, ctrl_chan_dma);
    
    dma_channel_configure(
        data_chan_dma,
        &data_cfg,
        &spi_get_hw(port)->dr,
        dma_buffer,
        dma_buffer_length,
        false
    );
    
    // Configure control channel
    dma_channel_config ctrl_cfg = dma_channel_get_default_config(ctrl_chan_dma);

    channel_config_set_transfer_data_size(&ctrl_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&ctrl_cfg, false);
    channel_config_set_write_increment(&ctrl_cfg, false);
    channel_config_set_chain_to(&ctrl_cfg, data_chan_dma);
    
    // dma_channel_configure(
    //     ctrl_chan_dma,
    //     &ctrl_cfg,
    //     &dma_hw->ch[data_chan_dma].read_addr,
    //     &ctrl_dma_read_address,
    //     1,
    //     false
    // );
    dma_channel_configure(
        ctrl_chan_dma,
        &ctrl_cfg,
        &dma_hw->ch[data_chan_dma].read_addr,
        &ctrl_dma_read_address,
        1,
        false
    );
    
    // irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    // irq_set_enabled(DMA_IRQ_0, true);
}

void dac_start_streaming() {
    dma_channel_start(ctrl_chan_dma);
    // dma_channel_start(data_chan_dma);
}

void dac_single_write(uint16_t val) {
    uint16_t data[1] = { val };
    gpio_put(cs_pin, 0);
    spi_write16_blocking(port, data, 1);
    gpio_put(cs_pin, 1);
}