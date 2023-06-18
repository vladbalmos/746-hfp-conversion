#include <stdio.h>
#include "pico/stdio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"

#define SPI_BAUD_RATE_HZ 10 * 1000 * 1000

static uint port;
static uint8_t mosi_pin;
static uint8_t clk_pin;
static uint8_t cs_pin;

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
    
    gpio_put(cs_pin, 1);
}

void dac_single_write(uint16_t val) {
    uint16_t data[1] = { val };
    gpio_put(cs_pin, 0);
    spi_write16_blocking(port, data, 1);
    gpio_put(cs_pin, 1);
}