/**
 * Helper functions for TLC5615: https://www.ti.com/lit/ds/symlink/tlc5615.pdf
 */

void dac_init(spi_inst_t *spi_port, uint8_t mosi, uint8_t clk, uint8_t cs);

void dac_setup_streaming(uint16_t *buffer, uint16_t buffer_length);

void dac_start_streaming();

void dac_single_write(uint16_t val);