#include "aps6404.h"
#include "hardware/dma.h"
#include "aps6404.pio.h"

void aps6404_init(struct aps6404_config* config) {
    assert(config->pin_miso == config->pin_mosi + 1);

	gpio_init(config->pin_miso);
	gpio_disable_pulls(config->pin_miso);

	uint offset = pio_add_program(config->pio, &sram_reset_program);
	aps6404_reset_program_init(config->pio, config->pio_sm, offset, config->pin_csn, config->pin_mosi);

	sleep_us(200);
	pio_sm_put_blocking(config->pio, config->pio_sm, 0x00000007u);
	pio_sm_put_blocking(config->pio, config->pio_sm, 0x66000000u);
	pio_sm_put_blocking(config->pio, config->pio_sm, 0x00000007u);
	pio_sm_put_blocking(config->pio, config->pio_sm, 0x99000000u);
	sleep_us(300);
	pio_sm_set_enabled(config->pio, config->pio_sm, false);

	pio_remove_program(config->pio, &sram_reset_program, offset);

	offset = pio_add_program(config->pio, &sram_program);
	//printf("SRAM program loaded to PIO at offset %d\n", offset);
	aps6404_program_init(config->pio, config->pio_sm, offset, config->pin_csn, config->pin_mosi);

}

void aps6404_write(struct aps6404_config* config, uint32_t addr, uint32_t* data, uint32_t len_in_words) {
	dma_channel_wait_for_finish_blocking(config->dma_channel);

	dma_channel_config c = dma_channel_get_default_config(config->dma_channel);
	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_dreq(&c, pio_get_dreq(config->pio, config->pio_sm, true));
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
	
	dma_channel_configure(
		config->dma_channel, &c,
		&config->pio->txf[config->pio_sm],
		data,
		len_in_words,
		false
	);

	pio_sm_put_blocking(config->pio, config->pio_sm, 0x80000000u | ((len_in_words * 8) - 1 + 6));
	pio_sm_put_blocking(config->pio, config->pio_sm, 0x38000000u | addr);

	dma_channel_start(config->dma_channel);
}

void aps6404_read(struct aps6404_config* config, uint32_t addr, uint32_t* read_buf, uint32_t len_in_words) {
	dma_channel_wait_for_finish_blocking(config->dma_channel);

	dma_channel_config c = dma_channel_get_default_config(config->dma_channel);
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_dreq(&c, pio_get_dreq(config->pio, config->pio_sm, false));
	channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
	
	dma_channel_configure(
		config->dma_channel, &c,
		read_buf,
		&config->pio->rxf[config->pio_sm],
		len_in_words,
		true
	);

	pio_sm_put_blocking(config->pio, config->pio_sm, (len_in_words * 32) - 1);
	pio_sm_put_blocking(config->pio, config->pio_sm, 0x0b000000u | addr);

}

void aps6404_read_blocking(struct aps6404_config* config, uint32_t addr, uint32_t* read_buf, uint32_t len_in_words) {
	aps6404_read(config, addr, read_buf, len_in_words);

	dma_channel_wait_for_finish_blocking(config->dma_channel);
}
