#pragma once

#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

struct aps6404_config {
	uint pin_csn;  // CSn, SCK
	uint pin_mosi;
	uint pin_miso;

	PIO pio;
	uint pio_sm;

	uint dma_channel;
};

void aps6404_init(struct aps6404_config* config);

void aps6404_write(struct aps6404_config* config, uint32_t addr, uint32_t* data, uint32_t len_in_words);
void aps6404_read(struct aps6404_config* config, uint32_t addr, uint32_t* read_buf, uint32_t len_in_words);
