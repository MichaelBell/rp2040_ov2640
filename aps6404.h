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

// Start a write, this completes asynchronously, this function only blocks if another 
// transfer is already in progress
void aps6404_write(struct aps6404_config* config, uint32_t addr, uint32_t* data, uint32_t len_in_words);

// Start a read, this completes asynchronously, you can check / wait for completion
// by checking the DMA channel.
void aps6404_read(struct aps6404_config* config, uint32_t addr, uint32_t* read_buf, uint32_t len_in_words);

// Start a read and block until it completes.
void aps6404_read_blocking(struct aps6404_config* config, uint32_t addr, uint32_t* read_buf, uint32_t len_in_words);
