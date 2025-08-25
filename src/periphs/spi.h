/*
    Copyright (c) 2021 Alethea Katherine Flowers.
    Published under the standard MIT License.
    Full text available at: https://opensource.org/licenses/MIT
*/

#pragma once

/* SERCOM SPI driver for SAMD51 */

#include "sam.h"
#include "sercom.h"
#include <stddef.h>

typedef struct {
	SercomSpi* sercom;
	uint8_t dopo;
	uint8_t dipo;
	uint8_t phase;
	uint8_t polarity;

	uint8_t mosi;
	uint8_t mosi_alt;
	
	uint8_t miso;
	uint8_t miso_alt;

	uint8_t sck;
	uint8_t sck_alt;
	
	uint8_t cs;
} SPIConfig;

extern const SPIConfig SPI_CONF_GATE_DRIVER;
extern const SPIConfig SPI_CONF_MTR_ENCODER;

void spi_init(const SPIConfig* inst);
uint16_t spi_write_read16(const SPIConfig* inst, uint16_t data);
void spi_change_mode(const SPIConfig* inst);

