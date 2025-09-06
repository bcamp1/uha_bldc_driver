#include "spi_slave.h"
#include "../board.h"
#include "../periphs/gpio.h"
#include "../periphs/sercom.h"
#include "../periphs/uart.h"
#include "samd51j20a.h"
#include <sam.h>
//#include "../periphs/clocks.h"

static volatile uint16_t torque_command = 0;
static volatile uint16_t torque_command_dirty = 0;
static volatile uint16_t byte_index = 0;

void spi_slave_init() {
	/* Enable clocks */
	wntr_sercom_init_clock((Sercom*)SPI_SLAVE, GCLK_PCHCTRL_GEN_GCLK1);

	/* Reset and configure */
	SPI_SLAVE->CTRLA.bit.ENABLE = 0;
	while (SPI_SLAVE->SYNCBUSY.bit.ENABLE) {};

	SPI_SLAVE->CTRLA.bit.SWRST = 1;
	while (SPI_SLAVE->SYNCBUSY.bit.SWRST || SPI_SLAVE->CTRLA.bit.SWRST) {};

	/* Setup SPI controller and mode (0x3 = controller, 0x2 = slave) */
    SPI_SLAVE->CTRLA.bit.DIPO = SPI_SLAVE_DIPO;
    SPI_SLAVE->CTRLA.bit.DOPO = SPI_SLAVE_DOPO;
    SPI_SLAVE->CTRLA.bit.MODE = 0x2;

	if (SPI_SLAVE_PHASE) {
		SPI_SLAVE->CTRLA.bit.CPHA = 1;
	}

	if (SPI_SLAVE_POLARITY) {
		SPI_SLAVE->CTRLA.bit.CPOL = 1;
	}

	gpio_init_pin(PIN_SLAVE_SDI, GPIO_DIR_IN, GPIO_ALTERNATE_D_SERCOM_ALT);
	gpio_init_pin(PIN_SLAVE_SDO, GPIO_DIR_OUT, GPIO_ALTERNATE_C_SERCOM);
	gpio_init_pin(PIN_SLAVE_SCK, GPIO_DIR_IN, GPIO_ALTERNATE_C_SERCOM);
	gpio_init_pin(PIN_SLAVE_CS, GPIO_DIR_IN, GPIO_ALTERNATE_D_SERCOM_ALT);


    SPI_SLAVE->DATA.reg = 0x55;

    SPI_SLAVE->CTRLB.bit.RXEN = 1;
	while (SPI_SLAVE->SYNCBUSY.bit.CTRLB) {};

    // Enable interrupts
    SPI_SLAVE->INTENSET.bit.RXC = 1;
    SPI_SLAVE->INTENSET.bit.DRE = 1;
    SPI_SLAVE->INTENSET.bit.SSL = 1;

	/* Finally, enable it! */
	SPI_SLAVE->CTRLA.bit.ENABLE = 1;
	while (SPI_SLAVE->SYNCBUSY.bit.ENABLE) {};

    NVIC_EnableIRQ(SERCOM4_0_IRQn); 
    NVIC_EnableIRQ(SERCOM4_1_IRQn); 
    NVIC_EnableIRQ(SERCOM4_2_IRQn); 
    NVIC_EnableIRQ(SERCOM4_3_IRQn); 
}

uint16_t spi_slave_get_torque_command() {
    return torque_command;
}

static void spi_slave_isr() {
    if (SPI_SLAVE->INTFLAG.bit.DRE) {
        SPI_SLAVE->DATA.reg = 0x55;
    } else if (SPI_SLAVE->INTFLAG.bit.RXC) {
        uint16_t data = SPI_SLAVE->DATA.reg;
        byte_index++;
        if (byte_index == 1) {
            torque_command_dirty = (data << 8);
        } else if (byte_index == 2) {
            torque_command_dirty |= data;
            torque_command = torque_command_dirty;
            byte_index = 0;
        }
    } 
}

void SERCOM4_0_Handler() {
    spi_slave_isr();
}

void SERCOM4_1_Handler() {
    spi_slave_isr();
}

void SERCOM4_2_Handler() {
    spi_slave_isr();
}

void SERCOM4_3_Handler() {
    spi_slave_isr();
}

