#include "spi_slave.h"
#include "../board.h"
#include "../periphs/gpio.h"
#include "../periphs/sercom.h"
#include "samd51j20a.h"
#include <sam.h>
//#include "../periphs/clocks.h"

static volatile uint16_t torque_command = 0;

void spi_slave_init() {
	/* Enable clocks */
	wntr_sercom_init_clock((Sercom*)SPI_SLAVE, GCLK_PCHCTRL_GEN_GCLK1);

	/* Reset and configure */
	SPI_SLAVE->CTRLA.bit.ENABLE = 0;
	while (SPI_SLAVE->SYNCBUSY.bit.ENABLE) {};

	SPI_SLAVE->CTRLA.bit.SWRST = 1;
	while (SPI_SLAVE->SYNCBUSY.bit.SWRST || SPI_SLAVE->CTRLA.bit.SWRST) {};

	/* Setup SPI controller and mode (0x3 = controller, 0x2 = slave) */
	SPI_SLAVE->CTRLA.reg = (SERCOM_SPI_CTRLA_MODE(0x2) | SERCOM_SPI_CTRLA_DOPO(SPI_SLAVE_DOPO) | SERCOM_SPI_CTRLA_DIPO(SPI_SLAVE_DIPO));

	if (SPI_SLAVE_PHASE) {
		SPI_SLAVE->CTRLA.bit.CPHA = 1;
	}
	if (SPI_SLAVE_POLARITY) {
		SPI_SLAVE->CTRLA.bit.CPOL = 1;
	}

	/* Set baud to max (GCLK / 2) 6 MHz */
	//SPI_SLAVE->BAUD.reg = SERCOM_SPI_BAUD_BAUD(2000);
	//SPI_SLAVE->BAUD.reg = (uint8_t) 1000; //SERCOM_SPI_BAUD_BAUD(1000);

    // Enable receive complete (RXC) interrupt
    //SPI_SLAVE->INTENSET.bit.RXC = 1;

    // NVIC interrupt enable
	//NVIC_EnableIRQ(SERCOM4_2_IRQn);

	/* Configure pins for the correct function. */
	gpio_init_pin(PIN_SLAVE_SDI, GPIO_DIR_IN, GPIO_ALTERNATE_D_SERCOM_ALT);
	gpio_init_pin(PIN_SLAVE_SDO, GPIO_DIR_OUT, GPIO_ALTERNATE_D_SERCOM_ALT);
	gpio_init_pin(PIN_SLAVE_SCK, GPIO_DIR_IN, GPIO_ALTERNATE_D_SERCOM_ALT);
	gpio_init_pin(PIN_SLAVE_CS, GPIO_DIR_IN, GPIO_ALTERNATE_D_SERCOM_ALT);

    // Enable interrupts
    SPI_SLAVE->INTENSET.bit.RXC = 1;
    SPI_SLAVE->INTENSET.bit.TXC = 1;

    NVIC_EnableIRQ(SERCOM4_0_IRQn); 
    NVIC_EnableIRQ(SERCOM4_1_IRQn); 
    NVIC_EnableIRQ(SERCOM4_2_IRQn); 
    NVIC_EnableIRQ(SERCOM4_3_IRQn); 

	/* Finally, enable it! */
	SPI_SLAVE->CTRLB.bit.RXEN = 1;
	SPI_SLAVE->CTRLA.bit.ENABLE = 1;
	while (SPI_SLAVE->SYNCBUSY.bit.ENABLE) {};
}

uint16_t get_torque_command() {
    return torque_command;
}

void SERCOM4_0_Handler() {
    gpio_set_pin(PIN_DEBUG1);
}

void SERCOM4_1_Handler() {
    gpio_set_pin(PIN_DEBUG1);
}

void SERCOM4_2_Handler() {
    gpio_set_pin(PIN_DEBUG1);
}

void SERCOM4_3_Handler() {
    gpio_set_pin(PIN_DEBUG1);
}

