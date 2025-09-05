#include "spi.h"
#include "gpio.h"
#include "uart.h"
#include "spi_async.h"
#include "../periphs/delay.h"
#include "../board.h"
/* 12 MHz clock for SPI */
#define SERCOM_SPI_GCLK GCLK_PCHCTRL_GEN_GCLK4;

static const uint8_t *tx_buf_ptr = NULL;
static uint8_t *rx_buf_ptr = NULL;
static uint16_t transfer_len = 0;
static uint16_t tx_index = 0;
static uint16_t rx_index = 0;
static spi_callback_t user_callback;
static volatile bool spi_busy = false;
static const SPIConfig* current_spi_conf = NULL;

// Timing configs
#define BAUD_CYCLES         (30)
#define BETWEEN_BYTE_CYCLES (0xa)

void spi_async_init(const SPIConfig* inst) {
	/* Enable clocks */
	wntr_sercom_init_clock((Sercom*)inst->sercom, GCLK_PCHCTRL_GEN_GCLK1);

	/* Reset and configure */
	inst->sercom->CTRLA.bit.ENABLE = 0;
	while (inst->sercom->SYNCBUSY.bit.ENABLE) {};

	inst->sercom->CTRLA.bit.SWRST = 1;
	while (inst->sercom->SYNCBUSY.bit.SWRST || inst->sercom->CTRLA.bit.SWRST) {};

	/* Setup SPI controller and mode (0x3 = controller) */
	inst->sercom->CTRLA.reg = (SERCOM_SPI_CTRLA_MODE(0x3) | SERCOM_SPI_CTRLA_DOPO(inst->dopo) | SERCOM_SPI_CTRLA_DIPO(inst->dipo));

	if (inst->phase) {
		inst->sercom->CTRLA.bit.CPHA = 1;
	}
	if (inst->polarity) {
		inst->sercom->CTRLA.bit.CPOL = 1;
	}

	/* Set baud to max (GCLK / 2) 6 MHz */
	//inst->sercom->BAUD.reg = SERCOM_SPI_BAUD_BAUD(2000);
	inst->sercom->BAUD.reg = (uint8_t) BAUD_CYCLES; //SERCOM_SPI_BAUD_BAUD(1000);

    // Enable interrupt(s)
    //inst->sercom->INTENSET.bit.DRE = 1;
    //inst->sercom->INTENSET.bit.RXC = 1;

    // NVIC interrupt enable
    NVIC_SetPriority(SERCOM4_0_IRQn, 2);  // 0 is highest priority
    NVIC_SetPriority(SERCOM4_2_IRQn, 2);  // 0 is highest priority
	NVIC_EnableIRQ(SERCOM4_0_IRQn);
	NVIC_EnableIRQ(SERCOM4_2_IRQn);


	/* Configure pins for the correct function. */
	gpio_init_pin(inst->mosi, GPIO_DIR_OUT, inst->mosi_alt);
	gpio_init_pin(inst->miso, GPIO_DIR_IN, inst->miso_alt);
	gpio_init_pin(inst->sck, GPIO_DIR_OUT, inst->sck_alt);
	gpio_init_pin(inst->cs, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	
	gpio_set_pin(inst->cs);

	/* Finally, enable it! */
	inst->sercom->CTRLB.bit.RXEN = 1;
	inst->sercom->CTRLA.bit.ENABLE = 1;
	while (inst->sercom->SYNCBUSY.bit.ENABLE) {};
}

void spi_async_start_transfer(const SPIConfig* inst, const uint8_t *tx_buf, uint8_t *rx_buf, uint16_t length, spi_callback_t callback) {
    if (spi_busy || length == 0) return;
    //gpio_set_pin(DEBUG_PIN);

	// Bring nCS low
	gpio_clear_pin(inst->cs);

    current_spi_conf = inst;

    tx_buf_ptr = tx_buf;
    rx_buf_ptr = rx_buf;
    transfer_len = length;
    tx_index = 0;
    rx_index = 0;
    user_callback = callback;
    spi_busy = true;

    SERCOM4->SPI.INTENCLR.bit.RXC = 1;
    //gpio_clear_pin(DEBUG_PIN);
    SERCOM4->SPI.INTENSET.bit.DRE = 1;
}
    
void spi_async_change_mode(const SPIConfig* inst) {
    // Disable SPI
    inst->sercom->CTRLA.bit.ENABLE = 0;
    while (inst->sercom->SYNCBUSY.bit.ENABLE);

    // Modify CPOL and CPHA
    inst->sercom->CTRLA.bit.CPOL = inst->polarity; // or 0
    inst->sercom->CTRLA.bit.CPHA = inst->phase; // or 0

    // Enable SPI again
    inst->sercom->CTRLA.bit.ENABLE = 1;
    while (inst->sercom->SYNCBUSY.bit.ENABLE);
}

bool spi_async_is_busy() {
    return spi_busy;
}

static void spi_async_isr() {
    //gpio_clear_pin(DEBUG_PIN);
    //uart_put('.');
    if (SERCOM4->SPI.INTFLAG.bit.RXC) {
        // Byte recieved
        uint8_t data = SERCOM4->SPI.DATA.reg;
        if (rx_buf_ptr) rx_buf_ptr[rx_index] = data;

        rx_index++;
        if (rx_index < transfer_len) {
            SERCOM4->SPI.INTENCLR.bit.RXC = 1;
            SERCOM4->SPI.INTENSET.bit.DRE = 1;
        } else {
            // Full transfer complete
	        gpio_set_pin(current_spi_conf->cs);
            SERCOM4->SPI.INTENCLR.bit.DRE = 1;
            SERCOM4->SPI.INTENCLR.bit.RXC = 1;
            spi_busy = false;
            if (user_callback) user_callback();
        }
    } else if (SERCOM4->SPI.INTFLAG.bit.DRE) {
        // Send next byte
        if (tx_index < transfer_len) {
            SERCOM4->SPI.DATA.reg = tx_buf_ptr ? tx_buf_ptr[tx_index] : 0xFF;
            delay(BETWEEN_BYTE_CYCLES);
            tx_index++;
        }
        SERCOM4->SPI.INTENCLR.bit.DRE = 1;
        SERCOM4->SPI.INTENSET.bit.RXC = 1;
    }
}

void SERCOM4_0_Handler(void) {
    //uart_println("(0) Handler");  
    spi_async_isr();
}

void SERCOM4_2_Handler(void) {
    //uart_println("(2) Handler");  
    spi_async_isr();
}

