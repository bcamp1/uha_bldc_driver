#include "spi.h"
#include "gpio.h"
#include "uart.h"
#include "spi_async.h"
#include "../board.h"
#include "../periphs/delay.h"

// Hardcoded to SERCOM3 for motor encoder (SPI_CONF_MTR_ENCODER)
#define SPI_SERCOM SERCOM3
#define INTER_BYTE_DELAY (1)  // Delay between bytes for slave to prepare

static spi_callback_t user_callback = NULL;
static volatile bool spi_busy = false;
static volatile uint16_t result = 0;
static volatile uint16_t safe_result = 0;  // Last completed result
static volatile uint8_t tx_count = 0;      // Bytes sent
static volatile uint8_t rx_count = 0;      // Bytes received

static bool verify_spi_data(uint16_t data) {
/*
---- AMT22 Checksum Example ----
Bits: K1 K0 D14 D13 D12 D11 D10 D9 D8 D7 D6 D5 D4 D3 D2 D1 D0
Full response: 0x61AB
14-bit position: 0x21AB (8619 decimal)
Checkbit Formula
Odd: K1 = !(H5^H3^H1^L7^L5^L3^L1)
Even: K0 = !(H4^H2^H0^L6^L4^L2^L0)
From the above response 0x61AB:
Odd: 0 = !(1^0^0^1^1^1^1) = correct
Even: 1 = !(0^0^1^0^0^0^1) = correct
 */
    uint8_t high = (data >> 8) & 0xFF;
    uint8_t low = data & 0xFF;

    // Extract received checksum bits
    uint8_t k1_received = (data >> 15) & 0x01;
    uint8_t k0_received = (data >> 14) & 0x01;

    // Calculate odd parity (K1): !(H5^H3^H1^L7^L5^L3^L1)
    uint8_t odd_parity = ((high >> 5) & 0x01) ^
                         ((high >> 3) & 0x01) ^
                         ((high >> 1) & 0x01) ^
                         ((low >> 7) & 0x01) ^
                         ((low >> 5) & 0x01) ^
                         ((low >> 3) & 0x01) ^
                         ((low >> 1) & 0x01);
    uint8_t k1_calculated = !odd_parity;

    // Calculate even parity (K0): !(H4^H2^H0^L6^L4^L2^L0)
    uint8_t even_parity = ((high >> 4) & 0x01) ^
                          ((high >> 2) & 0x01) ^
                          ((high >> 0) & 0x01) ^
                          ((low >> 6) & 0x01) ^
                          ((low >> 4) & 0x01) ^
                          ((low >> 2) & 0x01) ^
                          ((low >> 0) & 0x01);
    uint8_t k0_calculated = !even_parity;

    // Return true if both checksum bits match
    return (k1_received == k1_calculated) && (k0_received == k0_calculated);
}

void spi_async_init() {
	// Uses SPI_CONF_MTR_ENCODER settings
	const SPIConfig* inst = &SPI_CONF_MTR_ENCODER;

	/* Enable clocks */
	wntr_sercom_init_clock((Sercom*)SPI_SERCOM, GCLK_PCHCTRL_GEN_GCLK1);

	/* Reset and configure */
	SPI_SERCOM->SPI.CTRLA.bit.ENABLE = 0;
	while (SPI_SERCOM->SPI.SYNCBUSY.bit.ENABLE) {};

	SPI_SERCOM->SPI.CTRLA.bit.SWRST = 1;
	while (SPI_SERCOM->SPI.SYNCBUSY.bit.SWRST || SPI_SERCOM->SPI.CTRLA.bit.SWRST) {};

	/* Setup SPI controller and mode (0x3 = controller) */
	SPI_SERCOM->SPI.CTRLA.reg = (SERCOM_SPI_CTRLA_MODE(0x3) |
	                              SERCOM_SPI_CTRLA_DOPO(inst->dopo) |
	                              SERCOM_SPI_CTRLA_DIPO(inst->dipo));

	if (inst->phase) {
		SPI_SERCOM->SPI.CTRLA.bit.CPHA = 1;
	}
	if (inst->polarity) {
		SPI_SERCOM->SPI.CTRLA.bit.CPOL = 1;
	}

	/* Set baud */
	SPI_SERCOM->SPI.BAUD.reg = (uint8_t) 600;

    // NVIC interrupt enable for SERCOM3
    NVIC_SetPriority(SERCOM3_0_IRQn, 2);  // DRE interrupt
    NVIC_SetPriority(SERCOM3_2_IRQn, 2);  // RXC interrupt
	NVIC_EnableIRQ(SERCOM3_0_IRQn);
	NVIC_EnableIRQ(SERCOM3_2_IRQn);

	/* Configure pins */
	gpio_init_pin(inst->mosi, GPIO_DIR_OUT, inst->mosi_alt);
	gpio_init_pin(inst->miso, GPIO_DIR_IN, inst->miso_alt);
	gpio_init_pin(inst->sck, GPIO_DIR_OUT, inst->sck_alt);
	gpio_init_pin(inst->cs, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);

	gpio_set_pin(inst->cs);

	/* Enable SPI */
	SPI_SERCOM->SPI.CTRLB.bit.CHSIZE = 0;  // 8-bit character size
	SPI_SERCOM->SPI.CTRLB.bit.RXEN = 1;
	while (SPI_SERCOM->SPI.SYNCBUSY.bit.CTRLB) {};

	SPI_SERCOM->SPI.CTRLA.bit.ENABLE = 1;
	while (SPI_SERCOM->SPI.SYNCBUSY.bit.ENABLE) {};
}

void spi_async_start_read(spi_callback_t callback) {
    if (spi_busy) return;

	// Bring nCS low
	gpio_clear_pin(SPI_CONF_MTR_ENCODER.cs);

    result = 0;
    tx_count = 0;
    rx_count = 0;
    user_callback = callback;
    spi_busy = true;

    // Disable both interrupts first
    SPI_SERCOM->SPI.INTENCLR.bit.DRE = 1;
    SPI_SERCOM->SPI.INTENCLR.bit.RXC = 1;

    // Enable DRE to send first byte
    SPI_SERCOM->SPI.INTENSET.bit.DRE = 1;
}

uint16_t spi_async_get_result() {
    return result;
}

uint16_t spi_async_get_safe_result() {
    return safe_result;
}

bool spi_async_is_busy() {
    return spi_busy;
}

// DRE interrupt - ready to send next byte
void SERCOM3_0_Handler(void) {
    //gpio_set_pin(PIN_DEBUG1);
    if (SPI_SERCOM->SPI.INTFLAG.bit.DRE) {
        // Always disable DRE first to prevent re-entry
        SPI_SERCOM->SPI.INTENCLR.bit.DRE = 1;

        if (tx_count < 2) {
            // Send dummy byte (0x00) to clock in data
            SPI_SERCOM->SPI.DATA.reg = 0x00;
            tx_count++;

            // Enable RXC to wait for received byte
            SPI_SERCOM->SPI.INTENSET.bit.RXC = 1;
        }
    }
    //gpio_clear_pin(PIN_DEBUG1);
}

// RXC interrupt - byte received
void SERCOM3_2_Handler(void) {
    //gpio_set_pin(PIN_DEBUG2);
    if (SPI_SERCOM->SPI.INTFLAG.bit.RXC) {
        uint8_t data = SPI_SERCOM->SPI.DATA.reg & 0xFF;
        rx_count++;

        if (rx_count == 1) {
            // First byte (MSB)
            result = (uint16_t)data << 8;

            // Delay to give slave time to prepare second byte
            //delay(INTER_BYTE_DELAY);

            // Prepare for second byte - only enable DRE if we haven't sent 2 yet
            SPI_SERCOM->SPI.INTENCLR.bit.RXC = 1;
            if (tx_count < 2) {
                SPI_SERCOM->SPI.INTENSET.bit.DRE = 1;
            } else {
                // Already sent 2 bytes, just wait for second RXC
                SPI_SERCOM->SPI.INTENSET.bit.RXC = 1;
            }
        } else if (rx_count == 2) {
            // Second byte (LSB) - transfer complete
            result |= (uint16_t)data;

            // Update safe result (atomic complete value)
            if (verify_spi_data(result)) {
                safe_result = result;
            } else {
                //uart_put('!');
            }

            // Bring nCS high
            gpio_set_pin(SPI_CONF_MTR_ENCODER.cs);

            // Disable interrupts
            SPI_SERCOM->SPI.INTENCLR.bit.DRE = 1;
            SPI_SERCOM->SPI.INTENCLR.bit.RXC = 1;

            spi_busy = false;

            // Call user callback if registered
            if (user_callback) user_callback();
        }
    }
    //gpio_clear_pin(PIN_DEBUG2);
}

