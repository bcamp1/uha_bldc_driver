/*
 * rs485.c
 *
 * Created: 2026-04-15
 *  Author: brans
 */

#include "rs485.h"
#include "gpio.h"
#include "samd51j20a.h"
#include "sercom.h"
#include "../board.h"
#include <stdint.h>
#include "../periphs/delay.h"

#define RS485_TX_PIN	PIN_PA04
#define RS485_RX_PIN	PIN_PA05
#define RS485_TX_PAD	(0)
#define RS485_RX_PAD	(1)
#define RS485_SERCOM	SERCOM0
#define RS485_SERCOM_REF_HZ	12000000.0f  // GCLK4

#define RS485_TXEN_PIN PIN_PA07

#define RS485_RX_BUF_SIZE 128  // Must be a power of two
#define RS485_RX_BUF_MASK (RS485_RX_BUF_SIZE - 1)

static volatile uint8_t rx_buf[RS485_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;  // Written by ISR
static volatile uint16_t rx_tail = 0;  // Read by consumer

static void (*rx_ready_cb)(void) = 0;

void rs485_init(void (*rx_ready)(void)) {
	const float baud_hz = 500000.0f;
	rx_ready_cb = rx_ready;

	// Init ports (PA04/PA05 use peripheral function D for SERCOM0)
	gpio_init_pin(RS485_TX_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_D_SERCOM_ALT);
	gpio_init_pin(RS485_RX_PIN, GPIO_DIR_IN,  GPIO_ALTERNATE_D_SERCOM_ALT);
    gpio_init_pin(RS485_TXEN_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
    gpio_clear_pin(RS485_TXEN_PIN);  // Start in receive mode

	// Init clock
	wntr_sercom_init_clock(RS485_SERCOM, GCLK_PCHCTRL_GEN_GCLK4);

	RS485_SERCOM->USART.CTRLA.bit.MODE = 1; // Enable internal clock
	RS485_SERCOM->USART.CTRLA.bit.TXPO = 0x0; // TX on PAD0
	RS485_SERCOM->USART.CTRLA.bit.RXPO = 0x1; // RX on PAD1
	RS485_SERCOM->USART.CTRLA.bit.DORD = 1; // LSB first

	// Asynchronous arithmetic baud generation, 16x oversampling:
	//   BAUD = 65536 * (1 - 16 * baud_hz / fref)
	RS485_SERCOM->USART.BAUD.reg = (uint16_t)(65536.0f * (1.0f - 16.0f * baud_hz / RS485_SERCOM_REF_HZ));

	RS485_SERCOM->USART.CTRLB.bit.CHSIZE = 0;
	RS485_SERCOM->USART.CTRLB.bit.TXEN = 1;
	RS485_SERCOM->USART.CTRLB.bit.RXEN = 1;
	while (RS485_SERCOM->USART.SYNCBUSY.bit.CTRLB) {}

	// Enable RX complete interrupt (SERCOM0_2 = RXC)
	RS485_SERCOM->USART.INTENSET.bit.RXC = 1;
	NVIC_SetPriority(SERCOM0_2_IRQn, PRIO_RS485_RX);
	NVIC_EnableIRQ(SERCOM0_2_IRQn);

	RS485_SERCOM->USART.CTRLA.bit.ENABLE = 1;
	while (RS485_SERCOM->USART.SYNCBUSY.bit.ENABLE) {}
}

void rs485_begin_transaction(void) {
	gpio_set_pin(RS485_TXEN_PIN);
	delay(1); // Settle before transmitting (TXEN spec is 55ns)
}

void rs485_end_transaction(void) {
	// Wait for the last byte (including stop bit) to fully shift out
	while (!RS485_SERCOM->USART.INTFLAG.bit.TXC) {}
	delay(1); // Settle before releasing bus
	gpio_clear_pin(RS485_TXEN_PIN);
}

// Low-level byte write. Caller must be inside a begin/end transaction.
void rs485_put(uint8_t byte) {
	RS485_SERCOM->USART.DATA.reg = byte;
	while (!RS485_SERCOM->USART.INTFLAG.bit.DRE) {}
}

void rs485_send_bytes(const uint8_t* data, uint16_t len) {
	rs485_begin_transaction();
	for (uint16_t i = 0; i < len; i++) {
		rs485_put(data[i]);
	}
	rs485_end_transaction();
}

int rs485_available(void) {
	return (int)((rx_head - rx_tail) & RS485_RX_BUF_MASK);
}

int rs485_get(void) {
	// Mask IRQs around tail update: the ISR can also mutate rx_tail on overflow,
	// so a preemption between the read and the write would let the consumer
	// clobber the ISR's tail bumps and lose extra bytes.
	__disable_irq();
	if (rx_head == rx_tail) {
		__enable_irq();
		return -1;
	}
	uint8_t ch = rx_buf[rx_tail];
	rx_tail = (rx_tail + 1) & RS485_RX_BUF_MASK;
	__enable_irq();
	return (int)ch;
}

void rs485_rx_flush(void) {
	rx_tail = rx_head;
}

// RXC interrupt: push received byte into ring buffer.
// On overflow, drop the oldest byte to make room for the newest.
void SERCOM0_2_Handler(void) {
	if (RS485_SERCOM->USART.INTFLAG.bit.RXC) {
		uint8_t data = RS485_SERCOM->USART.DATA.reg & 0xFF;
		uint16_t next = (rx_head + 1) & RS485_RX_BUF_MASK;
		if (next == rx_tail) {
			rx_tail = (rx_tail + 1) & RS485_RX_BUF_MASK;
		}
		rx_buf[rx_head] = data;
		rx_head = next;

		if (rx_ready_cb) {
			rx_ready_cb();
		}
	}
}
