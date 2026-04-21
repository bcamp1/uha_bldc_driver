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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../periphs/delay.h"

#define RS485_TX_PIN	PIN_PA04
#define RS485_RX_PIN	PIN_PA05
#define RS485_TX_PAD	(0)
#define RS485_RX_PAD	(1)
#define RS485_SERCOM	SERCOM0
#define RS485_BAUD	(0x9000)

#define RS485_RX_BUF_SIZE 128  // Must be a power of two
#define RS485_RX_BUF_MASK (RS485_RX_BUF_SIZE - 1)

static volatile uint8_t rx_buf[RS485_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;  // Written by ISR
static volatile uint16_t rx_tail = 0;  // Read by consumer

void rs485_init(void) {
	// Init ports (PA04/PA05 use peripheral function D for SERCOM0)
	gpio_init_pin(RS485_TX_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_D_SERCOM_ALT);
	gpio_init_pin(RS485_RX_PIN, GPIO_DIR_IN,  GPIO_ALTERNATE_D_SERCOM_ALT);

	// Init clock
	wntr_sercom_init_clock(RS485_SERCOM, GCLK_PCHCTRL_GEN_GCLK4);

	RS485_SERCOM->USART.CTRLA.bit.MODE = 1; // Enable internal clock
	RS485_SERCOM->USART.CTRLA.bit.TXPO = 0x0; // TX on PAD0
	RS485_SERCOM->USART.CTRLA.bit.RXPO = 0x1; // RX on PAD1
	RS485_SERCOM->USART.CTRLA.bit.DORD = 1; // LSB first

	RS485_SERCOM->USART.BAUD.reg = RS485_BAUD;

	RS485_SERCOM->USART.CTRLB.bit.CHSIZE = 0;
	RS485_SERCOM->USART.CTRLB.bit.TXEN = 1;
	RS485_SERCOM->USART.CTRLB.bit.RXEN = 1;
	while (RS485_SERCOM->USART.SYNCBUSY.bit.CTRLB) {}

	// Enable RX complete interrupt (SERCOM0_2 = RXC)
	RS485_SERCOM->USART.INTENSET.bit.RXC = 1;
	NVIC_SetPriority(SERCOM0_2_IRQn, 3);
	NVIC_EnableIRQ(SERCOM0_2_IRQn);

	RS485_SERCOM->USART.CTRLA.bit.ENABLE = 1;
	while (RS485_SERCOM->USART.SYNCBUSY.bit.ENABLE) {}
}

void rs485_put(char ch) {
	delay(0x1F);
	RS485_SERCOM->USART.DATA.reg = ch;
	while (!RS485_SERCOM->USART.INTFLAG.bit.DRE) {}
	if (ch == '\n') {
		rs485_put('\r');
	}
}

int rs485_available(void) {
	return (int)((rx_head - rx_tail) & RS485_RX_BUF_MASK);
}

int rs485_get(void) {
	if (rx_head == rx_tail) {
		return -1;
	}
	uint8_t ch = rx_buf[rx_tail];
	rx_tail = (rx_tail + 1) & RS485_RX_BUF_MASK;
	return (int)ch;
}

void rs485_rx_flush(void) {
	rx_tail = rx_head;
}

void rs485_print(char* str) {
	char* ch = str;
	while (*ch != '\0') {
		rs485_put(*ch);
		ch++;
	}
}

void rs485_println(char* str) {
	rs485_print(str);
	rs485_put('\n');
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
	}
}

void rs485_send_float(float num) {
	uint32_t raw = 0;
	memcpy(&raw, &num, 4);
	char byte0 = (raw >> 0)  & 0xFF;
    char byte1 = (raw >> 8)  & 0xFF;
    char byte2 = (raw >> 16) & 0xFF;
    char byte3 = (raw >> 24) & 0xFF;
	rs485_put(byte3);
	rs485_put(byte2);
	rs485_put(byte1);
	rs485_put(byte0);
}

void rs485_send_float_arr(float* data, int len) {
    for (int i = 0; i < len; i++) {
        rs485_send_float(data[i]);
        delay(0x3FF);
    }
    rs485_send_float(INFINITY);
    delay(0x3FF);
}

