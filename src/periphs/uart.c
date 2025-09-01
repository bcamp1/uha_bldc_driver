/*
 * uart.c
 *
 * Created: 7/29/2024 3:41:25 PM
 *  Author: brans
 */ 

#include "uart.h"
#include "gpio.h"
#include "samd51j20a.h"
#include "sercom.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../periphs/delay.h"

#define UART_TX_PIN		PIN_PA04
#define UART_RX_PIN		PIN_PA05
#define UART_TX_PAD		(0)
#define UART_RX_PAD		(1)
#define UART_SERCOM		SERCOM0
#define UART_BAUD_9600	(0xD8A0)

static char read_character = 0;

void uart_init(void) {
	// Init ports
	gpio_init_pin(UART_TX_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_D_SERCOM_ALT);
	gpio_init_pin(UART_RX_PIN, GPIO_DIR_IN, GPIO_ALTERNATE_D_SERCOM_ALT);
	
	// Init clock
	wntr_sercom_init_clock(UART_SERCOM, GCLK_PCHCTRL_GEN_GCLK4);
	
	// Enable clock TODO
	//PM->APBCMASK.bit.SERCOM5_ = 1; // Enable clock source
	//GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM5_CORE | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
	//while (GCLK->STATUS.bit.SYNCBUSY) {} // Wait for busy
	
	UART_SERCOM->USART.CTRLA.bit.MODE = 1; // Enable internal clock
	UART_SERCOM->USART.CTRLA.bit.TXPO = 0x0; // Pad 0
	UART_SERCOM->USART.CTRLA.bit.RXPO = 0x1; // Pad 1
	UART_SERCOM->USART.CTRLA.bit.DORD = 1; // Order

	UART_SERCOM->USART.BAUD.reg = UART_BAUD_9600;
	
	// Enable recieve interrupts
	//UART_SERCOM->USART.INTENSET.bit.RXC = 1;
	//NVIC_EnableIRQ(SERCOM0_2_IRQn);
	
	UART_SERCOM->USART.CTRLB.bit.CHSIZE = 0;
	UART_SERCOM->USART.CTRLB.bit.TXEN = 1;
	UART_SERCOM->USART.CTRLB.bit.RXEN = 1;
	while (UART_SERCOM->USART.SYNCBUSY.bit.CTRLB) {} // Wait for busy
	
	UART_SERCOM->USART.CTRLA.bit.ENABLE = 1;
	while (UART_SERCOM->USART.SYNCBUSY.bit.ENABLE) {} // Wait for busy
}

void uart_put(char ch) {
	delay(0x1F);
	//for (int i = 0; i < 0x8FF; i++);
	UART_SERCOM->USART.DATA.reg = ch; // Send data
	while (!UART_SERCOM->USART.INTFLAG.bit.DRE) {} // Wait for empty
	if (ch == '\n') {
		uart_put('\r');
	}
}

char uart_get() {
	if (UART_SERCOM->USART.INTFLAG.bit.RXC) {
		return (UART_SERCOM->USART.DATA.reg & 0xff);
	}
	return 0;
}

void uart_print(char* str) {
	char* ch = str;
	while (*ch != '\0') {
		uart_put(*ch);
		ch++;
	}
}

void uart_println(char* str) {
	uart_print(str);
	uart_put('\n');
}


#define MAX_FLOAT_STR_LEN 32  // Define max length for the float string (e.g., 32 chars)
#define FLOAT_PRECISION (4)

void uart_print_float(float num) {
	if (num < 0) {
		uart_put('-');
		num = -num;
	} else {
        uart_put('+');
    }
	static char buffer[MAX_FLOAT_STR_LEN];  // Static to keep the buffer valid after returning
	int integer_part = (int)num;  // Get the integer part
	float fractional_part = num - integer_part;  // Get the fractional part
	
	// Convert integer part to string
	int i = 0;
	if (integer_part == 0) {
		buffer[i++] = '0';  // Handle zero integer part
		} else {
		char int_buffer[10];  // Temporary buffer for integer part
		int idx = 0;
		while (integer_part > 0) {
			int_buffer[idx++] = (integer_part % 10) + '0';
			integer_part /= 10;
		}
		
		// Reverse integer part into buffer
		for (int j = idx - 1; j >= 0; j--) {
			buffer[i++] = int_buffer[j];
		}
	}

	// Add decimal point
	buffer[i++] = '.';

	// Convert fractional part to string (limited to 6 decimal places)
	int frac_digits = FLOAT_PRECISION;
	while (frac_digits-- > 0) {
		fractional_part *= 10;
		int digit = (int)fractional_part;
		buffer[i++] = digit + '0';
		fractional_part -= digit;
	}

	buffer[i] = '\0';  // Null-terminate the string
	uart_print(buffer);
}

void uart_println_float(float num) {
	uart_print_float(num);
	uart_put('\n');
}

#define MAX_INT_STR_LEN 50
void uart_print_int_base(int num, int base) {
	static char buffer[MAX_INT_STR_LEN];
	//buffer[0] = 0;
	itoa(num, buffer, base);
	uart_print(buffer);
}

void uart_println_int_base(int num, int base) {
	uart_print_int_base(num, base);
	uart_put('\n');
}

void uart_print_int(int num) {
	uart_print_int_base(num, 10);
}

void uart_println_int(int num) {
	uart_println_int_base(num, 10);
}

void SERCOM0_2_Handler(void) {
	char data = UART_SERCOM->USART.DATA.reg & 0xff;
	read_character = data;
}

void uart_send_float(float num) {
	uint32_t raw = 0;
	memcpy(&raw, &num, 4);
	char byte0 = (raw >> 0)  & 0xFF;  // Least significant byte
    char byte1 = (raw >> 8)  & 0xFF;
    char byte2 = (raw >> 16) & 0xFF;
    char byte3 = (raw >> 24) & 0xFF;  // Most significant byte
	uart_put(byte3);
	uart_put(byte2);
	uart_put(byte1);
	uart_put(byte0);
}

void uart_send_float_arr(float* data, int len) {
    for (int i = 0; i < len; i++) {
        uart_send_float(data[i]);
        delay(0x3FF); 
    }
    uart_send_float(INFINITY);
    delay(0x3FF); 
}

void uart_print_float_arr(float* data, int len) {
   // uart_put('[');
    for (int i = 0; i < len; i++) {
        uart_print_float(data[i]);
        if (i < len-1) {
            uart_put(',');
            //uart_put(' ');
        }
    }
   // uart_put(']');
}

void uart_println_float_arr(float* data, int len) {
    uart_print_float_arr(data, len);
    uart_put('\n');
}
