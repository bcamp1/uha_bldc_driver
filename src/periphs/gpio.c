/*
 * gpio.c
 *
 * Created: 8/9/2024 2:41:16 PM
 *  Author: brans
 */ 

#include "gpio.h"

/*
void gpio_init() {
	
}
*/

static void get_pin_port(uint8_t pin_id, uint8_t* pin, uint8_t* port) {
	if (pin_id > 31) {
		*pin = pin_id - 32;
		*port = GPIO_PORT_B;
	} else {
		*pin = pin_id;
		*port = GPIO_PORT_A;
	}
}

void gpio_init_pin(uint8_t pin, bool direction, uint8_t alternate_function) {
	uint8_t port;
	get_pin_port(pin, &pin, &port);
	
	uint32_t pin_mask = (1) << pin;
	
	// Set pin direction
	if (direction) { 
		// Output
		PORT->Group[port].DIRSET.reg = pin_mask;
	} else {
		// Input
		PORT->Group[port].DIRCLR.reg = pin_mask;
		PORT->Group[port].PINCFG[pin].bit.INEN = 1;
		//PORT->Group[port].PINCFG[pin].bit.PULLEN = 1;
	}
	
	// Set drivestrength
	PORT->Group[port].PINCFG[pin].bit.DRVSTR = 1;
	//PORT->Group[port].PINCFG[pin].bit.PULLEN = 1;
	
	// Set alternate function
	if (alternate_function == GPIO_ALTERNATE_NONE) {
		PORT->Group[port].PINCFG[pin].bit.PMUXEN = 0;
	} else {
		PORT->Group[port].PINCFG[pin].bit.PMUXEN = 1;
		uint8_t pmux_group = pin >> 1;
		uint8_t is_odd = pin % 2;
		uint8_t clear_mask = (0b1111) << (4*is_odd);
		uint8_t write_mask = (alternate_function) << (4*is_odd);
		PORT->Group[port].PMUX[pmux_group].reg &= ~clear_mask;
		PORT->Group[port].PMUX[pmux_group].reg |= write_mask;
	}
}

void gpio_set_pin(uint8_t pin) {
	uint8_t port;
	get_pin_port(pin, &pin, &port);
	uint32_t pin_mask = (1) << pin;
	
	PORT->Group[port].OUTSET.reg = pin_mask;
}

void gpio_clear_pin(uint8_t pin) {
	uint8_t port;
	get_pin_port(pin, &pin, &port);
	uint32_t pin_mask = (1) << pin;
	
	PORT->Group[port].OUTCLR.reg = pin_mask;
}

void gpio_toggle_pin(uint8_t pin) {
	uint8_t port;
	get_pin_port(pin, &pin, &port);
	uint32_t pin_mask = (1) << pin;
	
	PORT->Group[port].OUTTGL.reg = pin_mask;
}

bool gpio_get_pin(uint8_t pin) {
	uint8_t port;
	get_pin_port(pin, &pin, &port);
	uint32_t pin_mask = (1) << (pin);
	uint32_t gpio_read = PORT->Group[port].IN.reg;
	uint32_t pin_state = (gpio_read & pin_mask) >> (pin);
	return (bool) pin_state;
}

void gpio_set_drivestrength(uint8_t pin, uint8_t drivestrength) {
	uint8_t port;
	get_pin_port(pin, &pin, &port);
	PORT->Group[port].PINCFG[pin].bit.DRVSTR = drivestrength;
}

