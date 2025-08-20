/*
 * uha_motor_driver.c
 *
 * Created: 3/9/2025 4:24:01 PM
 *  Author: brans
 */ 

#include "uha_motor_driver.h"
#include "../periphs/gpio.h"
#include "../periphs/uart.h"
#include "../periphs/spi.h"
#include "../foc/foc.h"
#include "delay.h"
#include <stdbool.h>

const UHAMotorDriverConfig UHA_MTR_DRVR_CONF = {
	.pwm = &PWM_CONF,
	.spi = &SPI_CONF_MTR_DRVR,

	// Current Sense pins
	.soa = PIN_PA02,
	.sob = PIN_PB02,
	.soc = PIN_PB03,
		
	// General Pins
	.n_fault = PIN_PA20,
	.en = PIN_PB17,
	.cal = PIN_PB16,
	.inl = PIN_PA24,
};

void uha_motor_driver_init(const UHAMotorDriverConfig* config) {
	// Intialize PWM
	pwm_timer_init(config->pwm);
	
	// Initialize SPI
	spi_init(config->spi);
	
	// Initialize digital GPIO (CAL, EN, INL, nFault)
	gpio_init_pin(config->cal, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(config->en, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(config->inl, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(config->n_fault, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);
	
	
	// Enable Motor driver
	gpio_set_pin(config->en);
	
	// Set to 3x PWM mode
	uha_motor_driver_set_3x(config);
	
	// Set to 3x PWM mode
	//uha_motor_driver_set_3x(config);
}


void uha_motor_driver_write_reg(const UHAMotorDriverConfig* config, uint8_t address, uint16_t data) {
	uint16_t command = 0;
	command |= (address & 0xF) << 11;
	command |= data & 0x7FF;
	spi_write_read16(config->spi, command);
}

uint16_t uha_motor_driver_read_reg(const UHAMotorDriverConfig* config, uint8_t address) {
	uint16_t command = 0x8000;
	command |= (address & 0xF) << 11;
	return spi_write_read16(config->spi, command);
}

void uha_motor_driver_set_3x(const UHAMotorDriverConfig* config) {
	uint16_t data = 0b100000;
	// Delay to wait for enable signal to turn on chip
	delay(0xFFFF);
	uha_motor_driver_write_reg(config, DRV_REG_DRIVER_CONTROL, data);
}

void uha_motor_driver_set_pwm(const UHAMotorDriverConfig* config, uint8_t a, uint8_t b, uint8_t c) {
	gpio_set_pin(config->inl);
	pwm_set_duties_int(config->pwm, a, b, c);
}

void uha_motor_driver_set_high_z(const UHAMotorDriverConfig* config) {
	gpio_clear_pin(config->inl);
	pwm_set_duties_int(config->pwm, 0, 0, 0);
}

void uha_motor_driver_goto_theta(const UHAMotorDriverConfig* config, float theta) {
	float a = 0;
	float b = 0;
	float c = 0;
	float d = 0.5;
	float q = 0;

	foc_get_duties(theta, d, q, &a, &b, &c);
	
	float scale_factor = 0.3f;
	a *= scale_factor;
	b *= scale_factor;
	c *= scale_factor;
	
	// Convert duties to integers
	uint8_t a_int = (int) (255.0f * a);
	uint8_t b_int = (int) (255.0f * b);
	uint8_t c_int = (int) (255.0f * c);
	
	//uart_print_float(a);
	//uart_print("\t");
	//uart_print_float(b);
	//uart_print("\t");
	//uart_println_float(c);
	
	uha_motor_driver_set_pwm(config, a_int, b_int, c_int);
}

void uha_motor_driver_enable(const UHAMotorDriverConfig* config) {
	gpio_set_pin(config->en);
	uha_motor_driver_set_3x(config);
}

void uha_motor_driver_disable(const UHAMotorDriverConfig* config) {
	gpio_clear_pin(config->en);
    delay(0xFFF);
}

void uha_motor_driver_toggle(const UHAMotorDriverConfig* config) {
	gpio_toggle_pin(config->en);
	uha_motor_driver_set_3x(config);
}

