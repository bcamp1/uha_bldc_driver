/*
 * gate_driver.c
 *
 * Created: 3/9/2025 4:24:01 PM
 *  Author: brans
 */ 

#include "gate_driver.h"
#include "motor_pwm.h"
#include "../periphs/gpio.h"
#include "../periphs/spi.h"
#include "../foc/foc.h"
#include "../periphs/delay.h"
#include "../board.h"
#include <stdbool.h>

/*
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

*/

void gate_driver_init() {
	// Intialize PWM
	pwm_timer_init();
	
	// Initialize SPI
	spi_init(&SPI_CONF_GATE_DRIVER);
	
	// Initialize digital GPIO (CAL, EN, INL, nFault)
	gpio_init_pin(PIN_GATE_CAL, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(PIN_GATE_ENABLE, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(PIN_GATE_INL, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(PIN_GATE_NFAULT, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);
	
	// Start with motor driver disabled
    gate_driver_disable();
}

void gate_driver_set_3x() {
    // Set to 3x PWM mode
    uint16_t pwm_mode = 0b01; // 3x PWM Mode
    gate_driver_write_reg(DRV_REG_DRIVER_CONTROL, pwm_mode << 5);
}

void gate_driver_set_idrive(uint16_t hs_p, uint16_t hs_n, uint16_t ls_p, uint16_t ls_n) {
    uint16_t hs_register = 0b01100000000;
    uint16_t ls_register = 0b11100000000;
    
    hs_register |= (hs_p << 4);
    hs_register |= hs_n;

    ls_register |= (ls_p << 4);
    ls_register |= ls_n;

    gate_driver_write_reg(DRV_REG_GATE_DRIVER_HS, hs_register);
    gate_driver_write_reg(DRV_REG_GATE_DRIVER_LS, ls_register);
}

void gate_driver_write_reg(uint8_t address, uint16_t data) {
    spi_change_mode(&SPI_CONF_GATE_DRIVER);
	uint16_t command = 0;
	command |= (address & 0xF) << 11;
	command |= data & 0x7FF;
	spi_write_read16(&SPI_CONF_GATE_DRIVER, command);
    spi_change_mode(&SPI_CONF_MTR_ENCODER);
}

uint16_t gate_driver_read_reg(uint8_t address) {
    spi_change_mode(&SPI_CONF_GATE_DRIVER);
	uint16_t command = 0x8000;
	command |= (address & 0xF) << 11;
	uint16_t result = spi_write_read16(&SPI_CONF_GATE_DRIVER, command);
    spi_change_mode(&SPI_CONF_MTR_ENCODER);
    return result;
}

void gate_driver_set_pwm(uint8_t a, uint8_t b, uint8_t c) {
    //uint32_t primask = __get_PRIMASK();
    //__disable_irq();
	gpio_set_pin(PIN_GATE_INL);
	pwm_set_duties_int(a, b, c);
    //__set_PRIMASK(primask);
}

void gate_driver_set_high_z() {
	gpio_clear_pin(PIN_GATE_INL);
	pwm_set_duties_int(0, 0, 0);
}

void gate_driver_goto_theta(float theta) {
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
	
	gate_driver_set_pwm(a_int, b_int, c_int);
}

void gate_driver_enable() {
	gpio_set_pin(PIN_GATE_ENABLE);
	delay(0xFFF);
	gate_driver_set_3x();
}

void gate_driver_disable() {
	gpio_clear_pin(PIN_GATE_ENABLE);
    delay(0xFFF);
}

void gate_driver_toggle() {
	gpio_toggle_pin(PIN_GATE_ENABLE);
	gate_driver_set_3x();
}

