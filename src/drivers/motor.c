/*
 * ems22a.c
 *
 * Created: 3/20/2025 12:35:23 AM
 *  Author: brans
 */ 

#include <stdint.h>
#include "motor.h"
#include "gate_driver.h"
#include "../foc/foc_math_fpu.h"
#include "../periphs/uart.h"
#include "../periphs/spi.h"
#include "../foc/foc.h"

#define MOTOR_POLES 4
#define TORQUE_LIMIT (0.4f)

static float offset = 1.38105f;

void motor_init() {
    // Init gate driver
    gate_driver_init();
	// Init encoder SPI
	spi_init(&SPI_CONF_MTR_ENCODER);
}

float motor_get_position() {
    uint16_t result = spi_write_read16(&SPI_CONF_MTR_ENCODER, 0) & 0x3FFF;
	float revolution_fraction = 2.0f * PI * ((float) result / ((float) 0x3FFF));
	return revolution_fraction;
}

float motor_get_pole_position() {
	float theta = motor_get_position();
	theta -= offset;
	theta *= (float) MOTOR_POLES;
	while (theta < 0) theta += 2*PI;
	while (theta > 2*PI) theta -= 2*PI;
	return theta;
}

float motor_get_pole_pos_from_theta(float theta) {
	theta -= offset;
	theta *= (float) MOTOR_POLES;
	while (theta < 0) theta += 2*PI;
	while (theta > 2*PI) theta -= 2*PI;
	return theta;
}

void motor_set_torque(float torque, float pole_position) {
    // Scale down torque + saturate
    if (torque > 1.0f) torque = 1.0f;
    if (torque < -1.0f) torque = -1.0f;
    torque *= TORQUE_LIMIT;

    // Get PWM values
    float a = 0;
    float b = 0;
    float c = 0;
    float d = 0;
    float q = torque;

    //gpio_set_pin(DEBUG_PIN);
    foc_get_duties(pole_position, d, q, &a, &b, &c);
    //gpio_clear_pin(DEBUG_PIN);

    // Convert to integers
    uint8_t a_int = (int) (255.0f * a);
    uint8_t b_int = (int) (255.0f * b);
    uint8_t c_int = (int) (255.0f * c);

    // Write them to motor driver
    gate_driver_set_pwm(a_int, b_int, c_int);
}

void motor_energize_coils(float a, float b, float c) {
	// Convert to integers
	uint8_t a_int = (int) (255.0f * a);
	uint8_t b_int = (int) (255.0f * b);
	uint8_t c_int = (int) (255.0f * c);
	
	// Write them to motor driver
	gate_driver_set_pwm(a_int, b_int, c_int);
}

void motor_write_reg(uint8_t address, uint16_t data) {
    gate_driver_write_reg(address, data);
}

uint16_t motor_read_reg(uint8_t address) {
    return gate_driver_read_reg(address);
}

void motor_enable() {
    gate_driver_enable();
}

void motor_disable() {
    gate_driver_disable();
}

void motor_set_high_z() {
    gate_driver_set_high_z();
}

