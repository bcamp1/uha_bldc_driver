/*
 * motor_unit.c
 *
 * Created: 3/28/2025 10:48:53 PM
 *  Author: brans
 */ 

#include "motor_unit.h"
#include "../foc/foc_math_fpu.h"
#include "../foc/foc.h"
#include "../periphs/uart.h"
#include "../periphs/gpio.h"
#include "board.h"

const MotorUnitConfig MOTOR_UNIT_CONF = {
	.driver = &UHA_MTR_DRVR_CONF,
	.encoder = &MOTOR_ENCODER_CONF,
};

void motor_unit_init(const MotorUnitConfig* config) {
	// Initialize driver
	uha_motor_driver_init(config->driver);
	
	// Initialize Encoder
	motor_encoder_init(config->encoder);
}

#define TORQUE_LIMIT (0.4f)

void motor_unit_set_torque(const MotorUnitConfig* config, float torque, float pole_position) {
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
    uha_motor_driver_set_pwm(config->driver, a_int, b_int, c_int);
}

void motor_unit_energize_coils(const MotorUnitConfig* config, float a, float b, float c) {
	// Convert to integers
	uint8_t a_int = (int) (255.0f * a);
	uint8_t b_int = (int) (255.0f * b);
	uint8_t c_int = (int) (255.0f * c);
	
	// Write them to motor driver
	uha_motor_driver_set_pwm(config->driver, a_int, b_int, c_int);
}
