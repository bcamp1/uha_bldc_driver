/*
 * ems22a.c
 *
 * Created: 3/20/2025 12:35:23 AM
 *  Author: brans
 */ 

#include <stdint.h>
#include "motor.h"
#include "gate_driver.h"
#include "curr_sense.h"
#include "../foc/foc_math_fpu.h"
#include "../periphs/uart.h"
#include "../periphs/spi.h"
#include "../periphs/gpio.h"
#include "../foc/foc.h"
#include "../periphs/delay.h"
#include "../periphs/spi_async.h"
#include "../board.h"

static uint8_t identity = MOTOR_IDENT_UNKNOWN;

MotorConfig MOTOR_CONF_SUPPLY = {
    .offset = 0.5809f,
    .poles = 4,
    .speed_control = false,
    .max_torque = 0.6f,
};

MotorConfig MOTOR_CONF_TAKEUP = {
    .offset = 3.0237f,
    .poles = 4,
    .speed_control = false,
    .max_torque = 0.6f,
};

MotorConfig MOTOR_CONF_CAPSTAN = {
    .offset = 0.0676f,
    .poles = 3,
    .speed_control = true,
    .max_torque = 0.8f,
};

static MotorConfig* config = NULL;

void motor_init_from_ident() {
    gpio_init_pin(PIN_IDENT1, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);
    gpio_init_pin(PIN_IDENT0, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);
    uint8_t ident1 = (uint8_t) gpio_get_pin(PIN_IDENT1);
    uint8_t ident0 = (uint8_t) gpio_get_pin(PIN_IDENT0);
    identity = (ident1 << 1) | ident0; 

    switch (identity) {
        case MOTOR_IDENT_CAPSTAN:
            uart_println("Identity: capstan");
            motor_init(&MOTOR_CONF_CAPSTAN);
            break;
        case MOTOR_IDENT_TAKEUP:
            uart_println("Identity: takeup");
            motor_init(&MOTOR_CONF_TAKEUP);
            break;
        case MOTOR_IDENT_SUPPLY:
            uart_println("Identity: supply");
            motor_init(&MOTOR_CONF_SUPPLY);
            break;
        case MOTOR_IDENT_UNKNOWN:
            uart_println("ERROR TRIED TO INIT WITH INVALID IDENTITY");
            break;
        default:
            while (true) {
                uart_println("ERROR INVALID IDENTITY");
            }
    }
}

void motor_init(MotorConfig* motor_config) {
    config = motor_config;
    // Init current sense
    //curr_sense_init();

    // Init gate driver
    gate_driver_init();

	// Init encoder SPI
	//spi_init(&SPI_CONF_MTR_ENCODER);
    spi_async_init();
}

float motor_get_position() {
    uint16_t raw_result = spi_async_get_safe_result();
    uint16_t result = raw_result & 0x3FFF;
	float revolution_fraction = 2.0f * PI * ((float) result / ((float) 0x3FFF));
	return revolution_fraction;
}

float motor_get_pole_position() {
	float theta = motor_get_position();
	theta -= config->offset;
	theta *= (float) config->poles;
	while (theta < 0) theta += 2*PI;
	while (theta > 2*PI) theta -= 2*PI;
	return theta;
}

float motor_get_pole_pos_from_theta(float theta) {
	theta -= config->offset;
	theta *= (float) config->poles;
	while (theta < 0) theta += 2*PI;
	while (theta > 2*PI) theta -= 2*PI;
	return theta;
}

void motor_set_torque(float torque, float pole_position) {
    // Scale down torque + saturate
    if (torque > 1.0f) torque = 1.0f;
    if (torque < -1.0f) torque = -1.0f;
    torque *= config->max_torque;

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

void motor_set_align(float torque, float pole_position) {
    // Scale down torque + saturate
    if (torque > 1.0f) torque = 1.0f;
    if (torque < -1.0f) torque = -1.0f;
    torque *= config->max_torque;

    // Get PWM values
    float a = 0;
    float b = 0;
    float c = 0;
    float d = torque;
    float q = 0;

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

void motor_print_reg(uint8_t address, char* name) {
    uint16_t data = motor_read_reg(address);
    uart_print(name);
    uart_print(": ");
    uart_println_int_base(data, 2);
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

void motor_calibrate_encoder() {
    const int iterations = 4000;
    float alpha = (1.0f / 100.0f);
    config->offset = 0.0f;

    motor_energize_coils(0.15f, 0.0f, 0.0f);
    delay(0xFFFFF);
    float avg = motor_get_position();
    
    for (int i = 0; i < iterations; i++) {
        //uart_println_int(i+1);
        float position = motor_get_position();
        avg = avg + (alpha*(position - avg));
        uart_println_float(avg);
    }
    config->offset = avg;
    motor_energize_coils(0.0f, 0.0f, 0.0f);
}

uint8_t motor_get_identity() {
    return identity;
}

