/*
 * ems22a.c
 *
 * Created: 3/20/2025 12:35:23 AM
 *  Author: brans
 */ 

#include "motor_encoder.h"
#include "../foc/foc_math_fpu.h"
#include "../periphs/uart.h"

#define MOTOR_POLES 4

const MotorEncoderConfig MOTOR_ENCODER_A = {
	.spi = &SPI_CONF_MTR_ENCODER_A,
	.offset = 1.38105f,
};

const MotorEncoderConfig MOTOR_ENCODER_B = {
	.spi = &SPI_CONF_MTR_ENCODER_B,
	.offset = 4.59148f,
};

void motor_encoder_init(const MotorEncoderConfig* config) {
	// Init SPI
	spi_init(config->spi);
}

float motor_encoder_get_position(const MotorEncoderConfig* config) {
	uint16_t result = spi_write_read16(config->spi, 0) & 0x3FFF;
	float revolution_fraction = 2.0f * PI * ((float) result / ((float) 0x3FFF));
	return revolution_fraction;   
}

float motor_encoder_get_pole_position(const MotorEncoderConfig* config) {
	float theta = motor_encoder_get_position(config);
	theta -= config->offset;
	theta *= (float) MOTOR_POLES;
	while (theta < 0) theta += 2*PI;
	while (theta > 2*PI) theta -= 2*PI;
	return theta;
}
