/*
 * encoder.h
 *
 * Created: 7/28/2024 10:43:19 PM
 *  Author: brans
 */

#include "../periphs/spi.h"

#ifndef MOTOR_ENCODER_H_
#define MOTOR_ENCODER_H_

typedef struct {
	const SPIConfig* spi;
	float offset;
} MotorEncoderConfig;

extern const MotorEncoderConfig MOTOR_ENCODER_CONF;

float motor_encoder_get_position(const MotorEncoderConfig* config);
float motor_encoder_get_pole_position(const MotorEncoderConfig* config);
void motor_encoder_init(const MotorEncoderConfig* config);
float motor_encoder_get_pole_pos_from_theta(const MotorEncoderConfig* config, float theta);

#endif
