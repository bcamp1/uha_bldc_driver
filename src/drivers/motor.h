/*
 * encoder.h
 *
 * Created: 7/28/2024 10:43:19 PM
 *  Author: brans
 */

#pragma once
#include <stdint.h>
#include "gate_driver.h"

float motor_get_position();
float motor_get_pole_position();
void motor_init();
float motor_get_pole_pos_from_theta(float theta);
void motor_energize_coils(float a, float b, float c);
void motor_set_torque(float torque, float pole_position);
void motor_write_reg(uint8_t address, uint16_t data);
uint16_t motor_read_reg(uint8_t address);
void motor_enable();
void motor_disable();
void motor_set_high_z();

