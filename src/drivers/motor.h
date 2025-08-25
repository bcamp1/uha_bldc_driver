/*
 * encoder.h
 *
 * Created: 7/28/2024 10:43:19 PM
 *  Author: brans
 */

#pragma once

float motor_get_position();
float motor_get_pole_position();
void motor_init();
float motor_get_pole_pos_from_theta(float theta);
void motor_energize_coils(float a, float b, float c);
void motor_set_torque(float torque, float pole_position);

