/*
 * foc_math_fpu.c
 *
 * Created: 3/16/2025 12:46:37 AM
 *  Author: brans
 */ 

#include "foc_math_fpu.h"
#include "../board.h"
#include "../periphs/gpio.h"
#include "fast_sin_cos.h"

float to_rads(float angle) {
	return angle*0.0174533f;
}

float to_degrees(float angle) {
	return angle*57.2958f;
}

void print_float(float f) {
	//printf("%s\n", fpt_cstr(f, 5));
}

void park_transform(float theta, float alpha, float beta, float* d, float* q) {
	float sin_theta = 0.0f;
	float cos_theta = 0.0f;
    float theta_deg = theta * (180.0f / 3.1415f);

    arm_sin_cos_f32(theta_deg, &sin_theta, &cos_theta);

	*d = (alpha*cos_theta) + (beta*sin_theta);
	*q = (beta*cos_theta) - (alpha*sin_theta);
}

void inv_park_transform(float theta, float d, float q, float* alpha, float* beta) {
    //gpio_set_pin(DEBUG_PIN);
	float sin_theta = 0.0f;
	float cos_theta = 0.0f;
    float theta_deg = theta * 57.2957795f;

    arm_sin_cos_f32(theta_deg, &sin_theta, &cos_theta);

	*alpha = d*cos_theta - q*sin_theta;
	*beta = q*cos_theta + d*sin_theta;
    //gpio_clear_pin(DEBUG_PIN);
}

void inv_clark_transform(float alpha, float beta, float* a, float* b, float* c) {
	float sqrt_three_beta = 1.73205080757f*beta;
	
	*a = alpha;
	*b = 0.5 * (-1.0f*alpha + sqrt_three_beta);
	*c = 0.5 * (-1.0f*alpha - sqrt_three_beta);
}

void clark_transform(float a, float b, float c, float* alpha, float* beta) {
    *alpha = 0.6666f*a - 0.3333f * (b + c);
    *beta = (b - c) * 0.57735026919f;
}
void clark_park(float theta, float a, float b, float c, float* d, float* q) {
	float alpha, beta;
	clark_transform(a, b, c, &alpha, &beta);
	park_transform(theta, alpha, beta, d, q);
}

void inv_park_clark(float theta, float d, float q, float* a, float* b, float* c) {
	float alpha, beta;
	inv_park_transform(theta, d, q, &alpha, &beta);
	inv_clark_transform(alpha, beta, a, b, c);
}


