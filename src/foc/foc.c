/*
 * foc.c
 *
 * Created: 7/30/2024 5:36:34 PM
 *  Author: brans
 */ 

#include "foc.h"
#include "foc_math_fpu.h"
//#include "../drivers/motor_encoder.h"
//#include "../periphs/pwm.h"

void foc_get_duties(float theta, float d, float q, float* a, float* b, float* c) {
	inv_park_clark(theta, d, q, a, b, c);
	*a = 0.5f*(*a) + 0.5f;
	*b = 0.5f*(*b) + 0.5f;
	*c = 0.5f*(*c) + 0.5f;
}
