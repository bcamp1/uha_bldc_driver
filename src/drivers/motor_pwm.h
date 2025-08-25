/*
 * pwm.h
 *
 * Created: 7/10/2024 6:25:10 PM
 *  Author: brans
 */ 

#pragma once
#include <stdint.h>

void pwm_timer_init();
void pwm_set_duties_int(uint8_t a, uint8_t b, uint8_t c);

