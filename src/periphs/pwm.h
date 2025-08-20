/*
 * pwm.h
 *
 * Created: 7/10/2024 6:25:10 PM
 *  Author: brans
 */ 


#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>
#include "sam.h"

typedef struct {
	Tcc* timer;
	uint8_t pin_a;
	uint8_t pin_b;
	uint8_t pin_c;
	uint8_t a_index;
	uint8_t b_index;
	uint8_t c_index;
} PWMConfig;

extern const PWMConfig PWM_CONF;

void pwm_timer_init(const PWMConfig* pwm);
void pwm_timer_loop(void);
void pwm_set_duties_int(const PWMConfig* pwm, uint8_t a, uint8_t b, uint8_t c);

#endif /* PWM_H_ */
