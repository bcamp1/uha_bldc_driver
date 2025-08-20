/*
 * pwm.c
 *
 * Created: 7/10/2024 6:24:51 PM
 *  Author: brans
 */ 

#include "pwm.h"
#include "gpio.h"
#include "sam.h"

#define DSCRITICAL		(0x4)
#define DSBOTTOM		(0x5)
#define DSBOTH			(0x6)
#define DSTOP			(0x7)
#define TCC_CMD_START	(0x1)
#define TCC_CMD_STOP	(0x2)

#define PWM_PRESCALER	(0x2)

// Timer Settings
#define TIMER_TOP		(0xFF)
#define A_PIN_INDEX		(3)  
#define B_PIN_INDEX		(2)
#define C_PIN_INDEX		(1)

// Clock
#define GCLK_TCC0_TCC1_INDEX (25)
#define GCLK_TCC2_TCC3_INDEX (29)

const PWMConfig PWM_CONF = {
	.timer = TCC0,
	.pin_a = PIN_PA23,
	.pin_b = PIN_PA22,
	.pin_c = PIN_PA21,
	.a_index = 3,
	.b_index = 2,
	.c_index = 1
};

/*
Initialization steps:
1. Enable clock
2. CPTEN
3. CTRLA.PRESCALER
4. CTRLA.PRESCSYNC
5. WAVE.WAVEGEN
6. WAVE.POL
7. DRVCTRL.INVEN
*/
void pwm_timer_init(const PWMConfig* pwm) {
	// Init Pins
	//init_pwm_pins();
	gpio_init_pin(pwm->pin_a, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	gpio_init_pin(pwm->pin_b, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	gpio_init_pin(pwm->pin_c, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	
	// Configure clock
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1;
	
	// Enable GCLK0 to Peripheral
	GCLK->PCHCTRL[GCLK_TCC0_TCC1_INDEX].reg |= GCLK_PCHCTRL_CHEN;
 	
	// Configure timer
	pwm->timer->CTRLA.reg |= PWM_PRESCALER << 8;	// Set prescaler
	
	// Set Wavegen
	pwm->timer->WAVE.reg |= DSTOP 
		| TCC_WAVE_POL0
		| TCC_WAVE_POL1
		| TCC_WAVE_POL2
		| TCC_WAVE_POL3;

	pwm->timer->PER.reg = TIMER_TOP;
	
	pwm->timer->CTRLA.bit.ENABLE = 1; // Enable timer
	
	// timer->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER | TCC_CTRLBSET_CMD_READSYNC | TCC_CTRLBSET_DIR;
	pwm->timer->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
		
	while (pwm->timer->SYNCBUSY.bit.CTRLB); // Wait for busy
}

void pwm_set_duties_int(const PWMConfig* pwm, uint8_t a, uint8_t b, uint8_t c) {
	pwm->timer->CC[pwm->a_index].reg = a;
	pwm->timer->CC[pwm->b_index].reg = b;
	pwm->timer->CC[pwm->c_index].reg = c;
}

/*
void init_pwm_pins(void) {
	// TCC0 (Gate Driver A) Initialization
	gpio_init_pin(PIN_PA21, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	gpio_init_pin(PIN_PA22, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	gpio_init_pin(PIN_PA23, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	
	// TCC1 (Gate Driver B) Initialization
	gpio_init_pin(PIN_PA08, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	gpio_init_pin(PIN_PA09, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	gpio_init_pin(PIN_PA10, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
}
*/

/*
void pwm_timer_loop(void) {
	fpt theta = FPT_ZERO;
	fpt inc = fl2fpt(0.1);
	//fpt inc_inc = (fpt) 1;
	
	fpt d = FPT_ZERO;
	fpt q = fl2fpt(0.3);
	
	fpt a = FPT_ZERO;
	fpt b = FPT_ZERO;
	fpt c = FPT_ZERO;
	
	
	
	while (1) {
		//for (uint16_t i = 0; i < 0xFF; i++) {}
		//inc = fpt_add(inc, inc_inc);
		//if (inc > fl2fpt(0.02)) inc = FPT_ZERO;
		
		//theta = encoder_get_rads();
		uart_println(fpt_cstr(theta, 3));
		
		//theta = fpt_add(theta, inc);
		//if (theta > fl2fpt(360)) theta = FPT_ZERO;
		
		inv_park_clark(theta, d, q, &a, &b, &c);
		
		a = fpt_add(fpt_mul(a, FPT_ONE_HALF), FPT_ONE_HALF);
		b = fpt_add(fpt_mul(b, FPT_ONE_HALF), FPT_ONE_HALF);
		c = fpt_add(fpt_mul(c, FPT_ONE_HALF), FPT_ONE_HALF);
		
		//pwm_set_duties(a, b, c);
		
		
		
		
		//pwm_set_duties(fl2fpt(0.98), fl2fpt(0.37), fl2fpt(0.17));
	}
}*/


