/*
 * pwm.c
 *
 * Created: 7/10/2024 6:24:51 PM
 *  Author: brans
 */ 

#include "motor_pwm.h"
#include "../periphs/gpio.h"
#include "../board.h"
#include "samd51j20a.h"

#define PWM_PRESCALER	(0x2)

// Timer Settings
#define TIMER_TOP		(0xFF)

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
void pwm_timer_init() {
	// Init Pins
	//init_pwm_pins();
	gpio_init_pin(PIN_GATE_INHA, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	gpio_init_pin(PIN_GATE_INHB, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	gpio_init_pin(PIN_GATE_INHC, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	
	// Configure clock
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1;
	
	// Enable GCLK0 to Peripheral
	GCLK->PCHCTRL[GCLK_TCC0_TCC1_INDEX].reg |= GCLK_PCHCTRL_CHEN;
 	
	// Configure timer
	PWM_TIMER->CTRLA.reg |= PWM_PRESCALER << 8;	// Set prescaler
	
	// Set Wavegen
	PWM_TIMER->WAVE.reg |= DSTOP 
		| TCC_WAVE_POL0
		| TCC_WAVE_POL1
		| TCC_WAVE_POL2
		| TCC_WAVE_POL3;

	PWM_TIMER->PER.reg = TIMER_TOP;

    // The interrupt in question. CC0
    //PWM_TIMER->INTENSET.bit.MC0 = 1;
    //PWM_TIMER->INTENSET.bit.OVF = 1;
    PWM_TIMER->CC[PWM_CURR_SENSE_INDEX].reg = 50;

    PWM_TIMER->EVCTRL.bit.OVFEO = 1;
    PWM_TIMER->EVCTRL.bit.MCEO0 = 1;

    // Set CTRLB
    // timer->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER | TCC_CTRLBSET_CMD_READSYNC | TCC_CTRLBSET_DIR;
    PWM_TIMER->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
    while (PWM_TIMER->SYNCBUSY.bit.CTRLB); // Wait for busy

    // Enable timer
	PWM_TIMER->CTRLA.bit.ENABLE = 1; // Enable timer
    while (PWM_TIMER->SYNCBUSY.bit.ENABLE); // Wait for busy
	
    //NVIC_EnableIRQ(TCC0_0_IRQn);
}

void pwm_set_duties_int(uint8_t a, uint8_t b, uint8_t c) {
	PWM_TIMER->CC[PWM_INHA_INDEX].reg = a;
	PWM_TIMER->CC[PWM_INHB_INDEX].reg = b;
	PWM_TIMER->CC[PWM_INHC_INDEX].reg = c;
    while (PWM_TIMER->SYNCBUSY.bit.CC1);
    while (PWM_TIMER->SYNCBUSY.bit.CC2);
    while (PWM_TIMER->SYNCBUSY.bit.CC3);
}

//void TCC0_0_Handler() {
    //TCC0->INTFLAG.bit.OVF = 1;
    //gpio_set_pin(PIN_DEBUG2);
    //gpio_clear_pin(PIN_DEBUG2);
//}

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


