#include "pwm_capture.h"
#include "../periphs/gpio.h"
#include "../board.h"
#include <stdbool.h>
#include "samd51j20a.h"
#include "../periphs/stopwatch.h"

#define EIC_SENSE_NONE (0x0)
#define EIC_SENSE_RISE (0x1)
#define EIC_SENSE_FALL (0x2)
#define EIC_SENSE_BOTH (0x3)
#define EIC_SENSE_HIGH (0x4)
#define EIC_SENSE_LOW  (0x5)

static volatile uint32_t on_ticks = 1;
static volatile uint32_t off_ticks = 1;
static volatile bool is_high = false;

void pwm_capture_init() {
    // Init GPIO Pins
    // Using TRQDIR as magnitude, and PIN_SLAVE_CS as direction
    gpio_init_pin(PIN_TRQDIR, GPIO_DIR_IN, GPIO_ALTERNATE_A_EIC);
    gpio_init_pin(PIN_SLAVE_CS, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);

    // Enable clocks to EIC
    MCLK->APBAMASK.reg |= MCLK_APBAMASK_EIC;   // enable bus clock to EIC
    GCLK->PCHCTRL[EIC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
    while (!(GCLK->PCHCTRL[EIC_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));
    
    // Disable EIC     
    EIC->CTRLA.bit.ENABLE = 0;
    while (EIC->SYNCBUSY.bit.ENABLE);

    // Set sense mode = BOTH
    EIC->CONFIG[1].bit.SENSE3 = EIC_SENSE_BOTH; 
    EIC->INTENSET.reg = (1 << 11);
    EIC->EVCTRL.reg = (1 << 11);

    // Enable EIC     
    EIC->CTRLA.bit.ENABLE = 1;
    while (EIC->SYNCBUSY.bit.ENABLE);

    // Enable interrupt
    NVIC_EnableIRQ(EIC_11_IRQn);
    NVIC_SetPriority(EIC_11_IRQn, 0);

    // Enable clocks to TCC1
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC1;
	
	// Enable GCLK0 to Peripheral
	GCLK->PCHCTRL[GCLK_TCC0_TCC1_INDEX].reg |= GCLK_PCHCTRL_CHEN;
 	
	// Set Prescaler
	PWM_CAPTURE_TIMER->CTRLA.reg |= 0x2 << 8;

    // Enable capture mode
    PWM_CAPTURE_TIMER->EVCTRL.bit.MCEI1 = 1;
    PWM_CAPTURE_TIMER->CTRLA.bit.CPTEN1 = 1;

	//PWM_CAPTURE_TIMER->PER.reg = 0xFF;

    // The interrupt in question. CC0
    //PWM_CAPTURE_TIMER->INTENSET.bit.MC0 = 1;
    //PWM_CAPTURE_TIMER->INTENSET.bit.OVF = 1;
    //PWM_CAPTURE_TIMER->CC[PWM_CURR_SENSE_INDEX].reg = 50;

    PWM_CAPTURE_TIMER->EVCTRL.bit.OVFEO = 1;
    PWM_CAPTURE_TIMER->EVCTRL.bit.MCEO0 = 1;

    // Set CTRLB
    // timer->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER | TCC_CTRLBSET_CMD_READSYNC | TCC_CTRLBSET_DIR;
    PWM_CAPTURE_TIMER->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
    while (PWM_CAPTURE_TIMER->SYNCBUSY.bit.CTRLB); // Wait for busy

    // Enable timer
	PWM_CAPTURE_TIMER->CTRLA.bit.ENABLE = 1; // Enable timer
    while (PWM_CAPTURE_TIMER->SYNCBUSY.bit.ENABLE); // Wait for busy
    
    // Initialize stopwatch
    stopwatch_init();
    stopwatch_start(0);
}

float pwm_capture_get_mag() {
    return ((float) on_ticks) / (float) (on_ticks + off_ticks);
}

void EIC_11_Handler() {
    uint32_t ticks = stopwatch_stop(0, true);
    //gpio_set_pin(PIN_DEBUG2);
    if (is_high) {
        on_ticks = ticks;
        is_high = false;
    } else {
        off_ticks = ticks;
        is_high = true;
    }

    EIC->INTFLAG.reg = (1 << 11);
    //gpio_clear_pin(PIN_DEBUG2);
}

