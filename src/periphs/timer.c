/*
 * timer.c
 *
 * Created: 3/21/2025 2:19:31 AM
 *  Author: brans
 */ 

#include "timer.h"
#include "sam.h"
#include "samd51j20a.h"
#include "uart.h"
#include <stddef.h>

#define CLK_FREQ (120000000.0f)
#define CLK_DIV  (256.0f)

static const Tc* timers[4] = {TC0, TC1, TC2, TC3};
static volatile func_ptr_t callbacks[4] = {NULL, NULL, NULL, NULL};
	
static void process_interrupt(uint16_t timer_id);
static uint16_t calculate_count(float sample_rate);

static uint16_t priorities[4] = {1, 1, 1, 1};

void timer_init_all() {
	// Enable TCC bus clocks
	MCLK->APBAMASK.reg |= (MCLK_APBAMASK_TC0 | MCLK_APBAMASK_TC1); 
	MCLK->APBBMASK.reg |= (MCLK_APBBMASK_TC2 | MCLK_APBBMASK_TC3);
	MCLK->APBCMASK.reg |= (MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5); 
	
	// Enable gclocks
	GCLK->PCHCTRL[TC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
	while (!(GCLK->PCHCTRL[TC0_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));  // Wait for clock enable
	
	GCLK->PCHCTRL[TC1_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
	while (!(GCLK->PCHCTRL[TC1_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));  // Wait for clock enable
	
	GCLK->PCHCTRL[TC2_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
	while (!(GCLK->PCHCTRL[TC2_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));  // Wait for clock enable
	
	GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
	while (!(GCLK->PCHCTRL[TC3_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));  // Wait for clock enable

    // Set NVIC Priorities                                                            
    NVIC_SetPriority(TC0_IRQn, priorities[0]);
    NVIC_SetPriority(TC1_IRQn, priorities[1]);
    NVIC_SetPriority(TC2_IRQn, priorities[2]);
    NVIC_SetPriority(TC3_IRQn, priorities[3]);
	
	// Enable NVIC interrupts
	NVIC_EnableIRQ(TC0_IRQn);
	NVIC_EnableIRQ(TC1_IRQn);
	NVIC_EnableIRQ(TC2_IRQn);
	NVIC_EnableIRQ(TC3_IRQn);
}

void timer_schedule(uint16_t timer_id, float sample_rate, uint16_t priority, func_ptr_t callback) {
	TcCount16* TIMER = &timers[timer_id]->COUNT16;
	TIMER->CTRLA.bit.ENABLE = 0;
	// Wait for synchronization
	while (TIMER->SYNCBUSY.bit.ENABLE);
	
	// Add callback
	callbacks[timer_id] = callback;

    // Add priority
	priorities[timer_id] = priority;
	
	// Initialize Timers 1-5
	timer_init_all();

	// Enable MC0 interrupt
	TIMER->INTENSET.bit.MC0 = 1;
	
	// Set prescaler
	TIMER->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256;
	
	// Set MC0 value
	uint16_t count_value = calculate_count(sample_rate);
	TIMER->CC[0].reg = count_value;
	
	// Enable Match Frequency wavegen
	TIMER->WAVE.bit.WAVEGEN = 0x1;
	
	// Enable timer
	TIMER->CTRLA.bit.ENABLE = 1;
	
	// Wait for synchronization
	while (TIMER->SYNCBUSY.bit.ENABLE);
}

void timer_deschedule(uint16_t timer_id) {
	TcCount16* TIMER = &timers[timer_id]->COUNT16;

    // 1. Clear callback FIRST (with volatile, ensures fresh read in ISR)
    callbacks[timer_id] = NULL;
    __DSB();  // Data Synchronization Barrier - force write to complete

    // 2. Disable interrupt at peripheral level
    TIMER->INTENCLR.bit.MC0 = 1;

    // 3. Clear any pending interrupt flag at peripheral
    TIMER->INTFLAG.bit.MC0 = 1;

    // 4. Disable timer
    TIMER->CTRLA.bit.ENABLE = 0;
    while (TIMER->SYNCBUSY.bit.ENABLE) {}

    // 5. Clear NVIC pending interrupt
    if (timer_id == 0) NVIC_ClearPendingIRQ(TC0_IRQn);
    else if (timer_id == 1) NVIC_ClearPendingIRQ(TC1_IRQn);
    else if (timer_id == 2) NVIC_ClearPendingIRQ(TC2_IRQn);
    else if (timer_id == 3) NVIC_ClearPendingIRQ(TC3_IRQn);

    // 6. Memory barriers to ensure all operations complete before returning
    __DSB();  // Ensure all memory writes complete
    __ISB();  // Flush instruction pipeline
}

static uint16_t calculate_count(float sample_rate) {
	if (sample_rate < 1) sample_rate = 1;
	float sample_period = 1.0f / sample_rate;
	float clock_freq = CLK_FREQ / CLK_DIV;
	float clock_period = 1.0f / clock_freq;
	float k_samples = sample_period / clock_period;
	uint32_t k = (int) k_samples;
	if (k > 0xFFFF) {
		return 0xFFFF;
	}
	return (uint16_t) k;
}

static void process_interrupt(uint16_t timer_id) {
	TcCount16* TIMER = &timers[timer_id]->COUNT16;
	
	// Clear interrupt
	TIMER->INTFLAG.bit.MC0 = 1;

	// Execute callback
	func_ptr_t callback = callbacks[timer_id];	
	if (callback != NULL) {
		callback();
	}
}

// Interrupt Handlers
void TC0_Handler(void) {
	process_interrupt(0);
}

void TC1_Handler(void) {
	process_interrupt(1);
}

void TC2_Handler(void) {
	process_interrupt(2);
}

void TC3_Handler(void) {
	process_interrupt(3);
}

