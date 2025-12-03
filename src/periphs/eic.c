/*
 * eic.c
 *
 * Created: 3/20/2025 9:46:04 PM
 *  Author: brans
 */ 

#include "eic.h"
#include "sam.h"
#include "gpio.h"
#include "../board.h"

static func_ptr_t callbacks[16];

static void process_interrupt(uint16_t ext_num);

void eic_init() {
	MCLK->APBAMASK.bit.EIC_ = 1; // Enable clock
	
	// Select GCLK0 (or another suitable clock) as the source for EIC
	GCLK->PCHCTRL[EIC_GCLK_ID].bit.GEN = GCLK_PCHCTRL_GEN_GCLK0_Val;

	// Enable the peripheral channel
	GCLK->PCHCTRL[EIC_GCLK_ID].bit.CHEN = 1;

    // Set NVIC priorities - EIC_15 lower priority than TC0 to prevent preemption
    NVIC_SetPriority(EIC_15_IRQn, PRIO_EIC_ENABLE);

	// Enable EIC interrupts
	NVIC_EnableIRQ(EIC_0_IRQn);
	NVIC_EnableIRQ(EIC_1_IRQn);
	NVIC_EnableIRQ(EIC_2_IRQn);
	NVIC_EnableIRQ(EIC_3_IRQn);
	NVIC_EnableIRQ(EIC_4_IRQn);
	NVIC_EnableIRQ(EIC_5_IRQn);
	NVIC_EnableIRQ(EIC_6_IRQn);
	NVIC_EnableIRQ(EIC_7_IRQn);
	NVIC_EnableIRQ(EIC_8_IRQn);
	NVIC_EnableIRQ(EIC_9_IRQn);
	NVIC_EnableIRQ(EIC_10_IRQn);
	NVIC_EnableIRQ(EIC_11_IRQn);
	NVIC_EnableIRQ(EIC_12_IRQn);
	NVIC_EnableIRQ(EIC_13_IRQn);
	NVIC_EnableIRQ(EIC_14_IRQn);
	NVIC_EnableIRQ(EIC_15_IRQn);
	
	EIC->CTRLA.bit.ENABLE = 1; // Enable EIC
	while (EIC->SYNCBUSY.bit.ENABLE);
}

void eic_init_pin(uint16_t pin, uint16_t ext_num, uint16_t int_mode, func_ptr_t callback) {
	// Set up pin for EIC use
	gpio_init_pin(pin, GPIO_DIR_IN, GPIO_ALTERNATE_A_EIC);
	
	EIC->CTRLA.bit.ENABLE = 0; // Disable EIC
	while (EIC->SYNCBUSY.bit.ENABLE);
	
	// Set config_n register
	uint16_t config_num;
	uint16_t block_num;
	
	if (ext_num > 7) {
		config_num = 1;
		block_num = ext_num - 8;
	} else {
		config_num = 0;
		block_num = ext_num;
	}

    // Enable events
    //EIC->EVCTRL.reg |= (0b1 << ext_num);
	
	uint32_t config_mask = int_mode << (4*block_num);
	EIC->CONFIG[config_num].reg |= config_mask;
	
	// Enable external interrupt
	EIC->INTENSET.reg |= (0b1) << ext_num;
	
	// Register callback
	callbacks[ext_num] = callback;
	
	// Re-enable EIC
	EIC->CTRLA.bit.ENABLE = 1;
	while (EIC->SYNCBUSY.bit.ENABLE);
}

void EIC_15_Handler(void) {
	//gpio_set_pin(PIN_DEBUG2);      // DEBUG: Trace EIC_15 (ENABLE) firing
	process_interrupt(15);
	//gpio_clear_pin(PIN_DEBUG2);
}

static void process_interrupt(uint16_t ext_num) {
	// Clear interrupt
	EIC->INTFLAG.reg |= (0b1) << ext_num;
	
	// Call the callback
	func_ptr_t callback = callbacks[ext_num];
	callback();
}

