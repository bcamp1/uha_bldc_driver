#include <sam.h>
#include "stopwatch.h"
#include "../periphs/uart.h"
//#include "../periphs/gpio.h"

#define TIMER TC4->COUNT32

static volatile uint32_t start_timestamps[10];

static void wait_resync() {
    TIMER.CTRLBSET.bit.CMD = 0x4; // READSYNC
    while (TIMER.SYNCBUSY.bit.CTRLB) {} // Wait for sync
}

void stopwatch_init() {
    // Enable bus clock
	MCLK->APBCMASK.reg |= (MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5); 

    // Init gclock
	GCLK->PCHCTRL[TC4_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
	while (!(GCLK->PCHCTRL[TC4_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));  // Wait for clock enable
	GCLK->PCHCTRL[TC5_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0;
	while (!(GCLK->PCHCTRL[TC5_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));  // Wait for clock enable
    
	// Enable NVIC interrupt
	NVIC_EnableIRQ(TC4_IRQn);
	NVIC_EnableIRQ(TC5_IRQn);
   
    // Disable timer so we can modify it
	TIMER.CTRLA.bit.ENABLE = 0;
	while (TIMER.SYNCBUSY.bit.ENABLE) {}
    
    // Set to 32 bit mode
    TIMER.CTRLA.bit.MODE = 0x2;

	// Enable MC0 interrupt
    // TIMER.INTENSET.bit.MC0 = 1;
	
	// Set prescaler
	TIMER.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;
	
	// Set MC0 value
	uint32_t count_value = 0xFFFFFFFF;
	TIMER.CC[0].reg = count_value;
	
	// Enable Match Frequency wavegen
	TIMER.WAVE.bit.WAVEGEN = 0x1;
	
	// Enable timer
	TIMER.CTRLA.bit.ENABLE = 1;
	while (TIMER.SYNCBUSY.bit.ENABLE);
}

uint32_t stopwatch_underlying_time() {
    wait_resync();
    uint32_t time = TIMER.COUNT.reg;
    return time;
}

void stopwatch_start(int id) {
    wait_resync();
    uint32_t start_time = TIMER.COUNT.reg;
    start_timestamps[id] = start_time;
    //uart_print("SW_NEW ");
    //uart_println_int_base(start_time, 16);
}

uint32_t stopwatch_stop(int id, bool lap) {
    wait_resync();
    uint32_t stop_time = TIMER.COUNT.reg;
    uint32_t start_time = start_timestamps[id];
    if (lap) {
        start_timestamps[id] = stop_time;
    }
    uint32_t elapsed_counts = 0;
    if (stop_time < start_time) {
        elapsed_counts = (0xFFFFFFFF - start_time) + (stop_time - 0x0);
    } else {
        elapsed_counts = stop_time - start_time;
    }
    return elapsed_counts;
}

float ticks_to_time(uint32_t ticks) {
    const float time_per_count = 8.33e-9f;
    return time_per_count * (float) ticks;
}

void stopwatch_print(int id, bool lap) {
    uint32_t ticks = stopwatch_stop(id, false);
    float dt_ms = ticks_to_time(ticks) * 1000.0f;
    uart_print("Stopwatch [");
    uart_print_int(id);
    uart_print("]: ");
    uart_print_float(dt_ms);
    uart_println(" ms");
    if (lap) {
        stopwatch_start(id);
    }
}

void TC4_Handler(void) {
	// Clear interrupt
	//TIMER.INTFLAG.bit.MC0 = 1;
    
    //gpio_toggle_pin(LED_PIN);
    //gpio_toggle_pin(LED_PIN);
}

void TC5_Handler(void) {

}

