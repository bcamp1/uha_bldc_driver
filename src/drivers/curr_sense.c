#include "curr_sense.h"
#include "../board.h"
#include "../periphs/timer.h"
#include "../periphs/uart.h"
#include "../periphs/gpio.h"

#define CURR_SENSE_FREQ (240000.0f)
#define CURR_SENSE_TIMER_ID (1)
#define CURR_SENSE_PRIO (1)

static void timer_callback();

void adc_init(void) {
    // Enable bus clock for ADC0
    MCLK->APBDMASK.bit.ADC0_ = 1;

    // Connect GCLK0 (120 MHz) / 32 = 3.75 MHz to ADC0
    // (ADC max clock is ~3.5 MHz, so divide to stay within spec)
    GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
    while (!(GCLK->PCHCTRL[ADC0_GCLK_ID].bit.CHEN)); // Wait for sync

    // Reset ADC
    ADC0->CTRLA.bit.SWRST = 1;
    while (ADC0->SYNCBUSY.bit.SWRST);

    // 8-bit resolution
    ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;

    // Max clock prescaler = DIV1 (use full GCLK in range)
    ADC0->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV32_Val; 
    // -> 120 MHz / 32 = 3.75 MHz ADC clock

    // Sampling time: minimal (SAMPCTRL=0)
    ADC0->SAMPCTRL.reg = 0;

    // Select input PB03 = AIN[11]
    ADC0->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_AIN11_Val;
    ADC0->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val; // Single-ended

    while (ADC0->SYNCBUSY.bit.INPUTCTRL);

    // Reference: internal VDDANA (default 3.3V)
    ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;

    // Enable ADC
    ADC0->CTRLA.bit.ENABLE = 1;
    while (ADC0->SYNCBUSY.bit.ENABLE);
}

uint8_t adc_read(void) {
    // Start conversion
    ADC0->SWTRIG.bit.START = 1;
    while (ADC0->SYNCBUSY.bit.SWTRIG);

    // Wait until result ready
    while (!ADC0->INTFLAG.bit.RESRDY);

    // Read result (8-bit mode)
    return (uint8_t)ADC0->RESULT.reg;
}

void curr_sense_init() {
    //timer_schedule(CURR_SENSE_TIMER_ID, CURR_SENSE_FREQ, CURR_SENSE_PRIO, timer_callback);
}

static void timer_callback() {
    gpio_set_pin(PIN_DEBUG2);
    gpio_clear_pin(PIN_DEBUG2);
}


