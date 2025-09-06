#include "curr_sense.h"
#include "../board.h"
#include "../periphs/timer.h"
#include "../periphs/uart.h"
#include "../periphs/gpio.h"
#include "component/adc.h"
#include "component/tcc.h"

#define CURR_SENSE_FREQ (240000.0f)
#define CURR_SENSE_TIMER_ID (1)
#define CURR_SENSE_PRIO (1)

static void timer_callback();

uint16_t adc_indices[3] = {SOA_INDEX, SOB_INDEX, SOC_INDEX};
//uint16_t adc_indices[3] = {SOA_INDEX, SOA_INDEX, SOA_INDEX};
static uint16_t index = 1;

static volatile float curr_values[] = {0.0f, 0.0f, 0.0f};

void curr_sense_init(void) {
    uart_println("Initializing ADC");
    // Initialize EVSYS
    MCLK->APBBMASK.bit.EVSYS_ = 1;

    // Channel 0: TCC0 compare match → ADC0
    const uint16_t CHANNEL_TCC0_OVF = 0x29;
    EVSYS->Channel[EVENT_TCC_ADC_CHANNEL].CHANNEL.reg = CHANNEL_TCC0_OVF | (0b1 << 9);

    // User: ADC0 start
    EVSYS->USER[EVSYS_ID_USER_ADC0_START].bit.CHANNEL = EVENT_TCC_ADC_CHANNEL + 1;

    // Initialize pins
    gpio_init_pin(PIN_GATE_SOA, GPIO_DIR_IN, GPIO_ALTERNATE_B_ADC);
    gpio_init_pin(PIN_GATE_SOB, GPIO_DIR_IN, GPIO_ALTERNATE_B_ADC);
    gpio_init_pin(PIN_GATE_SOC, GPIO_DIR_IN, GPIO_ALTERNATE_B_ADC);

    // Enable bus clock for ADC0
    MCLK->APBDMASK.bit.ADC0_ = 1;

    GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
    while (!(GCLK->PCHCTRL[ADC0_GCLK_ID].bit.CHEN)); // Wait for sync

    // Disable ADC
    ADC0->CTRLA.bit.ENABLE = 0;
    while (ADC0->SYNCBUSY.bit.ENABLE);

    // 8-bit resolution
    ADC0->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;

    // Max clock prescaler = DIV1 (use full GCLK in range)
    // -> 120 MHz / 32 = 3.75 MHz ADC clock
    ADC0->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV32_Val; 

    // Sampling time: minimal (SAMPCTRL=0)
    ADC0->SAMPCTRL.reg = 10;

    // Select input PB03 = AIN[11]
    ADC0->INPUTCTRL.bit.MUXPOS = adc_indices[index];
    ADC0->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val; // Single-ended

    while (ADC0->SYNCBUSY.bit.INPUTCTRL);

    // Reference: internal VDDANA (default 3.3V)
    ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;

    // Enable start event + interrupt
    ADC0->EVCTRL.bit.STARTEI = 1;
    ADC0->INTENSET.bit.RESRDY = 1;

    // Enable ADC
    ADC0->CTRLA.bit.ENABLE = 1;
    while (ADC0->SYNCBUSY.bit.ENABLE);

    NVIC_EnableIRQ(ADC0_1_IRQn);

    uart_println("Starting conversion");

    // Start conversion
    //ADC0->SWTRIG.bit.START = 1;
    //while (ADC0->SYNCBUSY.bit.SWTRIG);
    EVSYS->SWEVT.bit.CHANNEL0 = 1;
}

float adc_read(void) {
    // Start conversion
    ADC0->SWTRIG.bit.START = 1;
    while (ADC0->SYNCBUSY.bit.SWTRIG);

    // Wait until result ready
    while (!ADC0->INTFLAG.bit.RESRDY);

    // Read result (8-bit mode)
    float data = (float) ADC0->RESULT.reg;

    return ((data / 256.0f) * 3.3f);
}

void ADC0_1_Handler() {
    //gpio_toggle_pin(PIN_DEBUG2);
    // Read current value
    uint8_t result = ADC0->RESULT.reg;
    curr_values[index] = (((float) result)/256.0f) * 3.3f;
    index++;
    if (index > 2) index = 0;
    ADC0->INPUTCTRL.bit.MUXPOS = adc_indices[index];
    ADC0->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val; // Single-ended
    while (ADC0->SYNCBUSY.bit.INPUTCTRL);
    //gpio_clear_pin(PIN_DEBUG2);
}

void curr_sense_get_values(float* a, float* b, float* c) {
    *a = curr_values[0];
    *b = curr_values[1];
    *c = curr_values[2];
}

