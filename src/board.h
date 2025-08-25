#pragma once

// General pins
#define PIN_DEBUG1 PIN_PA14
#define PIN_DEBUG2 PIN_PB15

#define PIN_TX PIN_PA04 
#define PIN_RX PIN_PA05 

#define PIN_MOSI PIN_PA17
#define PIN_MISO PIN_PA18
#define PIN_SCK PIN_PA16

// Encoder pins
#define PIN_ENCODER_CS PIN_PB14

// Gate Driver pins
#define PIN_GATE_CS PIN_PA19
#define PIN_GATE_SOA PIN_PA02
#define PIN_GATE_SOB PIN_PB02
#define PIN_GATE_SOC PIN_PB03
#define PIN_GATE_NFAULT PIN_PA20
#define PIN_GATE_ENABLE PIN_PB17
#define PIN_GATE_CAL PIN_PB16
#define PIN_GATE_INHA PIN_PA23
#define PIN_GATE_INHB PIN_PA22
#define PIN_GATE_INHC PIN_PA21
#define PIN_GATE_INL PIN_PA24

// PWM Settings
#define PWM_INHA_INDEX (3)
#define PWM_INHB_INDEX (2)
#define PWM_INHC_INDEX (1)
#define PWM_TIMER (TCC0)

