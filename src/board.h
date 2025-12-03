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
#define PIN_GATE_SOA PIN_PA02 // ADC0 AIN[0]
#define PIN_GATE_SOB PIN_PB02 // ADC0 AIN[14]
#define PIN_GATE_SOC PIN_PB03 // ADC0 AIN[15]
#define PIN_GATE_NFAULT PIN_PA20
#define PIN_GATE_ENABLE PIN_PB17
#define PIN_GATE_CAL PIN_PB16
#define PIN_GATE_INHA PIN_PA23
#define PIN_GATE_INHB PIN_PA22
#define PIN_GATE_INHC PIN_PA21
#define PIN_GATE_INL PIN_PA24

// Current Sense Settings
#define SOA_INDEX (0)
#define SOB_INDEX (14)
#define SOC_INDEX (15)

// PWM Generation
#define PWM_INHA_INDEX (3)
#define PWM_INHB_INDEX (2)
#define PWM_INHC_INDEX (1)
#define PWM_CURR_SENSE_INDEX (0)
#define PWM_TIMER (TCC0)
#define GCLK_TCC0_TCC1_INDEX (25)
#define GCLK_TCC2_TCC3_INDEX (29)
#define DSCRITICAL		(0x4)
#define DSBOTTOM		(0x5)
#define DSBOTH			(0x6)
#define DSTOP			(0x7)
#define TCC_CMD_START	(0x1)
#define TCC_CMD_STOP	(0x2)

// Interface with Motherboard
#define PIN_IDENT1      PIN_PA08
#define PIN_IDENT0      PIN_PA11
#define PIN_SCL         PIN_PA13
#define PIN_SDA         PIN_PA12
#define PIN_SLAVE_SDO   PIN_PB12 // SERCOM4[0] C
#define PIN_SLAVE_SDI   PIN_PB11 // SERCOM4[3] D
#define PIN_SLAVE_CS    PIN_PB10 // SERCOM4[2] D
#define PIN_SLAVE_SCK   PIN_PB13 // SERCOM4[1] C
#define PIN_ENABLE      PIN_PA15
#define PIN_ENABLE_EIC_INDEX 15 

// SPI Slave Settings
#define SPI_SLAVE_DIPO      (0x3) // PAD3 = Data in
#define SPI_SLAVE_DOPO      (0x0) // PAD0 = Data out, PAD1 = SCK, PAD2 = CS
#define SPI_SLAVE_POLARITY  (0)
#define SPI_SLAVE_PHASE     (1)
#define SPI_SLAVE ((SercomSpi*) SERCOM4)

// PWM Capture
#define PWM_CAPTURE_TIMER (TCC1)
#define PWM_CAPTURE_EIC_INDEX 11
// Current TRQMAG PIN (PA08) is EIC Non-Maskable Interrupt (NMI)

// Events
#define EVENT_TCC_ADC_CHANNEL (0)
#define EVENT_EIC_TCC_CHANNEL (1)

// Priorities
#define PRIO_FOC_LOOP (1)
#define PRIO_EIC_ENABLE (2)
#define PRIO_SPI_SLAVE (0)


