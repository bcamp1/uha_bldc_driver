/*
 * ems22a.c
 *
 * Created: 3/20/2025 12:35:23 AM
 *  Author: brans
 */ 

#include <stdint.h>
#include <sam.h>
#include "motor.h"
#include "gate_driver.h"
#include "curr_sense.h"
#include "../foc/foc_math_fpu.h"
#include "../periphs/uart.h"
#include "../periphs/spi.h"
#include "../periphs/gpio.h"
#include "../foc/foc.h"
#include "../periphs/delay.h"
#include "../periphs/spi_async.h"
#include "../periphs/timer.h"
#include "../board.h"
#include "seeprom.h"

MotorConfig MOTOR_CONF_SUPPLY = {
    .offset = 3.0237f,
    .poles = 4,
    .speed_control = false,
    .max_torque = 0.6f,
};

MotorConfig MOTOR_CONF_TAKEUP = {
    .offset = 3.9542f,
    .poles = 4,
    .speed_control = false,
    .max_torque = 0.6f,
};

MotorConfig MOTOR_CONF_CAPSTAN = {
    .offset = 0.0676f,
    .poles = 3,
    .speed_control = true,
    .max_torque = 0.8f,
};

static MotorConfig* config = &MOTOR_CONF_SUPPLY;
static MotorIdentity identity = MOTOR_IDENT_UNKNOWN;

// SmartEEPROM layout: byte 0 is the boot counter (see main.c). Encoder
// calibration offsets live past it, one float per motor identity, so the same
// firmware running on takeup/supply/capstan boards never clobbers each other's
// stored offset.
#define SEE_SLOT_CAL_BASE 4u
static uint16_t cal_slot(void) {
    return (uint16_t)(SEE_SLOT_CAL_BASE + (uint16_t)identity * sizeof(float));
}

// Encoder is sampled by a 1700 Hz timer that kicks off an async SPI read on
// SERCOM3. motor_enable/disable temporarily mask this IRQ to prevent
// SERCOM3 contention with the gate driver (which shares SERCOM3).
static void encoder_spi_callback(void) {
    spi_async_start_read(NULL);
}

void motor_init(MotorIdentity i) {
    identity = i;
    switch (identity) {
        case MOTOR_IDENT_CAPSTAN:
            //uart_println("Identity: capstan");
            config = &MOTOR_CONF_CAPSTAN;
            break;
        case MOTOR_IDENT_TAKEUP:
            //uart_println("Identity: takeup");
            config = &MOTOR_CONF_TAKEUP;
            break;
        case MOTOR_IDENT_SUPPLY:
            //uart_println("Identity: supply");
            config = &MOTOR_CONF_SUPPLY;
            break;
        default:
            identity = MOTOR_IDENT_UNKNOWN;
            uart_print("Unable to init motor: unknown identity: ");
            uart_println_int(identity);
            break;
    }

    // Init current sense
    //curr_sense_init();

    // Init gate driver
    gate_driver_init();

    // The capstan runs open-loop with no actively-used encoder, so skip all
    // encoder SPI setup (init + sample timer) for it.
    if (identity != MOTOR_IDENT_CAPSTAN) {
        // Init encoder SPI
        spi_async_init();

        // Take ownership of the encoder sample timer. It runs continuously from
        // boot; motor_enable/disable mask it briefly to keep SERCOM3 exclusive
        // while talking to the gate driver.
        timer_schedule(TIMER_ID_SPI_ENCODER, FREQ_SPI_ENCODER,
                       PRIO_SPI_ENCODER, encoder_spi_callback);
    }
}

float motor_get_position() {
    uint16_t raw_result = spi_async_get_safe_result();
    uint16_t result = raw_result & 0x3FFF;
	float revolution_fraction = 2.0f * PI * ((float) result / ((float) 0x3FFF));
	return revolution_fraction;
}

float motor_get_pole_position() {
	float theta = motor_get_position();
	theta -= config->offset;
	theta *= (float) config->poles;
	while (theta < 0) theta += 2*PI;
	while (theta > 2*PI) theta -= 2*PI;
	return theta;
}

float motor_get_pole_pos_from_theta(float theta) {
	theta -= config->offset;
	theta *= (float) config->poles;
	while (theta < 0) theta += 2*PI;
	while (theta > 2*PI) theta -= 2*PI;
	return theta;
}

void motor_set_torque(float torque, float pole_position) {
    // Scale down torque + saturate
    if (torque > 1.0f) torque = 1.0f;
    if (torque < -1.0f) torque = -1.0f;
    torque *= config->max_torque;

    // Get PWM values
    float a = 0;
    float b = 0;
    float c = 0;
    float d = 0;
    float q = torque;

    //gpio_set_pin(DEBUG_PIN);
    foc_get_duties(pole_position, d, q, &a, &b, &c);
    //gpio_clear_pin(DEBUG_PIN);

    // Convert to integers
    uint8_t a_int = (int) (255.0f * a);
    uint8_t b_int = (int) (255.0f * b);
    uint8_t c_int = (int) (255.0f * c);

    // Write them to motor driver
    gate_driver_set_pwm(a_int, b_int, c_int);
}

void motor_set_align(float torque, float pole_position) {
    // Scale down torque + saturate
    if (torque > 1.0f) torque = 1.0f;
    if (torque < -1.0f) torque = -1.0f;
    torque *= config->max_torque;

    // Get PWM values
    float a = 0;
    float b = 0;
    float c = 0;
    float d = torque;
    float q = 0;

    //gpio_set_pin(DEBUG_PIN);
    foc_get_duties(pole_position, d, q, &a, &b, &c);
    //gpio_clear_pin(DEBUG_PIN);

    // Convert to integers
    uint8_t a_int = (int) (255.0f * a);
    uint8_t b_int = (int) (255.0f * b);
    uint8_t c_int = (int) (255.0f * c);

    // Write them to motor driver
    gate_driver_set_pwm(a_int, b_int, c_int);
}

void motor_energize_coils(float a, float b, float c) {
	// Convert to integers
	uint8_t a_int = (int) (255.0f * a);
	uint8_t b_int = (int) (255.0f * b);
	uint8_t c_int = (int) (255.0f * c);
	
	// Write them to motor driver
	gate_driver_set_pwm(a_int, b_int, c_int);
}

void motor_write_reg(uint8_t address, uint16_t data) {
    gate_driver_write_reg(address, data);
}

uint16_t motor_read_reg(uint8_t address) {
    return gate_driver_read_reg(address);
}

void motor_print_reg(uint8_t address, char* name) {
    uint16_t data = motor_read_reg(address);
    uart_print(name);
    uart_print(": ");
    uart_println_int_base(data, 2);
}

bool motor_enable() {
    // SERCOM3 is shared with the encoder SPI. Mask the 1700 Hz encoder timer
    // (TC1 == TIMER_ID_SPI_ENCODER) and drain any in-flight encoder read
    // before driving the gate driver bus, otherwise the encoder ISR can
    // preempt mid-transaction and leave spi_busy=true forever.
    NVIC_DisableIRQ(TC1_IRQn);
    while (spi_async_is_busy()) { }
    bool spi_ok = gate_driver_enable();
    NVIC_EnableIRQ(TC1_IRQn);
    return spi_ok;
}

void motor_disable() {
    // gate_driver_disable is GPIO-only today, but keep the exclusion
    // symmetric so future SPI on this path stays safe.
    NVIC_DisableIRQ(TC1_IRQn);
    while (spi_async_is_busy()) { }
    gate_driver_disable();
    NVIC_EnableIRQ(TC1_IRQn);
}

void motor_set_high_z() {
    gate_driver_set_high_z();
}

void motor_calibrate_encoder() {
    const int iterations = 4000;
    float alpha = (1.0f / 100.0f);
    config->offset = 0.0f;

    motor_energize_coils(0.15f, 0.0f, 0.0f);
    delay(0xFFFFF);
    float avg = motor_get_position();
    
    for (int i = 0; i < iterations; i++) {
        //uart_println_int(i+1);
        float position = motor_get_position();
        avg = avg + (alpha*(position - avg));
        uart_println_float(avg);
    }
    config->offset = avg;
    motor_energize_coils(0.0f, 0.0f, 0.0f);

    // Persist the freshly-measured offset so later boots can restore it via
    // motor_get_calibration() instead of re-running this (motion-inducing) sweep.
    seeprom_write_float(cal_slot(), config->offset);
}

bool motor_get_calibration() {
    // Restore the encoder offset previously saved by motor_calibrate_encoder().
    // An erased / never-written SmartEEPROM cell reads back as 0xFFFFFFFF, which
    // as a float is NaN -- detect that and keep the compiled-in default instead.
    float stored = seeprom_read_float(cal_slot());
    if (stored != stored) {   // NaN => slot never written
        return false;
    }
    config->offset = stored;
    return true;
}

void motor_test_calibration() {
    while (true) {
        motor_energize_coils(0.15f, 0.0f, 0.0f);
        float pos = motor_get_pole_position();
       uart_println_float(pos); 
    }
}

uint8_t motor_get_identity() {
    return identity;
}

