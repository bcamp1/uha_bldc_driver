#include "foc_loop.h"
#include "motor.h"
#include "../periphs/eic.h"
#include "../periphs/delay.h"
#include "../periphs/spi_async.h"
#include "../periphs/uart.h"
#include "../periphs/gpio.h"
#include "../periphs/timer.h"
#include "../board.h"
#include <sam.h>
#include <stddef.h>
#include <stdbool.h>

#define CAPSTAN_POLE_FREQ       (285.805f)
//#define CAPSTAN_POLE_FREQ       (50.805f)
#define CAPSTAN_SAMPLE_RATE     (10000.0f)
#define CAPSTAN_MOTOR_STRENGTH  (0.8f)
#define TWOPI (6.28318f)

static void schedule_foc_timer();
static void foc_loop();
static void foc_loop_capstan();

static float curr_pos = 0.0f;
static float prev_pos = 0.0f;
static float speed = 0.0f;

// Written from RS485 RX ISR context (via command_center_cb → foc_loop_set_torque)
// and read from the FOC ISR. volatile prevents the compiler from caching or
// hoisting the read across the ISR boundary.
static volatile float torque_command = 0.0f;

static bool foc_loop_running = false;

static float sub_angles(float x, float y) {
    float a = x - y;
    float b = (x + TWOPI) - y;
    float c = x - (y + TWOPI);

    float abs_a = a;
    float abs_b = b;
    float abs_c = c;

    if (abs_a < 0.0f) abs_a = -abs_a;
    if (abs_b < 0.0f) abs_b = -abs_b;
    if (abs_c < 0.0f) abs_c = -abs_c;

    if (abs_a <= abs_b && abs_a <= abs_c) {
        return a;
    } else if (abs_b <= abs_a && abs_b <= abs_c) {
        return b;
    } else {
        return c;
    }
}

static void schedule_foc_timer() {
    if (motor_get_identity() == MOTOR_IDENT_CAPSTAN) {
        timer_schedule(TIMER_ID_FOC_LOOP, CAPSTAN_SAMPLE_RATE, PRIO_FOC_LOOP, foc_loop_capstan);
    } else {
        timer_schedule(TIMER_ID_FOC_LOOP, FREQ_FOC_LOOP, PRIO_FOC_LOOP, foc_loop);
    }
}

static void foc_loop() {
    gpio_set_pin(PIN_DEBUG1);
    prev_pos = curr_pos;
    curr_pos = motor_get_position();
    float pole_position = motor_get_pole_pos_from_theta(curr_pos);
    float torque = torque_command;

    // Calculate speed
    //speed = foc_loop_freq * sub_angles(curr_pos, prev_pos);

    motor_set_torque(torque, pole_position);
    //motor_set_torque(0.7f, pole_position);
    gpio_clear_pin(PIN_DEBUG1);
}

static float capstan_theta = 0.0f;

static void foc_loop_capstan() {
    float theta_increment = CAPSTAN_POLE_FREQ / CAPSTAN_SAMPLE_RATE;
    capstan_theta += theta_increment;
    if (capstan_theta > 6.28318f) {
        capstan_theta -= 6.28318f;
    }
    //float pole_position = motor_get_pole_position();
    float torque = CAPSTAN_MOTOR_STRENGTH;
    motor_set_align(torque, capstan_theta);
}

//#define INSTA_ENABLE

void foc_loop_init() {
    // One-shot HW config. The FOC timer is NOT started here — callers must
    // invoke foc_loop_start() (typically via main()'s enable reconcile).
    gate_driver_set_idrive(0b111, 0b111, 0b111, 0b111);
}

void foc_loop_start(void) {
    if (foc_loop_running) return;
    schedule_foc_timer();
    foc_loop_running = true;
}

void foc_loop_stop(void) {
    if (!foc_loop_running) return;
    // Must stop timer BEFORE high-Z so a pending FOC tick can't re-arm PWM.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    // DEBUG2 is now used to trace the 1700 Hz encoder read (spi_async.c);
    // disabled here so it doesn't inject a spurious pulse into that signal.
    //gpio_set_pin(PIN_DEBUG2);
    timer_deschedule(TIMER_ID_FOC_LOOP);
    motor_set_high_z();
    //gpio_clear_pin(PIN_DEBUG2);
    __set_PRIMASK(primask);
    foc_loop_running = false;
}

float foc_loop_get_speed() {
    return speed;
}

void foc_loop_set_torque(float torque) {
    torque_command = torque;
}

