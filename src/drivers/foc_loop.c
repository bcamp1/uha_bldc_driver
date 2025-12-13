#include "foc_loop.h"
#include "motor.h"
#include "spi_slave.h"
#include "../periphs/eic.h"
#include "../periphs/delay.h"
#include "../periphs/spi_async.h"
#include "../periphs/uart.h"
#include "../periphs/gpio.h"
#include "../periphs/timer.h"
#include "../board.h"
#include <sam.h>
#include <stddef.h>

#define CAPSTAN_POLE_FREQ       (285.805f)
//#define CAPSTAN_POLE_FREQ       (50.805f)
#define CAPSTAN_SAMPLE_RATE     (10000.0f)
#define CAPSTAN_MOTOR_STRENGTH  (0.8f)
#define TWOPI (6.28318f)

static void enable_callback();
static void initialize_motor_module();
static void deinitialize_motor_module();
static void foc_loop();
static void foc_loop_capstan();

static float foc_loop_freq = 1004.0f;

static float curr_pos = 0.0f;
static float prev_pos = 0.0f;
static float speed = 0.0f;

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

static void enable_callback() {
    if (gpio_get_pin(PIN_ENABLE)) {
        // Module has been enabled. Initialize everything and begin.    
        initialize_motor_module();
    } else {
        // Module has been shut down. Uninitialize anything here.
        deinitialize_motor_module();
    }
}

static void initialize_motor_module() {
    //motor_init_from_ident();
    //motor_enable();
    delay(0xFFF);
    uart_put('\n');
    uart_put('\n');
    //motor_calibrate_encoder();
    //gate_driver_set_idrive(0b111, 0b111, 0b111, 0b111);
    //motor_print_reg(DRV_REG_DRIVER_CONTROL, "Control");
    //motor_print_reg(DRV_REG_FAULT_STATUS_1, "Fault1");
    //motor_print_reg(DRV_REG_FAULT_STATUS_2, "Fault2");
    //motor_print_reg(DRV_REG_GATE_DRIVER_HS, "DriverHS");
    //motor_print_reg(DRV_REG_GATE_DRIVER_LS, "DriverLS");

    // Schedule FOC loop
    if (motor_get_identity() == MOTOR_IDENT_CAPSTAN) {
        uart_println("Running capstan control loop");
        timer_schedule(0, CAPSTAN_SAMPLE_RATE, PRIO_FOC_LOOP, foc_loop_capstan);
    } else {
        uart_println("Running standard control loop");
        timer_schedule(0, foc_loop_freq, PRIO_FOC_LOOP, foc_loop);
    }
    //uart_println("Motor module enabled");
} 

static void deinitialize_motor_module() {
    //motor_disable();
    // CRITICAL: Disable interrupts to prevent race condition
    // Must stop timer BEFORE setting high-Z to prevent FOC loop
    // from re-enabling PWM after we've disabled it
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    gpio_set_pin(PIN_DEBUG2);
    timer_deschedule(0);      // Stop FOC loop FIRST
    motor_set_high_z();       // Then safe to set high-Z
    gpio_clear_pin(PIN_DEBUG2);
    __set_PRIMASK(primask);
    //uart_println("Motor module disabled");
}

static void foc_loop() {
    //gpio_set_pin(PIN_DEBUG1);
    prev_pos = curr_pos;
    curr_pos = motor_get_position();
    float pole_position = motor_get_pole_pos_from_theta(curr_pos);
    float torque = spi_slave_get_torque_command();
    //torque = -0.57f;
    // Calculate speed
    speed = foc_loop_freq * sub_angles(curr_pos, prev_pos);

    motor_set_torque(torque, pole_position);
    //gpio_clear_pin(PIN_DEBUG1);
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

void foc_loop_init() {
    eic_init_pin(PIN_ENABLE, PIN_ENABLE_EIC_INDEX, EIC_MODE_BOTH, enable_callback);

    if (gpio_get_pin(PIN_ENABLE)) {
        initialize_motor_module();
    }
}

float foc_loop_get_speed() {
    return speed;
}

