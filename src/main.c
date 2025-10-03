/*
 * MotorControllerBoard.c
 *
 * Created: 3/9/2025 3:26:48 PM
 * Author : brans
 */ 

#include <sam.h>
#include "drivers/gate_driver.h"
#include "periphs/gpio.h"
#include "periphs/clocks.h"
#include "periphs/uart.h"
#include "drivers/motor.h"
#include "periphs/stopwatch.h"
#include "board.h"
#include "periphs/delay.h"
#include "periphs/timer.h"
#include "drivers/curr_sense.h"
#include "drivers/pwm_capture.h"
#include "drivers/spi_slave.h"
#include "periphs/spi.h"
#include "periphs/eic.h"

#define FIRMWARE_VERSION "UHA BLDC FIRMWARE v0.1"
#define FIRMWARE_AUTHOR "AUTHOR: BRANSON CAMP"
#define FIRMWARE_DATE "DATE: OCTOBER 2025"

#define CAPSTAN_POLE_FREQ       (285.805f)
#define CAPSTAN_SAMPLE_RATE     (20000.0f)
#define CAPSTAN_MOTOR_STRENGTH  (1.0f)

static void enable_fpu(void);
static void init_peripherals(void);
static void stopwatch_test();

static void init_peripherals(void) {
	// Init clock to use 32K OSC in closed-loop 48MHz mode
    // Then its boosted to 120MHz
	wntr_system_clocks_init();
	
	// Enable floating-point unit
	enable_fpu();

	// Init useful debugging GPIO pins
	gpio_init_pin(PIN_DEBUG1, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(PIN_DEBUG2, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	
	// Init the UART
	uart_init();
    
    // Stop Watch
    stopwatch_init();

    // EIC
    eic_init();
}

static void enable_fpu(void) {
	// Enable CP10 and CP11 (FPU coprocessors)
	SCB->CPACR |= (0xF << 20);
	// Ensure all memory accesses complete before continuing
	__DSB();  // Data Synchronization Barrier
	__ISB();  // Instruction Synchronization Barrier
}

static void stopwatch_test() {
    stopwatch_init();
    bool running = false;
    uart_println("UART Stopwatch Test (press s)");
    
    while (1) {
        //uint32_t time = stopwatch_underlying_time();
        //uart_println_int_base(time, 16);
        if (uart_get() == 's') {
            if (!running) {
                uart_println("Starting stopwatch... s to stop");
                running = true;
                stopwatch_start(0);
            } else {
                float dt = ticks_to_time(stopwatch_stop(0, false));
                running = false;
                uart_print("Stopped. Took ");
                uart_print_float(dt);
                uart_println(" seconds.");
            }
        }
    }
}

static void encoder_test() {
    uart_println("Starting motor encoder test");
    delay(0xFFF);
    uart_put('\n');
    uart_put('\n');
    motor_init(&MOTOR_CONF_SUPPLY);
    motor_enable();
    while (true) {
  float pos = motor_get_pole_position();
        uart_print("POS: ");
        uart_println_float(pos);
    }
}

void foc_loop() {
    float pole_position = motor_get_pole_position();
    float torque = spi_slave_get_torque_command();
    //torque = 0.4f;
    //uart_println_float(pole_position);
    //motor_set_torque(torque, pole_position);
}

static float capstan_theta = 0.0f;

static void foc_loop_capstan() {
    gpio_set_pin(PIN_DEBUG2);
    float theta_increment = CAPSTAN_POLE_FREQ / CAPSTAN_SAMPLE_RATE;
    capstan_theta += theta_increment;
    if (capstan_theta > 6.28318f) {
        capstan_theta -= 6.28318f;
    }
    //float pole_position = motor_get_pole_position();
    float torque = CAPSTAN_MOTOR_STRENGTH;
    //motor_set_align(torque, capstan_theta);
    gpio_clear_pin(PIN_DEBUG2);
}

void current_printer() {
    static float time = 0.0f;
    time += 0.01f;
    float currents[4];
    curr_sense_get_values(&currents[1], &currents[2], &currents[3]);
    currents[0] = time;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uart_println_float_arr(currents, 4);
    __set_PRIMASK(primask);
}

static void initialize_motor_module() {
    uart_println("Motor module enabled");
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
    gpio_set_pin(PIN_DEBUG1);

    // Schedule FOC loop
    if (motor_get_identity() == MOTOR_IDENT_CAPSTAN) {
        uart_println("Running capstan control loop");
        timer_schedule(0, CAPSTAN_SAMPLE_RATE, 1, foc_loop_capstan);
    } else {
        uart_println("Running standard control loop");
        timer_schedule(0, 2000, 1, foc_loop);
    }
} 

static void deinitialize_motor_module() {
    uart_println("Motor module disabled");
    //motor_disable();
    motor_set_high_z();
    gpio_clear_pin(PIN_DEBUG1);

    // Unschedule FOC loop
    timer_deschedule(0);
}

void enable_callback() {
    if (gpio_get_pin(PIN_ENABLE)) {
        // Module has been enabled. Initialize everything and begin.    
        initialize_motor_module();
    } else {
        // Module has been shut down. Uninitialize anything here.
        deinitialize_motor_module();
    }
}

int main(void) {
	init_peripherals();
    delay(0xFFF);

    // Print firmware info
    uart_println("\n");
    uart_println("--------------------");
    uart_println(FIRMWARE_VERSION);
    uart_println(FIRMWARE_AUTHOR);
    uart_println(FIRMWARE_DATE);
    uart_println("--------------------");
    delay(0xFFFF);

	gpio_init_pin(PIN_DEBUG1, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(PIN_DEBUG2, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);

    gpio_clear_pin(PIN_DEBUG1);
    gpio_clear_pin(PIN_DEBUG2);

    motor_init_from_ident();
    motor_enable();
    //motor_calibrate_encoder();
    spi_slave_init();
    eic_init_pin(PIN_ENABLE, PIN_ENABLE_EIC_INDEX, EIC_MODE_BOTH, enable_callback);
    //initialize_motor_module();

    if (gpio_get_pin(PIN_ENABLE)) {
        initialize_motor_module();
    }

	while (1) {
        if (gpio_get_pin(PIN_ENABLE)) {
            uart_println_int_base(spi_slave_get_torque_command_uint(), 16);
            delay(0x4FFF);
        }
        //uart_println_float(spi_slave_get_torque_command());
        //uart_println("HELLO");
	}
}

