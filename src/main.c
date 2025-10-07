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
#include "drivers/foc_loop.h"
#include "drivers/pwm_capture.h"
#include "drivers/spi_slave.h"
#include "periphs/spi.h"
#include "periphs/eic.h"

#define FIRMWARE_VERSION "UHA BLDC FIRMWARE v0.1"
#define FIRMWARE_AUTHOR "AUTHOR: BRANSON CAMP"
#define FIRMWARE_DATE "DATE: OCTOBER 2025"

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
    gpio_init_pin(PIN_DEBUG1, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
    gpio_init_pin(PIN_DEBUG2, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);

    gpio_clear_pin(PIN_DEBUG1);
    gpio_clear_pin(PIN_DEBUG2);

    delay(0x3FFFF);
    
    do {
        motor_init_from_ident();
    } while (motor_get_identity() == MOTOR_IDENT_UNKNOWN);

    motor_enable();
    spi_slave_init();
    foc_loop_init();
    
    //motor_calibrate_encoder();

	while (1) {
        if (gpio_get_pin(PIN_ENABLE)) {
            uart_println_int_base(spi_slave_get_torque_command_uint(), 16);
            delay(0x4FFF);
        }
	}
}

