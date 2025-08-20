/*
 * MotorControllerBoard.c
 *
 * Created: 3/9/2025 3:26:48 PM
 * Author : brans
 */ 

#include <sam.h>
#include "periphs/gpio.h"
#include "periphs/clocks.h"
#include "periphs/uart.h"
#include "drivers/motor_encoder.h"
#include "drivers/tension_arm.h"
#include "drivers/stopwatch.h"
#include "drivers/board.h"
#include "drivers/uha_motor_driver.h"
#include "drivers/motor_encoder.h"

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
	//gpio_init_pin(LED_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(DBG1_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(DBG2_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	
	// Init the UART
	uart_init();
    
    // Stop Watch
    stopwatch_init();
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
    motor_encoder_init(&MOTOR_ENCODER_CONF);

    while (true) {
        gpio_set_pin(DBG1_PIN);
        float pos = motor_encoder_get_position(&MOTOR_ENCODER_CONF);
        gpio_clear_pin(DBG1_PIN);
        uart_println_float(pos);
    }
}

int main(void) {
	init_peripherals();
	//print_welcome();
    //timer_schedule(1, 500.0f, timer_test);
    //delay(0x4FFF);
    //uart_println("\nStarting Controller Test");
    //controller_test();
    //delay(0x4FFF);
    //uart_println("\nStarting Encoder Test");
    //encoder_test();

    gpio_set_pin(DBG1_PIN);
    gpio_set_pin(DBG2_PIN);
    encoder_test();

	while (1) {
        //delay(0x4FF);
        //for (int i = 0; i < 0xFFFFF; i++) {}
        //gpio_toggle_pin(DBG1_PIN);
        //gpio_toggle_pin(DBG2_PIN);
        //uart_println("Hello world");
		//float ips = roller_get_ips();
		//float tape_pos = roller_get_tape_position(15.0f);
        //float data[2] = {ips, tape_pos};
        //uart_println_float_arr(data, 2);
	}
}

