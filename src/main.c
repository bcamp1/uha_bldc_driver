/*
 * MotorControllerBoard.c
 *
 * Created: 3/9/2025 3:26:48 PM
 * Author : brans
 */ 

#include <sam.h>
#include "drivers/uha_motor_driver.h"
#include "periphs/gpio.h"
#include "periphs/clocks.h"
#include "periphs/uart.h"
#include "drivers/motor_encoder.h"
#include "drivers/stopwatch.h"
#include "drivers/board.h"
#include "drivers/motor_unit.h"
#include "drivers/delay.h"

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
    motor_unit_init(&MOTOR_UNIT_CONF);

    while (true) {
        gpio_set_pin(DBG1_PIN);
        float pos = motor_encoder_get_pole_position(&MOTOR_ENCODER_CONF);
        motor_unit_energize_coils(&MOTOR_UNIT_CONF, 1.0f, 0.5f, 0.25f);
        gpio_clear_pin(DBG1_PIN);
        uart_println_float(pos);
    }
}


static void motor_test() {
    uha_motor_driver_init(&UHA_MTR_DRVR_CONF);
    delay(0xFFFFF);
    //motor_unit_energize_coils(&MOTOR_UNIT_CONF, 0.0f, 0.0f, 0.0f);

    int i = 0;
    while (true) {
        i++;
        gpio_set_pin(DBG1_PIN);
        int status1 = uha_motor_driver_read_reg(&UHA_MTR_DRVR_CONF, DRV_REG_DRIVER_CONTROL);
        gpio_clear_pin(DBG1_PIN);
        //int status2 = uha_motor_driver_read_reg(&UHA_MTR_DRVR_CONF, DRV_REG_FAULT_STATUS_2);
        uart_print("FAULT1: ");
        uart_print_int_base(status1 & 0x3FF, 2);
        uart_print(" ");
        uart_print_int(i);
        //uart_print(", FAULT2: ");
        //uart_print_int_base(status2, 2);
        uart_println(" ");
        if (i > 10000) {
            motor_unit_energize_coils(&MOTOR_UNIT_CONF, 0.0f, 0.0f, 0.0f);
        }
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
    //gpio_set_pin(DBG2_PIN);
    encoder_test();
    //motor_test();

	while (1) {
        delay(0xFFFF);
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

