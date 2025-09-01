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
    motor_init();

    while (true) {
        float pos = motor_get_pole_position();
        uart_print("POS: ");
        uart_println_float(pos);
    }
}

void foc_loop() {
    gpio_set_pin(PIN_DEBUG1);
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    float pole_position = motor_get_pole_position();
    __set_PRIMASK(primask);
    motor_set_torque(0.4, pole_position);
    gpio_clear_pin(PIN_DEBUG1);
}

void current_printer() {
    static float time = 0.0f;
    time += 0.02f;
    float currents[4];
    curr_sense_get_values(&currents[1], &currents[2], &currents[3]);
    currents[0] = time;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uart_println_float_arr(currents, 4);
    __set_PRIMASK(primask);
}

static void motor_test() {
    uart_println("Starting motor test");
    motor_init();
    motor_enable();
    delay(0xFFF);
    uart_put('\n');
    uart_put('\n');
    //motor_calibrate_encoder();
    gate_driver_set_idrive(0b111, 0b111, 0b111, 0b111);
    motor_print_reg(DRV_REG_DRIVER_CONTROL, "Control");
    motor_print_reg(DRV_REG_FAULT_STATUS_1, "Fault1");
    motor_print_reg(DRV_REG_FAULT_STATUS_2, "Fault2");
    motor_print_reg(DRV_REG_GATE_DRIVER_HS, "DriverHS");
    motor_print_reg(DRV_REG_GATE_DRIVER_LS, "DriverLS");
    //motor_energize_coils(0.2f, 0.3f, 0.0f);
    
    // Schedule FOC loop
    timer_schedule(0, 5000, 6, foc_loop);
    timer_schedule(1, 100, 7, current_printer);
}

int main(void) {
	init_peripherals();
    delay(0xFFFF);
    //delay(0xFFFFF);
	//print_welcome();
    //timer_schedule(1, 500.0f, timer_test);
    //delay(0x4FFF);
    //uart_println("\nStarting Controller Test");
    //controller_test();
    //delay(0x4FFF);
    //uart_println("\nStarting Encoder Test");
    //encoder_test();

    //encoder_test();
    motor_test();

	while (1) {
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

