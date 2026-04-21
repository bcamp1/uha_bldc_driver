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
#include "periphs/rs485.h"
#include "drivers/motor.h"
#include "periphs/stopwatch.h"
#include "board.h"
#include "periphs/delay.h"
#include "periphs/timer.h"
#include "drivers/curr_sense.h"
#include "drivers/foc_loop.h"
#include "periphs/spi.h"
#include "periphs/spi_async.h"
#include "periphs/eic.h"
#include "drivers/motor_comms_slave.h"

#define FIRMWARE_VERSION "UHA BLDC FIRMWARE v0.1"
#define FIRMWARE_AUTHOR "AUTHOR: BRANSON CAMP"
#define FIRMWARE_DATE "DATE: OCTOBER 2025"

static void enable_fpu(void);
static void init_peripherals(void);
static void stopwatch_test();
static void encoder_spi_callback();
static uint8_t get_address();

static uint8_t self_address = 0;

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
    //eic_init();
    
    // Set hardware address
    gpio_init_pin(PIN_ADDR2, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);
    gpio_init_pin(PIN_ADDR1, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);
    gpio_init_pin(PIN_ADDR0, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);
    self_address = get_address();
}

// Gets hardware address set by dip switches
static uint8_t get_address() {
    uint8_t addr = 0;
    addr |= (!gpio_get_pin(PIN_ADDR2) << 2);
    addr |= (!gpio_get_pin(PIN_ADDR1) << 1);
    addr |= (!gpio_get_pin(PIN_ADDR0) << 0);
    return addr;
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

static void encoder_spi_callback() {
    spi_async_start_read(NULL);
}

static void encoder_test() {
    uart_println("Starting motor encoder test");
    delay(0xFFF);
    uart_put('\n');
    uart_put('\n');
    motor_init(&MOTOR_CONF_TAKEUP);
    motor_enable();
    timer_schedule(1, FREQ_SPI_ENCODER, PRIO_SPI_ENCODER, encoder_spi_callback);
    while (true) {
        float pos = motor_get_pole_position();
        uart_print("POS: ");
        uart_println_float(pos);
        delay(0x7FFF);
    }
}

static void motor_test() {
    uart_println("Starting motor test");
    delay(0xFFF);
    uart_put('\n');
    uart_put('\n');
    motor_init(&MOTOR_CONF_TAKEUP);
    timer_schedule(1, FREQ_SPI_ENCODER, PRIO_SPI_ENCODER, encoder_spi_callback);

    delay(0xFFFF);
    motor_enable();
    while (true) {
        float pos = motor_get_pole_position();
        motor_set_torque(-0.3f, pos);
        delay(0xFF);
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

int main() {
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

    rs485_init();
    uart_println("RS485 TEST MODE");
    uart_print("SLAVE ADDRESS: ");
    uart_println_int(self_address);
    motor_comms_init(self_address);

    uint16_t buf_length = 20;
    uint8_t buf[buf_length];
    uint8_t data_len = 0;

    while (true) {
        RXError err = motor_comms_get_data(buf, &data_len, buf_length);
        motor_comms_println_error(err);


        if (err == RX_ERR_OK) {
            uart_println_int_base(buf[0], 10); 
        }
        
        delay(0xFFFF);
        gpio_toggle_pin(PIN_DEBUG1);
    }
}

int min(void) {
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

    gpio_init_pin(PIN_ADDR2, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);
    gpio_init_pin(PIN_ADDR1, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);
    gpio_init_pin(PIN_ADDR0, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);

    gpio_clear_pin(PIN_DEBUG1);
    gpio_clear_pin(PIN_DEBUG2);

    //encoder_test();
    //motor_init(&MOTOR_CONF_SUPPLY);
    //

    rs485_init();
    

    uart_println("Waiting before enabling motor...");
    delay(0xFFFFF);

    do {
        motor_init_from_ident();
    } while (motor_get_identity() == MOTOR_IDENT_UNKNOWN);

    if (motor_get_identity() != MOTOR_IDENT_CAPSTAN) {
        timer_schedule(1, FREQ_SPI_ENCODER, PRIO_SPI_ENCODER, encoder_spi_callback);
        delay(0x3FFFF);
    }

    motor_enable();
    uart_println("Motor enabled.");

    timer_schedule(1, FREQ_SPI_ENCODER, PRIO_SPI_ENCODER, encoder_spi_callback);

    motor_calibrate_encoder();

    //spi_slave_init();
    foc_loop_init();
    uart_println("FOC Loop enabled.");
    
    //motor_calibrate_encoder();
    //motor_test_calibration();

    foc_loop_set_torque(-0.3f);


	while (1) {
        /*
        if (rs485_available()) {
            uint8_t byte = rs485_get();
            float byte_percent = byte / 256.0f;
            uart_println_float(byte_percent);
            foc_loop_set_torque((byte_percent * 0.6f));
        }
        */

        foc_loop_set_torque(0.4f);
        
        /*
        uint16_t fault = gate_driver_read_reg(DRV_REG_FAULT_STATUS_1);

        if (fault != 0)
            uart_println_int_base(fault, 2);
        delay(0xFFFFF);
        */

        //gpio_toggle_pin(PIN_DEBUG1);
        //gpio_toggle_pin(PIN_DEBUG2);
        //delay(0xFFFF);
        /*
        uart_print_int(spi_slave_get_hit_count());
        uart_print(" : ");
        uart_print_int(spi_slave_get_miss_count());
        uart_print(" (");
        uart_print_float(spi_slave_get_success_rate() * 100.0f);
        uart_println("%)");
        delay(0x1FFFF);
        */
	}
}

