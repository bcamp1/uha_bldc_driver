/*
 * uha_motor_driver.h
 *
 * Created: 3/9/2025 4:24:16 PM
 *  Author: brans
 */ 

#pragma once
#include <stdint.h>

// Register Addresses
#define DRV_REG_FAULT_STATUS_1	(0)
#define DRV_REG_FAULT_STATUS_2	(1)
#define DRV_REG_DRIVER_CONTROL	(2)
#define DRV_REG_GATE_DRIVER_HS	(3)
#define DRV_REG_GATE_DRIVER_LS	(4)
#define DRV_REG_OCP_CONTROL		(5)
#define DRV_REG_CSA_CONTROL		(6)

void gate_driver_init();
void gate_driver_write_reg(uint8_t address, uint16_t data);
void gate_driver_set_3x();
void gate_driver_set_pwm(uint8_t a, uint8_t b, uint8_t c);
void gate_driver_set_high_z();
void gate_driver_goto_theta(float theta);
void gate_driver_enable();
void gate_driver_disable();
void gate_driver_toggle();
uint16_t gate_driver_read_reg(uint8_t address);

