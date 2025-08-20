/*
 * uha_motor_driver.h
 *
 * Created: 3/9/2025 4:24:16 PM
 *  Author: brans
 */ 

#ifndef UHA_MOTOR_DRIVER_H_
#define UHA_MOTOR_DRIVER_H_

#include <stdint.h>
#include "sam.h"
#include "../periphs/pwm.h"
#include "../periphs/spi.h"

// Register Addresses
#define DRV_REG_FAULT_STATUS_1	(0)
#define DRV_REG_FAULT_STATUS_2	(1)
#define DRV_REG_DRIVER_CONTROL	(2)
#define DRV_REG_GATE_DRIVER_HS	(3)
#define DRV_REG_GATE_DRIVER_LS	(4)
#define DRV_REG_OCP_CONTROL		(5)
#define DRV_REG_CSA_CONTROL		(6)

typedef struct {
	const PWMConfig* pwm;
	const SPIConfig* spi;

	// Non-SPI pins
	uint8_t soa;
	uint8_t sob;
	uint8_t soc;
	uint8_t n_fault;
	uint8_t en;
	uint8_t cal;
	uint8_t inl;
	
} UHAMotorDriverConfig;

extern const UHAMotorDriverConfig UHA_MTR_DRVR_CONF;

void uha_motor_driver_init(const UHAMotorDriverConfig* config);
void uha_motor_driver_write_reg(const UHAMotorDriverConfig* config, uint8_t address, uint16_t data);
uint16_t uha_motor_driver_read_reg(const UHAMotorDriverConfig* config, uint8_t address);
void uha_motor_driver_set_3x(const UHAMotorDriverConfig* config);
void uha_motor_driver_set_pwm(const UHAMotorDriverConfig* config, uint8_t a, uint8_t b, uint8_t c);
void uha_motor_driver_set_high_z(const UHAMotorDriverConfig* config);
void uha_motor_driver_goto_theta(const UHAMotorDriverConfig* config, float theta);
void uha_motor_driver_enable(const UHAMotorDriverConfig* config);
void uha_motor_driver_disable(const UHAMotorDriverConfig* config);
void uha_motor_driver_toggle(const UHAMotorDriverConfig* config);

#endif /* UHA_MOTOR_DRIVER_H_ */
