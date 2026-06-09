#pragma once
#include "motor.h"

typedef enum {
    CAPSTAN_CMD_15IPS,
    CAPSTAN_CMD_30IPS,
    CAPSTAN_CMD_7P5IPS,
    CAPSTAN_CMD_OFF,
    CAPSTAN_CMD_OTHER,
} CapstanSetting;

typedef enum {
    CMD_ENABLE,
    CMD_DISABLE,
    CMD_CALIB_ENCODER,
    CMD_CAPSTAN_SETTING,
    CMD_TORQUE_UPDATE,
} CommandCenterCmd;

typedef void (*CommandCenterCb)(CommandCenterCmd cmd);

void command_center_init(MotorIdentity i);
void command_center_register_cb(CommandCenterCb cb);

float command_center_get_torque();
CapstanSetting command_center_get_capstan_setting();
// Raw DRV8323 FAULT_STATUS_1/2 registers (hardware faults).
void command_center_set_fault_registers(uint16_t fs1, uint16_t fs2);

// Meta-fault register: firmware/system-level faults detected by this firmware,
// as opposed to the DRV's own FAULT_STATUS registers. Bit masks below; one
// uint8_t (up to 8 bits). Set/clear individual bits as conditions appear/clear.
#define META_FAULT_GATE_SPI     (1u << 0)  // gate-driver SPI not responding / readback mismatch
#define META_FAULT_ENCODER_SPI  (1u << 1)  // encoder SPI not delivering valid reads
#define META_FAULT_DRV          (1u << 2)  // DRV reported a hardware fault (see FAULT_STATUS regs)

void command_center_set_meta_fault(uint8_t mask);    // OR-set the given bits
void command_center_clear_meta_fault(uint8_t mask);  // clear the given bits
uint8_t command_center_get_meta_fault(void);


