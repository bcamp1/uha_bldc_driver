#include "command_center.h"
#include "../periphs/timer.h"
#include "../board.h"
#include "motor.h"
#include "motor_comms_slave.h"

MotorIdentity identity = MOTOR_IDENT_UNKNOWN;

#define MOTOR_COMMS_CMD_DISABLE (0x0)
#define MOTOR_COMMS_CMD_ENABLE (0x1)
#define MOTOR_COMMS_CMD_FAULT_STATUS (0x2)
#define MOTOR_COMMS_CMD_CALIB_ENCODER (0x3)
#define MOTOR_COMMS_CMD_CAPSTAN_15IPS (0x4)
#define MOTOR_COMMS_CMD_CAPSTAN_30IPS (0x5)
#define MOTOR_COMMS_CMD_CAPSTAN_7P5IPS (0x6)
#define MOTOR_COMMS_CMD_REEL_TORQUE (0x7)
#define MOTOR_COMMS_CMD_BAD_REQUEST (0x8)

static float commanded_torque = 0.0f;
static CapstanSetting capstan_setting = CAPSTAN_CMD_OFF;
static uint8_t fault_status = 0;

static CommandCenterCb command_cb = 0;

static void safe_cb(CommandCenterCmd cmd) {
    if (command_cb != 0) {
        command_cb(cmd);
    }
}

// INT16_MAX -> +1.0f, INT16_MIN -> -1.0f. Inverse of torque_float_to_int16.
static float torque_int16_to_float(int16_t raw) {
    if (raw >= 0) return (float)raw / 32767.0f;
    return (float)raw / 32768.0f;
}

static int16_t bytes_to_int16(uint8_t msb, uint8_t lsb) {
    return (int16_t)(((uint16_t)msb << 8) | lsb);
}

static void command_center_tick() {
    MotorCommsRxResult rx = motor_comms_get_data();
    if (rx.err != RX_ERR_OK || rx.data_len < 1) {
        return;
    }

    uint8_t command = rx.data[0];
    uint8_t response[4];

    switch (command) {
        case MOTOR_COMMS_CMD_DISABLE:
            if (!rx.broadcast) {
                motor_comms_send_byte(MOTOR_COMMS_CMD_DISABLE);
            }
            safe_cb(CMD_DISABLE);
            break;
        case MOTOR_COMMS_CMD_ENABLE:
            if (!rx.broadcast) {
                motor_comms_send_byte(MOTOR_COMMS_CMD_ENABLE);
            }
            safe_cb(CMD_ENABLE);
            break;
        case MOTOR_COMMS_CMD_FAULT_STATUS:
            if (!rx.broadcast) {
                response[0] = MOTOR_COMMS_CMD_FAULT_STATUS;
                response[1] = fault_status;
                motor_comms_send_data(response, 2);
            }
            break;
        case MOTOR_COMMS_CMD_CALIB_ENCODER:
            if (!rx.broadcast) {
                motor_comms_send_byte(MOTOR_COMMS_CMD_CALIB_ENCODER);
            }
            safe_cb(CMD_CALIB_ENCODER);
            break;
        case MOTOR_COMMS_CMD_CAPSTAN_15IPS:
            if (identity != MOTOR_IDENT_CAPSTAN) return;
            if (!rx.broadcast) {
                motor_comms_send_byte(command);
            }
            capstan_setting = CAPSTAN_CMD_15IPS;
            safe_cb(CMD_CAPSTAN_SETTING);
            break;
        case MOTOR_COMMS_CMD_CAPSTAN_30IPS:
            if (identity != MOTOR_IDENT_CAPSTAN) return;
            if (!rx.broadcast) {
                motor_comms_send_byte(command);
            }
            capstan_setting = CAPSTAN_CMD_30IPS;
            safe_cb(CMD_CAPSTAN_SETTING);
            break;
        case MOTOR_COMMS_CMD_CAPSTAN_7P5IPS:
            if (identity != MOTOR_IDENT_CAPSTAN) return;
            if (!rx.broadcast) {
                motor_comms_send_byte(command);
            }
            capstan_setting = CAPSTAN_CMD_7P5IPS;
            safe_cb(CMD_CAPSTAN_SETTING);
            break;
        case MOTOR_COMMS_CMD_REEL_TORQUE:
            if (!(identity == MOTOR_IDENT_TAKEUP || identity == MOTOR_IDENT_SUPPLY)) return;
            uint8_t msb = 0;
            uint8_t lsb = 0;
            if (rx.broadcast) {
                if (rx.data_len != 5) return;
                if (identity == MOTOR_IDENT_TAKEUP) {
                    msb = rx.data[1];
                    lsb = rx.data[2];
                } else {
                    msb = rx.data[3];
                    lsb = rx.data[4];
                }
            } else {
                if (rx.data_len != 3) {
                    motor_comms_send_byte(MOTOR_COMMS_CMD_BAD_REQUEST);
                    return;
                }
                msb = rx.data[1];
                lsb = rx.data[2];
            } 
            int16_t torque_int = bytes_to_int16(msb, lsb);
            commanded_torque = torque_int16_to_float(torque_int);
            safe_cb(CMD_TORQUE_UPDATE);
            break;
        default:
            break;
    }
    
}

void command_center_init(MotorIdentity i) {
    identity = i;
    timer_schedule(TIMER_ID_COMMAND_CENTER, FREQ_COMMAND_CENTER, PRIO_COMMAND_CENTER, command_center_tick); 
}

void command_center_register_cb(CommandCenterCb cb) {
    command_cb = cb;
}

