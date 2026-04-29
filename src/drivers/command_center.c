#include "command_center.h"
#include "../periphs/timer.h"
#include "../board.h"
#include "motor.h"
#include "motor_comms_slave.h"

MotorIdentity identity = MOTOR_IDENT_UNKNOWN;

static void command_center_tick() {
    MotorCommsRxResult rx = motor_comms_get_data();
    if (rx.err != RX_ERR_OK) {
        return;
    }
}

void command_center_init() {
    timer_schedule(TIMER_ID_COMMAND_CENTER, FREQ_COMMAND_CENTER, PRIO_COMMAND_CENTER, command_center_tick); 
}

