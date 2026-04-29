#pragma once
#include <stdint.h>
#include <stdbool.h>

#define MOTOR_COMMS_MAX_DATA 32
#define MOTOR_COMMS_BROADCAST_ADDR (0x55)

typedef enum {
    RX_ERR_OK,
    RX_ERR_NO_DATA,
} RXError;

typedef struct {
    RXError err;
    uint8_t data[MOTOR_COMMS_MAX_DATA];
    uint8_t data_len;
} MotorCommsRxResult;

void motor_comms_print_error(RXError err);
void motor_comms_println_error(RXError err);
void motor_comms_init(uint8_t self_addr);
void motor_comms_send_data(const uint8_t *data, uint8_t length);
void motor_comms_send_byte(uint8_t byte);
void motor_comms_send_float(uint8_t addr, const uint8_t command, float data);
float motor_comms_data_to_float(uint8_t* data);

// Returns the most recent fully-parsed frame's outcome, or { .err = RX_ERR_NO_DATA }
// if no result is pending. Consuming the result clears it. Frames addressed to
// other slaves, oversized frames, and checksum failures are dropped silently.
MotorCommsRxResult motor_comms_get_data(void);

// Fraction of received frames (addressed to us) that passed the checksum.
// Returns 1.0f if no frames have been seen yet.
float motor_comms_checksum_success_rate(void);
