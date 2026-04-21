#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    RX_ERR_OK,
    RX_ERR_TIMEOUT,
    RX_ERR_WRONG_ADDR,
    RX_ERR_BUF_OVF,
    RX_ERR_WRONG_CHECKSUM,
    RX_ERR_NO_DATA,
} RXError;

void motor_comms_print_error(RXError err);
void motor_comms_println_error(RXError err);
void motor_comms_init(uint8_t self_addr);
void motor_comms_send_bytes(uint8_t addr, const uint8_t *data, uint8_t length);
void motor_comms_send_cmd(uint8_t addr, uint8_t cmd);
void motor_comms_send_float(uint8_t addr, const uint8_t command, float data);
float motor_comms_data_to_float(uint8_t* data);

RXError motor_comms_get_data(uint8_t* data, uint8_t* data_len, uint8_t buf_size);

