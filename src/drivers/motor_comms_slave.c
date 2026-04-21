#include "motor_comms_slave.h"
#include "../periphs/rs485.h"
#include <string.h>
#include <stdint.h>
#include "../periphs/uart.h"

// Incoming data structure:
// [SOF][SLAVE_ADDR][CARGO_LENGTH][CHECKSUM][CARGO]
// Checksum is seeded with slave_addr+cargo_length

#define SOF_BYTE 0xAA

static uint8_t self_address = 0;

void motor_comms_init(uint8_t self_addr) {
    self_address = self_addr;
    rs485_init();
}

void motor_comms_send_bytes(uint8_t addr, const uint8_t *data, uint8_t length) {
    uint8_t checksum = addr + length;  // seed with addr + length
    for (uint16_t i = 0; i < length; i++)
        checksum += data[i];

    uint8_t frame[4 + 255];
    frame[0] = SOF_BYTE;
    frame[1] = addr;
    frame[2] = length;
    memcpy(&frame[3], data, length);
    frame[3 + length] = checksum;

    rs485_send_bytes(frame, 4 + length);
}


void motor_comms_send_cmd(uint8_t addr, uint8_t cmd) {
    motor_comms_send_bytes(addr, &cmd, 1);
}

void motor_comms_send_float(uint8_t addr, const uint8_t command, float data) {
    uint8_t buf[5];
    buf[0] = command;
    memcpy(&buf[1], &data, 4);
    motor_comms_send_bytes(addr, buf, 5);
}

static bool get_byte_with_timeout(uint8_t* byte, uint32_t timeout) {
    for (uint32_t i = 0; i < timeout; i++) {
        int16_t ch = rs485_get();
        if (ch != -1) {
            *byte = (uint8_t)ch;
            return true;
        }
    }
    return false;
}

// Incoming data structure:
// [SOF][SLAVE_ADDR][CARGO_LENGTH][CHECKSUM][CARGO]
// Checksum is seeded with slave_addr+cargo_length
RXError motor_comms_get_data(uint8_t* data, uint8_t* data_len, uint8_t buf_size) {
    int16_t ch;

    do {
        ch = rs485_get();
    } while (!(ch == -1 || ch == SOF_BYTE));

    if (ch == -1) {
        return RX_ERR_NO_DATA;
    }

    // get addr
    uint8_t addr = 0;
    if (!get_byte_with_timeout(&addr, 0xFFF)) {
       return RX_ERR_TIMEOUT;
    }
    if (addr != self_address) return RX_ERR_WRONG_ADDR;

    // get length
    uint8_t length = 0;
    if (!get_byte_with_timeout(&length, 0xFFF)) {
       return RX_ERR_TIMEOUT;
    }
    if (length > buf_size) return RX_ERR_BUF_OVF;

    // get checksum
    uint8_t given_checksum = 0;
    if (!get_byte_with_timeout(&given_checksum, 0xFFF)) {
       return RX_ERR_TIMEOUT;
    }

    // get data
    uint8_t data_byte = 0;
    uint8_t checksum = addr + length;
    for (uint32_t i = 0; i < length; i++) {
        if (!get_byte_with_timeout(&data_byte, 0xFFF)) {
           return RX_ERR_TIMEOUT;
        }

        data[i] = data_byte;
        checksum += data_byte;
    }

    // Verify checksum
    if (given_checksum != checksum) return RX_ERR_WRONG_CHECKSUM;

    *data_len = length;
    return RX_ERR_OK;
}

float motor_comms_data_to_float(uint8_t* data) {
    float result;
    memcpy(&result, data, 4);
    return result;
}

void motor_comms_print_error(RXError err) {
    switch (err) {
        case RX_ERR_OK:
            uart_print("RX_ERR_OK");
            break;
        case RX_ERR_TIMEOUT:
            uart_print("RX_ERR_TIMEOUT");
            break;
        case RX_ERR_WRONG_ADDR:
            uart_print("RX_ERR_WRONG_ADDR");
            break;
        case RX_ERR_BUF_OVF:
            uart_print("RX_ERR_BUF_OVF");
            break;
        case RX_ERR_WRONG_CHECKSUM:
            uart_print("RX_ERR_WRONG_CHECKSUM");
            break;
        case RX_ERR_NO_DATA:
            uart_print("RX_ERR_NO_DATA");
            break;
    }
}

void motor_comms_println_error(RXError err) {
    motor_comms_print_error(err);
    uart_println("");
}

