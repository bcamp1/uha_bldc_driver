#include "motor_comms_slave.h"
#include "../periphs/rs485.h"
#include "../periphs/uart.h"
#include "samd51j20a.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Wire format: [SOF][addr][length][checksum][data...]
// Checksum is seeded with addr+length and covers the data bytes.

#define SOF_BYTE 0xAA

typedef enum {
    ST_SOF,
    ST_ADDR,
    ST_LEN,
    ST_CKSUM,
    ST_DATA,
    ST_DROP_TAIL,  // Frame isn't for us (or too big); drain remaining bytes to stay aligned
} RxState;

static uint8_t self_address = 0;

static volatile RxState state = ST_SOF;
static volatile uint8_t expected_len = 0;
static volatile uint8_t given_checksum = 0;
static volatile uint8_t calc_checksum = 0;
static volatile uint8_t data_idx = 0;
static volatile uint8_t scratch[MOTOR_COMMS_MAX_DATA];

static volatile bool     for_us = false;
static volatile uint16_t drop_remaining = 0;

static volatile MotorCommsRxResult latched;
static volatile bool result_ready = false;

static volatile uint32_t checksum_success = 0;
static volatile uint32_t checksum_fail = 0;

// When either counter crosses this threshold, halve both. Preserves the ratio
// while keeping comfortably below UINT32_MAX so we can never overflow.
#define CHECKSUM_RESCALE_THRESHOLD (1u << 30)

static void discard(void) {
    state = ST_SOF;
}

static void latch_success(void) {
    latched.err = RX_ERR_OK;
    latched.data_len = expected_len;
    for (uint8_t i = 0; i < expected_len; i++) {
        latched.data[i] = scratch[i];
    }
    result_ready = true;
    state = ST_SOF;
}

static void finalize_checksum(void) {
    if (given_checksum == calc_checksum) {
        checksum_success++;
        latch_success();
    } else {
        checksum_fail++;
        discard();
    }

    if (checksum_success > CHECKSUM_RESCALE_THRESHOLD ||
        checksum_fail    > CHECKSUM_RESCALE_THRESHOLD) {
        checksum_success >>= 1;
        checksum_fail    >>= 1;
    }
}

static void feed(uint8_t byte) {
    switch (state) {
        case ST_SOF:
            if (byte == SOF_BYTE) {
                state = ST_ADDR;
            }
            break;
        case ST_ADDR:
            // Always advance — even if the frame isn't for us we need to keep
            // counting bytes so a 0xAA in the cargo doesn't get mis-parsed as SOF.
            for_us = (byte == self_address) || (byte == MOTOR_COMMS_BROADCAST_ADDR);
            if (for_us) {
                // Seed with the addr actually on the wire — sender's checksum
                // includes whichever address (self or broadcast) was used.
                calc_checksum = byte;
            }
            state = ST_LEN;
            break;
        case ST_LEN:
            if (!for_us || byte > MOTOR_COMMS_MAX_DATA) {
                // Skip the cksum byte plus `byte` cargo bytes, then resync.
                drop_remaining = (uint16_t)byte + 1;
                state = ST_DROP_TAIL;
                return;
            }
            expected_len = byte;
            calc_checksum += byte;
            data_idx = 0;
            state = ST_CKSUM;
            break;
        case ST_CKSUM:
            given_checksum = byte;
            if (expected_len == 0) {
                finalize_checksum();
            } else {
                state = ST_DATA;
            }
            break;
        case ST_DATA:
            scratch[data_idx++] = byte;
            calc_checksum += byte;
            if (data_idx == expected_len) {
                finalize_checksum();
            }
            break;
        case ST_DROP_TAIL:
            if (--drop_remaining == 0) {
                state = ST_SOF;
            }
            break;
    }
}

static void on_rx_ready(void) {
    int ch;
    while ((ch = rs485_get()) != -1) {
        feed((uint8_t)ch);
    }
}

void motor_comms_init(uint8_t self_addr) {
    self_address = self_addr;
    state = ST_SOF;
    drop_remaining = 0;
    for_us = false;
    result_ready = false;
    rs485_init(on_rx_ready);
}

// Outgoing frame: [SOF][self_addr][length][checksum][data...]
void motor_comms_send_data(const uint8_t *data, uint8_t length) {
    uint8_t checksum = self_address + length;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }

    uint8_t frame[4 + 255];
    frame[0] = SOF_BYTE;
    frame[1] = self_address;
    frame[2] = length;
    frame[3] = checksum;
    memcpy(&frame[4], data, length);

    rs485_send_bytes(frame, 4 + length);
}

void motor_comms_send_byte(uint8_t byte) {
    motor_comms_send_data(&byte, 1);
}

void motor_comms_send_float(uint8_t addr, const uint8_t command, float data) {
    (void)addr;
    uint8_t buf[5];
    buf[0] = command;
    memcpy(&buf[1], &data, 4);
    motor_comms_send_data(buf, 5);
}

MotorCommsRxResult motor_comms_get_data(void) {
    if (!result_ready) {
        MotorCommsRxResult empty = { .err = RX_ERR_NO_DATA, .data_len = 0 };
        return empty;
    }

    NVIC_DisableIRQ(SERCOM0_2_IRQn);
    MotorCommsRxResult out;
    out.err = latched.err;
    out.data_len = latched.data_len;
    for (uint8_t i = 0; i < latched.data_len; i++) {
        out.data[i] = latched.data[i];
    }
    result_ready = false;
    NVIC_EnableIRQ(SERCOM0_2_IRQn);
    return out;
}

float motor_comms_checksum_success_rate(void) {
    NVIC_DisableIRQ(SERCOM0_2_IRQn);
    uint32_t s = checksum_success;
    uint32_t f = checksum_fail;
    NVIC_EnableIRQ(SERCOM0_2_IRQn);

    uint32_t total = s + f;
    if (total == 0) return 1.0f;
    return (float)s / (float)total;
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
        case RX_ERR_NO_DATA:
            uart_print("RX_ERR_NO_DATA");
            break;
    }
}

void motor_comms_println_error(RXError err) {
    motor_comms_print_error(err);
    uart_println("");
}
