/*
 * rs485.h
 *
 * Created: 2026-04-15
 *  Author: brans
 */

#ifndef RS485_H_
#define RS485_H_

#include <stdint.h>

// rx_ready is invoked from ISR context whenever a byte is pushed into the
// RX ring buffer. Pass NULL if no callback is needed.
void rs485_init(void (*rx_ready)(void));

// Input (async ring buffer)
int rs485_get(void);        // Returns byte (0-255) or -1 if empty
int rs485_available(void);  // Number of bytes buffered
void rs485_rx_flush(void);  // Discard all buffered RX

// Output. Drive TXEN high for the duration of a transmission.
// rs485_send_bytes wraps a transaction internally; for hand-rolled sequences
// of rs485_put calls, wrap them in begin/end_transaction manually.
void rs485_begin_transaction(void);
void rs485_end_transaction(void);
void rs485_put(uint8_t byte);  // Caller must hold a transaction
void rs485_send_bytes(const uint8_t* data, uint16_t len);

#endif /* RS485_H_ */
