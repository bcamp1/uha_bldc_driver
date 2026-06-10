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

// Output (async ring buffer).
// Returns once the bytes have been queued into the 16-byte TX ring; the DRE/TXC
// ISRs handle the actual shift-out and TXEN release. Blocks (spins) only when
// the ring is full, so callers MUST run at a lower priority than PRIO_RS485_TX
// or the wait will deadlock.
void rs485_send_bytes(const uint8_t* data, uint16_t len);

// Safety net: if TXEN gets stranded high (a lost TXC completion), force the
// transceiver back to receive once the ring has been idle too long. Call
// periodically from the main loop (lowest priority). Cheap no-op when idle.
void rs485_txen_watchdog(void);

#endif /* RS485_H_ */
