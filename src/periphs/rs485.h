/*
 * rs485.h
 *
 * Created: 2026-04-15
 *  Author: brans
 */

#ifndef RS485_H_
#define RS485_H_


void rs485_init(void);

// Input (async ring buffer)
int rs485_get(void);        // Returns byte (0-255) or -1 if empty
int rs485_available(void);  // Number of bytes buffered
void rs485_rx_flush(void);  // Discard all buffered RX

// Output
void rs485_put(char ch);
void rs485_send_float(float num);
void rs485_send_float_arr(float* data, int len);

#endif /* RS485_H_ */
