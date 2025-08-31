/*
 * timer.h
 *
 * Created: 3/21/2025 2:20:49 AM
 *  Author: brans
 */ 


#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

// Timer IDs
//#define TIMER_ID_CONTROLLER     (1)
//#define TIMER_ID_STATE_RECORDER (2)
//#define TIMER_ID_SPI_COLLECTOR  (3)

// Timer Sample Rates
//#define TIMER_SAMPLE_RATE_STATE_RECORDER    (50.0f)
//#define TIMER_SAMPLE_RATE_SPI_COLLECTOR     (2500.0f)

// Timer Priorities
//#define TIMER_PRIORITY_CONTROLLER     (2)
//#define TIMER_PRIORITY_STATE_RECORDER (3)
//#define TIMER_PRIORITY_SPI_COLLECTOR  (1)

typedef void (*func_ptr_t)();

void timer_schedule(uint16_t timer_id, float sample_rate, uint16_t priority, func_ptr_t callback);
void timer_deschedule(uint16_t timer_id);
void timer_init_all();

#endif /* TIMER_H_ */
