#ifndef SPI_ASYNC
#define SPI_ASYNC
#include <stdint.h>
#include <stdbool.h>

typedef void (*spi_callback_t)(void);

// Initialize async SPI for motor encoder (SERCOM3, uses SPI_CONF_MTR_ENCODER settings)
void spi_async_init();

// Start a 2-byte read from encoder (non-blocking, callback called when complete)
void spi_async_start_read(spi_callback_t callback);

// Get the 16-bit result from the last completed read
uint16_t spi_async_get_result();

// Get the safe 16-bit result (only updated when transfer fully completes)
uint16_t spi_async_get_safe_result();

// Check if a transfer is in progress
bool spi_async_is_busy();

#endif

