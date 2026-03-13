#pragma once
#include <stdint.h>

void spi_slave_init();
float spi_slave_get_torque_command();
uint16_t spi_slave_get_torque_command_uint();
float spi_slave_get_success_rate();
uint32_t spi_slave_get_hit_count();
uint32_t spi_slave_get_miss_count();
int32_t spi_slave_last_rx_status();

