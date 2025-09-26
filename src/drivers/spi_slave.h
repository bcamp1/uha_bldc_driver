#pragma once
#include <stdint.h>

void spi_slave_init();
float spi_slave_get_torque_command();
uint16_t spi_slave_get_torque_command_uint();

