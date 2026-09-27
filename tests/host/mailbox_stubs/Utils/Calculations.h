#ifndef HOST_CALCULATIONS_H
#define HOST_CALCULATIONS_H

#include <stdbool.h>
#include <stdint.h>

float CalculatePressureTemperature(uint8_t MSB, uint8_t LSB, uint8_t XLSB, bool Temperature);

#endif
