#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include <stdbool.h>
#include <stdint.h>
#include "shared.h"

#define GAS_CONSTANT    287.0f 					// J/(kg*K)
#define GRAV_CONSTANT   9.80665f 				// m/s^2
#define PRESSURE_SCALE (1.0f / 64.0f)			// BMP581: 6 fractional bits
#define TEMPERATURE_SCALE (1.0f / 65536.0f)		// BMP581: 16 fractional bits

float CalculateGyroscope(uint8_t MSB, uint8_t LSB, float Factor);
float CalculateAcceleration(uint8_t MSB, uint8_t LSB, float Factor);
float CalculateBiasedGyroscope(SystemContext_t *SystemContext, float Value, float Bias);

float CalculateAltitude(SystemContext_t *SystemContext, float PressurePa, float Temperature);
float CalculateFilteredAltitude(SystemContext_t *SystemContext, float RawAltitude);

float CalculatePressureTemperature(uint8_t MSB, uint8_t LSB, uint8_t XLSB, bool Temperature);

float CalculateMagneticField(uint8_t MSB, uint8_t LSB);

float CalculateVerticalVelocity(float Altitude, uint32_t Tick);
void ResetVerticalVelocity(void);

uint32_t GetStateElapsedMs(SystemContext_t *SystemContext, SystemState_t State);

#endif //CALCULATIONS_H
