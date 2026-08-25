#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include <stdbool.h>
#include <stdint.h>
#include "shared.h"

#define GAS_CONSTANT    287.0f 					// J/(kg*K)
#define GRAV_CONSTANT   9.80665f 				// m/s^2
#define PRESSURE_SCALE (1.0f / 64.0f)			// BMP581: 6 fractional bits
#define TEMPERATURE_SCALE (1.0f / 65536.0f)		// BMP581: 16 fractional bits

float CalculateAltitude(SystemContext_t *SystemContext, float PressurePa, float Temperature);
float CalculateFilteredAltitude(SystemContext_t *SystemContext, float RawAltitude);

float CalculatePressureTemperature(uint8_t MSB, uint8_t LSB, uint8_t XLSB, bool Temperature);

float CalculateBarometricVerticalVelocity(float Altitude, uint32_t Tick);
void ResetBarometricVerticalVelocity(void);

float CalculateGPSVerticalVelocity(float Altitude, uint32_t Tick);
void ResetGPSVerticalVelocity(void);

static inline float CalculateGPSAltitudeAGL(float AltitudeASL) {
    return AltitudeASL - GPS_ALTITUDE_ASL_BASELINE;
}

static inline float CalculateKelvinFromCelsius(float TemperatureC) {
    return TemperatureC + 273.15f;
}

static inline float CalculateGyroscope(uint8_t MSB, uint8_t LSB, float Factor) {
    return ((int16_t)((MSB << 8) | LSB)) * Factor;
}

static inline float CalculateAcceleration(uint8_t MSB, uint8_t LSB, float Factor) {
    return ((int16_t)((MSB << 8) | LSB)) * Factor;
}

static inline float CalculateBiasedGyroscope(SystemContext_t *SystemContext, float Value, float Bias) {
    return SystemContext->GyroCalibrationValid ? Value - Bias : Value;
}

static inline float CalculateMagneticField(uint8_t MSB, uint8_t LSB) {
    int16_t Raw = (int16_t)((MSB << 8) | LSB);
    return (float)Raw * 1.5f;
}

static inline uint32_t GetStateElapsedMs(SystemContext_t *SystemContext, SystemState_t State) {
    return xTaskGetTickCount() - SystemContext->StateEntryTicks[State];
}

typedef struct {
    uint8_t Count;
    uint8_t Required;
} ConfirmCounter_t;

static inline bool ConfirmCounterCheck(ConfirmCounter_t *Counter, bool Condition) {
    if (Condition) {
        if (++Counter->Count >= Counter->Required) return true;
    } else {
        Counter->Count = 0;
    }
    return false;
}

#endif //CALCULATIONS_H
