#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "shared.h"

#define GAS_CONSTANT    287.0f 					// J/(kg*K)
#define GRAV_CONSTANT   9.80665f 				// m/s^2
#define PRESSURE_SCALE (1.0f / 64.0f)			// BMP581: 6 fractional bits
#define TEMPERATURE_SCALE (1.0f / 65536.0f)		// BMP581: 16 fractional bits

float CalculateAltitude(SystemContext_t *SystemContext, float PressurePa, float Temperature);
float CalculateFilteredAltitude(SystemContext_t *SystemContext, float RawAltitude, uint32_t Sequence);

float CalculatePressureTemperature(uint8_t MSB, uint8_t LSB, uint8_t XLSB, bool Temperature);

float CalculateBarometricVerticalVelocity(float Altitude, uint32_t Tick, uint32_t Sequence);
void ResetBarometricVerticalVelocity(void);

float CalculateGPSVerticalVelocity(float Altitude, uint32_t Tick);
void ResetGPSVerticalVelocity(void);

void CalculateAccelerometerAxisCalibration(float PositiveMean, float NegativeMean, float *Bias, float *Scale);

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

static inline int16_t CalculateAccelerationLSB(float Acceleration, float Factor) {
    return (int16_t)(Acceleration / Factor);
}

static inline float CalculateBiasedGyroscope(SystemContext_t *SystemContext, float Value, float Bias) {
    return SystemContext->GyroCalibrationValid ? Value - Bias : Value;
}

static inline bool IsGyroscopeStill(FlightData_t FlightData, float MaxDps) {
    float Mag2 = FlightData.GyroX * FlightData.GyroX
               + FlightData.GyroY * FlightData.GyroY
               + FlightData.GyroZ * FlightData.GyroZ;
    return Mag2 < MaxDps * MaxDps;
}

static inline float CalculateMagneticField(uint8_t MSB, uint8_t LSB) {
    int16_t Raw = (int16_t)((MSB << 8) | LSB);
    return (float)Raw * 1.5f;
}

static inline uint32_t GetStateElapsedMs(SystemContext_t *SystemContext, SystemState_t State) {
    return xTaskGetTickCount() - SystemContext->StateEntryTicks[State];
}

typedef struct {
    uint16_t Count;
    uint16_t Required;
} ConfirmCounter_t;

static inline void CalculateCalibratedAccel(const float *Raw, const float *M, float Scale, float *Out) {
    float lsb[3] = { Raw[0] / Scale, Raw[1] / Scale, Raw[2] / Scale };
    Out[0] = M[0]*lsb[0] + M[1]*lsb[1] + M[2]*lsb[2];
    Out[1] = M[3]*lsb[0] + M[4]*lsb[1] + M[5]*lsb[2];
    Out[2] = M[6]*lsb[0] + M[7]*lsb[1] + M[8]*lsb[2];
}

static inline void CalculateRotatedVector(const float *M, const float *In, float *Out) {
    Out[0] = M[0]*In[0] + M[1]*In[1] + M[2]*In[2];
    Out[1] = M[3]*In[0] + M[4]*In[1] + M[5]*In[2];
    Out[2] = M[6]*In[0] + M[7]*In[1] + M[8]*In[2];
}

static inline bool ConfirmCounterCheck(ConfirmCounter_t *Counter, bool Condition) {
    if (Condition) {
        if (++Counter->Count >= Counter->Required) return true;
    } else {
        Counter->Count = 0;
    }
    return false;
}

#endif //CALCULATIONS_H
