#include <math.h>
#include "Utils/shared.h"
#include "Utils/Calculations.h"

float CalculateAltitude(SystemContext_t *SystemContext, float PressurePa, float Temperature) {
    if (PressurePa <= 0.0f || SystemContext->ReferencePressurePa <= 0.0f || !(SystemContext->ReferencePressurePaValid)) {
        return 0.0f;
    }

    float TemperatureK = CalculateKelvinFromCelsius(Temperature);

    return (GAS_CONSTANT * TemperatureK / GRAV_CONSTANT) * logf(SystemContext->ReferencePressurePa / PressurePa);
}

float CalculateFilteredAltitude(SystemContext_t *SystemContext, float RawAltitude, uint32_t Sequence) {
    static float FilteredAltitude = 0.0f;
    static uint32_t PreviousSequence = 0;

    if (!SystemContext->ReferencePressurePaValid) {
        FilteredAltitude = 0.0f;
        PreviousSequence = 0;
        SystemContext->AltitudeFilterInitialized = false;
        return 0.0f;
    }

    if (!SystemContext->AltitudeFilterInitialized) {
        FilteredAltitude = RawAltitude;
        PreviousSequence = Sequence;
        SystemContext->AltitudeFilterInitialized = true;
        return FilteredAltitude;
    }

    if (Sequence == PreviousSequence) {
        return FilteredAltitude;
    }

    PreviousSequence = Sequence;
    FilteredAltitude = FilteredAltitude + ALTITUDE_IIR_FILTER_ALPHA * (RawAltitude - FilteredAltitude);
    return FilteredAltitude;
}

float CalculatePressureTemperature(uint8_t MSB, uint8_t LSB, uint8_t XLSB, bool Temperature) {
    int32_t RawValue = (int32_t)((MSB << 16) | (LSB << 8) | XLSB);
    int32_t SignValue = (RawValue << 8) >> 8;

    return (float)SignValue * (Temperature ? TEMPERATURE_SCALE : PRESSURE_SCALE);
}

static float BarometricPreviousAltitude;
static uint32_t BarometricPreviousTick;
static uint32_t BarometricPreviousSequence;
static float BarometricPreviousVelocity;

float CalculateBarometricVerticalVelocity(float Altitude, uint32_t Tick, uint32_t Sequence) {
    if (Sequence == BarometricPreviousSequence) {
        return BarometricPreviousVelocity;
    }

    uint32_t DeltaTick = Tick - BarometricPreviousTick;
    float Velocity = 0.0f;

    if (DeltaTick > 0 && BarometricPreviousTick > 0) {
        float DeltaSeconds = (float)DeltaTick / 1000.0f;
        Velocity = (Altitude - BarometricPreviousAltitude) / DeltaSeconds;
    }

    BarometricPreviousAltitude = Altitude;
    BarometricPreviousTick = Tick;
    BarometricPreviousSequence = Sequence;
    BarometricPreviousVelocity = Velocity;

    return Velocity;
}

void ResetBarometricVerticalVelocity(void) {
    BarometricPreviousAltitude = 0.0f;
    BarometricPreviousTick = 0;
    BarometricPreviousSequence = 0;
    BarometricPreviousVelocity = 0.0f;
}

static float GPSPreviousAltitude;
static uint32_t GPSPreviousTick;

float CalculateGPSVerticalVelocity(float Altitude, uint32_t Tick) {
    uint32_t DeltaTick = Tick - GPSPreviousTick;
    float Velocity = 0.0f;

    if (DeltaTick > 0 && GPSPreviousTick > 0) {
        float DeltaSeconds = (float)DeltaTick / 1000.0f;
        Velocity = (Altitude - GPSPreviousAltitude) / DeltaSeconds;
    }

    GPSPreviousAltitude = Altitude;
    GPSPreviousTick = Tick;

    return Velocity;
}

void ResetGPSVerticalVelocity(void) {
    GPSPreviousAltitude = 0.0f;
    GPSPreviousTick = 0;
}

void CalculateAccelerometerAxisCalibration(float PositiveMean, float NegativeMean, float *Bias, float *Scale) {
    *Bias = (PositiveMean + NegativeMean) / 2.0f;
    *Scale = 2.0f * GRAV_CONSTANT / (PositiveMean - NegativeMean);
}

