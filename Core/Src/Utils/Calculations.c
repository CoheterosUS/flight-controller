#include <math.h>
#include "Utils/shared.h"
#include "Utils/Calculations.h"
#include "KalmanLib.h"

float CalculateAltitude(SystemContext_t *SystemContext, float PressurePa) {
    if (SystemContext == NULL || !isfinite(PressurePa) || PressurePa <= 0.0f ||
        !isfinite(SystemContext->ReferencePressurePa) || SystemContext->ReferencePressurePa <= 0.0f ||
        !SystemContext->ReferencePressurePaValid || !isfinite(SystemContext->ReferenceTemperatureC) ||
        SystemContext->ReferenceTemperatureC < BARO_VALID_MIN_TEMP_C ||
        SystemContext->ReferenceTemperatureC > BARO_VALID_MAX_TEMP_C) {
        return NAN;
    }

    const float ReferencePressurePa = SystemContext->ReferencePressurePa;
    const float ReferenceTemperatureK = CalculateKelvinFromCelsius(SystemContext->ReferenceTemperatureC);
    const float PressureRatio = PressurePa / ReferencePressurePa;
    const float AirGasConstant = (float)R_AIR;
    const float AirLapseRate = (float)ALPHA_AIR;
    const float Exponent = AirGasConstant * AirLapseRate / BARO_GRAVITY_MS2;
    const float DownAltitude = (ReferenceTemperatureK / AirLapseRate) *
                               (powf(PressureRatio, Exponent) - 1.0f);

    // Sanity: ratio 0.7 at 298.15 K is about 3008.5 m up, ratio 1.0 is 0 m,
    // and the forward model round trip returns 0.7 within float precision.
    return -DownAltitude;
}

float CalculateFilteredAltitude(SystemContext_t *SystemContext, float RawAltitude) {
    static float FilteredAltitude = 0.0f;

    if (!SystemContext->ReferencePressurePaValid) {
        FilteredAltitude = 0.0f;
        SystemContext->AltitudeFilterInitialized = false;
        return 0.0f;
    }

    if (!SystemContext->AltitudeFilterInitialized) {
        FilteredAltitude = RawAltitude;
        SystemContext->AltitudeFilterInitialized = true;
        return FilteredAltitude;
    }

    FilteredAltitude = FilteredAltitude + ALTITUDE_IIR_FILTER_ALPHA * (RawAltitude - FilteredAltitude);
    return FilteredAltitude;
}

float CalculatePressureTemperature(uint8_t MSB, uint8_t LSB, uint8_t XLSB, bool Temperature) {
    int32_t RawValue = (int32_t)((MSB << 16) | (LSB << 8) | XLSB);
    int32_t SignValue = (RawValue << 8) >> 8;

    return (float)SignValue * (Temperature ? TEMPERATURE_SCALE : PRESSURE_SCALE);
}

#define BARO_VELOCITY_BUFFER_SIZE (BARO_ODR_HZ * BARO_VELOCITY_WINDOW_MS / 1000 + 1)

typedef struct {
    float Altitude;
    uint32_t Tick;
} BarometricVelocitySample_t;

static BarometricVelocitySample_t BarometricSamples[BARO_VELOCITY_BUFFER_SIZE];
static uint16_t BarometricSampleCount;
static uint16_t BarometricNextIndex;
static uint32_t BarometricLastSampleId;
static float BarometricVelocity;

float CalculateBarometricVerticalVelocity(float Altitude, uint32_t Tick, bool BaroValid, uint32_t BaroSampleId) {
    if (!BaroValid || BaroSampleId == 0 || BaroSampleId == BarometricLastSampleId || !isfinite(Altitude)) {
        return BarometricVelocity;
    }

    BarometricLastSampleId = BaroSampleId;
    BarometricSamples[BarometricNextIndex] = (BarometricVelocitySample_t){
        .Altitude = Altitude,
        .Tick = Tick
    };
    BarometricNextIndex = (uint16_t)((BarometricNextIndex + 1U) % BARO_VELOCITY_BUFFER_SIZE);
    if (BarometricSampleCount < BARO_VELOCITY_BUFFER_SIZE) {
        BarometricSampleCount++;
    }

    uint16_t OldestIndex = (BarometricSampleCount == BARO_VELOCITY_BUFFER_SIZE) ? BarometricNextIndex : 0;
    uint32_t DeltaTick = Tick - BarometricSamples[OldestIndex].Tick;
    if (DeltaTick >= (BARO_VELOCITY_WINDOW_MS / 2U) && DeltaTick > 0) {
        BarometricVelocity = (Altitude - BarometricSamples[OldestIndex].Altitude) /
                             ((float)DeltaTick / 1000.0f);
    } else {
        BarometricVelocity = 0.0f;
    }

    return BarometricVelocity;
}

void ResetBarometricVerticalVelocity(void) {
    BarometricSampleCount = 0;
    BarometricNextIndex = 0;
    BarometricLastSampleId = 0;
    BarometricVelocity = 0.0f;
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

