#include "Utils/shared.h"
#include "Utils/Calculations.h"

float CalculateKelvinFromCelsius(float TemperatureC) {
    return TemperatureC + 273.15f;
}

float CalculateAltitude(SystemContext_t *SystemContext, float PressurePa, float Temperature) {
    if (PressurePa <= 0.0f || SystemContext->ReferencePressurePa <= 0.0f || !(SystemContext->ReferencePressurePaValid)) {
        return 0.0f;
    }

    float TemperatureK = CalculateKelvinFromCelsius(Temperature);

    return (GAS_CONSTANT * TemperatureK / GRAV_CONSTANT) * logf(SystemContext->ReferencePressurePa / PressurePa);
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

float CalculateGyroscope(uint8_t MSB, uint8_t LSB, float Factor) {
    return ((int16_t)((MSB << 8) | LSB)) * Factor;
}

float CalculateAcceleration(uint8_t MSB, uint8_t LSB, float Factor) {
    return ((int16_t)((MSB << 8) | LSB)) * Factor;
}

float CalculateBiasedGyroscope(SystemContext_t *SystemContext, float Value, float Bias) {
    return SystemContext->GyroCalibrationValid ? Value - Bias : Value;
}

float CalculateMagneticField(uint8_t MSB, uint8_t LSB) {
    int16_t Raw = (int16_t)((MSB << 8) | LSB);
    return (float)Raw * 1.5f;
}

static float PreviousAltitude;
static uint32_t PreviousTick;

float CalculateVerticalVelocity(float Altitude, uint32_t Tick) {

    uint32_t DeltaTick = Tick - PreviousTick;
    float Velocity = 0.0f;

    if (DeltaTick > 0 && PreviousTick > 0) {
        float DeltaSeconds = (float)DeltaTick / 1000.0f;
        Velocity = (Altitude - PreviousAltitude) / DeltaSeconds;
    }

    PreviousAltitude = Altitude;
    PreviousTick = Tick;

    return Velocity;
}

void ResetVerticalVelocity(void) {
    PreviousAltitude = 0.0f;
    PreviousTick = 0;
}

uint32_t GetStateElapsedMs(SystemContext_t *SystemContext, SystemState_t State) {
    return xTaskGetTickCount() - SystemContext->StateEntryTicks[State];
}
