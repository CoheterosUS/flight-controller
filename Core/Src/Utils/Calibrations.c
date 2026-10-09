#include "Utils/Calibrations.h"
#include "Utils/Calculations.h"
#include <math.h>

static float PressureSumPa;
static uint16_t PressureSampleCount;
static uint16_t PressureDiscardCount;

static float TemperatureSumC;
static uint16_t TemperatureSampleCount;
static uint16_t TemperatureDiscardCount;

static float GyroSumX, GyroSumY, GyroSumZ;
static uint16_t GyroSampleCount;
static uint16_t GyroDiscardCount;

static float AccelBiasSumX, AccelBiasSumY, AccelBiasSumZ;
static uint16_t AccelBiasSampleCount;
static uint16_t AccelBiasDiscardCount;

void ResetCalibrationContext(SystemContext_t *ctx) {
    ctx->ReferencePressurePa = 0.0f;
    ctx->ReferencePressurePaValid = false;
    ctx->ReferenceTemperatureC = 0.0f;
    ctx->ReferenceTemperatureCValid = false;
    ctx->GyroBiasX = 0.0f;
    ctx->GyroBiasY = 0.0f;
    ctx->GyroBiasZ = 0.0f;
    ctx->GyroCalibrationValid = false;
    ctx->AltitudeFilterInitialized = false;
    ctx->GPSFixValid = false;
    ctx->AccelBiasBodyValid = false;

#if DEEP_CALIBRATION_ENABLED
    ctx->WaitingDeepCalibration = false;
#endif

    ResetBarometricVerticalVelocity();
    ResetGPSVerticalVelocity();

    PressureSumPa = 0.0f;
    PressureSampleCount = 0;
    PressureDiscardCount = 0;

    TemperatureSumC = 0.0f;
    TemperatureSampleCount = 0;
    TemperatureDiscardCount = 0;

    GyroSumX = 0.0f;
    GyroSumY = 0.0f;
    GyroSumZ = 0.0f;
    GyroSampleCount = 0;
    GyroDiscardCount = 0;

    AccelBiasSumX = 0.0f;
    AccelBiasSumY = 0.0f;
    AccelBiasSumZ = 0.0f;
    AccelBiasSampleCount = 0;
    AccelBiasDiscardCount = 0;
}

void CalibratePressure(FlightData_t FlightData, SystemContext_t *SystemContext) {
    float PressurePa = FlightData.PressurePa;

    if (PressurePa > 0.0f && !SystemContext->ReferencePressurePaValid) {
        if (PressureDiscardCount < PRESSURE_CALIBRATION_DISCARD_SAMPLES) {
            PressureDiscardCount++;
            return;
        }

        PressureSumPa += PressurePa;
        PressureSampleCount++;

        if (PressureSampleCount >= PRESSURE_CALIBRATION_SAMPLES) {
            SystemContext->ReferencePressurePa = PressureSumPa / (float)PressureSampleCount;
            SystemContext->ReferencePressurePaValid = true;
        }
    }
}

void CalibrateTemperature(FlightData_t FlightData, SystemContext_t *SystemContext) {
    if (SystemContext->ReferenceTemperatureCValid) {
        return;
    }

    if (TemperatureDiscardCount < TEMPERATURE_CALIBRATION_DISCARD_SAMPLES) {
        TemperatureDiscardCount++;
        return;
    }

    TemperatureSumC += FlightData.TemperatureC;
    TemperatureSampleCount++;

    if (TemperatureSampleCount >= TEMPERATURE_CALIBRATION_SAMPLES) {
        SystemContext->ReferenceTemperatureC = TemperatureSumC / (float)TemperatureSampleCount;
        SystemContext->ReferenceTemperatureCValid = true;
    }
}

void CalibrateGyroscope(FlightData_t FlightData, SystemContext_t *SystemContext) {
    if (SystemContext->GyroCalibrationValid) {
        return;
    }

    if (GyroDiscardCount < GYRO_CALIBRATION_DISCARD_SAMPLES) {
        GyroDiscardCount++;
        return;
    }

    GyroSumX += FlightData.GyroX;
    GyroSumY += FlightData.GyroY;
    GyroSumZ += FlightData.GyroZ;
    GyroSampleCount++;

    if (GyroSampleCount >= GYRO_CALIBRATION_SAMPLES) {
        const float InvCount = 1.0f / (float)GyroSampleCount;
        SystemContext->GyroBiasX = GyroSumX * InvCount;
        SystemContext->GyroBiasY = GyroSumY * InvCount;
        SystemContext->GyroBiasZ = GyroSumZ * InvCount;
        SystemContext->GyroCalibrationValid = true;
    }
}

void CalibrateAccelBias(FlightData_t FlightData, SystemContext_t *SystemContext) {
    if (SystemContext->AccelBiasBodyValid) {
        return;
    }

    if (!SystemContext->AccelCalibrationValid || !SystemContext->PitchReceived) {
        return;
    }

    if (AccelBiasDiscardCount < ACCEL_BIAS_CALIBRATION_DISCARD_SAMPLES) {
        AccelBiasDiscardCount++;
        return;
    }

    AccelBiasSumX += FlightData.BodyAccelX;
    AccelBiasSumY += FlightData.BodyAccelY;
    AccelBiasSumZ += FlightData.BodyAccelZ;
    AccelBiasSampleCount++;

    if (AccelBiasSampleCount >= ACCEL_BIAS_CALIBRATION_SAMPLES) {
        const float InvCount = 1.0f / (float)AccelBiasSampleCount;
        const float AvgX = AccelBiasSumX * InvCount;
        const float AvgY = AccelBiasSumY * InvCount;
        const float AvgZ = AccelBiasSumZ * InvCount;

        const float Pitch = SystemContext->PitchAngleRad;
        const float ExpectedX = sinf(Pitch) * GRAV_CONSTANT;
        const float ExpectedY = 0.0f;
        const float ExpectedZ = -cosf(Pitch) * GRAV_CONSTANT;

        SystemContext->AccelBiasBody[0] = AvgX - ExpectedX;
        SystemContext->AccelBiasBody[1] = AvgY - ExpectedY;
        SystemContext->AccelBiasBody[2] = AvgZ - ExpectedZ;
        SystemContext->AccelBiasBodyValid = true;
    }
}
