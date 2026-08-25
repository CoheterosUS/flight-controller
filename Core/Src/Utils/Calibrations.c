#include "Utils/Calibrations.h"
#include "Utils/Calculations.h"

static float PressureSumPa;
static uint16_t PressureSampleCount;
static uint16_t PressureDiscardCount;

static float GyroSumX, GyroSumY, GyroSumZ;
static uint16_t GyroSampleCount;
static uint16_t GyroDiscardCount;

void ResetCalibrationContext(SystemContext_t *ctx) {
    ctx->ReferencePressurePa = 0.0f;
    ctx->ReferencePressurePaValid = false;
    ctx->GyroBiasX = 0.0f;
    ctx->GyroBiasY = 0.0f;
    ctx->GyroBiasZ = 0.0f;
    ctx->GyroCalibrationValid = false;
    ctx->AltitudeFilterInitialized = false;

    ResetBarometricVerticalVelocity();
    ResetGPSVerticalVelocity();

    PressureSumPa = 0.0f;
    PressureSampleCount = 0;
    PressureDiscardCount = 0;

    GyroSumX = 0.0f;
    GyroSumY = 0.0f;
    GyroSumZ = 0.0f;
    GyroSampleCount = 0;
    GyroDiscardCount = 0;
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
