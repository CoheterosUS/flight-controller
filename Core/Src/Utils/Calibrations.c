#include "Utils/Calibrations.h"
#include "Utils/Calculations.h"
#include "Utils/ImuCal.h"

static float PressureSumPa;
static float TemperatureSumC;
static uint16_t PressureSampleCount;
static uint16_t PressureDiscardCount;

static ImuGyroBiasAccumulator_t GyroAccumulator;
static ImuAccelBiasAccumulator_t AccelBiasAccumulator;

void ResetCalibrationContext(SystemContext_t *ctx) {
    ctx->ReferencePressurePa = 0.0f;
    ctx->ReferenceTemperatureC = 0.0f;
    ctx->ReferencePressurePaValid = false;
    ctx->GyroBiasRawX = 0.0f;
    ctx->GyroBiasRawY = 0.0f;
    ctx->GyroBiasRawZ = 0.0f;
    ctx->GyroCalibrationValid = false;
    ctx->AccelBiasCalX = 0.0f;
    ctx->AccelBiasCalY = 0.0f;
    ctx->AccelBiasCalZ = 0.0f;
    ctx->AccelBiasCalValid = false;
    ctx->AltitudeFilterInitialized = false;
    ctx->GPSFixValid = false;
    ctx->KalmanInitialized = false;

    ResetBarometricVerticalVelocity();
    ResetGPSVerticalVelocity();

    PressureSumPa = 0.0f;
    TemperatureSumC = 0.0f;
    PressureSampleCount = 0;
    PressureDiscardCount = 0;

    ImuCal_GyroBiasReset(&GyroAccumulator);
    ImuCal_AccelBiasReset(&AccelBiasAccumulator);
}

void CalibratePressure(FlightData_t FlightData, SystemContext_t *SystemContext) {
    float PressurePa = FlightData.PressurePa;

    if (PressurePa > 0.0f && !SystemContext->ReferencePressurePaValid) {
        if (PressureDiscardCount < PRESSURE_CALIBRATION_DISCARD_SAMPLES) {
            PressureDiscardCount++;
            return;
        }

        PressureSumPa += PressurePa;
        TemperatureSumC += FlightData.TemperatureC;
        PressureSampleCount++;

        if (PressureSampleCount >= PRESSURE_CALIBRATION_SAMPLES) {
            float InvCount = 1.0f / (float)PressureSampleCount;
            SystemContext->ReferencePressurePa = PressureSumPa * InvCount;
            SystemContext->ReferenceTemperatureC = TemperatureSumC * InvCount;
            SystemContext->ReferencePressurePaValid = true;
        }
    }
}

void CalibrateGyroscope(FlightData_t FlightData, SystemContext_t *SystemContext) {
    const float RawGyro[3] = {FlightData.RawGyroX, FlightData.RawGyroY, FlightData.RawGyroZ};
    float BiasRaw[3];

    if (SystemContext->GyroCalibrationValid) {
        return;
    }

    if (ImuCal_GyroBiasAdd(&GyroAccumulator, RawGyro, BiasRaw) == IMU_CAL_READY) {
        SystemContext->GyroBiasRawX = BiasRaw[0];
        SystemContext->GyroBiasRawY = BiasRaw[1];
        SystemContext->GyroBiasRawZ = BiasRaw[2];
        SystemContext->GyroCalibrationValid = true;
    }
}

void CalibrateAccelBias(FlightData_t FlightData, SystemContext_t *SystemContext) {
    const float RawAccel[3] = {FlightData.RawAccelX, FlightData.RawAccelY, FlightData.RawAccelZ};
    const float RawGyro[3] = {FlightData.RawGyroX, FlightData.RawGyroY, FlightData.RawGyroZ};
    const float GyroBiasRaw[3] = {
        SystemContext->GyroBiasRawX,
        SystemContext->GyroBiasRawY,
        SystemContext->GyroBiasRawZ
    };
    float BiasCal[3];

    if (!SystemContext->ImuCal.Valid || SystemContext->AccelBiasCalValid) return;

    if (ImuCal_AccelBiasAdd(&AccelBiasAccumulator,
                            &SystemContext->ImuCal,
                            SystemContext->GyroCalibrationValid,
                            GyroBiasRaw,
                            RawAccel,
                            RawGyro,
                            BiasCal) == IMU_CAL_READY) {
        SystemContext->AccelBiasCalX = BiasCal[0];
        SystemContext->AccelBiasCalY = BiasCal[1];
        SystemContext->AccelBiasCalZ = BiasCal[2];
        SystemContext->AccelBiasCalValid = true;
    }
}
