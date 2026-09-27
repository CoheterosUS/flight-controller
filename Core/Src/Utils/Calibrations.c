#include "Utils/Calibrations.h"
#include "Utils/Calculations.h"
#include "Utils/ImuCal.h"
#include <math.h>

static float PressureSumPa;
static float TemperatureSumC;
static uint16_t PressureSampleCount;
static uint16_t PressureDiscardCount;
static uint32_t PressureLastSampleId;

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
    PressureLastSampleId = 0;

    ImuCal_GyroBiasReset(&GyroAccumulator);
    ImuCal_AccelBiasReset(&AccelBiasAccumulator);
}

static bool CalibrationBaroInRange(float PressurePa, float TemperatureC) {
    return isfinite(PressurePa) && PressurePa >= BARO_VALID_MIN_PA && PressurePa <= BARO_VALID_MAX_PA &&
           isfinite(TemperatureC) && TemperatureC >= BARO_VALID_MIN_TEMP_C && TemperatureC <= BARO_VALID_MAX_TEMP_C;
}

void CalibratePressure(FlightData_t FlightData, SystemContext_t *SystemContext) {
    if (SystemContext->ReferencePressurePaValid) {
        return;
    }

    // Only a NEW published barometer sample counts (a repeated read of the same conversion must not be averaged twice)
    if (FlightData.BaroSampleId == 0 || FlightData.BaroSampleId == PressureLastSampleId) {
        return;
    }
    PressureLastSampleId = FlightData.BaroSampleId;

    // An out of range or non finite sample means a flaky sensor: restart the averaging window
    if (!CalibrationBaroInRange(FlightData.PressurePa, FlightData.TemperatureC)) {
        PressureSumPa = 0.0f;
        TemperatureSumC = 0.0f;
        PressureSampleCount = 0;
        return;
    }

    if (PressureDiscardCount < PRESSURE_CALIBRATION_DISCARD_SAMPLES) {
        PressureDiscardCount++;
        return;
    }

    PressureSumPa += FlightData.PressurePa;
    TemperatureSumC += FlightData.TemperatureC;
    PressureSampleCount++;

    if (PressureSampleCount >= PRESSURE_CALIBRATION_SAMPLES) {
        float InvCount = 1.0f / (float)PressureSampleCount;
        float ReferencePressurePa = PressureSumPa * InvCount;
        float ReferenceTemperatureC = TemperatureSumC * InvCount;

        if (CalibrationBaroInRange(ReferencePressurePa, ReferenceTemperatureC)) {
            SystemContext->ReferencePressurePa = ReferencePressurePa;
            SystemContext->ReferenceTemperatureC = ReferenceTemperatureC;
            SystemContext->ReferencePressurePaValid = true;
        } else {
            PressureSumPa = 0.0f;
            TemperatureSumC = 0.0f;
            PressureSampleCount = 0;
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
