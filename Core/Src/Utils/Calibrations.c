#include "Utils/Calibrations.h"
#include "Utils/Calculations.h"
#include <math.h>

static float PressureSumPa;
static float TemperatureSumC;
static uint16_t PressureSampleCount;
static uint16_t PressureDiscardCount;
static uint32_t PressureLastSampleId;

static float GyroSumX, GyroSumY, GyroSumZ;
static uint16_t GyroSampleCount;
static uint16_t GyroDiscardCount;

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

    ResetBarometricVerticalVelocity();
    ResetGPSVerticalVelocity();

    PressureSumPa = 0.0f;
    TemperatureSumC = 0.0f;
    PressureSampleCount = 0;
    PressureDiscardCount = 0;
    PressureLastSampleId = 0;

    GyroSumX = 0.0f;
    GyroSumY = 0.0f;
    GyroSumZ = 0.0f;
    GyroSampleCount = 0;
    GyroDiscardCount = 0;
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
    if (SystemContext->GyroCalibrationValid) {
        return;
    }

    if (GyroDiscardCount < GYRO_CALIBRATION_DISCARD_SAMPLES) {
        GyroDiscardCount++;
        return;
    }

    GyroSumX += FlightData.RawGyroX;
    GyroSumY += FlightData.RawGyroY;
    GyroSumZ += FlightData.RawGyroZ;
    GyroSampleCount++;

    if (GyroSampleCount >= GYRO_CALIBRATION_SAMPLES) {
        const float InvCount = 1.0f / (float)GyroSampleCount;
        SystemContext->GyroBiasRawX = GyroSumX * InvCount;
        SystemContext->GyroBiasRawY = GyroSumY * InvCount;
        SystemContext->GyroBiasRawZ = GyroSumZ * InvCount;
        SystemContext->GyroCalibrationValid = true;
    }
}
