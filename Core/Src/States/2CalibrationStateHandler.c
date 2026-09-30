#include "States/StateHandlers.h"
#include "Utils/Calibrations.h"
#include "Utils/Calculations.h"

#if DEEP_CALIBRATION_ENABLED
static ConfirmCounter_t DeepCalibrationConfirm;

static bool WaitForDeepCalibration(SystemContext_t *ctx, FlightData_t FlightData) {
    if (!ctx->WaitingDeepCalibration) return false;

    if (ConfirmCounterCheck(&DeepCalibrationConfirm, IsGyroscopeStill(FlightData, DEEP_CALIBRATION_GYRO_MAX_DPS) && FlightData.AccelY >= DEEP_CALIBRATION_ACCEL_THRESHOLD)) {
        return true;
    }

    if (GetStateElapsedMs(ctx, STATE_CALIBRATION) >= DEEP_CALIBRATION_DURATION_MS) {
        ctx->WaitingDeepCalibration = false;
    }

    return false;
}
#endif

void CalibrationStateEntry(SystemContext_t *ctx) {
    ResetCalibrationContext(ctx);

#if DEEP_CALIBRATION_ENABLED
    if (!ctx->DeepCalibrationComplete) {
        ctx->WaitingDeepCalibration = true;
        DeepCalibrationConfirm = (ConfirmCounter_t){ .Required = DEEP_CALIBRATION_CONFIRM_SAMPLES };
    }
#endif

#if SD_LOGGING_ENABLED
    ctx->SDLoggingEnabled = true;
#endif
}

SystemState_t CalibrationStateHandler(SystemContext_t *Context, FlightData_t FlightData) {

#if DEEP_CALIBRATION_ENABLED
    if (WaitForDeepCalibration(Context, FlightData)) {
        return STATE_DEEP_CALIBRATION;
    }

    if (Context->WaitingDeepCalibration) {
        return STATE_CALIBRATION;
    }
#endif

    CalibratePressure(FlightData, Context);
    CalibrateGyroscope(FlightData, Context);

#if GPS_FIX_REQUIRED
    if (FlightData.GPSAltitude != 0.0f && FlightData.UnixTime != 0 && FlightData.Latitude != 0 && FlightData.Longitude != 0 && FlightData.Satellites >= GPS_FIX_MIN_SATELLITES) {
        Context->GPSFixValid = true;
    }
#else
    Context->GPSFixValid = true;
#endif

    if (Context->ReferencePressurePaValid && Context->GyroCalibrationValid && Context->GPSFixValid) {
        return STATE_PRELAUNCH;
    }

    // TODO: Refine
    if (SystemFaultFlags != 0) {
        return STATE_GROUND_ABORT;
    }

    return STATE_CALIBRATION;
}
