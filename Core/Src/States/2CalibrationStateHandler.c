#include "States/StateHandlers.h"
#include "Utils/Calibrations.h"

void CalibrationStateEntry(SystemContext_t *ctx) {
    ctx->SDLoggingEnabled = true;
    ResetCalibrationContext(ctx);
}

SystemState_t CalibrationStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	CalibratePressure(FlightData, Context);
	CalibrateGyroscope(FlightData, Context);

	if (FlightData.GPSAltitude != 0.0f && FlightData.UnixTime != 0 && FlightData.Latitude != 0 && FlightData.Longitude != 0 && FlightData.Satellites >= GPS_FIX_MIN_SATELLITES) {
		Context->GPSFixValid = true;
	}

	if (Context->ReferencePressurePaValid && Context->GyroCalibrationValid && Context->GPSFixValid) {
		return STATE_PRELAUNCH;
	}

    // TODO: Refine
    if (SystemFaultFlags != 0) {
        return STATE_GROUND_ABORT;
    }

    return STATE_CALIBRATION;
}
