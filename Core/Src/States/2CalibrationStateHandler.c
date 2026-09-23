#include "States/StateHandlers.h"
#include "Utils/Calibrations.h"
#include "Utils/FlightData.h"

void CalibrationStateEntry(SystemContext_t *ctx) {
#if SD_LOGGING_ENABLED
    ctx->SDLoggingEnabled = true;
#endif
    ResetCalibrationContext(ctx);
}

SystemState_t CalibrationStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	CalibratePressure(FlightData, Context);
	CalibrateGyroscope(FlightData, Context);

#if GPS_FIX_REQUIRED
	if (FlightData.GPSAltitude != 0.0f && FlightData.UnixTime != 0 && FlightData.Latitude != 0 && FlightData.Longitude != 0 && FlightData.Satellites >= GPS_FIX_MIN_SATELLITES) {
		Context->GPSFixValid = true;
	}
#else
	Context->GPSFixValid = true;
#endif

	if (Context->ReferencePressurePaValid && Context->GyroCalibrationValid && !Context->KalmanInitialized) {
		KalmanFilter_Init(Context);
		Context->KalmanInitialized = true;
	}

	if (Context->ReferencePressurePaValid && Context->GyroCalibrationValid && Context->GPSFixValid && Context->KalmanInitialized) {
		return STATE_PRELAUNCH;
	}

    // TODO: Refine
    if (SystemFaultFlags != 0) {
        return STATE_GROUND_ABORT;
    }

    return STATE_CALIBRATION;
}
