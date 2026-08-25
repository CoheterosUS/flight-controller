#include "States/StateHandlers.h"
#include "Utils/Calibrations.h"

void CalibrationStateEntry(SystemContext_t *ctx) {
    ctx->SDLoggingEnabled = true;
    ResetCalibrationContext(ctx);
}

SystemState_t CalibrationStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	CalibratePressure(FlightData, Context);
	CalibrateGyroscope(FlightData, Context);

	if (Context->ReferencePressurePaValid && Context->GyroCalibrationValid) {
		return STATE_PRELAUNCH;
	}

    // TODO: Refine
    if (SystemFaultFlags != 0) {
        return STATE_GROUND_ABORT;
    }

    return STATE_CALIBRATION;
}
