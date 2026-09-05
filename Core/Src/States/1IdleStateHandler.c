#include "States/StateHandlers.h"
#include "Utils/Calibrations.h"
#include "Utils/SD.h"
#include "stm32h7xx_hal.h"
#include "Utils/Pyro.h"

void IdleStateEntry(SystemContext_t *ctx) {
	PyroSafeAll();
    ResetCalibrationContext(ctx);
    MountAndOpen();
}

SystemState_t IdleStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
    // TODO: URGENT: Test code, remove later
    if (Context->SensorsIdleFinished) {
    	return STATE_CALIBRATION;
    }

    // TODO: Refine
    if (SystemFaultFlags != 0) {
        return STATE_GROUND_ABORT;
    }

    // Remain on Idle if not valid command
    return STATE_IDLE;
}
