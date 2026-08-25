#include "States/StateHandlers.h"
#include "Utils/Calculations.h"
#include "stm32h7xx_hal.h"

static ConfirmCounter_t BurnConfirm;

void PrelaunchStateEntry(SystemContext_t *ctx) {
    BurnConfirm = (ConfirmCounter_t){ .Required = PRELAUNCH_BURN_CONSECUTIVE_SAMPLES };
}

SystemState_t PrelaunchStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	if (ConfirmCounterCheck(&BurnConfirm, FlightData.AccelY > PRELAUNCH_BURN_ACCEL_Y_THRESHOLD)) {
		return STATE_BURN;
	}

    if (SystemFaultFlags != 0) {
        return STATE_GROUND_ABORT;
    }

    return STATE_PRELAUNCH;
}
