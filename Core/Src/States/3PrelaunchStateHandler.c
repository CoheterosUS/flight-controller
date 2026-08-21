#include "States/StateHandlers.h"
#include "stm32h7xx_hal.h"

static uint8_t BurnConfirmCount;

void PrelaunchStateEntry(SystemContext_t *ctx) {
    // TODO: Refine
    ctx->SDLoggingEnabled = true;
    BurnConfirmCount = 0;
}

SystemState_t PrelaunchStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	// TODO: Revise code
	if (FlightData.AccelY > PRELAUNCH_BURN_ACCEL_Y_THRESHOLD) {
		BurnConfirmCount++;
		if (BurnConfirmCount >= PRELAUNCH_BURN_CONSECUTIVE_SAMPLES) {
			return STATE_BURN;
		}
	} else {
		BurnConfirmCount = 0;
	}

	// TODO: Refine
    if (SystemFaultFlags != 0) {
        return STATE_GROUND_ABORT;
    }

    return STATE_PRELAUNCH;
}
