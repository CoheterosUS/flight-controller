#include "States/StateHandlers.h"
#include "Utils/Calculations.h"
#include "stm32h7xx_hal.h"

static ConfirmCounter_t PassiveBurnoutConfirm;

void BurnStateEntry(SystemContext_t *ctx) {
    PassiveBurnoutConfirm = (ConfirmCounter_t){ .Required = BURN_PASSIVE_BURNOUT_CONSECUTIVE_SAMPLES };
}

SystemState_t BurnStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	if (ConfirmCounterCheck(&PassiveBurnoutConfirm, FlightData.AccelY < BURN_PASSIVE_BURNOUT_ACCEL_Y_THRESHOLD)) {
		return STATE_PASSIVE_BURNOUT;
	}

    return STATE_BURN;
}
