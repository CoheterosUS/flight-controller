#include "States/StateHandlers.h"
#include "Utils/Calculations.h"
#include "stm32h7xx_hal.h"

static ConfirmCounter_t BoostConfirm;

void PrelaunchStateEntry(SystemContext_t *ctx) {
    BoostConfirm = (ConfirmCounter_t){ .Required = PRELAUNCH_BOOST_CONSECUTIVE_SAMPLES };
}

SystemState_t PrelaunchStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	if (ConfirmCounterCheck(&BoostConfirm, FlightData.AccelY > PRELAUNCH_BOOST_ACCEL_Y_THRESHOLD)) {
		return STATE_BOOST;
	}

    if (SystemFaultFlags != 0) {
        return STATE_GROUND_ABORT;
    }

    return STATE_PRELAUNCH;
}
