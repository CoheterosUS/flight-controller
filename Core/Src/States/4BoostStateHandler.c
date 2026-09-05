#include "States/StateHandlers.h"
#include "Utils/Calculations.h"
#include "stm32h7xx_hal.h"

static ConfirmCounter_t CoastConfirm;

void BoostStateEntry(SystemContext_t *ctx) {
    CoastConfirm = (ConfirmCounter_t){ .Required = BOOST_COAST_CONSECUTIVE_SAMPLES };
}

SystemState_t BoostStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	if (ConfirmCounterCheck(&CoastConfirm, FlightData.AccelY < BOOST_COAST_ACCEL_Y_THRESHOLD)) {
		return STATE_COAST;
	}

    return STATE_BOOST;
}
