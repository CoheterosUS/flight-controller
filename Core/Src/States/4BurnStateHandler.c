#include "States/StateHandlers.h"
#include "stm32h7xx_hal.h"

static bool VelocityReached;

void BurnStateEntry(SystemContext_t *ctx) {
    VelocityReached = false;
}

SystemState_t BurnStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	if (FlightData.BarometricVelocity > BURN_MIN_VEL_Y_REACHED) {
		VelocityReached = true;
	}

	if (VelocityReached && FlightData.BarometricVelocity < BURN_APOGEE_VEL_Y_THRESHOLD) {
		return STATE_PASSIVE_BURNOUT;
	}

    return STATE_BURN;
}
