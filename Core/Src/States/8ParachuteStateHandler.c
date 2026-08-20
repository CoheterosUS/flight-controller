#include <math.h>
#include "States/StateHandlers.h"
#include "Utils/Pyro.h"
#include "stm32h7xx_hal.h"

static bool DescentReached;

void ParachuteStateEntry(SystemContext_t *ctx) {
    PyroFire(PYRO_CHANNEL_PARACHUTE);
    DescentReached = false;
}

SystemState_t ParachuteStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	if (FlightData.BarometricVelocity < PARACHUTE_MIN_VEL_Y_REACHED) {
		DescentReached = true;
	}

	if (DescentReached && fabsf(FlightData.BarometricVelocity) < PARACHUTE_LANDED_VEL_Y_THRESHOLD) {
		return STATE_LANDED;
	}

    return STATE_PARACHUTE;
}
