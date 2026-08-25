#include <math.h>
#include "States/StateHandlers.h"
#include "Utils/Calculations.h"
#include "Utils/Pyro.h"
#include "stm32h7xx_hal.h"

static bool DescentReached;
static ConfirmCounter_t LandedConfirm;

void ParachuteStateEntry(SystemContext_t *ctx) {
    PyroFire(PYRO_CHANNEL_PARACHUTE);
    DescentReached = false;
    LandedConfirm = (ConfirmCounter_t){ .Required = PARACHUTE_LANDED_CONSECUTIVE_SAMPLES };
}

SystemState_t ParachuteStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	if (FlightData.GPSVelocity < PARACHUTE_MIN_GPS_VEL_Y_REACHED) {
		DescentReached = true;
	}

	if (DescentReached && ConfirmCounterCheck(&LandedConfirm, fabsf(FlightData.GPSVelocity) < PARACHUTE_LANDED_GPS_VEL_Y_THRESHOLD)) {
		return STATE_LANDED;
	}

    return STATE_PARACHUTE;
}
