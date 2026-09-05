#include <math.h>
#include "States/StateHandlers.h"
#include "Utils/Calculations.h"
#include "Utils/Pyro.h"
#include "stm32h7xx_hal.h"

static ConfirmCounter_t LandedConfirm;

void MainParachuteStateEntry(SystemContext_t *ctx) {
    PyroFire(PYRO_CHANNEL_PARACHUTE);
    LandedConfirm = (ConfirmCounter_t){ .Required = MAIN_PARACHUTE_LANDED_CONSECUTIVE_SAMPLES };
}

SystemState_t MainParachuteStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	bool AltitudeLow = FlightData.BarometricAltitude <= MAIN_PARACHUTE_LANDED_BAROM_ALT_THRESHOLD;
	bool VelocityLow = fabsf(FlightData.BarometricVelocity) <= MAIN_PARACHUTE_LANDED_BAROM_VEL_Y_THRESHOLD;

	if (ConfirmCounterCheck(&LandedConfirm, AltitudeLow && VelocityLow)) {
		return STATE_LANDED;
	}

    return STATE_MAIN_PARACHUTE;
}
