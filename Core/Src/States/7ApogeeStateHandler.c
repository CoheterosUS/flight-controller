#include "States/StateHandlers.h"
#include "Utils/Pyro.h"
#include "stm32h7xx_hal.h"

void ApogeeStateEntry(SystemContext_t *ctx) {
    PyroFire(PYRO_CHANNEL_DROGUE);
}

SystemState_t ApogeeStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	if (FlightData.BarometricAltitude < APOGEE_PARACHUTE_ALTITUDE_THRESHOLD) {
		return STATE_PARACHUTE;
	}

    return STATE_APOGEE;
}
