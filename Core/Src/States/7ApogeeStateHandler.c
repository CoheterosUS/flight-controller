#include "States/StateHandlers.h"
#include "Utils/Calculations.h"
#include "Utils/Pyro.h"
#include "stm32h7xx_hal.h"

void ApogeeStateEntry(SystemContext_t *ctx) {
    PyroFire(PYRO_CHANNEL_DROGUE);
}

SystemState_t ApogeeStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	// Altitude Threshold
	if (FlightData.BarometricAltitude <= APOGEE_PARACHUTE_BAROM_ALT_THRESHOLD) {
		return STATE_PARACHUTE;
	}

	// TODO: Discuss if GPS Altitude too

	// Delay
	if (GetStateElapsedMs(Context, STATE_APOGEE) >= APOGEE_PARACHUTE_DELAY_MS) {
		return STATE_PARACHUTE;
	}

    return STATE_APOGEE;
}
