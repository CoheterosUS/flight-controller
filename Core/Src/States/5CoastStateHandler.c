#include "States/StateHandlers.h"
#include "Utils/Calculations.h"

static ConfirmCounter_t ActiveControlConfirm;

void CoastStateEntry(SystemContext_t *ctx) {
    ActiveControlConfirm = (ConfirmCounter_t){ .Required = COAST_ACTIVE_CONTROL_CONSECUTIVE_SAMPLES };
}

SystemState_t CoastStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	if (ConfirmCounterCheck(&ActiveControlConfirm, FlightData.BarometricAltitude > COAST_ACTIVE_CONTROL_BAROM_ALT_THRESHOLD)) {
		return STATE_ACTIVE_CONTROL;
	}

    return STATE_COAST;
}
