#include "States/StateHandlers.h"
#include "Utils/Calculations.h"

static ConfirmCounter_t ActiveControlConfirm;
static uint32_t LastBaroSampleId;

void CoastStateEntry(SystemContext_t *ctx) {
    ActiveControlConfirm = (ConfirmCounter_t){ .Required = COAST_ACTIVE_CONTROL_CONSECUTIVE_SAMPLES };
    LastBaroSampleId = 0;
}

SystemState_t CoastStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	bool NewBaroSample = FlightData.BaroSampleId != 0 && FlightData.BaroSampleId != LastBaroSampleId;
	if (NewBaroSample) {
		LastBaroSampleId = FlightData.BaroSampleId;
	}

	if (NewBaroSample && FlightData.BaroValid && ConfirmCounterCheck(&ActiveControlConfirm, FlightData.BarometricAltitude > COAST_ACTIVE_CONTROL_BAROM_ALT_THRESHOLD)) {
		return STATE_ACTIVE_CONTROL;
	}

    return STATE_COAST;
}
