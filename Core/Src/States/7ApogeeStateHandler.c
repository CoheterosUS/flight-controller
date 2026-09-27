#include "States/StateHandlers.h"
#include "Utils/Calculations.h"
#include "Utils/Pyro.h"
#include "stm32h7xx_hal.h"

static ConfirmCounter_t MainParachuteConfirm;
static uint32_t LastBaroSampleId;

void ApogeeStateEntry(SystemContext_t *ctx) {
    PyroFire(PYRO_CHANNEL_DROGUE);
    MainParachuteConfirm = (ConfirmCounter_t){ .Required = APOGEE_CONFIRM_SAMPLES };
    LastBaroSampleId = 0;
}

SystemState_t ApogeeStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	bool NewBaroSample = FlightData.BaroSampleId != 0 && FlightData.BaroSampleId != LastBaroSampleId;
	if (NewBaroSample) {
		LastBaroSampleId = FlightData.BaroSampleId;
	}

	if (NewBaroSample && FlightData.BaroValid && ConfirmCounterCheck(&MainParachuteConfirm, FlightData.BarometricAltitude <= APOGEE_MAIN_PARACHUTE_BAROM_ALT_THRESHOLD)) {
		return STATE_MAIN_PARACHUTE;
	}

#if APOGEE_MAIN_PARACHUTE_GPS_ALT_ENABLED
	if (FlightData.GPSAltitude <= APOGEE_MAIN_PARACHUTE_GPS_ALT_THRESHOLD) {
		return STATE_MAIN_PARACHUTE;
	}
#endif


#if APOGEE_MAIN_PARACHUTE_DELAY_ENABLED
	if (GetStateElapsedMs(Context, STATE_APOGEE) >= APOGEE_MAIN_PARACHUTE_DELAY_MS) {
		return STATE_MAIN_PARACHUTE;
	}
#endif

    return STATE_APOGEE;
}
