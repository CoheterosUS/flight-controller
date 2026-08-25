#include "States/StateHandlers.h"
#include "Utils/Calculations.h"

void ActiveBurnoutStateEntry(SystemContext_t *ctx) {
}

SystemState_t ActiveBurnoutStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	// Barometric Altitude Threshold
	if (FlightData.BarometricAltitude >= ACTIVE_BURNOUT_APOGEE_BAROM_ALT_THRESHOLD) {
		return STATE_APOGEE;
	}

	// GPS Altitude (AGL) Threshold
	if (CalculateGPSAltitudeAGL(FlightData.GPSAltitude) >= ACTIVE_BURNOUT_APOGEE_GPS_ALT_THRESHOLD) {
		return STATE_APOGEE;
	}

	// GPS Vertical Velocity Threshold (NED)
	if (FlightData.GPSVelocity <= ACTIVE_BURNOUT_APOGEE_GPS_VEL_Y_THRESHOLD) {
		return STATE_APOGEE;
	}

	// Delay
	if (GetStateElapsedMs(Context, STATE_ACTIVE_BURNOUT) >= ACTIVE_BURNOUT_APOGEE_DELAY_MS) {
		return STATE_APOGEE;
	}

	return STATE_ACTIVE_BURNOUT;
}
