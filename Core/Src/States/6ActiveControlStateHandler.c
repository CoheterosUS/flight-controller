#include "States/StateHandlers.h"
#include "Utils/Calculations.h"

void ActiveControlStateEntry(SystemContext_t *ctx) {
}

SystemState_t ActiveControlStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	// Barometric Altitude Threshold
	if (FlightData.BarometricAltitude >= ACTIVE_CONTROL_APOGEE_BAROM_ALT_THRESHOLD) {
		return STATE_APOGEE;
	}

	// GPS Altitude (AGL) Threshold
	if (CalculateGPSAltitudeAGL(FlightData.GPSAltitude) >= ACTIVE_CONTROL_APOGEE_GPS_ALT_THRESHOLD) {
		return STATE_APOGEE;
	}

	// GPS Vertical Velocity Threshold (NED)
	if (FlightData.GPSVelocity <= ACTIVE_CONTROL_APOGEE_GPS_VEL_Y_THRESHOLD) {
		return STATE_APOGEE;
	}

	// Delay
#if ACTIVE_CONTROL_APOGEE_DELAY_ENABLED
	if (GetStateElapsedMs(Context, STATE_ACTIVE_CONTROL) >= ACTIVE_CONTROL_APOGEE_DELAY_MS) {
		return STATE_APOGEE;
	}
#endif

	return STATE_ACTIVE_CONTROL;
}
