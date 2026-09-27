#include "States/StateHandlers.h"
void ActiveControlStateEntry(SystemContext_t *ctx) {
    (void)ctx;
}

SystemState_t ActiveControlStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	(void)Context;
	(void)FlightData;
	return STATE_ACTIVE_CONTROL;
}
