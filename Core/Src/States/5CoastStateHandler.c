#include "States/StateHandlers.h"

void CoastStateEntry(SystemContext_t *ctx) {
}

SystemState_t CoastStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
    return STATE_ACTIVE_CONTROL;
}
