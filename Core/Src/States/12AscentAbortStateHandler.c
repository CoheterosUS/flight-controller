#include "States/StateHandlers.h"
#include "Utils/Pyro.h"

void AscentAbortStateEntry(SystemContext_t *Context) {
}

SystemState_t AscentAbortStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
    return STATE_ASCENT_ABORT;
}
