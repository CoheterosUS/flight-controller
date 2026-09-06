#include "States/StateHandlers.h"
#include "Utils/Pyro.h"
#include "Utils/SD.h"

void DescentAbortStateEntry(SystemContext_t *Context) {
}

SystemState_t DescentAbortStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
    return STATE_DESCENT_ABORT;
}
