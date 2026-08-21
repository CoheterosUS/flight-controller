#include "States/StateHandlers.h"
#include "Utils/Pyro.h"
#include "Utils/SD.h"

void GroundAbortStateEntry(SystemContext_t *Context) {
    PyroSafeAll();
    Context->SDLoggingEnabled = false;
    CloseFile();
}

SystemState_t GroundAbortStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
    return STATE_GROUND_ABORT;
}
