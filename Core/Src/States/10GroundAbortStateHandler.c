#include "States/StateHandlers.h"
#include "Utils/Pyro.h"
#include "Utils/SD.h"
#include "Sensors/W25Q32JV.h"

void GroundAbortStateEntry(SystemContext_t *Context) {
    PyroSafeAll();
    Context->SDLoggingEnabled = false;
    CloseFile();
    Context->FlashLoggingEnabled = false;
    W25Q_LoggingStop();
}

SystemState_t GroundAbortStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
    return STATE_GROUND_ABORT;
}
