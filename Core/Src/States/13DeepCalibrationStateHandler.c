#include "States/StateHandlers.h"

void DeepCalibrationStateEntry(SystemContext_t *Context) {
    (void)Context;
}

SystemState_t DeepCalibrationStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
    (void)Context;
    (void)FlightData;
    return STATE_IDLE;
}
