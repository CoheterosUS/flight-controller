#include "States/StateHandlers.h"

void DeepCalibrationStateEntry(SystemContext_t *ctx) {
    (void)ctx;
}

SystemState_t DeepCalibrationStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
    (void)Context;
    (void)FlightData;
    return STATE_DEEP_CALIBRATION;
}
