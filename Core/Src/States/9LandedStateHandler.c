#include "States/StateHandlers.h"
#include "Utils/Pyro.h"
#include "Utils/Calculations.h"
#include "stm32h7xx_hal.h"

void LandedStateEntry(SystemContext_t *ctx) {
    PyroSafeAll();
}

SystemState_t LandedStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
    if (Context->SDLoggingEnabled && GetStateElapsedMs(Context, STATE_LANDED) >= LANDED_SD_STOP_DELAY_MS) {
        Context->SDLoggingEnabled = false;
    }

    return STATE_LANDED;
}
