#include "States/StateHandlers.h"
#include "Utils/Pyro.h"
#include "Utils/Calculations.h"
#include "Utils/SD.h"
#include "Sensors/W25Q32JV.h"
#include "stm32h7xx_hal.h"

void LandedStateEntry(SystemContext_t *ctx) {
    PyroSafeAll();
}

SystemState_t LandedStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
    if (Context->SDLoggingEnabled
#if LANDED_SD_STOP_DELAY_ENABLED
        && GetStateElapsedMs(Context, STATE_LANDED) >= LANDED_SD_STOP_DELAY_MS
#endif
    ) {
        Context->SDLoggingEnabled = false;
        CloseFile();
    }

    if (Context->FlashLoggingEnabled
#if LANDED_FLASH_STOP_DELAY_ENABLED
        && GetStateElapsedMs(Context, STATE_LANDED) >= LANDED_FLASH_STOP_DELAY_MS
#endif
    ) {
        Context->FlashLoggingEnabled = false;
        W25Q_LoggingStop();
    }

    return STATE_LANDED;
}
