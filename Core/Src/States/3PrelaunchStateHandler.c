#include "States/StateHandlers.h"
#include "Utils/Calculations.h"
#include "Sensors/W25Q32JV.h"
#include "stm32h7xx_hal.h"
#include <string.h>

static ConfirmCounter_t BoostConfirm;

void PrelaunchStateEntry(SystemContext_t *ctx) {
    BoostConfirm = (ConfirmCounter_t){ .Required = PRELAUNCH_BOOST_CONSECUTIVE_SAMPLES };

    FlightSnapshot_t Snapshot = {0};
    memcpy(Snapshot.M, ctx->ImuCal.M, sizeof(Snapshot.M));
    Snapshot.AccelBiasCal[0] = ctx->AccelBiasCalX;
    Snapshot.AccelBiasCal[1] = ctx->AccelBiasCalY;
    Snapshot.AccelBiasCal[2] = ctx->AccelBiasCalZ;
    Snapshot.GyroBiasRaw[0] = ctx->GyroBiasRawX;
    Snapshot.GyroBiasRaw[1] = ctx->GyroBiasRawY;
    Snapshot.GyroBiasRaw[2] = ctx->GyroBiasRawZ;
    Snapshot.ReferencePressurePa = ctx->ReferencePressurePa;
    Snapshot.ReferenceTemperatureC = ctx->ReferenceTemperatureC;
    Snapshot.ImuOdrHz = IMU_ODR_HZ;
    Snapshot.ConfigVersion = CONFIG_VERSION;
    Snapshot.ImuCalSequence = W25Q_CalGetSequence();
    if ((ctx->CalStatus & CAL_STATUS_HIL_PRESEED) != 0u) {
        Snapshot.Flags |= W25Q_SNAPSHOT_FLAG_HIL_PRESEED;
    }
    if (!W25Q_SnapshotWrite(&Snapshot)) {
        SystemFaultSet(W25Q_SNAPSHOT_FAILED);
        ctx->FlashLoggingEnabled = false;
    } else {
        ctx->FlashLoggingEnabled = true;
    }
}

SystemState_t PrelaunchStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
	if (ConfirmCounterCheck(&BoostConfirm, FlightData.CalAccelX > PRELAUNCH_BOOST_ACCEL_X_THRESHOLD)) {
		return STATE_BOOST;
	}

    if (SystemFaultFlags != 0) {
        return STATE_GROUND_ABORT;
    }

    return STATE_PRELAUNCH;
}
