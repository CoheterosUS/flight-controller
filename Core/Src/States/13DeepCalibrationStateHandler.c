#include "States/StateHandlers.h"
#include "Utils/Calculations.h"
#include "Sensors/IIM42653.h"
#include "Sensors/W25Q32JV.h"
#include "Tasks/BuzzerTask.h"

typedef enum {
    PHASE_SETTLING,
    PHASE_DISCARDING,
    PHASE_SAMPLING,
    PHASE_WAITING_MOTION,
} DeepCalPhase_t;

static int16_t CalibrationMatrix[DEEP_CALIBRATION_FACE_COUNT][DEEP_CALIBRATION_SAMPLES][DEEP_CALIBRATION_AXES];
static uint16_t SampleCount;
static uint16_t Discarded;
static uint8_t CurrentStep;
static DeepCalPhase_t Phase;

void DeepCalibrationStateEntry(SystemContext_t *ctx) {
    ctx->AccelCalibrationValid = false;
    ctx->DeepCalFacesCaptured = 0;
    ctx->DeepCalCurrentFace = 0;
    CurrentStep = 0;
    SampleCount = 0;
    Discarded = 0;
    Phase = PHASE_SETTLING;
    xTaskNotify(BuzzerTaskHandle, 3, eSetValueWithOverwrite);
}

SystemState_t DeepCalibrationStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
    if (GetStateElapsedMs(Context, STATE_DEEP_CALIBRATION) >= DEEP_CALIBRATION_TIMEOUT_MS) {
        W25Q_LoadAccelCal(Context);
        return STATE_CALIBRATION;
    }

    const bool Still = IsGyroscopeStill(FlightData, DEEP_CALIBRATION_GYRO_MAX_DPS);

    switch (Phase) {
    case PHASE_SETTLING:
        if (Still) {
            Phase = PHASE_DISCARDING;
            Discarded = 0;
            SampleCount = 0;
        }
        return STATE_DEEP_CALIBRATION;

    case PHASE_DISCARDING:
        if (!Still) {
            Phase = PHASE_SETTLING;
            return STATE_DEEP_CALIBRATION;
        }
        if (++Discarded >= DEEP_CALIBRATION_DISCARD_SAMPLES) {
            Phase = PHASE_SAMPLING;
        }
        return STATE_DEEP_CALIBRATION;

    case PHASE_SAMPLING:
        if (!Still) {
            Phase = PHASE_SETTLING;
            return STATE_DEEP_CALIBRATION;
        }

        const float Accel[DEEP_CALIBRATION_AXES] = {
            FlightData.AccelX,
            FlightData.AccelY,
            FlightData.AccelZ
        };

        CalibrationMatrix[CurrentStep][SampleCount][0] = CalculateAccelerationLSB(Accel[0], ACCEL_SCALE);
        CalibrationMatrix[CurrentStep][SampleCount][1] = CalculateAccelerationLSB(Accel[1], ACCEL_SCALE);
        CalibrationMatrix[CurrentStep][SampleCount][2] = CalculateAccelerationLSB(Accel[2], ACCEL_SCALE);

        if (++SampleCount >= DEEP_CALIBRATION_SAMPLES) {
            Context->DeepCalFacesCaptured |= (1u << CurrentStep);
            Context->DeepCalCurrentFace = (int8_t)CurrentStep;
            xTaskNotify(BuzzerTaskHandle, CurrentStep + 1, eSetValueWithOverwrite);
            Context->StateEntryTicks[STATE_DEEP_CALIBRATION] = xTaskGetTickCount();

            if (++CurrentStep >= DEEP_CALIBRATION_FACE_COUNT) {
                Context->DeepCalibrationComplete = true;
                return STATE_CALIBRATION;
            }

            Phase = PHASE_WAITING_MOTION;
        }
        return STATE_DEEP_CALIBRATION;

    case PHASE_WAITING_MOTION:
        if (!Still) {
            Phase = PHASE_SETTLING;
        }
        return STATE_DEEP_CALIBRATION;
    }

    return STATE_DEEP_CALIBRATION;
}
