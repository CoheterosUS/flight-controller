#include "States/StateHandlers.h"

#include "Sensors/Sensors.h"
#include "Utils/DeepCalSequencer.h"
#include "Utils/Pyro.h"
#include "Sensors/W25Q32JV.h"

static DeepCalSeq_t DeepCalibrationSequencer;
static bool DeepCalibrationOutcomePending;
static bool DeepCalibrationOutcomeSuccess;
static BuzzerPattern_t DeepCalibrationOutcomePattern;
static uint32_t DeepCalibrationOutcomeStartMs;

static uint32_t DeepCalibrationPatternLength(BuzzerPattern_t Pattern)
{
    switch (Pattern) {
    case BUZZ_DEEPCAL_OK:
        return BUZZER_DEEPCAL_OK_COUNT * (BUZZER_DEEPCAL_OK_ON_MS + BUZZER_DEEPCAL_OK_OFF_MS);
    case BUZZ_DEEPCAL_FAIL:
        return BUZZER_DEEPCAL_FAIL_MS;
    default:
        return 0u;
    }
}

void DeepCalibrationStateEntry(SystemContext_t *Context)
{
    PyroSafeAll();
    Context->KalmanInitialized = false;
    Context->FlashLoggingEnabled = false;
    Context->CalStatus = CAL_STATUS_SET_POSE(Context->CalStatus, 0u);
    DeepCalibrationOutcomePending = false;
    DeepCalibrationOutcomeSuccess = false;
    DeepCalibrationOutcomePattern = BUZZ_NONE;
    DeepCalibrationOutcomeStartMs = 0u;
    DeepCalSeq_Start(&DeepCalibrationSequencer, Context->StateEntryTick);
    Buzzer_Play(BUZZ_DEEPCAL_ENTERED);
}

static void DeepCalibrationClearPose(SystemContext_t *Context)
{
    Context->CalStatus = CAL_STATUS_SET_POSE(Context->CalStatus, 0u);
}

SystemState_t DeepCalibrationStateHandler(SystemContext_t *Context, FlightData_t FlightData)
{
    if (SystemFaultFlags != 0u) {
        DeepCalibrationClearPose(Context);
        Buzzer_Play(BUZZ_STOP);
        return STATE_GROUND_ABORT;
    }

    if (DeepCalibrationOutcomePending) {
        if ((uint32_t)(FlightData.Tick - DeepCalibrationOutcomeStartMs)
            >= DeepCalibrationPatternLength(DeepCalibrationOutcomePattern)) {
            DeepCalibrationClearPose(Context);
            return STATE_IDLE;
        }
        return STATE_DEEP_CALIBRATION;
    }

    float RawAccel[3] = {FlightData.RawAccelX, FlightData.RawAccelY, FlightData.RawAccelZ};
    float RawGyro[3] = {FlightData.RawGyroX, FlightData.RawGyroY, FlightData.RawGyroZ};
    DeepCalSeqOut_t Output;
    DeepCalSeq_Step(&DeepCalibrationSequencer, FlightData.Tick, RawAccel, RawGyro, &Output);

    Context->CalStatus = CAL_STATUS_SET_POSE(Context->CalStatus, Output.Pose);
    if (Output.PlayPattern) {
        Buzzer_Play(Output.Pattern);
    }

    if (!Output.Done) {
        return STATE_DEEP_CALIBRATION;
    }

    DeepCalibrationOutcomeSuccess = Output.Success && W25Q_CalAppend(Output.Result.M);
    if (DeepCalibrationOutcomeSuccess) {
        Context->ImuCal = Output.Result;
        Context->CalStatus |= CAL_STATUS_IMU_CAL_VALID;
        DeepCalibrationOutcomePattern = BUZZ_DEEPCAL_OK;
    } else {
        DeepCalibrationOutcomePattern = BUZZ_DEEPCAL_FAIL;
    }
    DeepCalibrationOutcomeStartMs = FlightData.Tick;
    DeepCalibrationOutcomePending = true;
    Buzzer_Play(DeepCalibrationOutcomePattern);
    return STATE_DEEP_CALIBRATION;
}
