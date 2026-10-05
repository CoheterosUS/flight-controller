#include "States/StateHandlers.h"
#include "Utils/Calculations.h"
#include "Sensors/IIM42653.h"
#include "Sensors/W25Q32JV.h"
#include "Tasks/BuzzerTask.h"
#include "../Kalman/Cal.h"

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

static float32_t CalW[DEEP_CALIBRATION_FACE_COUNT * DEEP_CALIBRATION_SAMPLES * 4];
static float32_t CalX[4 * 3];
static float32_t CalM[3 * 3];
static float32_t CalAm[3 * 3];

static void RunSixPointCal(void) {
    const int16_t *Raw = &CalibrationMatrix[0][0][0];
    const uint32_t Rows = DEEP_CALIBRATION_FACE_COUNT * DEEP_CALIBRATION_SAMPLES;

    for (uint32_t i = 0; i < Rows; i++) {
        CalW[i * 4 + 0] = Raw[i * 3 + 0];
        CalW[i * 4 + 1] = Raw[i * 3 + 1];
        CalW[i * 4 + 2] = Raw[i * 3 + 2];
        CalW[i * 4 + 3] = 1.0f;
    }

    arm_matrix_instance_f32 W, X, M, Am;
    arm_mat_init_f32(&W, Rows, 4, CalW);
    arm_mat_init_f32(&X, 4, 3, CalX);
    arm_mat_init_f32(&M, 3, 3, CalM);
    arm_mat_init_f32(&Am, 3, 3, CalAm);

    six_point_cal(&W, &X, &Am, &M);
}

void DeepCalibrationStateEntry(SystemContext_t *ctx) {
    ctx->AccelCalibrationValid = false;
    ctx->DeepCalFacesCaptured = 0;
    ctx->DeepCalCurrentFace = 0;
    CurrentStep = 0;
    SampleCount = 0;
    Discarded = 0;
    Phase = PHASE_WAITING_MOTION;
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
                RunSixPointCal();

                memcpy(Context->AccelA_m, CalAm, sizeof(CalAm));
                memcpy(Context->AccelBias, &CalX[9], 3 * sizeof(float));
                memcpy(Context->AccelM, CalM, sizeof(CalM));
                Context->AccelCalibrationValid = true;

                AccelCalibration_t FlashCal;
                memcpy(FlashCal.A_m, CalAm, sizeof(CalAm));
                memcpy(FlashCal.Bias, &CalX[9], 3 * sizeof(float));
                memcpy(FlashCal.M, CalM, sizeof(CalM));
                W25Q_WriteAccelCal(&FlashCal);

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
