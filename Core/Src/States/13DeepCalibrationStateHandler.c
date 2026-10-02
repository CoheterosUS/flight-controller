#include "States/StateHandlers.h"
#include "Utils/Calculations.h"
#include "Sensors/IIM42653.h"
#include "Sensors/W25Q32JV.h"
#include "Tasks/BuzzerTask.h"

typedef struct {
    uint8_t Axis;
    float Sign;
} Face_t;

static int16_t CalibrationMatrix[DEEP_CALIBRATION_FACE_COUNT][DEEP_CALIBRATION_SAMPLES][DEEP_CALIBRATION_AXES];
static uint16_t FaceSampleCount[DEEP_CALIBRATION_FACE_COUNT];
static uint16_t Discarded;
static const Face_t Faces[DEEP_CALIBRATION_FACE_COUNT] = {
    { 1, +1.0f }, { 1, -1.0f },
    { 0, +1.0f }, { 0, -1.0f },
    { 2, +1.0f }, { 2, -1.0f },
};

static int8_t DetectFace(const float Accel[DEEP_CALIBRATION_AXES]) {
    for (uint8_t face = 0; face < DEEP_CALIBRATION_FACE_COUNT; face++) {
        if (Accel[Faces[face].Axis] * Faces[face].Sign >= DEEP_CALIBRATION_ACCEL_THRESHOLD) {
            return (int8_t)face;
        }
    }
    return -1;
}

void DeepCalibrationStateEntry(SystemContext_t *ctx) {
    ctx->AccelCalibrationValid = false;
    ctx->DeepCalFacesCaptured = 0;
    ctx->DeepCalCurrentFace = -1;
    Discarded = 0;
    xTaskNotify(BuzzerTaskHandle, 3, eSetValueWithOverwrite);
}

SystemState_t DeepCalibrationStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
    if (GetStateElapsedMs(Context, STATE_DEEP_CALIBRATION) >= DEEP_CALIBRATION_TIMEOUT_MS) {
        W25Q_LoadAccelCal(Context);
        return STATE_CALIBRATION;
    }

    const float Accel[DEEP_CALIBRATION_AXES] = {
        FlightData.AccelX,
        FlightData.AccelY,
        FlightData.AccelZ
    };

    const int8_t DetectedFace = IsGyroscopeStill(FlightData, DEEP_CALIBRATION_GYRO_MAX_DPS) ? DetectFace(Accel) : -1;

    if (DetectedFace != Context->DeepCalCurrentFace) {
        Context->DeepCalCurrentFace = DetectedFace;
        Discarded = 0;
        if (DetectedFace >= 0) {
            FaceSampleCount[DetectedFace] = 0;
        }
    }

    if (DetectedFace < 0 || (Context->DeepCalFacesCaptured & (1u << DetectedFace))) {
        return STATE_DEEP_CALIBRATION;
    }

    if (Discarded < DEEP_CALIBRATION_DISCARD_SAMPLES) {
        Discarded++;
        return STATE_DEEP_CALIBRATION;
    }

    uint16_t idx = FaceSampleCount[DetectedFace];
    CalibrationMatrix[DetectedFace][idx][0] = CalculateAccelerationLSB(Accel[0], ACCEL_SCALE);
    CalibrationMatrix[DetectedFace][idx][1] = CalculateAccelerationLSB(Accel[1], ACCEL_SCALE);
    CalibrationMatrix[DetectedFace][idx][2] = CalculateAccelerationLSB(Accel[2], ACCEL_SCALE);

    if (++FaceSampleCount[DetectedFace] >= DEEP_CALIBRATION_SAMPLES) {
        Context->DeepCalFacesCaptured |= (1u << DetectedFace);
        xTaskNotify(BuzzerTaskHandle, __builtin_popcount(Context->DeepCalFacesCaptured), eSetValueWithOverwrite);
        Context->StateEntryTicks[STATE_DEEP_CALIBRATION] = xTaskGetTickCount();

        if (Context->DeepCalFacesCaptured == DEEP_CALIBRATION_ALL_FACES) {
            Context->DeepCalibrationComplete = true;
            return STATE_CALIBRATION;
        }
    }

    return STATE_DEEP_CALIBRATION;
}
