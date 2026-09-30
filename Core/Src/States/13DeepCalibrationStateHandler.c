#include "States/StateHandlers.h"
#include "Utils/Calculations.h"
#include "Sensors/W25Q32JV.h"

typedef struct {
    uint8_t Axis;
    float Sign;
} Face_t;

typedef struct {
    float Sum[DEEP_CALIBRATION_AXES];
    uint16_t Samples;
    uint16_t Discarded;
} FaceAccumulator_t;

static float FaceMean[DEEP_CALIBRATION_FACE_COUNT][DEEP_CALIBRATION_AXES];
static uint8_t FacesCaptured;
static int8_t CurrentFace;
static FaceAccumulator_t Accumulator;
static const Face_t Faces[DEEP_CALIBRATION_FACE_COUNT] = {
    { 1, +1.0f }, { 1, -1.0f },     // Y
    { 0, +1.0f }, { 0, -1.0f },     // X
    { 2, +1.0f }, { 2, -1.0f },     // Z
};

static void ResetAccumulator(void) {
    Accumulator = (FaceAccumulator_t){0};
}

static int8_t DetectFace(const float Accel[DEEP_CALIBRATION_AXES]) {
    for (uint8_t face = 0; face < DEEP_CALIBRATION_FACE_COUNT; face++) {
        if (Accel[Faces[face].Axis] * Faces[face].Sign >= DEEP_CALIBRATION_ACCEL_THRESHOLD) {
            return (int8_t)face;
        }
    }
    return -1;
}

static bool SaveCalibration(SystemContext_t *ctx) {
    float Bias[DEEP_CALIBRATION_AXES];
    float Scale[DEEP_CALIBRATION_AXES];

    for (uint8_t pair = 0; pair < DEEP_CALIBRATION_FACE_COUNT / 2; pair++) {
        const uint8_t Axis = Faces[2 * pair].Axis;
        CalculateAccelerometerAxisCalibration(FaceMean[2 * pair][Axis], FaceMean[2 * pair + 1][Axis], &Bias[Axis], &Scale[Axis]);
    }

    const AccelCalibration_t Cal = {
        .BiasX = Bias[0], .BiasY = Bias[1], .BiasZ = Bias[2],
        .ScaleX = Scale[0], .ScaleY = Scale[1], .ScaleZ = Scale[2],
    };

    if (W25Q_WriteAccelCal(&Cal) != HAL_OK) {
        return false;
    }

    W25Q_LoadAccelCal(ctx);
    return true;
}

void DeepCalibrationStateEntry(SystemContext_t *ctx) {
    (void)ctx;

    FacesCaptured = 0;
    CurrentFace = -1;
    ResetAccumulator();
}

SystemState_t DeepCalibrationStateHandler(SystemContext_t *Context, FlightData_t FlightData) {
    const float Accel[DEEP_CALIBRATION_AXES] = {
        FlightData.AccelX,
        FlightData.AccelY,
        FlightData.AccelZ
    };

    const int8_t DetectedFace = IsGyroscopeStill(FlightData, DEEP_CALIBRATION_GYRO_MAX_DPS) ? DetectFace(Accel) : -1;

    if (DetectedFace != CurrentFace) {
        CurrentFace = DetectedFace;
        ResetAccumulator();
    }

    if (DetectedFace < 0 || (FacesCaptured & (1u << DetectedFace))) {
        return STATE_DEEP_CALIBRATION;
    }

    if (Accumulator.Discarded < DEEP_CALIBRATION_DISCARD_SAMPLES) {
        Accumulator.Discarded++;
        return STATE_DEEP_CALIBRATION;
    }

    for (uint8_t i = 0; i < DEEP_CALIBRATION_AXES; i++) {
        Accumulator.Sum[i] += Accel[i];
    }

    if (++Accumulator.Samples >= DEEP_CALIBRATION_SAMPLES) {
        for (uint8_t i = 0; i < DEEP_CALIBRATION_AXES; i++) {
            FaceMean[DetectedFace][i] = Accumulator.Sum[i] / (float)Accumulator.Samples;
        }

        FacesCaptured |= (1u << DetectedFace);
        ResetAccumulator();

        if (FacesCaptured == DEEP_CALIBRATION_ALL_FACES) {
            Context->DeepCalibrationComplete = SaveCalibration(Context);
            return STATE_CALIBRATION;
        }
    }

    return STATE_DEEP_CALIBRATION;
}
