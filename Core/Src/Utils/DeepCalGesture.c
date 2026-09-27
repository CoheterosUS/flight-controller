#include "Utils/DeepCalGesture.h"

#include <math.h>
#include <string.h>

#include "Utils/configuration.h"

#define DEEP_CAL_G 9.81f

void DeepCalGesture_Reset(DeepCalGesture_t *G)
{
    if (G != NULL) {
        memset(G, 0, sizeof(*G));
    }
}

static bool DeepCalGesture_IsValid(const float RawAccel[3], const float RawGyro[3])
{
    const uint8_t HoldAxis = DEEP_CAL_HOLD_AXIS_RAW;
    const float HoldValue = RawAccel[HoldAxis] * (float)DEEP_CAL_HOLD_AXIS_SIGN;
    const float Band = DEEP_CAL_G * ((float)DEEP_CAL_HOLD_G_BAND_PCT / 100.0f);
    const float GyroNorm = sqrtf(RawGyro[0] * RawGyro[0]
                               + RawGyro[1] * RawGyro[1]
                               + RawGyro[2] * RawGyro[2]);

    if (fabsf(HoldValue - DEEP_CAL_G) > Band) {
        return false;
    }

    for (uint8_t Axis = 0u; Axis < 3u; Axis++) {
        if (Axis != HoldAxis && fabsf(RawAccel[Axis]) > DEEP_CAL_HOLD_LATERAL_MAX_G * DEEP_CAL_G) {
            return false;
        }
    }

    return GyroNorm <= DEEP_CAL_HOLD_GYRO_MAX_DPS;
}

bool DeepCalGesture_Update(DeepCalGesture_t *G,
                           const float RawAccel[3],
                           const float RawGyro[3],
                           uint32_t NowMs)
{
    if (G == NULL || RawAccel == NULL || RawGyro == NULL) {
        return false;
    }

    if (!DeepCalGesture_IsValid(RawAccel, RawGyro)) {
        G->Holding = false;
        G->HoldStartMs = 0u;
        return false;
    }

    if (!G->Holding) {
        G->Holding = true;
        G->HoldStartMs = NowMs;
        return false;
    }

    return (uint32_t)(NowMs - G->HoldStartMs) >= DEEP_CAL_HOLD_MS;
}
