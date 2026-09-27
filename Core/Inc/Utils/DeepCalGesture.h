#ifndef DEEP_CAL_GESTURE_H
#define DEEP_CAL_GESTURE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool Holding;
    uint32_t HoldStartMs;
} DeepCalGesture_t;

void DeepCalGesture_Reset(DeepCalGesture_t *G);
bool DeepCalGesture_Update(DeepCalGesture_t *G,
                           const float RawAccel[3],
                           const float RawGyro[3],
                           uint32_t NowMs);

#endif
