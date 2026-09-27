#ifndef DEEP_CAL_SEQUENCER_H
#define DEEP_CAL_SEQUENCER_H

#include <stdbool.h>
#include <stdint.h>

#include "Sensors/Sensors.h"
#include "Utils/ImuTumbleCal.h"

typedef struct {
    uint32_t StartMs;
    uint32_t PromptStartMs;
    uint32_t CheckStartMs;
    uint32_t SampleStartMs;
    uint8_t Pose;
    uint8_t Restarts[IMU_TUMBLE_POSE_COUNT];
    uint8_t Phase;
    uint32_t CheckCount;
    double CheckSum[3];
    float LastAccel[3];
    bool HasLastAccel;
    bool Done;
    bool Success;
    ImuTumbleFailureReason_t Failure;
    ImuTumble_t Tumble;
    ImuCalibration_t Result;
} DeepCalSeq_t;

typedef struct {
    bool PlayPattern;
    BuzzerPattern_t Pattern;
    uint8_t Pose;
    bool Done;
    bool Success;
    ImuCalibration_t Result;
    ImuTumbleFailureReason_t Failure;
} DeepCalSeqOut_t;

void DeepCalSeq_Start(DeepCalSeq_t *S, uint32_t NowMs);
void DeepCalSeq_Step(DeepCalSeq_t *S,
                     uint32_t NowMs,
                     const float RawAccel[3],
                     const float RawGyro[3],
                     DeepCalSeqOut_t *Out);

#endif
