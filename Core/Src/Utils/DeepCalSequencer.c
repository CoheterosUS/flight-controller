#include "Utils/DeepCalSequencer.h"

#include <math.h>
#include <string.h>

#include "Utils/configuration.h"

#define DEEP_CAL_G 9.81f

_Static_assert(DEEP_CAL_POSE_SETTLE_MS >
               (6u * (BUZZER_POSE_ON_MS + BUZZER_POSE_OFF_MS)),
               "deep calibration settle time must exceed the longest pose prompt");

enum {
    DEEP_CAL_SEQ_WAIT_ENTERED = 0u,
    DEEP_CAL_SEQ_SETTLE,
    DEEP_CAL_SEQ_CHECK,
    DEEP_CAL_SEQ_SAMPLE
};

static uint32_t DeepCalSeq_Elapsed(uint32_t NowMs, uint32_t SinceMs)
{
    return (uint32_t)(NowMs - SinceMs);
}

static BuzzerPattern_t DeepCalSeq_Pattern(uint8_t Pose)
{
    return (BuzzerPattern_t)(BUZZ_POSE_1 + Pose);
}

static void DeepCalSeq_ResetCheck(DeepCalSeq_t *S)
{
    S->CheckStartMs = 0u;
    S->CheckCount = 0u;
    memset(S->CheckSum, 0, sizeof(S->CheckSum));
    S->HasLastAccel = false;
}

static void DeepCalSeq_Fail(DeepCalSeq_t *S, ImuTumbleFailureReason_t Failure)
{
    S->Done = true;
    S->Success = false;
    S->Failure = Failure;
}

static bool DeepCalSeq_Motion(const float RawGyro[3])
{
    const float Norm = sqrtf(RawGyro[0] * RawGyro[0]
                           + RawGyro[1] * RawGyro[1]
                           + RawGyro[2] * RawGyro[2]);
    return Norm > DEEP_CAL_GYRO_MOTION_MAX_DPS;
}

static void DeepCalSeq_Restart(DeepCalSeq_t *S, uint32_t NowMs, DeepCalSeqOut_t *Out)
{
    if (S->Restarts[S->Pose] >= DEEP_CAL_POSE_MAX_RESTARTS) {
        DeepCalSeq_Fail(S, IMU_TUMBLE_FAILURE_INSUFFICIENT_SAMPLES);
        return;
    }

    S->Restarts[S->Pose]++;
    ImuTumble_PoseBegin(&S->Tumble, S->Pose);
    DeepCalSeq_ResetCheck(S);
    S->PromptStartMs = NowMs;
    S->Phase = DEEP_CAL_SEQ_SETTLE;
    Out->PlayPattern = true;
    Out->Pattern = DeepCalSeq_Pattern(S->Pose);
}

static void DeepCalSeq_AccumulateCheck(DeepCalSeq_t *S, const float RawAccel[3])
{
    for (uint8_t Axis = 0u; Axis < 3u; Axis++) {
        S->CheckSum[Axis] += (double)RawAccel[Axis];
    }
    S->CheckCount++;
}

static bool DeepCalSeq_CheckMean(const DeepCalSeq_t *S, float Mean[3])
{
    if (S->CheckCount == 0u) {
        return false;
    }

    for (uint8_t Axis = 0u; Axis < 3u; Axis++) {
        Mean[Axis] = (float)(S->CheckSum[Axis] / (double)S->CheckCount);
    }
    return true;
}

static bool DeepCalSeq_PoseIsValid(const DeepCalSeq_t *S, const float Mean[3])
{
    return ImuTumble_CheckPose(&S->Tumble, S->Pose, Mean) == IMU_TUMBLE_POSE_OK;
}
    return S->Pose >= 4u && Status == IMU_TUMBLE_POSE_INCONSISTENT_WITH_PREVIOUS
        && DeepCalSeq_ValidThirdAxis(S, Mean);
}

static void DeepCalSeq_SetOutput(const DeepCalSeq_t *S, DeepCalSeqOut_t *Out)
{
    Out->Pose = (S->Pose < IMU_TUMBLE_POSE_COUNT) ? (uint8_t)(S->Pose + 1u) : 0u;
    Out->Done = S->Done;
    Out->Success = S->Success;
    Out->Result = S->Result;
    Out->Failure = S->Failure;
}

void DeepCalSeq_Start(DeepCalSeq_t *S, uint32_t NowMs)
{
    if (S == NULL) {
        return;
    }

    memset(S, 0, sizeof(*S));
    S->StartMs = NowMs;
    S->Phase = DEEP_CAL_SEQ_WAIT_ENTERED;
    S->Failure = IMU_TUMBLE_FAILURE_NONE;
    ImuTumble_Reset(&S->Tumble);
}

void DeepCalSeq_Step(DeepCalSeq_t *S,
                     uint32_t NowMs,
                     const float RawAccel[3],
                     const float RawGyro[3],
                     DeepCalSeqOut_t *Out)
{
    if (Out == NULL) {
        return;
    }
    memset(Out, 0, sizeof(*Out));
    Out->Pattern = BUZZ_NONE;

    if (S == NULL || RawAccel == NULL || RawGyro == NULL) {
        Out->Done = true;
        Out->Failure = IMU_TUMBLE_FAILURE_ARGUMENT;
        return;
    }

    if (!S->Done && DeepCalSeq_Elapsed(NowMs, S->StartMs) >= DEEP_CAL_TIMEOUT_MS) {
        DeepCalSeq_Fail(S, IMU_TUMBLE_FAILURE_INSUFFICIENT_SAMPLES);
    }

    if (!S->Done) {
        switch (S->Phase) {
        case DEEP_CAL_SEQ_WAIT_ENTERED:
            if (DeepCalSeq_Elapsed(NowMs, S->StartMs) >= BUZZER_DEEPCAL_ENTERED_MS) {
                S->Pose = 0u;
                ImuTumble_PoseBegin(&S->Tumble, S->Pose);
                DeepCalSeq_ResetCheck(S);
                S->PromptStartMs = NowMs;
                S->Phase = DEEP_CAL_SEQ_SETTLE;
                Out->PlayPattern = true;
                Out->Pattern = DeepCalSeq_Pattern(S->Pose);
            }
            break;

        case DEEP_CAL_SEQ_SETTLE:
            if (DeepCalSeq_Elapsed(NowMs, S->PromptStartMs) >= DEEP_CAL_POSE_SETTLE_MS) {
                S->CheckStartMs = NowMs;
                S->CheckCount = 0u;
                memset(S->CheckSum, 0, sizeof(S->CheckSum));
                S->Phase = DEEP_CAL_SEQ_CHECK;
            }
            break;

        case DEEP_CAL_SEQ_CHECK:
            if (DeepCalSeq_Motion(RawGyro)) {
                DeepCalSeq_Restart(S, NowMs, Out);
                break;
            }
            DeepCalSeq_AccumulateCheck(S, RawAccel);
            if (DeepCalSeq_Elapsed(NowMs, S->CheckStartMs) >= DEEP_CAL_POSE_CHECK_WINDOW_MS) {
                float Mean[3];
                if (!DeepCalSeq_CheckMean(S, Mean)
                    || !DeepCalSeq_PoseIsValid(S, Mean)) {
                    DeepCalSeq_Restart(S, NowMs, Out);
                    break;
                }
                S->SampleStartMs = NowMs;
                S->HasLastAccel = false;
                S->Phase = DEEP_CAL_SEQ_SAMPLE;
            }
            break;

        case DEEP_CAL_SEQ_SAMPLE:
            if (DeepCalSeq_Motion(RawGyro)) {
                DeepCalSeq_Restart(S, NowMs, Out);
                break;
            }
            if (!S->HasLastAccel || memcmp(S->LastAccel, RawAccel, sizeof(S->LastAccel)) != 0) {
                ImuTumble_AddSample(&S->Tumble, S->Pose, RawAccel);
                memcpy(S->LastAccel, RawAccel, sizeof(S->LastAccel));
                S->HasLastAccel = true;
            }
            if (DeepCalSeq_Elapsed(NowMs, S->SampleStartMs) >= DEEP_CAL_POSE_SAMPLE_MS) {
                if (!ImuTumble_PoseCommit(&S->Tumble, S->Pose)) {
                    DeepCalSeq_Fail(S, IMU_TUMBLE_FAILURE_INSUFFICIENT_SAMPLES);
                    break;
                }
                S->Pose++;
                if (S->Pose >= IMU_TUMBLE_POSE_COUNT) {
                    ImuTumbleQuality_t Quality;
                    S->Success = ImuTumble_Solve(&S->Tumble, &S->Result, &Quality);
                    S->Failure = Quality.Failure;
                    S->Done = true;
                    if (!S->Success && S->Failure == IMU_TUMBLE_FAILURE_NONE) {
                        S->Failure = IMU_TUMBLE_FAILURE_RESIDUAL;
                    }
                } else {
                    ImuTumble_PoseBegin(&S->Tumble, S->Pose);
                    DeepCalSeq_ResetCheck(S);
                    S->PromptStartMs = NowMs;
                    S->Phase = DEEP_CAL_SEQ_SETTLE;
                    Out->PlayPattern = true;
                    Out->Pattern = DeepCalSeq_Pattern(S->Pose);
                }
            }
            break;

        default:
            DeepCalSeq_Fail(S, IMU_TUMBLE_FAILURE_ARGUMENT);
            break;
        }
    }

    DeepCalSeq_SetOutput(S, Out);
}
