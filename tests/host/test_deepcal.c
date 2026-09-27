#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Utils/DeepCalGesture.h"
#include "Utils/DeepCalSequencer.h"
#include "Utils/configuration.h"

#define TEST_G 9.81f
#define TEST_EPSILON 0.01f

static int Check(bool Condition, const char *Message)
{
    if (!Condition) {
        fprintf(stderr, "FAIL: %s\n", Message);
        return 1;
    }
    return 0;
}

static const float MTrue[9] = {
    1.015f, 0.012f, -0.008f,
   -0.006f, 0.985f,  0.018f,
    0.009f,-0.011f,  1.025f
};

static const float Offset[3] = {0.12f, -0.08f, 0.05f};

static bool Invert3(const float A[9], float Out[9])
{
    const float Det = A[0] * (A[4] * A[8] - A[5] * A[7])
                    - A[1] * (A[3] * A[8] - A[5] * A[6])
                    + A[2] * (A[3] * A[7] - A[4] * A[6]);
    if (fabsf(Det) < 0.001f) {
        return false;
    }

    Out[0] = (A[4] * A[8] - A[5] * A[7]) / Det;
    Out[1] = (A[2] * A[7] - A[1] * A[8]) / Det;
    Out[2] = (A[1] * A[5] - A[2] * A[4]) / Det;
    Out[3] = (A[5] * A[6] - A[3] * A[8]) / Det;
    Out[4] = (A[0] * A[8] - A[2] * A[6]) / Det;
    Out[5] = (A[2] * A[3] - A[0] * A[5]) / Det;
    Out[6] = (A[3] * A[7] - A[4] * A[6]) / Det;
    Out[7] = (A[1] * A[6] - A[0] * A[7]) / Det;
    Out[8] = (A[0] * A[4] - A[1] * A[3]) / Det;
    return true;
}

static void PoseTarget(uint8_t Pose, float Target[3])
{
    memset(Target, 0, 3u * sizeof(float));
    Target[Pose / 2u] = ((Pose & 1u) == 0u) ? TEST_G : -TEST_G;
}

static void RawForPose(uint8_t Pose, float Raw[3])
{
    float Inverse[9];
    float Target[3];
    PoseTarget(Pose, Target);
    for (uint8_t Axis = 0u; Axis < 3u; Axis++) {
        Target[Axis] -= Offset[Axis];
    }
    (void)Invert3(MTrue, Inverse);
    for (uint8_t Row = 0u; Row < 3u; Row++) {
        Raw[Row] = Inverse[Row * 3u] * Target[0]
                 + Inverse[Row * 3u + 1u] * Target[1]
                 + Inverse[Row * 3u + 2u] * Target[2];
    }
}

static void SampleForPose(uint8_t Pose, uint32_t Sample, float Raw[3])
{
    RawForPose(Pose, Raw);
    Raw[0] += ((float)(Sample % 3u) - 1.0f) * 0.001f;
    Raw[1] += ((float)(Sample % 5u) - 2.0f) * 0.001f;
    Raw[2] += ((float)(Sample % 7u) - 3.0f) * 0.001f;
}

static int TestGesture(void)
{
    DeepCalGesture_t Gesture;
    const float Gyro[3] = {0.0f, 0.0f, 0.0f};
    float Accel[3] = {0.0f, TEST_G, 0.0f};
    DeepCalGesture_Reset(&Gesture);

    if (Check(!DeepCalGesture_Update(&Gesture, Accel, Gyro, 100u), "gesture starts on first valid sample") != 0
        || Check(!DeepCalGesture_Update(&Gesture, Accel, Gyro, 100u + DEEP_CAL_HOLD_MS - 1u), "gesture waits for the full hold") != 0
        || Check(DeepCalGesture_Update(&Gesture, Accel, Gyro, 100u + DEEP_CAL_HOLD_MS), "gesture fires at the hold deadline") != 0) {
        return 1;
    }

    Accel[0] = DEEP_CAL_HOLD_LATERAL_MAX_G * TEST_G + 0.01f;
    if (Check(!DeepCalGesture_Update(&Gesture, Accel, Gyro, 101u + DEEP_CAL_HOLD_MS), "lateral violation resets") != 0) {
        return 1;
    }
    Accel[0] = 0.0f;
    if (Check(!DeepCalGesture_Update(&Gesture, Accel, Gyro, 101u + 2u * DEEP_CAL_HOLD_MS), "lateral reset needs a new hold") != 0) {
        return 1;
    }

    Accel[1] = TEST_G * (1.0f + (float)DEEP_CAL_HOLD_G_BAND_PCT / 100.0f + 0.01f);
    if (Check(!DeepCalGesture_Update(&Gesture, Accel, Gyro, 200u), "band violation blocks gesture") != 0) {
        return 1;
    }
    Accel[1] = TEST_G;
    Accel[0] = 0.0f;
    if (Check(!DeepCalGesture_Update(&Gesture, Accel, Gyro, 300u), "valid sample starts after band violation") != 0) {
        return 1;
    }

    Accel[1] = -TEST_G;
    if (Check(!DeepCalGesture_Update(&Gesture, Accel, Gyro, 301u), "axis violation blocks gesture") != 0) {
        return 1;
    }
    Accel[1] = TEST_G;
    if (Check(!DeepCalGesture_Update(&Gesture, Accel, Gyro, 400u), "valid sample starts after axis violation") != 0) {
        return 1;
    }

    float MotionGyro[3] = {DEEP_CAL_HOLD_GYRO_MAX_DPS + 0.01f, 0.0f, 0.0f};
    if (Check(!DeepCalGesture_Update(&Gesture, Accel, MotionGyro, 401u), "gyro violation blocks gesture") != 0) {
        return 1;
    }

    DeepCalGesture_Reset(&Gesture);
    const uint32_t WrappedStart = UINT32_MAX - 100u;
    if (Check(!DeepCalGesture_Update(&Gesture, Accel, Gyro, WrappedStart), "wrap test starts") != 0
        || Check(DeepCalGesture_Update(&Gesture, Accel, Gyro, WrappedStart + DEEP_CAL_HOLD_MS), "gesture handles tick wrap") != 0) {
        return 1;
    }
    return 0;
}

static int RunSequence(bool Mirrored, bool ForceTimeout, bool ForceMotion, bool WrongPose,
                       uint32_t *DoneMs, uint8_t *PromptCounts, ImuCalibration_t *Result)
{
    DeepCalSeq_t Sequence;
    DeepCalSeqOut_t Output;
    uint32_t Now = 0u;
    uint32_t PromptStart = 0u;
    uint32_t SampleNumber = 0u;
    uint8_t LastPose = 0u;
    bool WrongInjected = false;
    bool WrongActive = false;
    bool MotionInjected = false;
    DeepCalSeq_Start(&Sequence, 0u);

    for (uint32_t Iteration = 0u; Iteration < 6000u; Iteration++, Now += 100u) {
        float RawAccel[3] = {123.0f, -456.0f, 789.0f};
        float RawGyro[3] = {0.0f, 0.0f, 0.0f};
        uint8_t PoseIndex = LastPose == 0u ? 0u : (uint8_t)(LastPose - 1u);
        if (LastPose != 0u) {
            uint8_t SourcePose = PoseIndex;
            if (Mirrored && SourcePose >= 4u) {
                SourcePose = (uint8_t)(SourcePose == 4u ? 5u : 4u);
            }
            SampleForPose(SourcePose, SampleNumber++, RawAccel);
        }

        if (ForceTimeout) {
            RawAccel[0] = 0.0f;
            RawAccel[1] = 0.0f;
            RawAccel[2] = 0.0f;
        }
        if (ForceMotion && LastPose == 1u && !MotionInjected
            && (uint32_t)(Now - PromptStart) >= DEEP_CAL_POSE_SETTLE_MS + DEEP_CAL_POSE_CHECK_WINDOW_MS + 100u) {
            RawGyro[0] = DEEP_CAL_GYRO_MOTION_MAX_DPS + 1.0f;
            MotionInjected = true;
        }
        if (WrongPose && LastPose == 2u && !WrongInjected
            && (uint32_t)(Now - PromptStart) >= DEEP_CAL_POSE_SETTLE_MS) {
            WrongActive = true;
            WrongInjected = true;
        }
        if (WrongPose && WrongActive) {
            RawForPose(0u, RawAccel);
        }
        DeepCalSeq_Step(&Sequence, Now, RawAccel, RawGyro, &Output);
        if (Output.PlayPattern) {
            if (PromptCounts != NULL && Output.Pattern >= BUZZ_POSE_1 && Output.Pattern <= BUZZ_POSE_6) {
                PromptCounts[Output.Pattern - BUZZ_POSE_1]++;
            }
            PromptStart = Now;
            if (WrongActive && Output.Pattern == BUZZ_POSE_2) {
                WrongActive = false;
            }
        }
        LastPose = Output.Pose;
        if (Output.Done) {
            if (DoneMs != NULL) {
                *DoneMs = Now;
            }
            if (Result != NULL) {
                *Result = Output.Result;
            }
            return Output.Success ? 1 : 0;
        }
    }
    return -1;
}

static int TestNominal(void)
{
    uint32_t DoneMs = 0u;
    uint8_t PromptCounts[6] = {0};
    ImuCalibration_t Result;
    int SequenceResult = RunSequence(false, false, false, false, &DoneMs, PromptCounts, &Result);
    if (Check(SequenceResult == 1, "nominal sequence succeeds") != 0
        || Check(DoneMs >= 240000u && DoneMs <= 270000u, "nominal sequence duration is bounded") != 0) {
        return 1;
    }
    for (uint8_t Pose = 0u; Pose < 6u; Pose++) {
        if (Check(PromptCounts[Pose] == 1u, "nominal prompts are not repeated") != 0) {
            return 1;
        }
    }
    for (uint8_t Index = 0u; Index < 9u; Index++) {
        if (Check(fabsf(Result.M[Index] - MTrue[Index]) < TEST_EPSILON, "nominal result matches M_true") != 0) {
            return 1;
        }
    }
    return 0;
}

static int TestRestarts(void)
{
    uint8_t PromptCounts[6] = {0};
    int MotionResult = RunSequence(false, false, true, false, NULL, PromptCounts, NULL);
    if (Check(MotionResult == 1,
              "motion during sampling recovers with a restart") != 0
        || Check(PromptCounts[0] == 2u, "motion restart replays the prompt") != 0) {
        return 1;
    }

    memset(PromptCounts, 0, sizeof(PromptCounts));
    if (Check(RunSequence(false, false, false, true, NULL, PromptCounts, NULL) == 1,
              "wrong-way pose recovers with a restart") != 0
        || Check(PromptCounts[1] == 2u, "wrong-way restart replays the prompt") != 0) {
        return 1;
    }

    if (Check(RunSequence(false, true, false, false, NULL, NULL, NULL) == 0,
              "overall timeout fails") != 0
        || Check(RunSequence(true, false, false, false, NULL, NULL, NULL) == 0,
                 "mirrored pose sequence fails") != 0) {
        return 1;
    }
    return 0;
}

static int TestMaximumRestarts(void)
{
    uint8_t PromptCounts[6] = {0};
    if (Check(RunSequence(false, false, true, false, NULL, PromptCounts, NULL) == 1,
              "single motion restart remains recoverable") != 0) {
        return 1;
    }

    DeepCalSeq_t Sequence;
    DeepCalSeqOut_t Output;
    uint32_t Now = 0u;
    uint32_t PromptStart = 0u;
    uint8_t LastPose = 0u;
    DeepCalSeq_Start(&Sequence, 0u);
    for (uint32_t Iteration = 0u; Iteration < 6000u; Iteration++, Now += 100u) {
        float RawAccel[3] = {TEST_G, 0.0f, 0.0f};
        float RawGyro[3] = {0.0f, 0.0f, 0.0f};
        if (LastPose == 1u && (uint32_t)(Now - PromptStart) >= DEEP_CAL_POSE_SETTLE_MS) {
            RawGyro[0] = DEEP_CAL_GYRO_MOTION_MAX_DPS + 1.0f;
        }
        DeepCalSeq_Step(&Sequence, Now, RawAccel, RawGyro, &Output);
        if (Output.PlayPattern) {
            PromptStart = Now;
        }
        LastPose = Output.Pose;
        if (Output.Done) {
            return Check(!Output.Success, "more than the maximum restarts fails");
        }
    }
    return Check(false, "maximum restart test completed");
}

int main(void)
{
    if (TestGesture() != 0 || TestNominal() != 0 || TestRestarts() != 0
        || TestMaximumRestarts() != 0) {
        return 1;
    }
    puts("deep calibration tests: PASS");
    return 0;
}
