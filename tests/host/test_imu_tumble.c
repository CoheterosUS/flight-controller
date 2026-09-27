#include "Utils/ImuTumbleCal.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define G 9.81f
#define SAMPLE_COUNT 6000u
#define NOISE_SIGMA 0.02f

static uint32_t RandomState = 0x13579BDFu;
static bool HaveGaussian;
static float GaussianValue;

static void Check(bool Condition, const char *Message)
{
    if (!Condition) {
        fprintf(stderr, "FAIL: %s\n", Message);
        exit(EXIT_FAILURE);
    }
}

static float UniformRandom(void)
{
    RandomState = RandomState * 1664525u + 1013904223u;
    return ((float)(RandomState >> 8) + 1.0f) / 16777217.0f;
}

static float GaussianRandom(void)
{
    if (HaveGaussian) {
        HaveGaussian = false;
        return GaussianValue;
    }

    const float U1 = UniformRandom();
    const float U2 = UniformRandom();
    const float Radius = sqrtf(-2.0f * logf(U1));
    const float Angle = 6.2831853071795864769f * U2;
    GaussianValue = Radius * sinf(Angle);
    HaveGaussian = true;
    return Radius * cosf(Angle);
}

static void Multiply3(const float M[9], const float V[3], float Result[3])
{
    for (unsigned Row = 0u; Row < 3u; Row++) {
        Result[Row] = M[Row * 3u] * V[0] + M[Row * 3u + 1u] * V[1] + M[Row * 3u + 2u] * V[2];
    }
}

static bool Invert3(const float M[9], float Inverse[9])
{
    const float Det = M[0] * (M[4] * M[8] - M[5] * M[7])
                    - M[1] * (M[3] * M[8] - M[5] * M[6])
                    + M[2] * (M[3] * M[7] - M[4] * M[6]);
    if (fabsf(Det) < 1.0e-6f) {
        return false;
    }
    Inverse[0] = (M[4] * M[8] - M[5] * M[7]) / Det;
    Inverse[1] = (M[2] * M[7] - M[1] * M[8]) / Det;
    Inverse[2] = (M[1] * M[5] - M[2] * M[4]) / Det;
    Inverse[3] = (M[5] * M[6] - M[3] * M[8]) / Det;
    Inverse[4] = (M[0] * M[8] - M[2] * M[6]) / Det;
    Inverse[5] = (M[2] * M[3] - M[0] * M[5]) / Det;
    Inverse[6] = (M[3] * M[7] - M[4] * M[6]) / Det;
    Inverse[7] = (M[1] * M[6] - M[0] * M[7]) / Det;
    Inverse[8] = (M[0] * M[4] - M[1] * M[3]) / Det;
    return true;
}

static void BuildTrueMap(float M[9], float Q[9])
{
    const float Rotation[9] = {
         0.0f, -1.0f, 0.0f,
         1.0f,  0.0f, 0.0f,
         0.0f,  0.0f, 1.0f
    };
    const float Upper[9] = {
        1.01f,  0.012f, -0.006f,
        0.0f,  0.98f,    0.009f,
        0.0f,  0.0f,     1.03f
    };
    memcpy(Q, Rotation, sizeof(Rotation));
    for (unsigned Row = 0u; Row < 3u; Row++) {
        for (unsigned Col = 0u; Col < 3u; Col++) {
            M[Row * 3u + Col] = Rotation[Row * 3u] * Upper[Col]
                              + Rotation[Row * 3u + 1u] * Upper[3u + Col]
                              + Rotation[Row * 3u + 2u] * Upper[6u + Col];
        }
    }
}

static void TargetForPose(unsigned Pose, float Target[3])
{
    memset(Target, 0, 3u * sizeof(float));
    Target[Pose / 2u] = (Pose % 2u == 0u) ? G : -G;
}

static void FillSynthetic(ImuTumble_t *T, const float M[9], const float Offset[3], bool Mirrored)
{
    float Inverse[9];
    Check(Invert3(M, Inverse), "synthetic map is invertible");
    for (unsigned Pose = 0u; Pose < 6u; Pose++) {
        unsigned ActualPose = Pose;
        if (Mirrored && Pose == 4u) {
            ActualPose = 5u;
        } else if (Mirrored && Pose == 5u) {
            ActualPose = 4u;
        }
        float Target[3];
        float Adjusted[3];
        TargetForPose(ActualPose, Target);
        for (unsigned I = 0u; I < 3u; I++) {
            Adjusted[I] = Target[I] - Offset[I];
        }
        for (unsigned Sample = 0u; Sample < SAMPLE_COUNT; Sample++) {
            float Raw[3];
            Multiply3(Inverse, Adjusted, Raw);
            for (unsigned I = 0u; I < 3u; I++) {
                Raw[I] += NOISE_SIGMA * GaussianRandom();
            }
            ImuTumble_AddSample(T, (uint8_t)Pose, Raw);
        }
    }
}

static void TestRecovery(void)
{
    float MTrue[9];
    float QTrue[9];
    const float Offset[3] = {0.18f, -0.11f, 0.07f};
    ImuTumble_t T;
    ImuCalibration_t Calibration;
    ImuTumbleQuality_t Quality;
    ImuTumble_Reset(&T);
    BuildTrueMap(MTrue, QTrue);
    FillSynthetic(&T, MTrue, Offset, false);
    Check(ImuTumble_Solve(&T, &Calibration, &Quality), "synthetic recovery solves");
    Check(Calibration.Valid, "synthetic result is valid");

    float ErrorSquared = 0.0f;
    float TrueSquared = 0.0f;
    for (unsigned I = 0u; I < 9u; I++) {
        const float Error = Calibration.M[I] - MTrue[I];
        ErrorSquared += Error * Error;
        TrueSquared += MTrue[I] * MTrue[I];
        Check(fabsf(Calibration.Q[I] - QTrue[I]) < 0.01f, "QR rotation is recovered");
    }
    Check(sqrtf(ErrorSquared / TrueSquared) < 0.005f, "map recovery is within 0.5 percent");
    for (unsigned Pose = 0u; Pose < 6u; Pose++) {
        float Mean[3];
        float Corrected[3];
        float Target[3];
        Check(ImuTumble_PoseMean(&T, (uint8_t)Pose, Mean), "pose mean is available");
        Multiply3(Calibration.M, Mean, Corrected);
        for (unsigned I = 0u; I < 3u; I++) {
            Corrected[I] += Quality.Offsets[I];
        }
        TargetForPose(Pose, Target);
        for (unsigned I = 0u; I < 3u; I++) {
            Check(fabsf(Corrected[I] - Target[I]) < 0.05f, "calibrated pose is accurate");
        }
        Check(Quality.PoseResidual[Pose] < DEEP_CAL_NORM_RESIDUAL_MAX, "pose residual is within limit");
    }
    Check(fabsf(Quality.DetQ - 1.0f) < 0.01f, "accepted Q has determinant plus one");
    for (unsigned I = 0u; I < 3u; I++) {
        Check(Quality.R[I * 3u + I] > 0.0f, "R diagonal is positive");
    }
}

static void TestMirrored(void)
{
    float MTrue[9];
    float QTrue[9];
    const float Offset[3] = {0.18f, -0.11f, 0.07f};
    ImuTumble_t T;
    ImuCalibration_t Calibration;
    ImuTumbleQuality_t Quality;
    ImuTumble_Reset(&T);
    BuildTrueMap(MTrue, QTrue);
    FillSynthetic(&T, MTrue, Offset, true);
    Check(!ImuTumble_Solve(&T, &Calibration, &Quality), "mirrored order is rejected");
    Check(Quality.Failure == IMU_TUMBLE_FAILURE_DET_Q, "mirrored order fails determinant check");
    Check(Quality.DetQ < -0.9f, "mirrored order has negative determinant");
}

static void AddPoseMean(ImuTumble_t *T, unsigned Pose, float X, float Y, float Z)
{
    const float Mean[3] = {X, Y, Z};
    for (unsigned Sample = 0u; Sample < 10u; Sample++) {
        ImuTumble_AddSample(T, (uint8_t)Pose, Mean);
    }
    Check(ImuTumble_PoseCommit(T, (uint8_t)Pose), "pose commits");
}

static void TestPoseChecks(void)
{
    ImuTumble_t T;
    ImuTumble_Reset(&T);
    AddPoseMean(&T, 0u, G, 0.0f, 0.0f);
    AddPoseMean(&T, 1u, -G, 0.0f, 0.0f);
    AddPoseMean(&T, 2u, 0.0f, G, 0.0f);
    const float WrongWay[3] = {0.0f, G, 0.0f};
    Check(ImuTumble_CheckPose(&T, 3u, WrongWay) == IMU_TUMBLE_POSE_INCONSISTENT_WITH_PREVIOUS,
          "wrong-way pose is rejected");
    const float WrongMagnitude[3] = {0.5f * G, 0.0f, 0.0f};
    Check(ImuTumble_CheckPose(&T, 3u, WrongMagnitude) == IMU_TUMBLE_POSE_BAD_MAGNITUDE,
          "wrong magnitude is rejected");

    const float NegativeY[3] = {0.0f, -G, 0.0f};
    Check(ImuTumble_CheckPose(&T, 3u, NegativeY) == IMU_TUMBLE_POSE_OK, "-Y pose is accepted");
    AddPoseMean(&T, 3u, 0.0f, -G, 0.0f);
    const float PositiveZ[3] = {0.0f, 0.0f, G};
    const float SameAxisAsY[3] = {0.0f, G, 0.0f};
    Check(ImuTumble_CheckPose(&T, 4u, PositiveZ) == IMU_TUMBLE_POSE_OK,
          "+Z pose on the third axis is accepted");
    Check(ImuTumble_CheckPose(&T, 4u, SameAxisAsY) == IMU_TUMBLE_POSE_INCONSISTENT_WITH_PREVIOUS,
          "pose 4 on an already used axis is rejected");
    AddPoseMean(&T, 4u, 0.0f, 0.0f, G);
    const float NegativeZ[3] = {0.0f, 0.0f, -G};
    Check(ImuTumble_CheckPose(&T, 5u, NegativeZ) == IMU_TUMBLE_POSE_OK,
          "-Z pose opposite to +Z is accepted");
    Check(ImuTumble_CheckPose(&T, 5u, PositiveZ) == IMU_TUMBLE_POSE_INCONSISTENT_WITH_PREVIOUS,
          "pose 5 not opposite to pose 4 is rejected");
}

static void TestSingular(void)
{
    ImuTumble_t T;
    ImuCalibration_t Calibration;
    ImuTumbleQuality_t Quality;
    const float Raw[3] = {G, 0.0f, 0.0f};
    ImuTumble_Reset(&T);
    for (unsigned Pose = 0u; Pose < 6u; Pose++) {
        for (unsigned Sample = 0u; Sample < 20u; Sample++) {
            ImuTumble_AddSample(&T, (uint8_t)Pose, Raw);
        }
    }
    Check(!ImuTumble_Solve(&T, &Calibration, &Quality), "singular data is rejected");
    Check(Quality.Failure == IMU_TUMBLE_FAILURE_SINGULAR, "singular data reports failure");
}

static void TestNonFiniteInputs(void)
{
    ImuTumble_t T;
    ImuCalibration_t Calibration;
    ImuTumbleQuality_t Quality;
    const float Good[3] = {G, 0.0f, 0.0f};
    const float Bad[3] = {NAN, 0.0f, 0.0f};
    float Q[9];
    float DetQ;

    ImuTumble_Reset(&T);
    ImuTumble_AddSample(&T, 0u, Bad);
    Check(T.Count[0] == 0u, "non-finite tumble sample is ignored");
    for (unsigned Pose = 0u; Pose < 6u; Pose++) {
        for (unsigned Sample = 0u; Sample < 20u; Sample++) {
            ImuTumble_AddSample(&T, (uint8_t)Pose, Good);
        }
    }
    T.Outer[0][0] = NAN;
    Check(!ImuTumble_Solve(&T, &Calibration, &Quality), "non-finite tumble accumulator is rejected");

    const float BadM[9] = {1.0f, 0.0f, 0.0f, 0.0f, INFINITY, 0.0f,
                           0.0f, 0.0f, 1.0f};
    Check(!ImuTumble_DeriveQ(BadM, Q, &DetQ), "non-finite M is rejected by derive-Q");
}

int main(void)
{
    TestRecovery();
    TestMirrored();
    TestPoseChecks();
    TestSingular();
    TestNonFiniteInputs();
    puts("imu tumble host tests passed");
    return EXIT_SUCCESS;
}
