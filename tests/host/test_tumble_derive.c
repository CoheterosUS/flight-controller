#include "Utils/ImuTumbleCal.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int Failures;

static void Check(bool Condition, const char *Message)
{
    if (!Condition) {
        fprintf(stderr, "FAIL: %s\n", Message);
        Failures++;
    }
}

int TestImuTumbleDeriveQ(void)
{
    const float Targets[6][3] = {
        {9.81f, 0.0f, 0.0f}, {-9.81f, 0.0f, 0.0f},
        {0.0f, 9.81f, 0.0f}, {0.0f, -9.81f, 0.0f},
        {0.0f, 0.0f, 9.81f}, {0.0f, 0.0f, -9.81f}
    };
    ImuTumble_t T;
    ImuCalibration_t Solved;
    ImuTumbleQuality_t Quality;
    float DerivedQ[9];
    float DetQ;

    ImuTumble_Reset(&T);
    for (uint8_t Pose = 0u; Pose < 6u; Pose++) {
        float Raw[3] = {
            Targets[Pose][1],
            -Targets[Pose][0] * 0.5f,
            Targets[Pose][2] / 1.5f
        };
        for (int Sample = 0; Sample < 4; Sample++) ImuTumble_AddSample(&T, Pose, Raw);
    }

    Check(ImuTumble_Solve(&T, &Solved, &Quality), "tumble solve succeeds for derive-Q fixture");
    Check(ImuTumble_DeriveQ(Solved.M, DerivedQ, &DetQ), "derive-Q succeeds");
    for (int I = 0; I < 9; I++) {
        Check(fabsf(DerivedQ[I] - Solved.Q[I]) < 0.00001f,
              "derive-Q matches solve-Q");
    }
    Check(fabsf(DetQ - Quality.DetQ) < 0.00001f, "derive-Q determinant matches solve");
    return Failures;
}
