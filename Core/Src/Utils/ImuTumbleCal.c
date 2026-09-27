#include "Utils/ImuTumbleCal.h"

#include <math.h>
#include <string.h>

#include "dsp/matrix_functions.h"

#ifndef DEEP_CAL_POSE_G_BAND_PCT
#define DEEP_CAL_POSE_G_BAND_PCT 10
#endif
#ifndef DEEP_CAL_POSE_DOMINANT_MIN_G
#define DEEP_CAL_POSE_DOMINANT_MIN_G 0.8f
#endif
#ifndef DEEP_CAL_DET_TOL
#define DEEP_CAL_DET_TOL 0.1f
#endif
#ifndef DEEP_CAL_NORM_RESIDUAL_MAX
#define DEEP_CAL_NORM_RESIDUAL_MAX 0.02f
#endif

#define IMU_TUMBLE_G 9.81f
/* Normalized equations reject a pivot below 1e-12 of the largest entry. */
#define IMU_TUMBLE_PIVOT_REL_TOL 1.0e-12
/* CMSIS QR must produce QQ^T within 0.01 of the identity. */
#define IMU_TUMBLE_Q_ORTHOGONAL_TOL 0.01f
#define IMU_TUMBLE_QR_THRESHOLD 1.0e-12f

static const float ImuTumbleTargets[IMU_TUMBLE_POSE_COUNT][3] = {
    { IMU_TUMBLE_G, 0.0f, 0.0f },
    {-IMU_TUMBLE_G, 0.0f, 0.0f },
    { 0.0f, IMU_TUMBLE_G, 0.0f },
    { 0.0f,-IMU_TUMBLE_G, 0.0f },
    { 0.0f, 0.0f, IMU_TUMBLE_G },
    { 0.0f, 0.0f,-IMU_TUMBLE_G }
};

static float ImuTumble_Dot(const float A[3], const float B[3])
{
    return A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
}

static float ImuTumble_Norm(const float A[3])
{
    return sqrtf(ImuTumble_Dot(A, A));
}

static uint8_t ImuTumble_DominantAxis(const float A[3])
{
    uint8_t Axis = 0u;
    float Largest = fabsf(A[0]);

    for (uint8_t I = 1u; I < 3u; I++) {
        const float Value = fabsf(A[I]);
        if (Value > Largest) {
            Largest = Value;
            Axis = I;
        }
    }
    return Axis;
}

static bool ImuTumble_Opposite(const float A[3], const float B[3], float CosTolerance)
{
    const float Denominator = ImuTumble_Norm(A) * ImuTumble_Norm(B);
    if (Denominator <= 0.0f) {
        return false;
    }
    return ImuTumble_Dot(A, B) / Denominator <= -CosTolerance;
}

static bool ImuTumble_Perpendicular(const float A[3], const float B[3], float SinTolerance)
{
    const float Denominator = ImuTumble_Norm(A) * ImuTumble_Norm(B);
    if (Denominator <= 0.0f) {
        return false;
    }
    return fabsf(ImuTumble_Dot(A, B) / Denominator) <= SinTolerance;
}

static bool ImuTumble_IsCommitted(const ImuTumble_t *T, uint8_t Pose)
{
    return T != NULL && Pose < IMU_TUMBLE_POSE_COUNT && T->PoseCommitted[Pose];
}

static bool ImuTumble_ValidResultArguments(const ImuTumble_t *T,
                                           const ImuCalibration_t *Out,
                                           const ImuTumbleQuality_t *Quality)
{
    return T != NULL && Out != NULL && Quality != NULL;
}

void ImuTumble_Reset(ImuTumble_t *T)
{
    if (T != NULL) {
        memset(T, 0, sizeof(*T));
    }
}

void ImuTumble_PoseBegin(ImuTumble_t *T, uint8_t Pose)
{
    if (T == NULL || Pose >= IMU_TUMBLE_POSE_COUNT) {
        return;
    }

    T->Count[Pose] = 0u;
    memset(T->Sum[Pose], 0, sizeof(T->Sum[Pose]));
    memset(T->Outer[Pose], 0, sizeof(T->Outer[Pose]));
    T->PoseCommitted[Pose] = false;
    memset(T->PoseMean[Pose], 0, sizeof(T->PoseMean[Pose]));
}

bool ImuTumble_PoseMean(const ImuTumble_t *T, uint8_t Pose, float Mean[3])
{
    if (T == NULL || Mean == NULL || Pose >= IMU_TUMBLE_POSE_COUNT || T->Count[Pose] == 0u) {
        return false;
    }

    const double Count = (double)T->Count[Pose];
    for (uint8_t I = 0u; I < 3u; I++) {
        Mean[I] = (float)(T->Sum[Pose][I] / Count);
    }
    return true;
}

bool ImuTumble_PoseCommit(ImuTumble_t *T, uint8_t Pose)
{
    float Mean[3];
    if (T == NULL || Pose >= IMU_TUMBLE_POSE_COUNT || !ImuTumble_PoseMean(T, Pose, Mean)) {
        return false;
    }

    memcpy(T->PoseMean[Pose], Mean, sizeof(Mean));
    T->PoseCommitted[Pose] = true;
    return true;
}

void ImuTumble_AddSample(ImuTumble_t *T, uint8_t Pose, const float RawAccel[3])
{
    if (T == NULL || RawAccel == NULL || Pose >= IMU_TUMBLE_POSE_COUNT) {
        return;
    }

    T->Count[Pose]++;
    for (uint8_t I = 0u; I < 3u; I++) {
        T->Sum[Pose][I] += (double)RawAccel[I];
    }
    T->Outer[Pose][0] += (double)RawAccel[0] * (double)RawAccel[0];
    T->Outer[Pose][1] += (double)RawAccel[0] * (double)RawAccel[1];
    T->Outer[Pose][2] += (double)RawAccel[0] * (double)RawAccel[2];
    T->Outer[Pose][3] += (double)RawAccel[1] * (double)RawAccel[1];
    T->Outer[Pose][4] += (double)RawAccel[1] * (double)RawAccel[2];
    T->Outer[Pose][5] += (double)RawAccel[2] * (double)RawAccel[2];
}

ImuTumblePoseStatus_t ImuTumble_CheckPose(const ImuTumble_t *T, uint8_t Pose, const float MeanRaw[3])
{
    if (T == NULL || MeanRaw == NULL || Pose >= IMU_TUMBLE_POSE_COUNT) {
        return IMU_TUMBLE_POSE_INCONSISTENT_WITH_PREVIOUS;
    }

    const float Magnitude = ImuTumble_Norm(MeanRaw);
    const float MagnitudeBand = IMU_TUMBLE_G * ((float)DEEP_CAL_POSE_G_BAND_PCT / 100.0f);
    if (fabsf(Magnitude - IMU_TUMBLE_G) > MagnitudeBand) {
        return IMU_TUMBLE_POSE_BAD_MAGNITUDE;
    }

    const uint8_t Axis = ImuTumble_DominantAxis(MeanRaw);
    if (fabsf(MeanRaw[Axis]) < IMU_TUMBLE_G * DEEP_CAL_POSE_DOMINANT_MIN_G) {
        return IMU_TUMBLE_POSE_NOT_AXIS_ALIGNED;
    }

    if (Pose == 0u) {
        return IMU_TUMBLE_POSE_OK;
    }

    const float ToleranceRadians = (float)DEEP_CAL_POSE_ANGLE_TOL_DEG * 0.01745329251994329577f;
    const float CosTolerance = cosf(ToleranceRadians);
    const float SinTolerance = sinf(ToleranceRadians);

    switch (Pose) {
    case 1u:
        return ImuTumble_IsCommitted(T, 0u) && ImuTumble_Opposite(T->PoseMean[0], MeanRaw, CosTolerance)
            ? IMU_TUMBLE_POSE_OK : IMU_TUMBLE_POSE_INCONSISTENT_WITH_PREVIOUS;
    case 2u:
        return ImuTumble_IsCommitted(T, 1u) &&
               Axis != ImuTumble_DominantAxis(T->PoseMean[0]) &&
               ImuTumble_Perpendicular(T->PoseMean[0], MeanRaw, SinTolerance)
            ? IMU_TUMBLE_POSE_OK : IMU_TUMBLE_POSE_INCONSISTENT_WITH_PREVIOUS;
    case 3u:
        return ImuTumble_IsCommitted(T, 2u) &&
               Axis == ImuTumble_DominantAxis(T->PoseMean[2]) &&
               ImuTumble_Opposite(T->PoseMean[2], MeanRaw, CosTolerance)
            ? IMU_TUMBLE_POSE_OK : IMU_TUMBLE_POSE_INCONSISTENT_WITH_PREVIOUS;
    case 4u:
        return ImuTumble_IsCommitted(T, 3u) &&
               Axis != ImuTumble_DominantAxis(T->PoseMean[0]) &&
               Axis != ImuTumble_DominantAxis(T->PoseMean[2]) &&
               ImuTumble_Perpendicular(T->PoseMean[0], MeanRaw, SinTolerance) &&
               ImuTumble_Perpendicular(T->PoseMean[2], MeanRaw, SinTolerance)
            ? IMU_TUMBLE_POSE_OK : IMU_TUMBLE_POSE_INCONSISTENT_WITH_PREVIOUS;
    case 5u:
        return ImuTumble_IsCommitted(T, 4u) &&
               Axis == ImuTumble_DominantAxis(T->PoseMean[4]) &&
               ImuTumble_Opposite(T->PoseMean[4], MeanRaw, CosTolerance)
            ? IMU_TUMBLE_POSE_OK : IMU_TUMBLE_POSE_INCONSISTENT_WITH_PREVIOUS;
    default:
        return IMU_TUMBLE_POSE_INCONSISTENT_WITH_PREVIOUS;
    }
}

static void ImuTumble_BuildNormalEquations(const ImuTumble_t *T, double A[4][4], double B[4][3])
{
    memset(A, 0, sizeof(double) * 16u);
    memset(B, 0, sizeof(double) * 12u);

    double TotalCount = 0.0;
    for (uint8_t Pose = 0u; Pose < IMU_TUMBLE_POSE_COUNT; Pose++) {
        const double Count = (double)T->Count[Pose];
        const double Sum[4] = {T->Sum[Pose][0], T->Sum[Pose][1], T->Sum[Pose][2], Count};
        A[0][0] += T->Outer[Pose][0];
        A[0][1] += T->Outer[Pose][1];
        A[0][2] += T->Outer[Pose][2];
        A[1][1] += T->Outer[Pose][3];
        A[1][2] += T->Outer[Pose][4];
        A[2][2] += T->Outer[Pose][5];
        A[0][3] += Sum[0];
        A[1][3] += Sum[1];
        A[2][3] += Sum[2];
        A[3][3] += Sum[3];
        TotalCount += Count;

        for (uint8_t Row = 0u; Row < 4u; Row++) {
            for (uint8_t Col = 0u; Col < 3u; Col++) {
                B[Row][Col] += Sum[Row] * (double)ImuTumbleTargets[Pose][Col];
            }
        }
    }

    A[1][0] = A[0][1];
    A[2][0] = A[0][2];
    A[3][0] = A[0][3];
    A[2][1] = A[1][2];
    A[3][1] = A[1][3];
    A[3][2] = A[2][3];

    if (TotalCount > 0.0) {
        for (uint8_t Row = 0u; Row < 4u; Row++) {
            for (uint8_t Col = 0u; Col < 4u; Col++) {
                A[Row][Col] /= TotalCount;
            }
            for (uint8_t Col = 0u; Col < 3u; Col++) {
                B[Row][Col] /= TotalCount;
            }
        }
    }
}

static bool ImuTumble_SolveNormalEquations(double A[4][4], double B[4][3], double X[4][3], float *PivotRatio)
{
    double LargestEntry = 0.0;
    for (uint8_t Row = 0u; Row < 4u; Row++) {
        for (uint8_t Col = 0u; Col < 4u; Col++) {
            const double Entry = fabs(A[Row][Col]);
            if (Entry > LargestEntry) {
                LargestEntry = Entry;
            }
        }
    }
    if (LargestEntry == 0.0) {
        *PivotRatio = 0.0f;
        return false;
    }

    double SmallestPivot = LargestEntry;
    double LargestPivot = 0.0;
    for (uint8_t Col = 0u; Col < 4u; Col++) {
        uint8_t PivotRow = Col;
        for (uint8_t Row = (uint8_t)(Col + 1u); Row < 4u; Row++) {
            if (fabs(A[Row][Col]) > fabs(A[PivotRow][Col])) {
                PivotRow = Row;
            }
        }
        const double Pivot = fabs(A[PivotRow][Col]);
        if (Pivot < LargestEntry * IMU_TUMBLE_PIVOT_REL_TOL) {
            *PivotRatio = (float)(SmallestPivot / (LargestPivot > 0.0 ? LargestPivot : LargestEntry));
            return false;
        }
        if (Pivot < SmallestPivot) {
            SmallestPivot = Pivot;
        }
        if (Pivot > LargestPivot) {
            LargestPivot = Pivot;
        }
        if (PivotRow != Col) {
            for (uint8_t J = Col; J < 4u; J++) {
                const double Temp = A[Col][J];
                A[Col][J] = A[PivotRow][J];
                A[PivotRow][J] = Temp;
            }
            for (uint8_t J = 0u; J < 3u; J++) {
                const double Temp = B[Col][J];
                B[Col][J] = B[PivotRow][J];
                B[PivotRow][J] = Temp;
            }
        }

        for (uint8_t Row = (uint8_t)(Col + 1u); Row < 4u; Row++) {
            const double Factor = A[Row][Col] / A[Col][Col];
            A[Row][Col] = 0.0;
            for (uint8_t J = (uint8_t)(Col + 1u); J < 4u; J++) {
                A[Row][J] -= Factor * A[Col][J];
            }
            for (uint8_t J = 0u; J < 3u; J++) {
                B[Row][J] -= Factor * B[Col][J];
            }
        }
    }

    *PivotRatio = (float)(SmallestPivot / LargestPivot);
    for (int Row = 3; Row >= 0; Row--) {
        for (uint8_t Col = 0u; Col < 3u; Col++) {
            double Value = B[Row][Col];
            for (uint8_t J = (uint8_t)(Row + 1); J < 4u; J++) {
                Value -= A[Row][J] * X[J][Col];
            }
            X[Row][Col] = Value / A[Row][Row];
        }
    }
    return true;
}

static float ImuTumble_Determinant3(const float M[9])
{
    return M[0] * (M[4] * M[8] - M[5] * M[7])
         - M[1] * (M[3] * M[8] - M[5] * M[6])
         + M[2] * (M[3] * M[7] - M[4] * M[6]);
}

static bool ImuTumble_CheckOrthogonal(const float Q[9])
{
    for (uint8_t Row = 0u; Row < 3u; Row++) {
        for (uint8_t Col = 0u; Col < 3u; Col++) {
            float Value = 0.0f;
            for (uint8_t K = 0u; K < 3u; K++) {
                Value += Q[Row * 3u + K] * Q[Col * 3u + K];
            }
            const float Expected = (Row == Col) ? 1.0f : 0.0f;
            if (fabsf(Value - Expected) > IMU_TUMBLE_Q_ORTHOGONAL_TOL) {
                return false;
            }
        }
    }
    return true;
}

static void ImuTumble_CopyAndNormalizeQR(const float M[9], float Q[9], float R[9], float *DetQ,
                                         arm_status *Status)
{
    float SourceData[9];
    float RData[9];
    float QData[9];
    float Tau[3];
    float TmpA[3];
    float TmpB[3];
    arm_matrix_instance_f32 Source = {3u, 3u, SourceData};
    arm_matrix_instance_f32 OutR = {3u, 3u, RData};
    arm_matrix_instance_f32 OutQ = {3u, 3u, QData};

    memcpy(SourceData, M, sizeof(SourceData));
    *Status = arm_mat_qr_f32(&Source, IMU_TUMBLE_QR_THRESHOLD, &OutR, &OutQ, Tau, TmpA, TmpB);
    if (*Status != ARM_MATH_SUCCESS) {
        *DetQ = 0.0f;
        memset(Q, 0, 9u * sizeof(float));
        memset(R, 0, 9u * sizeof(float));
        return;
    }

    for (uint8_t I = 0u; I < 3u; I++) {
        if (RData[I * 3u + I] < 0.0f) {
            for (uint8_t J = 0u; J < 3u; J++) {
                QData[J * 3u + I] = -QData[J * 3u + I];
                RData[I * 3u + J] = -RData[I * 3u + J];
            }
        }
    }

    memcpy(Q, QData, sizeof(QData));
    memset(R, 0, 9u * sizeof(float));
    for (uint8_t Row = 0u; Row < 3u; Row++) {
        for (uint8_t Col = Row; Col < 3u; Col++) {
            R[Row * 3u + Col] = RData[Row * 3u + Col];
        }
    }
    *DetQ = ImuTumble_Determinant3(Q);
}

bool ImuTumble_DeriveQ(const float M[9], float Q[9], float *DetQ)
{
    float R[9];
    arm_status Status;

    if (M == NULL || Q == NULL || DetQ == NULL) return false;
    ImuTumble_CopyAndNormalizeQR(M, Q, R, DetQ, &Status);
    return Status == ARM_MATH_SUCCESS;
}

bool ImuTumble_Solve(const ImuTumble_t *T, ImuCalibration_t *Out, ImuTumbleQuality_t *Quality)
{
    double A[4][4];
    double B[4][3];
    double X[4][3] = {{0.0}};
    float M[9];
    arm_status QRStatus;

    if (!ImuTumble_ValidResultArguments(T, Out, Quality)) {
        return false;
    }
    memset(Out, 0, sizeof(*Out));
    memset(Quality, 0, sizeof(*Quality));
    Quality->Failure = IMU_TUMBLE_FAILURE_ARGUMENT;

    for (uint8_t Pose = 0u; Pose < IMU_TUMBLE_POSE_COUNT; Pose++) {
        if (T->Count[Pose] == 0u) {
            Quality->Failure = IMU_TUMBLE_FAILURE_INSUFFICIENT_SAMPLES;
            return false;
        }
    }

    ImuTumble_BuildNormalEquations(T, A, B);
    if (!ImuTumble_SolveNormalEquations(A, B, X, &Quality->PivotRatio)) {
        Quality->Failure = IMU_TUMBLE_FAILURE_SINGULAR;
        return false;
    }

    for (uint8_t Row = 0u; Row < 3u; Row++) {
        Quality->Offsets[Row] = (float)X[3][Row];
        for (uint8_t Col = 0u; Col < 3u; Col++) {
            M[Row * 3u + Col] = (float)X[Col][Row];
        }
    }

    memcpy(Out->M, M, sizeof(M));
    for (uint8_t Pose = 0u; Pose < IMU_TUMBLE_POSE_COUNT; Pose++) {
        float Mean[3];
        float Corrected[3];
        if (!ImuTumble_PoseMean(T, Pose, Mean)) {
            Quality->Failure = IMU_TUMBLE_FAILURE_INSUFFICIENT_SAMPLES;
            return false;
        }
        for (uint8_t Row = 0u; Row < 3u; Row++) {
            Corrected[Row] = Quality->Offsets[Row];
            for (uint8_t Col = 0u; Col < 3u; Col++) {
                Corrected[Row] += M[Row * 3u + Col] * Mean[Col];
            }
        }
        Quality->PoseResidual[Pose] = fabsf(ImuTumble_Norm(Corrected) - IMU_TUMBLE_G) / IMU_TUMBLE_G;
        if (Quality->PoseResidual[Pose] > DEEP_CAL_NORM_RESIDUAL_MAX) {
            Quality->Failure = IMU_TUMBLE_FAILURE_RESIDUAL;
            return false;
        }
    }

    ImuTumble_CopyAndNormalizeQR(M, Out->Q, Quality->R, &Quality->DetQ, &QRStatus);
    if (QRStatus != ARM_MATH_SUCCESS) {
        Quality->Failure = IMU_TUMBLE_FAILURE_QR;
        return false;
    }
    if (fabsf(Quality->DetQ - 1.0f) > DEEP_CAL_DET_TOL) {
        Quality->Failure = IMU_TUMBLE_FAILURE_DET_Q;
        return false;
    }
    if (!ImuTumble_CheckOrthogonal(Out->Q)) {
        Quality->Failure = IMU_TUMBLE_FAILURE_Q_NOT_ORTHOGONAL;
        return false;
    }

    Out->Valid = true;
    Quality->Failure = IMU_TUMBLE_FAILURE_NONE;
    return true;
}
