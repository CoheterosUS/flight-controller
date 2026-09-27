#include "Utils/ImuCal.h"

#include <math.h>
#include <string.h>

#include "Utils/configuration.h"

#define IMU_CAL_GYRO_MIN_STILL_SAMPLES 10u

#ifndef IMU_CAL_HOST
#include "Sensors/W25Q32JV.h"
#include "Utils/ImuTumbleCal.h"
#endif

static float ImuCal_Norm(const float V[3])
{
    if (V == NULL || !isfinite(V[0]) || !isfinite(V[1]) || !isfinite(V[2])) {
        return NAN;
    }
    return sqrtf(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]);
}

static bool ImuCal_FiniteVec3(const float V[3])
{
    return V != NULL && isfinite(V[0]) && isfinite(V[1]) && isfinite(V[2]);
}

static bool ImuCal_FiniteMatrix3(const float Matrix[9])
{
    if (Matrix == NULL) return false;
    for (uint8_t I = 0u; I < 9u; I++) {
        if (!isfinite(Matrix[I])) return false;
    }
    return true;
}

static void ImuCal_Multiply3(const float Matrix[9], const float Vector[3], float Result[3])
{
    for (uint8_t Row = 0u; Row < 3u; Row++) {
        Result[Row] = Matrix[Row * 3u] * Vector[0]
                    + Matrix[Row * 3u + 1u] * Vector[1]
                    + Matrix[Row * 3u + 2u] * Vector[2];
    }
}

void ImuCal_Apply(const ImuCalibration_t *Cal,
                  bool AccelBiasValid,
                  const float AccelBias[3],
                  bool GyroBiasValid,
                  const float GyroBiasRaw[3],
                  const float RawAccel[3],
                  const float RawGyro[3],
                  float CalAccel[3],
                  float CalGyro[3])
{
    float GyroRawCorrected[3];

    if (Cal != NULL && Cal->Valid) {
        ImuCal_Multiply3(Cal->M, RawAccel, CalAccel);
        if (AccelBiasValid && AccelBias != NULL) {
            for (uint8_t I = 0u; I < 3u; I++) CalAccel[I] -= AccelBias[I];
        }
    } else {
        memcpy(CalAccel, RawAccel, 3u * sizeof(float));
    }

    for (uint8_t I = 0u; I < 3u; I++) {
        GyroRawCorrected[I] = RawGyro[I];
        if (GyroBiasValid && GyroBiasRaw != NULL) GyroRawCorrected[I] -= GyroBiasRaw[I];
    }

    if (Cal != NULL && Cal->Valid) {
        ImuCal_Multiply3(Cal->Q, GyroRawCorrected, CalGyro);
    } else {
        memcpy(CalGyro, GyroRawCorrected, 3u * sizeof(float));
    }
}

void ImuCal_GyroBiasReset(ImuGyroBiasAccumulator_t *Accumulator)
{
    if (Accumulator != NULL) memset(Accumulator, 0, sizeof(*Accumulator));
}

ImuCalAccumulatorResult_t ImuCal_GyroBiasAdd(ImuGyroBiasAccumulator_t *Accumulator,
                                             const float RawGyro[3],
                                             float BiasRaw[3])
{
    float Mean[3];
    float Delta[3];

    if (Accumulator == NULL || RawGyro == NULL || BiasRaw == NULL) return IMU_CAL_RESTARTED;
    if (!ImuCal_FiniteVec3(RawGyro) || ImuCal_Norm(RawGyro) > GYRO_CAL_BIAS_MAX_DPS + GYRO_CAL_STILL_MAX_DPS) {
        ImuCal_GyroBiasReset(Accumulator);
        return IMU_CAL_RESTARTED;
    }

    if (Accumulator->DiscardCount < GYRO_CALIBRATION_DISCARD_SAMPLES) {
        Accumulator->DiscardCount++;
        return IMU_CAL_ACCUMULATING;
    }

    if (Accumulator->SampleCount >= IMU_CAL_GYRO_MIN_STILL_SAMPLES) {
        const float InvCount = 1.0f / (float)Accumulator->SampleCount;
        for (uint8_t I = 0u; I < 3u; I++) {
            Mean[I] = (float)(Accumulator->Sum[I] * (double)InvCount);
            Delta[I] = RawGyro[I] - Mean[I];
        }
        if (!ImuCal_FiniteVec3(Mean) || !ImuCal_FiniteVec3(Delta) ||
            ImuCal_Norm(Delta) > GYRO_CAL_STILL_MAX_DPS) {
            ImuCal_GyroBiasReset(Accumulator);
            return IMU_CAL_RESTARTED;
        }
    }

    for (uint8_t I = 0u; I < 3u; I++) Accumulator->Sum[I] += (double)RawGyro[I];
    Accumulator->SampleCount++;

    if (Accumulator->SampleCount < GYRO_CALIBRATION_SAMPLES) return IMU_CAL_ACCUMULATING;

    const float InvCount = 1.0f / (float)Accumulator->SampleCount;
    for (uint8_t I = 0u; I < 3u; I++) BiasRaw[I] = (float)(Accumulator->Sum[I] * (double)InvCount);
    if (!ImuCal_FiniteVec3(BiasRaw) || ImuCal_Norm(BiasRaw) > GYRO_CAL_BIAS_MAX_DPS) {
        ImuCal_GyroBiasReset(Accumulator);
        return IMU_CAL_RESTARTED;
    }
    return IMU_CAL_READY;
}

void ImuCal_AccelBiasReset(ImuAccelBiasAccumulator_t *Accumulator)
{
    if (Accumulator != NULL) memset(Accumulator, 0, sizeof(*Accumulator));
}

ImuCalAccumulatorResult_t ImuCal_AccelBiasAdd(ImuAccelBiasAccumulator_t *Accumulator,
                                              const ImuCalibration_t *Cal,
                                              bool GyroBiasValid,
                                              const float GyroBiasRaw[3],
                                              const float RawAccel[3],
                                              const float RawGyro[3],
                                              float BiasCal[3])
{
    float GyroCorrected[3];
    float BodyAccel[3];

    if (Accumulator == NULL || Cal == NULL || !Cal->Valid || RawAccel == NULL ||
        RawGyro == NULL || BiasCal == NULL) return IMU_CAL_RESTARTED;
    if (!ImuCal_FiniteVec3(RawAccel) || !ImuCal_FiniteVec3(RawGyro) ||
        !ImuCal_FiniteMatrix3(Cal->M) ||
        ImuCal_Norm(RawGyro) > GYRO_CAL_BIAS_MAX_DPS + ACCEL_BIAS_STILL_GYRO_MAX_DPS) {
        ImuCal_AccelBiasReset(Accumulator);
        return IMU_CAL_RESTARTED;
    }
    if (GyroBiasValid && !ImuCal_FiniteVec3(GyroBiasRaw)) {
        ImuCal_AccelBiasReset(Accumulator);
        return IMU_CAL_RESTARTED;
    }

    for (uint8_t I = 0u; I < 3u; I++) {
        GyroCorrected[I] = RawGyro[I];
        if (GyroBiasValid && GyroBiasRaw != NULL) GyroCorrected[I] -= GyroBiasRaw[I];
    }
    if (!ImuCal_FiniteVec3(GyroCorrected) || ImuCal_Norm(GyroCorrected) > ACCEL_BIAS_STILL_GYRO_MAX_DPS) {
        ImuCal_AccelBiasReset(Accumulator);
        return IMU_CAL_RESTARTED;
    }

    if (Accumulator->DiscardCount < ACCEL_BIAS_CAL_DISCARD_SAMPLES) {
        Accumulator->DiscardCount++;
        return IMU_CAL_ACCUMULATING;
    }

    ImuCal_Multiply3(Cal->M, RawAccel, BodyAccel);
    if (!ImuCal_FiniteVec3(BodyAccel)) {
        ImuCal_AccelBiasReset(Accumulator);
        return IMU_CAL_RESTARTED;
    }
    for (uint8_t I = 0u; I < 3u; I++) Accumulator->Sum[I] += (double)BodyAccel[I];
    Accumulator->SampleCount++;

    if (Accumulator->SampleCount < ACCEL_BIAS_CAL_SAMPLES) return IMU_CAL_ACCUMULATING;

    const float InvCount = 1.0f / (float)Accumulator->SampleCount;
    const float MeanX = (float)(Accumulator->Sum[0] * (double)InvCount);
    const float MeanY = (float)(Accumulator->Sum[1] * (double)InvCount);
    const float MeanZ = (float)(Accumulator->Sum[2] * (double)InvCount);
    const float Mean[3] = {MeanX, MeanY, MeanZ};
    if (!ImuCal_FiniteVec3(Mean)) {
        ImuCal_AccelBiasReset(Accumulator);
        return IMU_CAL_RESTARTED;
    }
    /* Gross-tilt gate: the rocket must be nose up. Checking only the lateral axes would also
       accept nose down or upside down, where X reads about -9.81 and the bias would be nonsense. */
    if (fabsf(MeanX - CAL_EXPECTED_NOSE_UP_X) > ACCEL_BIAS_LATERAL_MAX_G * 9.81f ||
        fabsf(MeanY) > ACCEL_BIAS_LATERAL_MAX_G * 9.81f ||
        fabsf(MeanZ) > ACCEL_BIAS_LATERAL_MAX_G * 9.81f) {
        ImuCal_AccelBiasReset(Accumulator);
        return IMU_CAL_RESTARTED;
    }

    BiasCal[0] = MeanX - CAL_EXPECTED_NOSE_UP_X;
    BiasCal[1] = MeanY;
    BiasCal[2] = MeanZ;
    if (!ImuCal_FiniteVec3(BiasCal)) {
        ImuCal_AccelBiasReset(Accumulator);
        return IMU_CAL_RESTARTED;
    }
    return IMU_CAL_READY;
}

bool KalmanStateAllowsStepping(SystemState_t State)
{
    return State >= STATE_PRELAUNCH && State <= STATE_MAIN_PARACHUTE;
}

void ImuCal_UpdateStatus(SystemContext_t *Ctx, bool KalmanStepping)
{
    uint16_t Status;

    if (Ctx == NULL) return;
    Status = Ctx->CalStatus & (CAL_STATUS_HIL_PRESEED | CAL_STATUS_POSE_MASK);
#if HIL_MODE
    Status |= CAL_STATUS_HIL_MODE;
#endif
    if (Ctx->ImuCal.Valid) Status |= CAL_STATUS_IMU_CAL_VALID;
    if (Ctx->GyroCalibrationValid) Status |= CAL_STATUS_GYRO_BIAS_VALID;
    if (Ctx->AccelBiasCalValid) Status |= CAL_STATUS_ACCEL_BIAS_VALID;
    if (Ctx->ReferencePressurePaValid) Status |= CAL_STATUS_PRESSURE_REF_VALID;
    if (Ctx->KalmanInitialized) Status |= CAL_STATUS_KALMAN_INITIALIZED;
    if (KalmanStepping) Status |= CAL_STATUS_KALMAN_STEPPING;
    Ctx->CalStatus = Status;
}

void ImuCal_SanitizeVec3(float V[3], float Last[3])
{
    if (V == NULL || Last == NULL) return;
    for (uint8_t I = 0u; I < 3u; I++) {
        if (isfinite(V[I])) {
            Last[I] = V[I];
        } else {
            V[I] = Last[I];
        }
    }
}

#ifndef IMU_CAL_HOST
bool ImuCal_LoadFromFlash(SystemContext_t *Ctx)
{
    float M[9];
    float Q[9];
    float DetQ;

    if (Ctx == NULL) return false;

#if HIL_PRESEED_M
    memset(&Ctx->ImuCal, 0, sizeof(Ctx->ImuCal));
    for (uint8_t I = 0u; I < 3u; I++) {
        Ctx->ImuCal.M[I * 3u + I] = 1.0f;
        Ctx->ImuCal.Q[I * 3u + I] = 1.0f;
    }
    Ctx->ImuCal.Valid = true;
    Ctx->CalStatus |= CAL_STATUS_HIL_PRESEED;
    return true;
#else
    Ctx->CalStatus &= (uint16_t)~CAL_STATUS_HIL_PRESEED;
    if (!W25Q_CalLoad(M) || !ImuCal_FiniteMatrix3(M) || !ImuTumble_DeriveQ(M, Q, &DetQ) ||
        !ImuCal_FiniteMatrix3(Q) || !isfinite(DetQ) ||
        fabsf(DetQ - 1.0f) > DEEP_CAL_DET_TOL) {
        Ctx->ImuCal.Valid = false;
        return false;
    }
    memcpy(Ctx->ImuCal.M, M, sizeof(M));
    memcpy(Ctx->ImuCal.Q, Q, sizeof(Q));
    Ctx->ImuCal.Valid = true;
    return true;
#endif
}
#endif
