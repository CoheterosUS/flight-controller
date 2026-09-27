#include "Utils/ImuCal.h"
#include "Utils/configuration.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

int TestImuTumbleDeriveQ(void);

static int Failures;

static void Check(bool Condition, const char *Message)
{
    if (!Condition) {
        fprintf(stderr, "FAIL: %s\n", Message);
        Failures++;
    }
}

static bool Near(float Actual, float Expected, float Tolerance)
{
    return fabsf(Actual - Expected) <= Tolerance;
}

static void TestApply(void)
{
    const float RawAccel[3] = {1.0f, 2.0f, 3.0f};
    const float RawGyro[3] = {4.0f, 5.0f, 6.0f};
    const float GyroBias[3] = {1.0f, 1.0f, 1.0f};
    const float AccelBias[3] = {0.5f, 0.5f, 0.5f};
    float CalAccel[3];
    float CalGyro[3];
    ImuCalibration_t Cal = {0};

    ImuCal_Apply(&Cal, true, AccelBias, true, GyroBias,
                 RawAccel, RawGyro, CalAccel, CalGyro);
    Check(Near(CalAccel[0], 1.0f, 0.001f) && Near(CalAccel[2], 3.0f, 0.001f),
          "invalid calibration passes acceleration through");
    Check(Near(CalGyro[0], 3.0f, 0.001f) && Near(CalGyro[2], 5.0f, 0.001f),
          "invalid calibration subtracts a valid gyro bias");

    Cal.Valid = true;
    Cal.M[0] = 0.0f; Cal.M[1] = -2.0f; Cal.M[2] = 0.0f;
    Cal.M[3] = 1.0f; Cal.M[4] = 0.0f; Cal.M[5] = 0.0f;
    Cal.M[6] = 0.0f; Cal.M[7] = 0.0f; Cal.M[8] = 1.5f;
    Cal.Q[0] = 0.0f; Cal.Q[1] = -1.0f; Cal.Q[2] = 0.0f;
    Cal.Q[3] = 1.0f; Cal.Q[4] = 0.0f; Cal.Q[5] = 0.0f;
    Cal.Q[6] = 0.0f; Cal.Q[7] = 0.0f; Cal.Q[8] = 1.0f;
    ImuCal_Apply(&Cal, true, AccelBias, true, GyroBias,
                 RawAccel, RawGyro, CalAccel, CalGyro);
    Check(Near(CalAccel[0], -4.5f, 0.001f) && Near(CalAccel[1], 0.5f, 0.001f) &&
          Near(CalAccel[2], 4.0f, 0.001f), "calibrated acceleration applies M and bias");
    Check(Near(CalGyro[0], -4.0f, 0.001f) && Near(CalGyro[1], 3.0f, 0.001f) &&
          Near(CalGyro[2], 5.0f, 0.001f), "calibrated gyro subtracts raw bias before Q");
}

static void TestGyroBias(void)
{
    ImuGyroBiasAccumulator_t Accumulator;
    const float RawGyro[3] = {1.0f, 2.0f, 3.0f};
    const float Zero[3] = {0.0f, 0.0f, 0.0f};
    float Bias[3] = {0.0f, 0.0f, 0.0f};
    float CalGyro[3];
    float PassAccel[3] = {0.0f, 0.0f, 0.0f};
    ImuCalibration_t Cal = {0};

    ImuCal_GyroBiasReset(&Accumulator);
    for (uint32_t I = 0u; I < GYRO_CALIBRATION_DISCARD_SAMPLES + GYRO_CALIBRATION_SAMPLES; I++) {
        Check(ImuCal_GyroBiasAdd(&Accumulator, RawGyro, Bias) != IMU_CAL_RESTARTED,
              "constant gyro bias does not restart");
    }
    Check(Near(Bias[0], 1.0f, 0.001f) && Near(Bias[1], 2.0f, 0.001f) &&
          Near(Bias[2], 3.0f, 0.001f), "gyro bias is measured in the raw frame");

    Cal.Valid = true;
    Cal.Q[0] = 0.0f; Cal.Q[1] = -1.0f; Cal.Q[2] = 0.0f;
    Cal.Q[3] = 1.0f; Cal.Q[4] = 0.0f; Cal.Q[5] = 0.0f;
    Cal.Q[6] = 0.0f; Cal.Q[7] = 0.0f; Cal.Q[8] = 1.0f;
    ImuCal_Apply(&Cal, false, Zero, true, Bias, Zero, RawGyro, PassAccel, CalGyro);
    Check(Near(CalGyro[0], 0.0f, 0.01f) && Near(CalGyro[1], 0.0f, 0.01f) &&
          Near(CalGyro[2], 0.0f, 0.01f), "raw gyro bias leaves calibrated gyro at zero");
}

static void BodyToRaw(const float Body[3], float Raw[3])
{
    Raw[0] = Body[1];
    Raw[1] = -Body[0] * 0.5f;
    Raw[2] = Body[2] / 1.5f;
}

static void TestAccelBias(void)
{
    ImuAccelBiasAccumulator_t Accumulator;
    ImuCalibration_t Cal = {0};
    const float Gyro[3] = {0.0f, 0.0f, 0.0f};
    const float Body[3] = {10.31f, 0.0f, 0.0f};
    float Raw[3];
    float Bias[3] = {0.0f, 0.0f, 0.0f};
    ImuCalAccumulatorResult_t Result = IMU_CAL_ACCUMULATING;

    Cal.Valid = true;
    Cal.M[0] = 0.0f; Cal.M[1] = -2.0f; Cal.M[2] = 0.0f;
    Cal.M[3] = 1.0f; Cal.M[4] = 0.0f; Cal.M[5] = 0.0f;
    Cal.M[6] = 0.0f; Cal.M[7] = 0.0f; Cal.M[8] = 1.5f;
    BodyToRaw(Body, Raw);

    ImuCal_AccelBiasReset(&Accumulator);
    for (uint32_t I = 0u; I < ACCEL_BIAS_CAL_DISCARD_SAMPLES + ACCEL_BIAS_CAL_SAMPLES; I++) {
        Result = ImuCal_AccelBiasAdd(&Accumulator, &Cal, false, NULL, Raw, Gyro, Bias);
    }
    Check(Result == IMU_CAL_READY && Near(Bias[0], 0.5f, 0.01f) &&
          Near(Bias[1], 0.0f, 0.01f) && Near(Bias[2], 0.0f, 0.01f),
          "accel bias uses the calibrated body frame");

    const float TiltedBody[3] = {9.81f, ACCEL_BIAS_LATERAL_MAX_G * 9.81f + 0.1f, 0.0f};
    BodyToRaw(TiltedBody, Raw);
    ImuCal_AccelBiasReset(&Accumulator);
    for (uint32_t I = 0u; I < ACCEL_BIAS_CAL_DISCARD_SAMPLES + ACCEL_BIAS_CAL_SAMPLES; I++) {
        Result = ImuCal_AccelBiasAdd(&Accumulator, &Cal, false, NULL, Raw, Gyro, Bias);
    }
    Check(Result == IMU_CAL_RESTARTED && Accumulator.SampleCount == 0u &&
          Accumulator.DiscardCount == 0u, "tilted accel window is rejected and restarted");

    const float NoseDownBody[3] = {-9.81f, 0.0f, 0.0f};
    BodyToRaw(NoseDownBody, Raw);
    ImuCal_AccelBiasReset(&Accumulator);
    for (uint32_t I = 0u; I < ACCEL_BIAS_CAL_DISCARD_SAMPLES + ACCEL_BIAS_CAL_SAMPLES; I++) {
        Result = ImuCal_AccelBiasAdd(&Accumulator, &Cal, false, NULL, Raw, Gyro, Bias);
    }
    Check(Result == IMU_CAL_RESTARTED && Accumulator.SampleCount == 0u,
          "nose down window is rejected (X must read about +9.81)");

    const float SidewaysBody[3] = {0.0f, 9.81f, 0.0f};
    BodyToRaw(SidewaysBody, Raw);
    ImuCal_AccelBiasReset(&Accumulator);
    for (uint32_t I = 0u; I < ACCEL_BIAS_CAL_DISCARD_SAMPLES + ACCEL_BIAS_CAL_SAMPLES; I++) {
        Result = ImuCal_AccelBiasAdd(&Accumulator, &Cal, false, NULL, Raw, Gyro, Bias);
    }
    Check(Result == IMU_CAL_RESTARTED, "sideways window is rejected");

    ImuCal_AccelBiasReset(&Accumulator);
    float NominalRaw[3];
    BodyToRaw(Body, NominalRaw);
    const float MotionGyro[3] = {ACCEL_BIAS_STILL_GYRO_MAX_DPS + 1.0f, 0.0f, 0.0f};
    for (uint32_t I = 0u; I < ACCEL_BIAS_CAL_DISCARD_SAMPLES + 20u; I++) {
        const float *CurrentGyro = I == ACCEL_BIAS_CAL_DISCARD_SAMPLES + 10u ? MotionGyro : Gyro;
        Result = ImuCal_AccelBiasAdd(&Accumulator, &Cal, false, NULL, NominalRaw, CurrentGyro, Bias);
    }
    (void)Result;
    Check(Accumulator.SampleCount == 0u, "accel motion restarts the window");
}

static void TestGyroStillness(void)
{
    ImuGyroBiasAccumulator_t Accumulator;
    const float Still[3] = {0.0f, 0.0f, 0.0f};
    const float Disturbed[3] = {GYRO_CAL_STILL_MAX_DPS + 1.0f, 0.0f, 0.0f};
    float Bias[3] = {0.0f, 0.0f, 0.0f};

    ImuCal_GyroBiasReset(&Accumulator);
    for (uint32_t I = 0u; I < GYRO_CALIBRATION_DISCARD_SAMPLES + 20u; I++) {
        ImuCal_GyroBiasAdd(&Accumulator, Still, Bias);
    }
    Check(ImuCal_GyroBiasAdd(&Accumulator, Disturbed, Bias) == IMU_CAL_RESTARTED &&
          Accumulator.SampleCount == 0u, "gyro disturbance restarts the window");
}

static void TestStatusAndStates(void)
{
    SystemContext_t Context = {0};
    Context.CalStatus = CAL_STATUS_HIL_PRESEED | (3u << 8);
    Context.ImuCal.Valid = true;
    Context.GyroCalibrationValid = true;
    Context.AccelBiasCalValid = true;
    Context.ReferencePressurePaValid = true;
    Context.KalmanInitialized = true;
    ImuCal_UpdateStatus(&Context, true);
    Check((Context.CalStatus & 0x3Fu) == 0x3Fu &&
          (Context.CalStatus & CAL_STATUS_HIL_PRESEED) != 0u &&
          ((Context.CalStatus >> 8) & 0x7u) == 3u, "status sets bits and preserves HIL and pose");

    Context.GyroCalibrationValid = false;
    ImuCal_UpdateStatus(&Context, false);
    Check((Context.CalStatus & (CAL_STATUS_GYRO_BIAS_VALID | CAL_STATUS_KALMAN_STEPPING)) == 0u,
          "status clears stale validity and stepping bits");

    for (int State = STATE_IDLE; State < STATE_MAX; State++) {
        const bool Expected = State >= STATE_PRELAUNCH && State <= STATE_MAIN_PARACHUTE;
        Check(KalmanStateAllowsStepping((SystemState_t)State) == Expected,
              "Kalman state gate matches the flight states");
    }
}

int main(void)
{
    TestApply();
    TestGyroBias();
    TestAccelBias();
    TestGyroStillness();
    TestStatusAndStates();
    Failures += TestImuTumbleDeriveQ();

    if (Failures != 0) {
        fprintf(stderr, "%d host test(s) failed\n", Failures);
        return 1;
    }
    puts("host calibration flow tests passed");
    return 0;
}
