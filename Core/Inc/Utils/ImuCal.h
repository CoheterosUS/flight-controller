#ifndef IMU_CAL_H
#define IMU_CAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef IMU_CAL_HOST
typedef enum {
    STATE_IDLE,
    STATE_CALIBRATION,
    STATE_PRELAUNCH,
    STATE_BOOST,
    STATE_COAST,
    STATE_ACTIVE_CONTROL,
    STATE_APOGEE,
    STATE_MAIN_PARACHUTE,
    STATE_LANDED,
    STATE_GROUND_ABORT,
    STATE_DESCENT_ABORT,
    STATE_ASCENT_ABORT,
    STATE_DEEP_CALIBRATION,
    STATE_MAX
} SystemState_t;

#define CAL_STATUS_IMU_CAL_VALID       (1u << 0)
#define CAL_STATUS_GYRO_BIAS_VALID     (1u << 1)
#define CAL_STATUS_ACCEL_BIAS_VALID    (1u << 2)
#define CAL_STATUS_PRESSURE_REF_VALID  (1u << 3)
#define CAL_STATUS_KALMAN_INITIALIZED  (1u << 4)
#define CAL_STATUS_KALMAN_STEPPING     (1u << 5)
#define CAL_STATUS_HIL_PRESEED         (1u << 6)
#define CAL_STATUS_HIL_MODE            (1u << 7)
#define CAL_STATUS_POSE_MASK           (0x7u << 8)

typedef struct {
    float M[9];
    float Q[9];
    bool Valid;
} ImuCalibration_t;

typedef struct {
    bool ImuCalValid;
    bool GyroCalibrationValid;
    bool AccelBiasCalValid;
    bool ReferencePressurePaValid;
    bool KalmanInitialized;
    uint16_t CalStatus;
    ImuCalibration_t ImuCal;
} SystemContext_t;
#else
#include "Utils/shared.h"
#endif

typedef struct {
    double Sum[3];
    uint32_t DiscardCount;
    uint32_t SampleCount;
} ImuGyroBiasAccumulator_t;

typedef struct {
    double Sum[3];
    uint32_t DiscardCount;
    uint32_t SampleCount;
} ImuAccelBiasAccumulator_t;

typedef enum {
    IMU_CAL_ACCUMULATING = 0,
    IMU_CAL_READY,
    IMU_CAL_RESTARTED
} ImuCalAccumulatorResult_t;

void ImuCal_Apply(const ImuCalibration_t *Cal,
                  bool AccelBiasValid,
                  const float AccelBias[3],
                  bool GyroBiasValid,
                  const float GyroBiasRaw[3],
                  const float RawAccel[3],
                  const float RawGyro[3],
                  float CalAccel[3],
                  float CalGyro[3]);

void ImuCal_GyroBiasReset(ImuGyroBiasAccumulator_t *Accumulator);
ImuCalAccumulatorResult_t ImuCal_GyroBiasAdd(ImuGyroBiasAccumulator_t *Accumulator,
                                             const float RawGyro[3],
                                             float BiasRaw[3]);

void ImuCal_AccelBiasReset(ImuAccelBiasAccumulator_t *Accumulator);
ImuCalAccumulatorResult_t ImuCal_AccelBiasAdd(ImuAccelBiasAccumulator_t *Accumulator,
                                              const ImuCalibration_t *Cal,
                                              bool GyroBiasValid,
                                              const float GyroBiasRaw[3],
                                              const float RawAccel[3],
                                              const float RawGyro[3],
                                              float BiasCal[3]);

void ImuCal_UpdateStatus(SystemContext_t *Ctx, bool KalmanStepping);
void ImuCal_SanitizeVec3(float V[3], float Last[3]);
bool KalmanStateAllowsStepping(SystemState_t State);

#ifndef IMU_CAL_HOST
bool ImuCal_LoadFromFlash(SystemContext_t *Ctx);
#endif

#endif
