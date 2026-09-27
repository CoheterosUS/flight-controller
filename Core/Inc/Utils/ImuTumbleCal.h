#ifndef IMU_TUMBLE_CAL_H
#define IMU_TUMBLE_CAL_H

#include <stdbool.h>
#include <stdint.h>

#include "configuration.h"

#if defined(IMU_TUMBLE_HOST)
typedef struct {
    float M[9];
    float Q[9];
    bool Valid;
} ImuCalibration_t;
#else
#include "shared.h"
#endif

#define IMU_TUMBLE_POSE_COUNT 6u

typedef enum {
    IMU_TUMBLE_POSE_OK = 0,
    IMU_TUMBLE_POSE_BAD_MAGNITUDE,
    IMU_TUMBLE_POSE_NOT_AXIS_ALIGNED,
    IMU_TUMBLE_POSE_INCONSISTENT_WITH_PREVIOUS
} ImuTumblePoseStatus_t;

typedef enum {
    IMU_TUMBLE_FAILURE_NONE = 0,
    IMU_TUMBLE_FAILURE_ARGUMENT,
    IMU_TUMBLE_FAILURE_INSUFFICIENT_SAMPLES,
    IMU_TUMBLE_FAILURE_SINGULAR,
    IMU_TUMBLE_FAILURE_QR,
    IMU_TUMBLE_FAILURE_DET_Q,
    IMU_TUMBLE_FAILURE_Q_NOT_ORTHOGONAL,
    IMU_TUMBLE_FAILURE_RESIDUAL
} ImuTumbleFailureReason_t;

typedef struct {
    float PoseResidual[IMU_TUMBLE_POSE_COUNT];
    float DetQ;
    float PivotRatio;
    float Offsets[3];
    float R[9];
    ImuTumbleFailureReason_t Failure;
} ImuTumbleQuality_t;

typedef struct {
    uint32_t Count[IMU_TUMBLE_POSE_COUNT];
    double Sum[IMU_TUMBLE_POSE_COUNT][3];
    double Outer[IMU_TUMBLE_POSE_COUNT][6];
    bool PoseCommitted[IMU_TUMBLE_POSE_COUNT];
    float PoseMean[IMU_TUMBLE_POSE_COUNT][3];
} ImuTumble_t;

void ImuTumble_Reset(ImuTumble_t *T);
ImuTumblePoseStatus_t ImuTumble_CheckPose(const ImuTumble_t *T, uint8_t Pose, const float MeanRaw[3]);
void ImuTumble_AddSample(ImuTumble_t *T, uint8_t Pose, const float RawAccel[3]);
bool ImuTumble_Solve(const ImuTumble_t *T, ImuCalibration_t *Out, ImuTumbleQuality_t *Quality);

void ImuTumble_PoseBegin(ImuTumble_t *T, uint8_t Pose);
bool ImuTumble_PoseCommit(ImuTumble_t *T, uint8_t Pose);
bool ImuTumble_PoseMean(const ImuTumble_t *T, uint8_t Pose, float Mean[3]);

#endif
