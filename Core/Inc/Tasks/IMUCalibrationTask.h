#ifndef IMU_CALIBRATION_TASK_H
#define IMU_CALIBRATION_TASK_H

#include "FreeRTOS.h"
#include "task.h"

#define IMU_CAL_DISCARD_SAMPLES  500
#define IMU_CAL_SAMPLES          2000
#define IMU_CAL_STACK_SIZE       1024

void CreateIMUCalibrationTask(const UBaseType_t Priority, const uint16_t StackSize);
void IMUCalibrationTask(void *pvParameters);

#endif
