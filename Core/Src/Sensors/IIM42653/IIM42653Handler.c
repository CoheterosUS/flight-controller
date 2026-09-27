#include "Sensors/IIM42653.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Sensors/Sensors.h"

#if IMU_ODR_HZ == 100
#define IIM42653_GYRO_ODR_REGISTER  IIM42653_REG_GYRO_CONFIG0_GYRO_ODR_HZ_100
#define IIM42653_ACCEL_ODR_REGISTER IIM42653_REG_ACCEL_CONFIG0_ACCEL_ODR_HZ_100
#elif IMU_ODR_HZ == 200
#define IIM42653_GYRO_ODR_REGISTER  IIM42653_REG_GYRO_CONFIG0_GYRO_ODR_HZ_200
#define IIM42653_ACCEL_ODR_REGISTER IIM42653_REG_ACCEL_CONFIG0_ACCEL_ODR_HZ_200
#elif IMU_ODR_HZ == 500
#define IIM42653_GYRO_ODR_REGISTER  IIM42653_REG_GYRO_CONFIG0_GYRO_ODR_HZ_500
#define IIM42653_ACCEL_ODR_REGISTER IIM42653_REG_ACCEL_CONFIG0_ACCEL_ODR_HZ_500
#elif IMU_ODR_HZ == 1000
#define IIM42653_GYRO_ODR_REGISTER  IIM42653_REG_GYRO_CONFIG0_GYRO_ODR_HZ_1000
#define IIM42653_ACCEL_ODR_REGISTER IIM42653_REG_ACCEL_CONFIG0_ACCEL_ODR_HZ_1000
#else
#error "IMU_ODR_HZ must be 100, 200, 500 or 1000"
#endif

HAL_StatusTypeDef IIM42653_Mode_Idle(SPI_HandleTypeDef *IIM42653_Handle) {
    if (IIM42653_Handle == NULL) {
        return HAL_ERROR;
    }

    if (IIM42653_Reset(IIM42653_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    if (IIM42653_CheckChipID(IIM42653_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    if (IIM42653_ResetInt(IIM42653_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    if (IIM42653_SetEndianness(IIM42653_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    if (IIM42653_EnableFIFOHold(IIM42653_Handle) != HAL_OK) {
    	return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    if (IIM42653_EnableDataReady(IIM42653_Handle) != HAL_OK) {
      return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

//    if (IIM42653_EnableIdle(IIM42653_Handle) != HAL_OK) {
//        return HAL_ERROR;
//    }

    vTaskDelay(pdMS_TO_TICKS(100));

    return HAL_OK;
}

HAL_StatusTypeDef IIM42653_Mode_Performance(SPI_HandleTypeDef *IIM42653_Handle) {
    if (IIM42653_Handle == NULL) {
        return HAL_ERROR;
    }

//    if (IIM42653_DisableIdle(IIM42653_Handle) != HAL_OK) {
//        return HAL_ERROR;
//    }

    // TODO: Revise timeout
    vTaskDelay(pdMS_TO_TICKS(100));

    // AAF configuration for the selected ODR (datasheet Section 5.3)
    if (IIM42653_SetGyroAAF(IIM42653_Handle, 5, 25, 10) != HAL_OK) {
        return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    if (IIM42653_SetGyroConfig(IIM42653_Handle, IIM42653_GYRO_ODR_REGISTER, IIM42653_REG_GYRO_CONFIG0_GYRO_UI_FS_SEL_DPS_2000) != HAL_OK) {
        return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    if (IIM42653_SetAccelConfig(IIM42653_Handle, IIM42653_ACCEL_ODR_REGISTER, IIM42653_REG_ACCEL_CONFIG0_ACCEL_UI_FS_SEL_G_32) != HAL_OK) {
        return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    // TODO: 5.5.1: 1st order (select GYRO_UI_FILT_ORD)
    if (IIM42653_SetFilterOrder(IIM42653_Handle, IIM42653_REG_ACCEL_CONFIG1_ACCEL_FILT_ORD_1, IIM42653_REG_GYRO_CONFIG1_GYRO_FILT_ORD_1) != HAL_OK) {
    	return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    if (IIM42653_SetFilterBandwidth(IIM42653_Handle, IIM42653_REG_GYRO_ACCEL_CONFIG0_FILT_BW_16, IIM42653_REG_GYRO_ACCEL_CONFIG0_FILT_BW_16) != HAL_OK) {
        return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    if (IIM42653_SetPowerMode(IIM42653_Handle, IIM42653_REG_PWR_MGMT0_MODE_LN, IIM42653_REG_PWR_MGMT0_MODE_LN) != HAL_OK) {
        return HAL_ERROR;
    }

    // TODO: IIM42653 Might need this much delay
    vTaskDelay(pdMS_TO_TICKS(2000));

    return HAL_OK;
}
