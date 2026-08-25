#include "Sensors/BMP581.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Sensors/Sensors.h"

HAL_StatusTypeDef BMP581_Mode_Idle(I2C_HandleTypeDef *BMP581_Handle) {
    if (BMP581_Handle == NULL) {
        return HAL_ERROR;
    }

    // TODO: If HAL_ERROR or HAL_TIMEOUT reminder: Write to this register doesn't return ACK
    if (BMP581_Reset(BMP581_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    // TODO: Revise delay
    vTaskDelay(pdMS_TO_TICKS(100));

    if (BMP581_CheckChipID(BMP581_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (BMP581_CheckNVMReady(BMP581_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (BMP581_CheckPOR(BMP581_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (BMP581_DisableFIFO(BMP581_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (BMP581_DisableIIR(BMP581_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (BMP581_EnableDeepStandby(BMP581_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (BMP581_SetODR(BMP581_Handle, BMP581_ODR_HZ_0_125) != HAL_OK) {
        return HAL_ERROR;
    }

    if (BMP581_SetPowerMode(BMP581_Handle, BMP581_PWR_MODE_STANDBY) != HAL_OK) {
        return HAL_ERROR;
    }

    // TODO: Revise delay
    vTaskDelay(pdMS_TO_TICKS(100));

    return HAL_OK;
}

HAL_StatusTypeDef BMP581_Mode_Performance(I2C_HandleTypeDef *BMP581_Handle) {
    if (BMP581_Handle == NULL) {
        return HAL_ERROR;
    }

    if (BMP581_DisableDeepStandby(BMP581_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // TODO: Max Out, DONE
    if (BMP581_SetOSR(BMP581_Handle, BMP581_REG_OSR_CONFIG_OVERSAMPLING_X128, BMP581_REG_OSR_CONFIG_OVERSAMPLING_X8) != HAL_OK) {
        return HAL_ERROR;
    }

    // TODO: Setup IIR filter, MAX X1: 0x1, DONE
    if (BMP581_SetIIR(BMP581_Handle, BMP581_REG_DSP_IIR_COEFFICIENT_X1, BMP581_REG_DSP_IIR_COEFFICIENT_X1) != HAL_OK) {
        return HAL_ERROR;
    }

    if (BMP581_EnableCompensation(BMP581_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (BMP581_EnablePressure(BMP581_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

     if (BMP581_SetPowerMode(BMP581_Handle, BMP581_PWR_MODE_CONTINUOUS) != HAL_OK) {
        return HAL_ERROR;
    }

    // TODO: Revise delay
    vTaskDelay(pdMS_TO_TICKS(100));

    return HAL_OK;
}
