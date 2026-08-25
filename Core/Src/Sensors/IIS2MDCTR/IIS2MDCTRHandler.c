#include "Sensors/IIS2MDCTR.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Sensors/Sensors.h"

HAL_StatusTypeDef IIS2MDCTR_Mode_Idle(I2C_HandleTypeDef *IIS2MDCTR_Handle) {
    if (IIS2MDCTR_Handle == NULL) {
        return HAL_ERROR;
    }

    if (IIS2MDCTR_Reset(IIS2MDCTR_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    // TODO: Revise delay
    vTaskDelay(pdMS_TO_TICKS(100));

    if (IIS2MDCTR_CheckChipID(IIS2MDCTR_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (IIS2MDCTR_EnableTempComp(IIS2MDCTR_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (IIS2MDCTR_SetMode(IIS2MDCTR_Handle, IIS2MDCTR_MD_IDLE) != HAL_OK) {
        return HAL_ERROR;
    }

    if (IIS2MDCTR_EnableLP(IIS2MDCTR_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (IIS2MDCTR_DisablePulseFrequency(IIS2MDCTR_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (IIS2MDCTR_DisableOffsetCancellation(IIS2MDCTR_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    // TODO: Revise delay
    vTaskDelay(pdMS_TO_TICKS(100));

    return HAL_OK;
}

HAL_StatusTypeDef IIS2MDCTR_Mode_Performance(I2C_HandleTypeDef *IIS2MDCTR_Handle) {
    if (IIS2MDCTR_Handle == NULL) {
        return HAL_ERROR;
    }

    if (IIS2MDCTR_DisableLP(IIS2MDCTR_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    if (IIS2MDCTR_EnableBDU(IIS2MDCTR_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    // TODO: 20Hz, DONE
    if (IIS2MDCTR_SetODR(IIS2MDCTR_Handle, IIS2MDCTR_ODR_HZ_20) != HAL_OK) {
        return HAL_ERROR;
    }

    if (IIS2MDCTR_EnableLPF(IIS2MDCTR_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    // TODO: Review this
//    if (IIS2MDCTR_EnableOffsetCancellation(IIS2MDCTR_Handle) != HAL_OK) {
//        return HAL_ERROR;
//    }

    if (IIS2MDCTR_EnablePulseFrequency(IIS2MDCTR_Handle) != HAL_OK) {
        return HAL_ERROR;
    }

    if (IIS2MDCTR_SetMode(IIS2MDCTR_Handle, IIS2MDCTR_MD_CONTINUOUS) != HAL_OK) {
        return HAL_ERROR;
    }

    // TODO: Revise delay
    vTaskDelay(pdMS_TO_TICKS(100));

    return HAL_OK;
}
