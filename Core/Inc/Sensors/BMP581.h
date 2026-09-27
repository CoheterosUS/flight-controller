#ifndef BMP581_H
#define BMP581_H

#include "stm32h7xx_hal.h"

/* I2C Address (SDO tied to GND) */
#define BMP581_I2C_ADDRESS          (0x46 << 1)
#define BMP581_SENSOR_DATA_SIZE     6

// REG_CHIP_ID
#define BMP581_REG_CHIP_ID                  0x01
#define BMP581_ID                           0x50

// REG_FIFO_SEL
#define BMP581_REG_FIFO_SEL                 0x18
#define BMP581_REG_FIFO_SEL_FIFO_FRAME_SEL  (0x01 << 0)

// REG_TEMPERATURE_XLSB
#define BMP581_REG_TEMPERATURE_XLSB         0x1D

// REG_INT_STATUS
#define BMP581_REG_INT_STATUS               0x27
#define BMP581_REG_INT_STATUS_POR           (0x01 << 4)

// REG_STATUS
#define BMP581_REG_STATUS                   0x28
#define BMP581_REG_STATUS_NVM_RDY           (0x01 << 1)
#define BMP581_REG_STATUS_NVM_ERR           (0x01 << 2)

// REG_DSP_CONFIG
#define BMP581_REG_DSP_CONFIG                   0x30

#define BMP581_REG_DSP_CONFIG_COMP_PT_EN        (0x03 << 0)
#define BMP581_REG_DSP_CONFIG_SHDW_SEL_IIR_T    (0x01 << 3)
#define BMP581_REG_DSP_CONFIG_SHDW_SEL_IIR_P    (0x01 << 5)

// REG_DSP_IIR
#define BMP581_REG_DSP_IIR                  0x31

#define BMP581_REG_DSP_IIR_SET_IIR_T_MASK       (0x07 << 0)
#define BMP581_REG_DSP_IIR_SET_IIR_P_MASK       (0x07 << 3)
#define BMP581_REG_DSP_IIR_COEFFICIENT_X0       0x00
#define BMP581_REG_DSP_IIR_COEFFICIENT_X1       0x01
#define BMP581_REG_DSP_IIR_COEFFICIENT_X3       0x02
#define BMP581_REG_DSP_IIR_COEFFICIENT_X7       0x03
#define BMP581_REG_DSP_IIR_COEFFICIENT_X15      0x04
#define BMP581_REG_DSP_IIR_COEFFICIENT_X31      0x05
#define BMP581_REG_DSP_IIR_COEFFICIENT_X63      0x06
#define BMP581_REG_DSP_IIR_COEFFICIENT_X127     0x07

#define BMP581_REG_DSP_IIR_TEMP_COEFFICIENT_OFFSET(c) ((c) << 0)
#define BMP581_REG_DSP_IIR_PRESS_COEFFICIENT_OFFSET(c) ((c) << 3)

// REG_OSR_CONFIG
#define BMP581_REG_OSR_CONFIG               0x36
#define BMP581_REG_OSR_CONFIG_PRESS_EN      (0x01 << 6)

#define BMP581_REG_OSR_CONFIG_OSR_T_MASK            (0x07 << 0)
#define BMP581_REG_OSR_CONFIG_OSR_P_MASK            (0x07 << 3)
#define BMP581_REG_OSR_CONFIG_OVERSAMPLING_X1       0x00
#define BMP581_REG_OSR_CONFIG_OVERSAMPLING_X2       0x01
#define BMP581_REG_OSR_CONFIG_OVERSAMPLING_X4       0x02
#define BMP581_REG_OSR_CONFIG_OVERSAMPLING_X8       0x03
#define BMP581_REG_OSR_CONFIG_OVERSAMPLING_X16      0x04
#define BMP581_REG_OSR_CONFIG_OVERSAMPLING_X32      0x05
#define BMP581_REG_OSR_CONFIG_OVERSAMPLING_X64      0x06
#define BMP581_REG_OSR_CONFIG_OVERSAMPLING_X128     0x07

#define BMP581_REG_OSR_CONFIG_TEMP_OVERSAMPLING_OFFSET(o)  ((o) << 0)
#define BMP581_REG_OSR_CONFIG_PRESS_OVERSAMPLING_OFFSET(o) ((o) << 3)

// REG_ODR_CONFIG
#define BMP581_REG_ODR_CONFIG               0x37
#define BMP581_REG_ODR_CONFIG_DEEP_DIS      (0x01 << 7)

#define BMP581_REG_ODR_CONFIG_PWR_MODE_MASK (0x03 << 0)
#define BMP581_PWR_MODE_STANDBY             (0x00 << 0)
#define BMP581_PWR_MODE_NORMAL              (0x01 << 0)
#define BMP581_PWR_MODE_FORCED              (0x02 << 0)
#define BMP581_PWR_MODE_CONTINUOUS          (0x03 << 0)

#define BMP581_REG_ODR_CONFIG_ODR_MASK      (0x1F << 2)
#define BMP581_ODR_HZ_0_125                 (0x1F << 2)

// REG_CMD
#define BMP581_REG_CMD                      0x7E
#define BMP581_RESET                        0xB6

typedef struct {
    float TemperatureC;
    float PressurePa;
    uint32_t SampleId;
} BMP581_SensorData_t;

HAL_StatusTypeDef BMP581_Reset(I2C_HandleTypeDef *BMP581_Handle);
HAL_StatusTypeDef BMP581_CheckChipID(I2C_HandleTypeDef *BMP581_Handle);
HAL_StatusTypeDef BMP581_CheckNVMReady(I2C_HandleTypeDef *BMP581_Handle);
HAL_StatusTypeDef BMP581_CheckPOR(I2C_HandleTypeDef *BMP581_Handle);
HAL_StatusTypeDef BMP581_EnableDeepStandby(I2C_HandleTypeDef *BMP581_Handle);
HAL_StatusTypeDef BMP581_DisableDeepStandby(I2C_HandleTypeDef *BMP581_Handle);
HAL_StatusTypeDef BMP581_DisableFIFO(I2C_HandleTypeDef *BMP581_Handle);
HAL_StatusTypeDef BMP581_DisableIIR(I2C_HandleTypeDef *BMP581_Handle);
HAL_StatusTypeDef BMP581_SetIIR(I2C_HandleTypeDef *BMP581_Handle, uint8_t PressureIIR, uint8_t TemperatureIIR);
HAL_StatusTypeDef BMP581_EnableCompensation(I2C_HandleTypeDef *BMP581_Handle);
HAL_StatusTypeDef BMP581_SetPowerMode(I2C_HandleTypeDef *BMP581_Handle, uint8_t PowerMode);
HAL_StatusTypeDef BMP581_SetODR(I2C_HandleTypeDef *BMP581_Handle, uint8_t ODR);
HAL_StatusTypeDef BMP581_SetOSR(I2C_HandleTypeDef *BMP581_Handle, uint8_t PressureOSR, uint8_t TemperatureOSR);
HAL_StatusTypeDef BMP581_EnablePressure(I2C_HandleTypeDef *BMP581_Handle);
HAL_StatusTypeDef BMP581_GetData(I2C_HandleTypeDef *BMP581_Handle, BMP581_SensorData_t *SensorData);

#endif //BMP581_H
