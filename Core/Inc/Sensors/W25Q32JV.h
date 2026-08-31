#ifndef W25Q32JV_H
#define W25Q32JV_H

#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Managers/StructManager.h"

#define W25Q_CS_GPIO_PORT       GPIOB
#define W25Q_CS_GPIO_PIN        GPIO_PIN_0
#define W25Q_HOLD_GPIO_PORT     GPIOB
#define W25Q_HOLD_GPIO_PIN      GPIO_PIN_1
#define W25Q_WP_GPIO_PORT       GPIOB
#define W25Q_WP_GPIO_PIN        GPIO_PIN_2

#define W25Q_PAGE_SIZE          256
#define W25Q_SECTOR_SIZE        4096
#define W25Q_BLOCK_SIZE         65536
#define W25Q_TOTAL_SIZE         (4 * 1024 * 1024)
#define W25Q_PAGE_COUNT         (W25Q_TOTAL_SIZE / W25Q_PAGE_SIZE)
#define W25Q_SECTOR_COUNT       (W25Q_TOTAL_SIZE / W25Q_SECTOR_SIZE)

#define W25Q_JEDEC_MFR          0xEF
#define W25Q_JEDEC_TYPE         0x40
#define W25Q_JEDEC_CAPACITY     0x16

#define W25Q_SPI_TIMEOUT        100

// CMD
#define W25Q_CMD_WRITE_ENABLE           0x06
#define W25Q_CMD_WRITE_DISABLE          0x04
#define W25Q_CMD_READ_STATUS_REG1       0x05
#define W25Q_CMD_READ_STATUS_REG2       0x35
#define W25Q_CMD_WRITE_STATUS_REG       0x01
#define W25Q_CMD_READ_DATA              0x03
#define W25Q_CMD_PAGE_PROGRAM           0x02
#define W25Q_CMD_SECTOR_ERASE           0x20
#define W25Q_CMD_BLOCK_ERASE_32K        0x52
#define W25Q_CMD_BLOCK_ERASE_64K        0xD8
#define W25Q_CMD_CHIP_ERASE             0xC7
#define W25Q_CMD_READ_JEDEC_ID          0x9F
#define W25Q_CMD_POWER_DOWN             0xB9
#define W25Q_CMD_RELEASE_POWER_DOWN     0xAB

// SR1
#define W25Q_SR1_BUSY                   (1 << 0)
#define W25Q_SR1_WEL                    (1 << 1)
#define W25Q_SR1_BP0                    (1 << 2)
#define W25Q_SR1_BP1                    (1 << 3)
#define W25Q_SR1_BP2                    (1 << 4)
#define W25Q_SR1_SEC                    (1 << 5)
#define W25Q_SR1_TB                     (1 << 6)
#define W25Q_SR1_SRP                    (1 << 7)

#define W25Q_SR1_PROTECT_ALL            0x9C
#define W25Q_SR1_PROTECT_NONE           0x00

#define FLASH_PAGE_RECORDS              FLASH_RECORDS_PER_PAGE

typedef struct {
    FlashLogRecord_t Records[FLASH_PAGE_RECORDS];
} FlashPage_t;

extern SemaphoreHandle_t FlashSPISemaphore;

void W25Q_SelectCS(void);
void W25Q_DeselectCS(void);
void W25Q_WP_Enable(void);
void W25Q_WP_Disable(void);

HAL_StatusTypeDef W25Q_ReadJEDECID(SPI_HandleTypeDef *Handle, uint8_t *ManufacturerID, uint8_t *DeviceType, uint8_t *Capacity);
HAL_StatusTypeDef W25Q_WriteEnable(SPI_HandleTypeDef *Handle);
HAL_StatusTypeDef W25Q_WriteDisable(SPI_HandleTypeDef *Handle);
HAL_StatusTypeDef W25Q_ReadStatusReg1(SPI_HandleTypeDef *Handle, uint8_t *Status);
HAL_StatusTypeDef W25Q_WriteStatusReg(SPI_HandleTypeDef *Handle, uint8_t SR1);
HAL_StatusTypeDef W25Q_WaitBusy(SPI_HandleTypeDef *Handle, uint32_t TimeoutMs);
HAL_StatusTypeDef W25Q_ReadData(SPI_HandleTypeDef *Handle, uint32_t Address, uint8_t *Data, uint32_t Length);
HAL_StatusTypeDef W25Q_PageProgram(SPI_HandleTypeDef *Handle, uint32_t Address, const uint8_t *Data, uint16_t Length);
HAL_StatusTypeDef W25Q_PageProgramDMA(SPI_HandleTypeDef *Handle, uint32_t Address, uint8_t *DMABuffer, uint16_t Length);
HAL_StatusTypeDef W25Q_SectorErase(SPI_HandleTypeDef *Handle, uint32_t SectorAddress);
HAL_StatusTypeDef W25Q_ChipErase(SPI_HandleTypeDef *Handle);
HAL_StatusTypeDef W25Q_ProtectAll(SPI_HandleTypeDef *Handle);
HAL_StatusTypeDef W25Q_UnprotectAll(SPI_HandleTypeDef *Handle);

bool W25Q_Init(void);
bool W25Q_LoggingInit(void);
void W25Q_LoggingStop(void);
void W25Q_NewFlight(void);
uint32_t W25Q_GetWritePointer(void);
void W25Q_AdvanceWritePointer(uint16_t Bytes);
HAL_StatusTypeDef W25Q_UpdateHeader(void);
bool W25Q_HasSpace(uint16_t Bytes);
HAL_StatusTypeDef W25Q_EraseAll(void);

#endif //W25Q32JV_H
