#include "Sensors/W25Q32JV.h"
#include "flash_sim.h"
#include <string.h>

static uint8_t Flash[W25Q_TOTAL_SIZE];
static int ProgramLimit = -1;
static size_t ProgramCount;
static uint8_t Status = 0;

void flash_reset(void) {
    memset(Flash, 0xFF, sizeof(Flash));
    ProgramLimit = -1;
    ProgramCount = 0;
    Status = 0;
}

uint8_t *flash_bytes(void) {
    return Flash;
}

void flash_set_program_limit(int Limit) {
    ProgramLimit = Limit;
}

size_t flash_program_count(void) {
    return ProgramCount;
}

void flash_corrupt(uint32_t Address, uint8_t Value) {
    Flash[Address] = Value;
}

HAL_StatusTypeDef W25Q_ReadJEDECID(SPI_HandleTypeDef *Handle, uint8_t *ManufacturerID, uint8_t *DeviceType, uint8_t *Capacity) {
    (void)Handle;
    *ManufacturerID = W25Q_JEDEC_MFR;
    *DeviceType = W25Q_JEDEC_TYPE;
    *Capacity = W25Q_JEDEC_CAPACITY;
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_ReadData(SPI_HandleTypeDef *Handle, uint32_t Address, uint8_t *Data, uint32_t Length) {
    (void)Handle;
    if (Address > W25Q_TOTAL_SIZE || Length > W25Q_TOTAL_SIZE - Address) return HAL_ERROR;
    memcpy(Data, &Flash[Address], Length);
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_ReadStatusReg1(SPI_HandleTypeDef *Handle, uint8_t *ReadStatus) {
    (void)Handle;
    *ReadStatus = Status;
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_UnprotectAll(SPI_HandleTypeDef *Handle) {
    (void)Handle;
    Status &= (uint8_t)~(W25Q_SR1_BP0 | W25Q_SR1_BP1 | W25Q_SR1_BP2);
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_ProtectAll(SPI_HandleTypeDef *Handle) {
    (void)Handle;
    Status |= (uint8_t)(W25Q_SR1_BP0 | W25Q_SR1_BP1 | W25Q_SR1_BP2);
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_SectorErase(SPI_HandleTypeDef *Handle, uint32_t Address) {
    (void)Handle;
    Address &= ~(W25Q_SECTOR_SIZE - 1u);
    if (Address >= W25Q_TOTAL_SIZE || Address + W25Q_SECTOR_SIZE > W25Q_TOTAL_SIZE) return HAL_ERROR;
    memset(&Flash[Address], 0xFF, W25Q_SECTOR_SIZE);
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_PageProgram(SPI_HandleTypeDef *Handle, uint32_t Address, const uint8_t *Data, uint16_t Length) {
    (void)Handle;
    uint32_t PageStart = Address & ~(W25Q_PAGE_SIZE - 1u);
    uint32_t PageOffset = Address & (W25Q_PAGE_SIZE - 1u);
    uint16_t Bytes = Length;

    ProgramCount++;
    if (ProgramLimit >= 0 && Bytes > (uint16_t)ProgramLimit) Bytes = (uint16_t)ProgramLimit;
    for (uint16_t i = 0; i < Bytes; i++) {
        uint32_t Position = PageStart + ((PageOffset + i) % W25Q_PAGE_SIZE);
        if (Position >= W25Q_TOTAL_SIZE) return HAL_ERROR;
        Flash[Position] &= Data[i];
    }
    return Bytes == Length ? HAL_OK : HAL_ERROR;
}
