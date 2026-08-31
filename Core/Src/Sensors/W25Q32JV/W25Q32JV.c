#include "Sensors/W25Q32JV.h"
#include "FreeRTOS.h"
#include "task.h"

void W25Q_SelectCS(void) {
    HAL_GPIO_WritePin(W25Q_CS_GPIO_PORT, W25Q_CS_GPIO_PIN, GPIO_PIN_RESET);
}

void W25Q_DeselectCS(void) {
    HAL_GPIO_WritePin(W25Q_CS_GPIO_PORT, W25Q_CS_GPIO_PIN, GPIO_PIN_SET);
}

void W25Q_WP_Enable(void) {
    HAL_GPIO_WritePin(W25Q_WP_GPIO_PORT, W25Q_WP_GPIO_PIN, GPIO_PIN_RESET);
}

void W25Q_WP_Disable(void) {
    HAL_GPIO_WritePin(W25Q_WP_GPIO_PORT, W25Q_WP_GPIO_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef W25Q_ReadJEDECID(SPI_HandleTypeDef *Handle, uint8_t *ManufacturerID, uint8_t *DeviceType, uint8_t *Capacity) {
    uint8_t TX[4] = { W25Q_CMD_READ_JEDEC_ID, 0, 0, 0 };
    uint8_t RX[4] = { 0 };

    W25Q_SelectCS();
    HAL_StatusTypeDef Status = HAL_SPI_TransmitReceive(Handle, TX, RX, 4, W25Q_SPI_TIMEOUT);
    W25Q_DeselectCS();

    if (Status == HAL_OK) {
        *ManufacturerID = RX[1];
        *DeviceType = RX[2];
        *Capacity = RX[3];
    }

    return Status;
}

HAL_StatusTypeDef W25Q_WriteEnable(SPI_HandleTypeDef *Handle) {
    uint8_t Cmd = W25Q_CMD_WRITE_ENABLE;

    W25Q_SelectCS();
    HAL_StatusTypeDef Status = HAL_SPI_Transmit(Handle, &Cmd, 1, W25Q_SPI_TIMEOUT);
    W25Q_DeselectCS();

    return Status;
}

HAL_StatusTypeDef W25Q_WriteDisable(SPI_HandleTypeDef *Handle) {
    uint8_t Cmd = W25Q_CMD_WRITE_DISABLE;

    W25Q_SelectCS();
    HAL_StatusTypeDef Status = HAL_SPI_Transmit(Handle, &Cmd, 1, W25Q_SPI_TIMEOUT);
    W25Q_DeselectCS();

    return Status;
}

HAL_StatusTypeDef W25Q_ReadStatusReg1(SPI_HandleTypeDef *Handle, uint8_t *Status) {
    uint8_t TX[2] = { W25Q_CMD_READ_STATUS_REG1, 0 };
    uint8_t RX[2] = { 0 };

    W25Q_SelectCS();
    HAL_StatusTypeDef Result = HAL_SPI_TransmitReceive(Handle, TX, RX, 2, W25Q_SPI_TIMEOUT);
    W25Q_DeselectCS();

    if (Result == HAL_OK) {
        *Status = RX[1];
    }

    return Result;
}

HAL_StatusTypeDef W25Q_WriteStatusReg(SPI_HandleTypeDef *Handle, uint8_t SR1) {
    if (W25Q_WriteEnable(Handle) != HAL_OK) return HAL_ERROR;

    uint8_t TX[2] = { W25Q_CMD_WRITE_STATUS_REG, SR1 };

    W25Q_SelectCS();
    HAL_StatusTypeDef Status = HAL_SPI_Transmit(Handle, TX, 2, W25Q_SPI_TIMEOUT);
    W25Q_DeselectCS();

    if (Status != HAL_OK) return Status;

    return W25Q_WaitBusy(Handle, 20);
}

HAL_StatusTypeDef W25Q_WaitBusy(SPI_HandleTypeDef *Handle, uint32_t TimeoutMs) {
    TickType_t Start = xTaskGetTickCount();
    TickType_t Timeout = pdMS_TO_TICKS(TimeoutMs);

    for (;;) {
        uint8_t SR1;
        HAL_StatusTypeDef Status = W25Q_ReadStatusReg1(Handle, &SR1);
        if (Status != HAL_OK) return Status;

        if (!(SR1 & W25Q_SR1_BUSY)) return HAL_OK;

        if ((xTaskGetTickCount() - Start) >= Timeout) return HAL_TIMEOUT;

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

HAL_StatusTypeDef W25Q_ReadData(SPI_HandleTypeDef *Handle, uint32_t Address, uint8_t *Data, uint32_t Length) {
    uint8_t Cmd[4] = {
        W25Q_CMD_READ_DATA,
        (Address >> 16) & 0xFF,
        (Address >> 8) & 0xFF,
        Address & 0xFF
    };

    W25Q_SelectCS();
    HAL_StatusTypeDef Status = HAL_SPI_Transmit(Handle, Cmd, 4, W25Q_SPI_TIMEOUT);
    if (Status == HAL_OK) {
        Status = HAL_SPI_Receive(Handle, Data, Length, W25Q_SPI_TIMEOUT);
    }
    W25Q_DeselectCS();

    return Status;
}

HAL_StatusTypeDef W25Q_PageProgram(SPI_HandleTypeDef *Handle, uint32_t Address, const uint8_t *Data, uint16_t Length) {
    if (Length > W25Q_PAGE_SIZE) return HAL_ERROR;

    if (W25Q_WriteEnable(Handle) != HAL_OK) return HAL_ERROR;

    uint8_t Cmd[4] = {
        W25Q_CMD_PAGE_PROGRAM,
        (Address >> 16) & 0xFF,
        (Address >> 8) & 0xFF,
        Address & 0xFF
    };

    W25Q_SelectCS();
    HAL_StatusTypeDef Status = HAL_SPI_Transmit(Handle, Cmd, 4, W25Q_SPI_TIMEOUT);
    if (Status == HAL_OK) {
        Status = HAL_SPI_Transmit(Handle, (uint8_t *)Data, Length, W25Q_SPI_TIMEOUT);
    }
    W25Q_DeselectCS();

    if (Status != HAL_OK) return Status;

    return W25Q_WaitBusy(Handle, 5);
}

HAL_StatusTypeDef W25Q_PageProgramDMA(SPI_HandleTypeDef *Handle, uint32_t Address, uint8_t *DMABuffer, uint16_t Length) {
    if (Length > W25Q_PAGE_SIZE) return HAL_ERROR;

    if (W25Q_WriteEnable(Handle) != HAL_OK) return HAL_ERROR;

    DMABuffer[0] = W25Q_CMD_PAGE_PROGRAM;
    DMABuffer[1] = (Address >> 16) & 0xFF;
    DMABuffer[2] = (Address >> 8) & 0xFF;
    DMABuffer[3] = Address & 0xFF;

    W25Q_SelectCS();
    return HAL_SPI_Transmit_DMA(Handle, DMABuffer, 4 + Length);
}

HAL_StatusTypeDef W25Q_SectorErase(SPI_HandleTypeDef *Handle, uint32_t SectorAddress) {
    if (W25Q_WriteEnable(Handle) != HAL_OK) return HAL_ERROR;

    uint8_t Cmd[4] = {
        W25Q_CMD_SECTOR_ERASE,
        (SectorAddress >> 16) & 0xFF,
        (SectorAddress >> 8) & 0xFF,
        SectorAddress & 0xFF
    };

    W25Q_SelectCS();
    HAL_StatusTypeDef Status = HAL_SPI_Transmit(Handle, Cmd, 4, W25Q_SPI_TIMEOUT);
    W25Q_DeselectCS();

    if (Status != HAL_OK) return Status;

    return W25Q_WaitBusy(Handle, 500);
}

HAL_StatusTypeDef W25Q_ChipErase(SPI_HandleTypeDef *Handle) {
    if (W25Q_WriteEnable(Handle) != HAL_OK) return HAL_ERROR;

    uint8_t Cmd = W25Q_CMD_CHIP_ERASE;

    W25Q_SelectCS();
    HAL_StatusTypeDef Status = HAL_SPI_Transmit(Handle, &Cmd, 1, W25Q_SPI_TIMEOUT);
    W25Q_DeselectCS();

    if (Status != HAL_OK) return Status;

    return W25Q_WaitBusy(Handle, 60000);
}

HAL_StatusTypeDef W25Q_ProtectAll(SPI_HandleTypeDef *Handle) {
    W25Q_WP_Disable();
    HAL_StatusTypeDef Status = W25Q_WriteStatusReg(Handle, W25Q_SR1_PROTECT_ALL);
    if (Status == HAL_OK) {
        W25Q_WP_Enable();
    }
    return Status;
}

HAL_StatusTypeDef W25Q_UnprotectAll(SPI_HandleTypeDef *Handle) {
    W25Q_WP_Disable();
    return W25Q_WriteStatusReg(Handle, W25Q_SR1_PROTECT_NONE);
}
