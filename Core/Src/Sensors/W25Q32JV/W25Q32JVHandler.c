#include "Sensors/W25Q32JV.h"
#include "Utils/shared.h"
#include "Managers/StructManager.h"
#include "fatfs.h"
#include <string.h>
#include <stdio.h>

#define FLASH_HEADER_MAGIC      0x464C5348
#define FLASH_HEADER_ADDRESS    0x00000000
#define FLASH_DATA_START        W25Q_SECTOR_SIZE

static uint32_t WritePointer;
static bool W25Q_Initialized = false;

static bool W25Q_VerifyJEDECID(SPI_HandleTypeDef *Handle) {
    uint8_t MFR, Type, Cap;
    if (W25Q_ReadJEDECID(Handle, &MFR, &Type, &Cap) != HAL_OK) return false;
    return (MFR == W25Q_JEDEC_MFR && Type == W25Q_JEDEC_TYPE && Cap == W25Q_JEDEC_CAPACITY);
}

static HAL_StatusTypeDef W25Q_FindWritePointer(SPI_HandleTypeDef *Handle, uint32_t *Result) {
    uint32_t Low = FLASH_DATA_START / W25Q_PAGE_SIZE;
    uint32_t High = W25Q_PAGE_COUNT;

    while (Low < High) {
        uint32_t Mid = Low + (High - Low) / 2;
        uint8_t Byte;
        if (W25Q_ReadData(Handle, Mid * W25Q_PAGE_SIZE, &Byte, 1) != HAL_OK) return HAL_ERROR;

        if (Byte == 0xFF) {
            High = Mid;
        } else {
            Low = Mid + 1;
        }
    }

    *Result = Low * W25Q_PAGE_SIZE;
    return HAL_OK;
}

static HAL_StatusTypeDef W25Q_WriteMagic(SPI_HandleTypeDef *Handle) {
    const uint32_t Magic = FLASH_HEADER_MAGIC;
    return W25Q_PageProgram(Handle, FLASH_HEADER_ADDRESS, (const uint8_t *)&Magic, sizeof(Magic));
}

bool W25Q_Init(void) {
    SPI_HandleTypeDef *Handle = W25Q_HANDLE;

    W25Q_WP_Disable();

    if (!W25Q_VerifyJEDECID(Handle)) {
        SystemFaultFlags |= W25Q_JEDEC_ID_FAILED;
        return false;
    }

    if (W25Q_UnprotectAll(Handle) != HAL_OK) {
        SystemFaultFlags |= W25Q_INIT_FAILED;
        return false;
    }

    uint32_t Magic;
    if (W25Q_ReadData(Handle, FLASH_HEADER_ADDRESS, (uint8_t *)&Magic, sizeof(Magic)) != HAL_OK) {
        SystemFaultFlags |= W25Q_INIT_FAILED;
        return false;
    }

    if (Magic == 0xFFFFFFFF) {
        if (W25Q_WriteMagic(Handle) != HAL_OK) {
            SystemFaultFlags |= W25Q_INIT_FAILED;
            return false;
        }
    } else if (Magic != FLASH_HEADER_MAGIC) {
        SystemFaultFlags |= W25Q_INIT_FAILED;
        return false;
    }

    if (W25Q_FindWritePointer(Handle, &WritePointer) != HAL_OK) {
        SystemFaultFlags |= W25Q_INIT_FAILED;
        return false;
    }

    return true;
}

void W25Q_NewFlight(void) {
    FlashLogRecord_t Marker = {0};
    Marker.Sync = PACKET_HEADER;
    Marker.State = 0xFF;
    Marker.SyncEnd = PACKET_FOOTER;

    if (W25Q_HasSpace(W25Q_PAGE_SIZE)) {
        W25Q_PageProgram(W25Q_HANDLE, WritePointer, (const uint8_t *)&Marker, sizeof(FlashLogRecord_t));
        WritePointer += W25Q_PAGE_SIZE;
    }
}

uint32_t W25Q_GetWritePointer(void) {
    return WritePointer;
}

void W25Q_AdvanceWritePointer(uint16_t Bytes) {
    WritePointer += Bytes;
}

bool W25Q_HasSpace(uint16_t Bytes) {
    return (WritePointer + Bytes) <= W25Q_TOTAL_SIZE;
}

HAL_StatusTypeDef W25Q_EraseAll(void) {
    SPI_HandleTypeDef *Handle = W25Q_HANDLE;

    W25Q_WP_Disable();

    if (!W25Q_VerifyJEDECID(Handle)) return HAL_ERROR;
    if (W25Q_UnprotectAll(Handle) != HAL_OK) return HAL_ERROR;
    if (W25Q_ChipErase(Handle) != HAL_OK) return HAL_ERROR;

    WritePointer = FLASH_DATA_START;
    return W25Q_WriteMagic(Handle);
}

bool W25Q_LoggingInit(void) {
    if (!W25Q_Init()) return false;

    W25Q_NewFlight();
    W25Q_Initialized = true;
    return true;
}

void W25Q_LoggingStop(void) {
    if (!W25Q_Initialized) return;

    W25Q_WaitBusy(W25Q_HANDLE, 5);
    W25Q_ProtectAll(W25Q_HANDLE);
    W25Q_Initialized = false;
}

bool W25Q_MaintenanceMode(void) {
#if FLASH_DUMP_TO_SD && !FLASH_ERASE_ALL
    return W25Q_DumpToSD();
#elif FLASH_ERASE_ALL && !FLASH_DUMP_TO_SD
    return W25Q_EraseAll() == HAL_OK;
#else
    return false;
#endif
}

bool W25Q_DumpToSD(void) {
    if (!W25Q_Init()) return false;

    if (f_mount(&SDFatFS, SDPath, 1) != FR_OK) return false;

    uint32_t Address = FLASH_DATA_START;
    uint32_t End = WritePointer;
    uint16_t FlightNum = 0;
    bool FileOpen = false;
    FIL File;
    FlashLogRecord_t Record;

    while (Address < End) {
        if (W25Q_ReadData(W25Q_HANDLE, Address, (uint8_t *)&Record, sizeof(FlashLogRecord_t)) != HAL_OK) break;

        if (Record.Sync == PACKET_HEADER && Record.State == 0xFF) {
            if (FileOpen) f_close(&File);
            char Name[16];
            snprintf(Name, sizeof(Name), "FLASH_%u.BIN", FlightNum++);
            if (f_open(&File, Name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) break;
            FileOpen = true;
            Address += W25Q_PAGE_SIZE;
            continue;
        }

        if (FileOpen && Record.Sync == PACKET_HEADER) {
            UINT BytesWritten;
            f_write(&File, &Record, sizeof(FlashLogRecord_t), &BytesWritten);
        }
        Address += sizeof(FlashLogRecord_t);
    }

    if (FileOpen) f_close(&File);
    f_mount(NULL, SDPath, 1);
    return true;
}
