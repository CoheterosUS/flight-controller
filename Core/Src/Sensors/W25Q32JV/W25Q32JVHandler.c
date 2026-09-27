#include "Sensors/W25Q32JV.h"
#include "Utils/shared.h"
#include "Managers/StructManager.h"
#include "fatfs.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "task.h"

#define FLASH_HEADER_MAGIC      0x464C5348
#define FLASH_HEADER_ADDRESS    0x00000000
#define FLASH_DATA_START        W25Q_SECTOR_SIZE

#pragma pack(push, 1)
typedef struct {
    uint32_t Magic;
    uint32_t FlightCount;
    uint32_t WritePointer;
} FlashHeader_t;
#pragma pack(pop)

static FlashHeader_t Header;
static bool W25Q_Initialized = false;
static bool LastPageIsFlightMarker = false;
static SemaphoreHandle_t W25QMutex;

_Static_assert(sizeof(FlightSnapshot_t) <= W25Q_PAGE_SIZE, "snapshot must fit in one page");

void W25Q_CreateLock(void) {
    if (W25QMutex == NULL) {
        W25QMutex = xSemaphoreCreateRecursiveMutex();
    }
}

void W25Q_Lock(void) {
    if (W25QMutex == NULL || xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return;
    (void)xSemaphoreTakeRecursive(W25QMutex, portMAX_DELAY);
}

void W25Q_Unlock(void) {
    if (W25QMutex == NULL || xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return;
    (void)xSemaphoreGiveRecursive(W25QMutex);
}

static bool W25Q_VerifyJEDECID(SPI_HandleTypeDef *Handle) {
    uint8_t MFR, Type, Cap;
    if (W25Q_ReadJEDECID(Handle, &MFR, &Type, &Cap) != HAL_OK) return false;
    return (MFR == W25Q_JEDEC_MFR && Type == W25Q_JEDEC_TYPE && Cap == W25Q_JEDEC_CAPACITY);
}

static bool W25Q_IsFlightMarker(const FlashLogRecord_t *Record) {
    return Record->Sync == PACKET_HEADER && Record->State == 0xFF && Record->SyncEnd == PACKET_FOOTER;
}

// Reflected CRC-32 with polynomial 0xEDB88320.
static uint32_t W25Q_SnapshotCrc32(const uint8_t *Data, size_t Length) {
    uint32_t Crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < Length; i++) {
        Crc ^= Data[i];
        for (uint8_t Bit = 0; Bit < 8; Bit++) {
            Crc = (Crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(Crc & 1u));
        }
    }

    return ~Crc;
}

static bool W25Q_IsSnapshotPage(const uint8_t *Page) {
    const FlightSnapshot_t *Snapshot = (const FlightSnapshot_t *)Page;

    // The fixed version, valid CRC and erased tail distinguish snapshots from log pages.
    if (Snapshot->Magic != PACKET_HEADER || Snapshot->Version != W25Q_SNAPSHOT_VERSION) return false;
    if (Snapshot->Crc32 != W25Q_SnapshotCrc32(Page, offsetof(FlightSnapshot_t, Crc32))) return false;

    for (size_t i = sizeof(FlightSnapshot_t); i < W25Q_PAGE_SIZE; i++) {
        if (Page[i] != 0xFFu) return false;
    }

    return true;
}

// Every written page (marker or data) starts with PACKET_HEADER_LSB; an erased page starts with 0xFF.
// The marker only occupies the first record of its page, so scanning must step by whole pages.
static uint32_t W25Q_ScanForWritePointer(SPI_HandleTypeDef *Handle, uint32_t StartAddress) {
    uint8_t Byte;
    uint8_t Page[W25Q_PAGE_SIZE];
    uint32_t Address = StartAddress & ~(uint32_t)(W25Q_PAGE_SIZE - 1);

    if (Address >= W25Q_LOG_END) return W25Q_LOG_END;

    while (Address < W25Q_LOG_END) {
        if (W25Q_ReadData(Handle, Address, &Byte, 1) != HAL_OK) return W25Q_LOG_END;
        if (Byte == 0xFF) {
            if (W25Q_ReadData(Handle, Address, Page, sizeof(Page)) != HAL_OK) return W25Q_LOG_END;
            if (W25Q_PageIsFullyErased(Page)) return Address;
        }
        Address += W25Q_PAGE_SIZE;
    }

    return Address < W25Q_LOG_END ? Address : W25Q_LOG_END;
}

static HAL_StatusTypeDef W25Q_WriteHeader(SPI_HandleTypeDef *Handle) {
    if (W25Q_SectorErase(Handle, FLASH_HEADER_ADDRESS) != HAL_OK) return HAL_ERROR;
    return W25Q_PageProgram(Handle, FLASH_HEADER_ADDRESS, (const uint8_t *)&Header, sizeof(FlashHeader_t));
}

bool W25Q_Init(void) {
    SPI_HandleTypeDef *Handle = W25Q_HANDLE;
    bool Success = false;

    W25Q_Lock();

    W25Q_DeselectCS();
    W25Q_WP_Disable();
    if (W25Q_WaitBusy(Handle, 60000) != HAL_OK) {
        SystemFaultSet(W25Q_INIT_FAILED);
        goto cleanup;
    }

    if (!W25Q_VerifyJEDECID(Handle)) {
        SystemFaultSet(W25Q_JEDEC_ID_FAILED);
        goto cleanup;
    }

    if (W25Q_UnprotectAll(Handle) != HAL_OK) {
        SystemFaultSet(W25Q_INIT_FAILED);
        goto cleanup;
    }

    FlashHeader_t ReadHeader;
    if (W25Q_ReadData(Handle, FLASH_HEADER_ADDRESS, (uint8_t *)&ReadHeader, sizeof(FlashHeader_t)) != HAL_OK) {
        SystemFaultSet(W25Q_INIT_FAILED);
        goto cleanup;
    }

    if (ReadHeader.Magic == 0xFFFFFFFF) {
        Header.Magic = FLASH_HEADER_MAGIC;
        Header.FlightCount = 0;
        // The header is only a hint; a lost header must not hide data still present in the array.
        Header.WritePointer = W25Q_ScanForWritePointer(Handle, FLASH_DATA_START);

        if (W25Q_WriteHeader(Handle) != HAL_OK) {
            SystemFaultSet(W25Q_INIT_FAILED);
            goto cleanup;
        }
    } else if (ReadHeader.Magic == FLASH_HEADER_MAGIC) {
        Header = ReadHeader;
        Header.WritePointer = W25Q_ScanForWritePointer(Handle, Header.WritePointer);
    } else {
        SystemFaultSet(W25Q_INIT_FAILED);
        goto cleanup;
    }

    LastPageIsFlightMarker = false;
    if (Header.WritePointer > FLASH_DATA_START) {
        FlashPage_t PreviousPage;
        if (W25Q_ReadData(Handle,
                          Header.WritePointer - W25Q_PAGE_SIZE,
                          (uint8_t *)&PreviousPage,
                          sizeof(PreviousPage)) != HAL_OK) {
            SystemFaultSet(W25Q_INIT_FAILED);
            goto cleanup;
        }
        LastPageIsFlightMarker = W25Q_IsFlightMarker(&PreviousPage.Records[0]);
    }

    Success = true;

cleanup:
    W25Q_Unlock();
    return Success;
}

bool W25Q_NewFlight(void) {
    bool Success = false;

    W25Q_Lock();

    if (!W25Q_ShouldCreateFlightMarker(Header.WritePointer > FLASH_DATA_START,
                                       LastPageIsFlightMarker)) {
        Success = true;
        goto cleanup;
    }

    if (!W25Q_HasSpace(W25Q_PAGE_SIZE)) {
        SystemFaultSet(W25Q_LOG_FULL);
        goto cleanup;
    }

    FlashLogRecord_t Marker = {0};
    Marker.Sync = PACKET_HEADER;
    Marker.State = 0xFF;
    Marker.SyncEnd = PACKET_FOOTER;

    if (W25Q_PageProgram(W25Q_HANDLE, Header.WritePointer, (const uint8_t *)&Marker, sizeof(FlashLogRecord_t)) != HAL_OK) {
        SystemFaultSet(W25Q_INIT_FAILED);
        goto cleanup;
    }

    Header.WritePointer += W25Q_PAGE_SIZE;
    Header.FlightCount++;
    if (W25Q_WriteHeader(W25Q_HANDLE) != HAL_OK) {
        SystemFaultSet(W25Q_INIT_FAILED);
        goto cleanup;
    }
    LastPageIsFlightMarker = true;

    Success = true;

cleanup:
    W25Q_Unlock();
    return Success;
}

uint32_t W25Q_GetWritePointer(void) {
    return Header.WritePointer;
}

void W25Q_AdvanceWritePointer(uint16_t Bytes) {
    if (W25Q_LogHasSpaceAt(Header.WritePointer, Bytes)) {
        Header.WritePointer += Bytes;
        if (Bytes != 0) LastPageIsFlightMarker = false;
    }
}

bool W25Q_HasSpace(uint16_t Bytes) {
    return W25Q_LogHasSpaceAt(Header.WritePointer, Bytes);
}

uint32_t W25Q_LogEndAddress(void) {
    return W25Q_LOG_END;
}

HAL_StatusTypeDef W25Q_EraseAll(void) {
    SPI_HandleTypeDef *Handle = W25Q_HANDLE;
    HAL_StatusTypeDef Result = HAL_ERROR;

    W25Q_Lock();

    W25Q_WP_Disable();

    if (!W25Q_VerifyJEDECID(Handle)) goto cleanup;
    if (W25Q_UnprotectAll(Handle) != HAL_OK) goto cleanup;

    if (W25Q_EraseLogRegion(Handle) != HAL_OK) goto cleanup;

    Header.Magic = FLASH_HEADER_MAGIC;
    Header.FlightCount = 0;
    Header.WritePointer = FLASH_DATA_START;
    LastPageIsFlightMarker = false;

    Result = W25Q_WriteHeader(Handle);

cleanup:
    W25Q_Unlock();
    return Result;
}

bool W25Q_LoggingInit(void) {
    bool Success = false;

    W25Q_Lock();

    W25Q_Initialized = false;
    if (!W25Q_Init()) goto cleanup;
    if (!W25Q_NewFlight()) goto cleanup;
    W25Q_Initialized = true;
    Success = true;

cleanup:
    W25Q_Unlock();
    return Success;
}

void W25Q_LoggingStop(void) {
    W25Q_Lock();

    if (!W25Q_Initialized) goto cleanup;

    W25Q_WaitBusy(W25Q_HANDLE, 5);
    W25Q_WriteHeader(W25Q_HANDLE);
    W25Q_ProtectAll(W25Q_HANDLE);
    W25Q_Initialized = false;

cleanup:
    W25Q_Unlock();
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
    bool Success = true;
    W25Q_Lock();

    if (!W25Q_Init()) {
        Success = false;
        goto cleanup;
    }

    if (f_mount(&SDFatFS, SDPath, 1) != FR_OK) {
        Success = false;
        goto cleanup;
    }

    uint32_t Address = FLASH_DATA_START;
    uint32_t End = Header.WritePointer < W25Q_LOG_END ? Header.WritePointer : W25Q_LOG_END;
    uint16_t FlightNum = 0;
    bool FileOpen = false;
    FIL File;
    FlashPage_t Page;

    while (Address < End) {
        if (W25Q_ReadData(W25Q_HANDLE, Address, (uint8_t *)&Page, sizeof(FlashPage_t)) != HAL_OK) {
            Success = false;
            break;
        }

        if (W25Q_IsSnapshotPage((const uint8_t *)&Page)) {
            Address += W25Q_PAGE_SIZE;
            continue;
        }

        if (W25Q_IsFlightMarker(&Page.Records[0])) {
            if (FileOpen) {
                if (f_close(&File) != FR_OK) {
                    Success = false;
                    break;
                }
                FileOpen = false;
            }
            char Name[16];
            snprintf(Name, sizeof(Name), "F_%02u.BIN", FlightNum);
            if (f_open(&File, Name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
                Success = false;
                break;
            }
            FileOpen = true;
            FlightNum++;
            Address += W25Q_PAGE_SIZE;
            continue;
        }

        if (FileOpen) {
            for (uint8_t i = 0; i < FLASH_PAGE_RECORDS; i++) {
                if (Page.Records[i].Sync != PACKET_HEADER) break;
                UINT BytesWritten;
                FRESULT Result = f_write(&File, &Page.Records[i], sizeof(FlashLogRecord_t), &BytesWritten);
                if (Result != FR_OK || BytesWritten != sizeof(FlashLogRecord_t)) {
                    Success = false;
                    break;
                }
            }
            if (!Success) break;
        }
        Address += W25Q_PAGE_SIZE;
    }

    if (FileOpen && f_close(&File) != FR_OK) Success = false;
    if (f_mount(NULL, SDPath, 1) != FR_OK) Success = false;
    if (FlightNum == 0) Success = false;

cleanup:
    W25Q_Unlock();
    return Success;
}

bool W25Q_SnapshotWrite(const FlightSnapshot_t *S) {
    FlightSnapshot_t Snapshot;
    uint8_t Page[W25Q_PAGE_SIZE];

    bool Success = false;
    uint32_t Address;

    W25Q_Lock();

    if (S == NULL || !W25Q_HasSpace(W25Q_PAGE_SIZE)) goto cleanup;
    Address = W25Q_GetWritePointer();
    if ((Address & (W25Q_PAGE_SIZE - 1u)) != 0) goto cleanup;

    memset(Page, 0xFF, sizeof(Page));
    memcpy(&Snapshot, S, sizeof(Snapshot));
    Snapshot.Magic = PACKET_HEADER;
    Snapshot.Version = W25Q_SNAPSHOT_VERSION;
    Snapshot.Crc32 = W25Q_SnapshotCrc32((const uint8_t *)&Snapshot, offsetof(FlightSnapshot_t, Crc32));
    memcpy(Page, &Snapshot, sizeof(Snapshot));

    if (W25Q_PageProgram(W25Q_HANDLE,
                         Address,
                         Page,
                         W25Q_PAGE_SIZE) != HAL_OK) goto cleanup;
    if (!W25Q_SnapshotVerify(Address)) goto cleanup;

    W25Q_AdvanceWritePointer(W25Q_PAGE_SIZE);
    Success = true;

cleanup:
    W25Q_Unlock();
    return Success;
}

bool W25Q_SnapshotVerify(uint32_t Address) {
    uint8_t Page[W25Q_PAGE_SIZE];
    bool Success = false;

    W25Q_Lock();

    if ((Address & (W25Q_PAGE_SIZE - 1u)) == 0
        && W25Q_LogHasSpaceAt(Address, W25Q_PAGE_SIZE)
        && W25Q_ReadData(W25Q_HANDLE, Address, Page, sizeof(Page)) == HAL_OK) {
        Success = W25Q_IsSnapshotPage(Page);
    }

    W25Q_Unlock();
    return Success;
}
