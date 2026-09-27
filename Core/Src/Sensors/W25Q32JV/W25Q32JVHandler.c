#include "Sensors/W25Q32JV.h"
#include "Utils/shared.h"
#include "Managers/StructManager.h"
#include "fatfs.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

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

_Static_assert(sizeof(FlightSnapshot_t) <= W25Q_PAGE_SIZE, "snapshot must fit in one page");

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
    uint32_t Address = StartAddress & ~(uint32_t)(W25Q_PAGE_SIZE - 1);

    if (Address >= W25Q_LOG_END) return W25Q_LOG_END;

    while (Address < W25Q_LOG_END) {
        if (W25Q_ReadData(Handle, Address, &Byte, 1) != HAL_OK) break;
        if (Byte == 0xFF) return Address;
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

    W25Q_DeselectCS();
    W25Q_WP_Disable();
    W25Q_WaitBusy(Handle, 60000);

    if (!W25Q_VerifyJEDECID(Handle)) {
        SystemFaultFlags |= W25Q_JEDEC_ID_FAILED;
        return false;
    }

    if (W25Q_UnprotectAll(Handle) != HAL_OK) {
        SystemFaultFlags |= W25Q_INIT_FAILED;
        return false;
    }

    FlashHeader_t ReadHeader;
    if (W25Q_ReadData(Handle, FLASH_HEADER_ADDRESS, (uint8_t *)&ReadHeader, sizeof(FlashHeader_t)) != HAL_OK) {
        SystemFaultFlags |= W25Q_INIT_FAILED;
        return false;
    }

    if (ReadHeader.Magic == 0xFFFFFFFF) {
        Header.Magic = FLASH_HEADER_MAGIC;
        Header.FlightCount = 0;
        // The header is only a hint; a lost header must not hide data still present in the array.
        Header.WritePointer = W25Q_ScanForWritePointer(Handle, FLASH_DATA_START);

        if (W25Q_WriteHeader(Handle) != HAL_OK) {
            SystemFaultFlags |= W25Q_INIT_FAILED;
            return false;
        }
    } else if (ReadHeader.Magic == FLASH_HEADER_MAGIC) {
        Header = ReadHeader;
        Header.WritePointer = W25Q_ScanForWritePointer(Handle, Header.WritePointer);
    } else {
        SystemFaultFlags |= W25Q_INIT_FAILED;
        return false;
    }

    return true;
}

void W25Q_NewFlight(void) {
    if (!W25Q_HasSpace(W25Q_PAGE_SIZE)) return;

    FlashLogRecord_t Marker = {0};
    Marker.Sync = PACKET_HEADER;
    Marker.State = 0xFF;
    Marker.SyncEnd = PACKET_FOOTER;

    if (W25Q_PageProgram(W25Q_HANDLE, Header.WritePointer, (const uint8_t *)&Marker, sizeof(FlashLogRecord_t)) == HAL_OK) {
        Header.WritePointer += W25Q_PAGE_SIZE;
        Header.FlightCount++;
        W25Q_WriteHeader(W25Q_HANDLE);
    }
}

uint32_t W25Q_GetWritePointer(void) {
    return Header.WritePointer;
}

void W25Q_AdvanceWritePointer(uint16_t Bytes) {
    if (W25Q_LogHasSpaceAt(Header.WritePointer, Bytes)) Header.WritePointer += Bytes;
}

bool W25Q_HasSpace(uint16_t Bytes) {
    return W25Q_LogHasSpaceAt(Header.WritePointer, Bytes);
}

uint32_t W25Q_LogEndAddress(void) {
    return W25Q_LOG_END;
}

HAL_StatusTypeDef W25Q_EraseAll(void) {
    SPI_HandleTypeDef *Handle = W25Q_HANDLE;

    W25Q_WP_Disable();

    if (!W25Q_VerifyJEDECID(Handle)) return HAL_ERROR;
    if (W25Q_UnprotectAll(Handle) != HAL_OK) return HAL_ERROR;

    for (uint32_t Address = 0; Address < W25Q_TOTAL_SIZE - W25Q_BLOCK_SIZE; Address += W25Q_BLOCK_SIZE) {
        if (W25Q_BlockErase64K(Handle, Address) != HAL_OK) return HAL_ERROR;
    }

    for (uint32_t Address = W25Q_TOTAL_SIZE - W25Q_BLOCK_SIZE;
         Address < W25Q_LOG_END;
         Address += W25Q_SECTOR_SIZE) {
        if (W25Q_SectorErase(Handle, Address) != HAL_OK) return HAL_ERROR;
    }

    Header.Magic = FLASH_HEADER_MAGIC;
    Header.FlightCount = 0;
    Header.WritePointer = FLASH_DATA_START;

    return W25Q_WriteHeader(Handle);
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
    W25Q_WriteHeader(W25Q_HANDLE);
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
    uint32_t End = Header.WritePointer;
    uint16_t FlightNum = 0;
    bool FileOpen = false;
    FIL File;
    FlashPage_t Page;

    while (Address < End) {
        if (W25Q_ReadData(W25Q_HANDLE, Address, (uint8_t *)&Page, sizeof(FlashPage_t)) != HAL_OK) break;

        if (W25Q_IsSnapshotPage((const uint8_t *)&Page)) {
            Address += W25Q_PAGE_SIZE;
            continue;
        }

        if (W25Q_IsFlightMarker(&Page.Records[0])) {
            if (FileOpen) f_close(&File);
            char Name[16];
            snprintf(Name, sizeof(Name), "F_%02u.BIN", FlightNum++);
            if (f_open(&File, Name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) break;
            FileOpen = true;
            Address += W25Q_PAGE_SIZE;
            continue;
        }

        if (FileOpen) {
            for (uint8_t i = 0; i < FLASH_PAGE_RECORDS; i++) {
                if (Page.Records[i].Sync != PACKET_HEADER) break;
                UINT BytesWritten;
                f_write(&File, &Page.Records[i], sizeof(FlashLogRecord_t), &BytesWritten);
            }
        }
        Address += W25Q_PAGE_SIZE;
    }

    if (FileOpen) f_close(&File);
    f_mount(NULL, SDPath, 1);
    return FlightNum > 0;
}

bool W25Q_SnapshotWrite(const FlightSnapshot_t *S) {
    FlightSnapshot_t Snapshot;
    uint8_t Page[W25Q_PAGE_SIZE];

    if (S == NULL || !W25Q_HasSpace(W25Q_PAGE_SIZE)) return false;
    if ((W25Q_GetWritePointer() & (W25Q_PAGE_SIZE - 1u)) != 0) return false;

    memset(Page, 0xFF, sizeof(Page));
    memcpy(&Snapshot, S, sizeof(Snapshot));
    Snapshot.Magic = PACKET_HEADER;
    Snapshot.Version = W25Q_SNAPSHOT_VERSION;
    Snapshot.Crc32 = W25Q_SnapshotCrc32((const uint8_t *)&Snapshot, offsetof(FlightSnapshot_t, Crc32));
    memcpy(Page, &Snapshot, sizeof(Snapshot));

    if (W25Q_PageProgram(W25Q_HANDLE,
                         W25Q_GetWritePointer(),
                         Page,
                         W25Q_PAGE_SIZE) != HAL_OK) return false;

    W25Q_AdvanceWritePointer(W25Q_PAGE_SIZE);
    return true;
}
