#include "Sensors/W25Q32JV.h"
#include <stddef.h>
#include <string.h>

#define W25Q_CAL_MAGIC             0x314C4143u
#define W25Q_CAL_VERSION           1u
#define W25Q_CAL_SLOT_COUNT        (W25Q_SECTOR_SIZE / 64u)
#define W25Q_CAL_SLOT_SIZE         64u
#define W25Q_CAL_RECORD_LENGTH     W25Q_CAL_SLOT_SIZE

#pragma pack(push, 1)
typedef struct {
    uint32_t Magic;
    uint16_t Version;
    uint16_t Length;
    uint32_t Sequence;
    float M[9];
    uint8_t Reserved[12];
    uint32_t Crc32;
} W25Q_CalRecord_t;
#pragma pack(pop)

_Static_assert(sizeof(W25Q_CalRecord_t) == W25Q_CAL_SLOT_SIZE, "calibration record must fill one slot");
_Static_assert(FLASH_CAL_SECTOR_B_ADDRESS == W25Q_TOTAL_SIZE - W25Q_SECTOR_SIZE,
               "calibration sector B must be the final flash sector");
_Static_assert(FLASH_CAL_SECTOR_A_ADDRESS + W25Q_SECTOR_SIZE == FLASH_CAL_SECTOR_B_ADDRESS,
               "calibration sectors must be adjacent");

typedef struct {
    W25Q_CalRecord_t Newest;
    uint32_t LastNonErased;
    bool FoundValid;
    bool ReadOk;
} W25Q_CalSectorScan_t;

static uint32_t CachedSequence;
static bool SequenceCached;

// Reflected CRC-32 with polynomial 0xEDB88320.
static uint32_t W25Q_CalCrc32(const uint8_t *Data, size_t Length) {
    uint32_t Crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < Length; i++) {
        Crc ^= Data[i];
        for (uint8_t Bit = 0; Bit < 8; Bit++) {
            Crc = (Crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(Crc & 1u));
        }
    }

    return ~Crc;
}

static bool W25Q_CalIsErased(const uint8_t *Data, size_t Length) {
    for (size_t i = 0; i < Length; i++) {
        if (Data[i] != 0xFFu) return false;
    }
    return true;
}

static bool W25Q_CalRecordValid(const W25Q_CalRecord_t *Record) {
    return Record->Magic == W25Q_CAL_MAGIC &&
           Record->Version == W25Q_CAL_VERSION &&
           Record->Length == W25Q_CAL_RECORD_LENGTH &&
           Record->Crc32 == W25Q_CalCrc32((const uint8_t *)Record, offsetof(W25Q_CalRecord_t, Crc32));
}

static bool W25Q_CalFlashPresent(void) {
    uint8_t ManufacturerID;
    uint8_t DeviceType;
    uint8_t Capacity;

    if (W25Q_ReadJEDECID(W25Q_HANDLE, &ManufacturerID, &DeviceType, &Capacity) != HAL_OK) return false;
    return ManufacturerID == W25Q_JEDEC_MFR &&
           DeviceType == W25Q_JEDEC_TYPE &&
           Capacity == W25Q_JEDEC_CAPACITY;
}

static W25Q_CalSectorScan_t W25Q_CalScanSector(uint32_t SectorAddress) {
    W25Q_CalSectorScan_t Scan = {0};
    Scan.LastNonErased = W25Q_CAL_SLOT_COUNT;
    Scan.ReadOk = true;

    for (uint32_t Slot = 0; Slot < W25Q_CAL_SLOT_COUNT; Slot++) {
        W25Q_CalRecord_t Record;
        uint32_t Address = SectorAddress + Slot * W25Q_CAL_SLOT_SIZE;

        if (W25Q_ReadData(W25Q_HANDLE, Address, (uint8_t *)&Record, sizeof(Record)) != HAL_OK) {
            Scan.ReadOk = false;
            return Scan;
        }
        if (!W25Q_CalIsErased((const uint8_t *)&Record, sizeof(Record))) Scan.LastNonErased = Slot;

        if (W25Q_CalRecordValid(&Record)
            && (!Scan.FoundValid || Record.Sequence > Scan.Newest.Sequence)) {
            Scan.Newest = Record;
            Scan.FoundValid = true;
        }
    }

    return Scan;
}

static const W25Q_CalSectorScan_t *W25Q_CalNewestSector(const W25Q_CalSectorScan_t *A,
                                                        const W25Q_CalSectorScan_t *B) {
    if (!A->FoundValid) return B->FoundValid ? B : NULL;
    if (!B->FoundValid) return A;
    return B->Newest.Sequence > A->Newest.Sequence ? B : A;
}

bool W25Q_CalLoad(float M[9]) {
    bool Success = false;

    W25Q_Lock();

    SequenceCached = true;
    CachedSequence = 0;
    if (M == NULL || !W25Q_CalFlashPresent()) goto cleanup;

    W25Q_CalSectorScan_t A = W25Q_CalScanSector(FLASH_CAL_SECTOR_A_ADDRESS);
    W25Q_CalSectorScan_t B = W25Q_CalScanSector(FLASH_CAL_SECTOR_B_ADDRESS);
    const W25Q_CalSectorScan_t *Newest = W25Q_CalNewestSector(&A, &B);
    if (!A.ReadOk || !B.ReadOk || Newest == NULL) goto cleanup;

    memcpy(M, Newest->Newest.M, sizeof(Newest->Newest.M));
    CachedSequence = Newest->Newest.Sequence;
    Success = true;

cleanup:
    W25Q_Unlock();
    return Success;
}

uint32_t W25Q_CalGetSequence(void) {
    if (!SequenceCached) {
        float M[9];
        (void)W25Q_CalLoad(M);
    }
    return CachedSequence;
}

static bool W25Q_CalRestoreProtection(uint8_t OriginalStatus) {
    if (OriginalStatus & (W25Q_SR1_BP0 | W25Q_SR1_BP1 | W25Q_SR1_BP2)) {
        return W25Q_ProtectAll(W25Q_HANDLE) == HAL_OK;
    }

    return true;
}

bool W25Q_CalAppend(const float M[9]) {
    W25Q_CalRecord_t NewRecord = {0};
    W25Q_CalRecord_t ReadBack;
    uint32_t SectorAddress;
    uint32_t Slot;
    uint8_t OriginalStatus;
    bool Success = false;
    bool Protected = false;

    W25Q_Lock();

    if (M == NULL || !W25Q_CalFlashPresent()) goto cleanup;
    if (W25Q_ReadStatusReg1(W25Q_HANDLE, &OriginalStatus) != HAL_OK) goto cleanup;

    Protected = (OriginalStatus & (W25Q_SR1_BP0 | W25Q_SR1_BP1 | W25Q_SR1_BP2)) != 0;
    if (W25Q_UnprotectAll(W25Q_HANDLE) != HAL_OK) goto cleanup;

    W25Q_CalSectorScan_t A = W25Q_CalScanSector(FLASH_CAL_SECTOR_A_ADDRESS);
    W25Q_CalSectorScan_t B = W25Q_CalScanSector(FLASH_CAL_SECTOR_B_ADDRESS);
    const W25Q_CalSectorScan_t *Newest = W25Q_CalNewestSector(&A, &B);
    if (!A.ReadOk || !B.ReadOk) goto cleanup;

    const W25Q_CalSectorScan_t *Active = Newest;
    if (Active == NULL) {
        if (A.LastNonErased != W25Q_CAL_SLOT_COUNT - 1u) {
            Active = &A;
            SectorAddress = FLASH_CAL_SECTOR_A_ADDRESS;
        } else if (B.LastNonErased != W25Q_CAL_SLOT_COUNT - 1u) {
            Active = &B;
            SectorAddress = FLASH_CAL_SECTOR_B_ADDRESS;
        } else {
            SectorAddress = FLASH_CAL_SECTOR_B_ADDRESS;
            if (W25Q_SectorErase(W25Q_HANDLE, SectorAddress) != HAL_OK) goto cleanup;
            Slot = 0;
        }
    } else {
        SectorAddress = Active == &A ? FLASH_CAL_SECTOR_A_ADDRESS : FLASH_CAL_SECTOR_B_ADDRESS;
    }

    if (Active != NULL && Active->LastNonErased == W25Q_CAL_SLOT_COUNT - 1u) {
        SectorAddress = SectorAddress == FLASH_CAL_SECTOR_A_ADDRESS
            ? FLASH_CAL_SECTOR_B_ADDRESS
            : FLASH_CAL_SECTOR_A_ADDRESS;
        if (W25Q_SectorErase(W25Q_HANDLE, SectorAddress) != HAL_OK) goto cleanup;
        Slot = 0;
    } else if (Active != NULL && Active->LastNonErased == W25Q_CAL_SLOT_COUNT) {
        Slot = 0;
    } else if (Active != NULL) {
        Slot = Active->LastNonErased + 1u;
    }

    NewRecord.Magic = W25Q_CAL_MAGIC;
    NewRecord.Version = W25Q_CAL_VERSION;
    NewRecord.Length = W25Q_CAL_RECORD_LENGTH;
    NewRecord.Sequence = Newest != NULL
        ? Newest->Newest.Sequence + 1u
        : 1u;
    if (NewRecord.Sequence == 0) NewRecord.Sequence = 1;
    memcpy(NewRecord.M, M, sizeof(NewRecord.M));
    memset(NewRecord.Reserved, 0xFF, sizeof(NewRecord.Reserved));
    NewRecord.Crc32 = W25Q_CalCrc32((const uint8_t *)&NewRecord, offsetof(W25Q_CalRecord_t, Crc32));

    if (W25Q_PageProgram(W25Q_HANDLE,
                         SectorAddress + Slot * W25Q_CAL_SLOT_SIZE,
                         (const uint8_t *)&NewRecord,
                         sizeof(NewRecord)) != HAL_OK) goto cleanup;
    if (W25Q_ReadData(W25Q_HANDLE,
                      SectorAddress + Slot * W25Q_CAL_SLOT_SIZE,
                      (uint8_t *)&ReadBack,
                      sizeof(ReadBack)) != HAL_OK) goto cleanup;

    Success = W25Q_CalRecordValid(&ReadBack) &&
              memcmp(ReadBack.M, M, sizeof(ReadBack.M)) == 0 &&
              ReadBack.Sequence == NewRecord.Sequence;
    if (Success) {
        CachedSequence = NewRecord.Sequence;
        SequenceCached = true;
    }

cleanup:
    if (Protected && !W25Q_CalRestoreProtection(OriginalStatus)) Success = false;
    W25Q_Unlock();
    return Success;
}

bool W25Q_PageIsFullyErased(const uint8_t *Page) {
    return Page != NULL && W25Q_CalIsErased(Page, W25Q_PAGE_SIZE);
}

bool W25Q_ShouldCreateFlightMarker(bool PreviousPageExists, bool PreviousIsMarker) {
    return !PreviousPageExists || !PreviousIsMarker;
}

HAL_StatusTypeDef W25Q_EraseLogRegion(SPI_HandleTypeDef *Handle) {
    uint32_t Address = 0;

    while (Address + W25Q_BLOCK_SIZE <= W25Q_LOG_END) {
        if (W25Q_BlockErase64K(Handle, Address) != HAL_OK) return HAL_ERROR;
        Address += W25Q_BLOCK_SIZE;
    }
    while (Address < W25Q_LOG_END) {
        if (W25Q_SectorErase(Handle, Address) != HAL_OK) return HAL_ERROR;
        Address += W25Q_SECTOR_SIZE;
    }
    return HAL_OK;
}
