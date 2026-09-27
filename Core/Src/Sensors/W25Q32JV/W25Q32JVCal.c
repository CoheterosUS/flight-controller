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
_Static_assert(FLASH_CAL_SECTOR_ADDRESS == W25Q_TOTAL_SIZE - W25Q_SECTOR_SIZE,
               "calibration sector must be the final flash sector");

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

static bool W25Q_CalScan(W25Q_CalRecord_t *Newest, uint32_t *LastNonErased, bool *ReadOk) {
    bool FoundValid = false;
    *LastNonErased = W25Q_CAL_SLOT_COUNT;
    *ReadOk = true;

    for (uint32_t Slot = 0; Slot < W25Q_CAL_SLOT_COUNT; Slot++) {
        W25Q_CalRecord_t Record;
        uint32_t Address = FLASH_CAL_SECTOR_ADDRESS + Slot * W25Q_CAL_SLOT_SIZE;

        if (W25Q_ReadData(W25Q_HANDLE, Address, (uint8_t *)&Record, sizeof(Record)) != HAL_OK) {
            *ReadOk = false;
            return false;
        }
        if (!W25Q_CalIsErased((const uint8_t *)&Record, sizeof(Record))) *LastNonErased = Slot;

        if (W25Q_CalRecordValid(&Record) && (!FoundValid || Record.Sequence > Newest->Sequence)) {
            *Newest = Record;
            FoundValid = true;
        }
    }

    return FoundValid;
}

bool W25Q_CalLoad(float M[9]) {
    W25Q_CalRecord_t Newest;
    uint32_t LastNonErased;
    bool ReadOk;

    if (M == NULL || !W25Q_CalFlashPresent()) return false;
    if (!W25Q_CalScan(&Newest, &LastNonErased, &ReadOk) || !ReadOk) return false;

    memcpy(M, Newest.M, sizeof(Newest.M));
    return true;
}

uint32_t W25Q_CalGetSequence(void) {
    W25Q_CalRecord_t Newest;
    uint32_t LastNonErased;
    bool ReadOk;

    if (!W25Q_CalFlashPresent()) return 0;
    if (!W25Q_CalScan(&Newest, &LastNonErased, &ReadOk) || !ReadOk) return 0;
    return Newest.Sequence;
}

static bool W25Q_CalRestoreProtection(uint8_t OriginalStatus) {
    if (OriginalStatus & (W25Q_SR1_BP0 | W25Q_SR1_BP1 | W25Q_SR1_BP2)) {
        return W25Q_ProtectAll(W25Q_HANDLE) == HAL_OK;
    }

    return true;
}

bool W25Q_CalAppend(const float M[9]) {
    W25Q_CalRecord_t NewRecord = {0};
    W25Q_CalRecord_t Newest = {0};
    W25Q_CalRecord_t ReadBack;
    uint32_t LastNonErased;
    uint32_t Slot;
    uint8_t OriginalStatus;
    bool Success = false;
    bool Protected;
    bool ReadOk;

    if (M == NULL || !W25Q_CalFlashPresent()) return false;
    if (W25Q_ReadStatusReg1(W25Q_HANDLE, &OriginalStatus) != HAL_OK) return false;

    Protected = (OriginalStatus & (W25Q_SR1_BP0 | W25Q_SR1_BP1 | W25Q_SR1_BP2)) != 0;
    if (W25Q_UnprotectAll(W25Q_HANDLE) != HAL_OK) return false;

    (void)W25Q_CalScan(&Newest, &LastNonErased, &ReadOk);
    if (!ReadOk) goto cleanup;

    if (LastNonErased == W25Q_CAL_SLOT_COUNT) {
        Slot = 0;
    } else if (LastNonErased < W25Q_CAL_SLOT_COUNT - 1u) {
        Slot = LastNonErased + 1u;
    } else {
        if (W25Q_SectorErase(W25Q_HANDLE, FLASH_CAL_SECTOR_ADDRESS) != HAL_OK) goto cleanup;
        Slot = 0;
    }

    NewRecord.Magic = W25Q_CAL_MAGIC;
    NewRecord.Version = W25Q_CAL_VERSION;
    NewRecord.Length = W25Q_CAL_RECORD_LENGTH;
    NewRecord.Sequence = Newest.Magic == W25Q_CAL_MAGIC && W25Q_CalRecordValid(&Newest)
        ? Newest.Sequence + 1u
        : 1u;
    if (NewRecord.Sequence == 0) NewRecord.Sequence = 1;
    memcpy(NewRecord.M, M, sizeof(NewRecord.M));
    memset(NewRecord.Reserved, 0xFF, sizeof(NewRecord.Reserved));
    NewRecord.Crc32 = W25Q_CalCrc32((const uint8_t *)&NewRecord, offsetof(W25Q_CalRecord_t, Crc32));

    if (W25Q_PageProgram(W25Q_HANDLE,
                         FLASH_CAL_SECTOR_ADDRESS + Slot * W25Q_CAL_SLOT_SIZE,
                         (const uint8_t *)&NewRecord,
                         sizeof(NewRecord)) != HAL_OK) goto cleanup;
    if (W25Q_ReadData(W25Q_HANDLE,
                      FLASH_CAL_SECTOR_ADDRESS + Slot * W25Q_CAL_SLOT_SIZE,
                      (uint8_t *)&ReadBack,
                      sizeof(ReadBack)) != HAL_OK) goto cleanup;

    Success = W25Q_CalRecordValid(&ReadBack) &&
              memcmp(ReadBack.M, M, sizeof(ReadBack.M)) == 0 &&
              ReadBack.Sequence == NewRecord.Sequence;

cleanup:
    if (Protected && !W25Q_CalRestoreProtection(OriginalStatus)) Success = false;
    return Success;
}
