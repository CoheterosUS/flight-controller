#ifndef HOST_W25Q32JV_H
#define HOST_W25Q32JV_H

#include <stdbool.h>
#include <stdint.h>

#define FLASH_CAL_SECTOR_ADDRESS 0x003FF000u
#define W25Q_PAGE_SIZE 256u
#define W25Q_SECTOR_SIZE 4096u
#define W25Q_TOTAL_SIZE (4u * 1024u * 1024u)
#define W25Q_LOG_END FLASH_CAL_SECTOR_ADDRESS
#define W25Q_JEDEC_MFR 0xEFu
#define W25Q_JEDEC_TYPE 0x40u
#define W25Q_JEDEC_CAPACITY 0x16u
#define W25Q_CAL_SECTOR_ADDRESS FLASH_CAL_SECTOR_ADDRESS
#define W25Q_SR1_BP0 (1u << 2)
#define W25Q_SR1_BP1 (1u << 3)
#define W25Q_SR1_BP2 (1u << 4)
#define W25Q_SNAPSHOT_VERSION 1u

typedef struct { int Unused; } SPI_HandleTypeDef;
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
#define W25Q_HANDLE ((SPI_HandleTypeDef *)0)

void W25Q_Lock(void);
void W25Q_Unlock(void);

static inline bool W25Q_LogHasSpaceAt(uint32_t Address, uint32_t Bytes) {
    return Address <= W25Q_LOG_END && Bytes <= W25Q_LOG_END - Address;
}

typedef struct {
    uint16_t Magic;
    uint16_t Version;
    uint16_t Flags;
    uint32_t ImuCalSequence;
    float M[9];
    float AccelBiasCal[3];
    float GyroBiasRaw[3];
    float ReferencePressurePa;
    float ReferenceTemperatureC;
    uint16_t ImuOdrHz;
    uint16_t ConfigVersion;
    uint32_t Crc32;
} FlightSnapshot_t;

HAL_StatusTypeDef W25Q_ReadJEDECID(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint8_t *);
HAL_StatusTypeDef W25Q_ReadData(SPI_HandleTypeDef *, uint32_t, uint8_t *, uint32_t);
HAL_StatusTypeDef W25Q_ReadStatusReg1(SPI_HandleTypeDef *, uint8_t *);
HAL_StatusTypeDef W25Q_UnprotectAll(SPI_HandleTypeDef *);
HAL_StatusTypeDef W25Q_ProtectAll(SPI_HandleTypeDef *);
HAL_StatusTypeDef W25Q_SectorErase(SPI_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef W25Q_PageProgram(SPI_HandleTypeDef *, uint32_t, const uint8_t *, uint16_t);

bool W25Q_CalLoad(float M[9]);
bool W25Q_CalAppend(const float M[9]);
uint32_t W25Q_CalGetSequence(void);

#endif
