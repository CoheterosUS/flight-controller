#ifndef SD_H
#define SD_H

#include "fatfs.h"
#include "Managers/StructManager.h"

typedef struct {
    SDLogRecord_t Records[SD_LOGGING_RECORDS_PER_BUFFER];
    uint16_t Count;
} SDLoggingBuffer_t;

FRESULT MountAndOpen(void);
void CloseFile(void);
FRESULT WriteLoggingBuffer(SDLoggingBuffer_t *Buffer);

#endif //SD_H
