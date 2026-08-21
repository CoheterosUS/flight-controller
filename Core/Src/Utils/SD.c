#include "Utils/SD.h"
#include <stdio.h>

extern SD_HandleTypeDef hsd1;

static FRESULT NextLogName(char *Out, size_t Size) {
    FILINFO FileInfo;

    for (uint32_t Index = 0; Index < 100; Index++) {
        snprintf(Out, Size, "LOG_%02lu.BIN", (unsigned long)Index);

        FRESULT Result = f_stat(Out, &FileInfo);
        if (Result == FR_NO_FILE) {
            return FR_OK;
        }
        if (Result != FR_OK) {
            return Result;
        }
    }

    return FR_DENIED;
}

FRESULT MountAndOpen(void) {
    FRESULT Result = f_mount(&SDFatFS, SDPath, 1);
    if (Result != FR_OK) {
        SystemFaultFlags |= SD_MOUNT_FAILED;
        return Result;
    }

    // TODO: Revise, comment out if necessary
    // Card identification done at 390 kHz (ClockDiv=162).
    // Switch to 8 MHz for data transfers: 64 MHz / (6+2) = 8 MHz
    // Improvement from around 1.4s to around 140ms
    hsd1.Init.ClockDiv = 6;
    SDMMC_Init(hsd1.Instance, hsd1.Init);

    char LogName[13];
    Result = NextLogName(LogName, sizeof(LogName));
    if (Result != FR_OK) {
        SystemFaultFlags |= SD_OPEN_FAILED;
        f_mount(NULL, SDPath, 1);
        return Result;
    }

    Result = f_open(&SDFile, LogName, FA_CREATE_NEW | FA_WRITE);
    if (Result != FR_OK) {
        SystemFaultFlags |= SD_OPEN_FAILED;
        f_mount(NULL, SDPath, 1);
        return Result;
    }

    return FR_OK;
}

void CloseFile(void) {
    f_sync(&SDFile);
    f_close(&SDFile);
    f_mount(NULL, SDPath, 1);
}

FRESULT WriteLoggingBuffer(SDLoggingBuffer_t *Buffer) {
    if (Buffer == NULL || Buffer->Count == 0) {
        return FR_OK;
    }

    UINT BytesWritten = 0;
    UINT BytesToWrite = Buffer->Count * sizeof(SDLogRecord_t);

    FRESULT Result = f_write(&SDFile, Buffer->Records, BytesToWrite, &BytesWritten);
    if (Result == FR_OK && BytesWritten != BytesToWrite) {
        return FR_INT_ERR;
    }

    return Result;
}
