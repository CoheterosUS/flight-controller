#include <Tasks/SDLoggingTask.h>
#include "Utils/SD.h"

TaskHandle_t SDProducerTaskHandle;
TaskHandle_t SDWriterTaskHandle;

static SDLoggingBuffer_t BufferA;
static SDLoggingBuffer_t BufferB;
static SDLoggingBuffer_t *ActiveBuffer = &BufferA;
static SDLoggingBuffer_t *WriteBuffer = NULL;

static SemaphoreHandle_t WriteSemaphore;

volatile uint64_t dbg_write_fault = 0;

void CreateSDLoggingTask(SystemContext_t *SystemContext, const UBaseType_t Priority, const uint16_t StackSize) {
    WriteSemaphore = xSemaphoreCreateBinary();

    xTaskCreate(
        SDProducerTask,
        "SD_PRODUCER",
        StackSize,
        SystemContext,
        Priority + 1,
        &SDProducerTaskHandle
    );

    xTaskCreate(
        SDWriterTask,
        "SD_WRITER",
        StackSize,
        SystemContext,
        Priority,
        &SDWriterTaskHandle
    );
}

void SDProducerTask(void *pvParameters) {
    SystemContext_t *SystemContext = pvParameters;

    for (;;) {
        SDLogRecord_t Record;

        BaseType_t SDQueueStatus = xQueueReceive(SDLoggingQueue, &Record, portMAX_DELAY);

        if (SDQueueStatus == pdPASS && SystemContext->SDLoggingEnabled) {
            ActiveBuffer->Records[ActiveBuffer->Count++] = Record;

            if (ActiveBuffer->Count >= SD_LOGGING_RECORDS_PER_BUFFER) {
                WriteBuffer = ActiveBuffer;
                ActiveBuffer = (ActiveBuffer == &BufferA) ? &BufferB : &BufferA;
                ActiveBuffer->Count = 0;

                xSemaphoreGive(WriteSemaphore);
            }
        }
    }
}

void SDWriterTask(void *pvParameters) {
    MountAndOpen();

    for (;;) {
        xSemaphoreTake(WriteSemaphore, portMAX_DELAY);

        if (WriteLoggingBuffer(WriteBuffer) != FR_OK) {
            dbg_write_fault++;
        }

        WriteBuffer->Count = 0;
        f_sync(&SDFile);
    }
}
