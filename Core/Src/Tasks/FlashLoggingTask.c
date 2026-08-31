#include "Tasks/FlashLoggingTask.h"
#include "Sensors/W25Q32JV.h"
#include "semphr.h"
#include <string.h>

TaskHandle_t FlashProducerTaskHandle;
TaskHandle_t FlashWriterTaskHandle;

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t W25Q_DMABuffer[4 + W25Q_PAGE_SIZE];

static FlashPage_t PageA;
static FlashPage_t PageB;
static FlashPage_t *ActivePage = &PageA;
static FlashPage_t *WritePage = NULL;
static uint8_t ActiveCount = 0;

static SemaphoreHandle_t PageReadySemaphore;

SemaphoreHandle_t FlashSPISemaphore;

void CreateFlashLoggingTask(SystemContext_t *SystemContext, const UBaseType_t Priority, const uint16_t StackSize) {
    PageReadySemaphore = xSemaphoreCreateBinary();
    FlashSPISemaphore = xSemaphoreCreateBinary();

    xTaskCreate(
        FlashProducerTask,
        "FLASH_PRODUCER",
        StackSize,
        SystemContext,
        Priority + 1,
        &FlashProducerTaskHandle
    );

    xTaskCreate(
        FlashWriterTask,
        "FLASH_WRITER",
        StackSize,
        SystemContext,
        Priority,
        &FlashWriterTaskHandle
    );
}

void FlashProducerTask(void *pvParameters) {
    SystemContext_t *SystemContext = pvParameters;

    for (;;) {
        FlashLogRecord_t Record;

        if (xQueueReceive(FlashLoggingQueue, &Record, portMAX_DELAY) != pdPASS) continue;

        if (!SystemContext->FlashLoggingEnabled) continue;

        ActivePage->Records[ActiveCount++] = Record;

        if (ActiveCount >= FLASH_PAGE_RECORDS) {
            WritePage = ActivePage;
            ActivePage = (ActivePage == &PageA) ? &PageB : &PageA;
            ActiveCount = 0;

            xSemaphoreGive(PageReadySemaphore);
        }
    }
}

void FlashWriterTask(void *pvParameters) {
    for (;;) {
        xSemaphoreTake(PageReadySemaphore, portMAX_DELAY);

        if (WritePage == NULL) continue;

        uint32_t Address = W25Q_GetWritePointer();
        if (!W25Q_HasSpace(W25Q_PAGE_SIZE)) continue;

        memcpy(&W25Q_DMABuffer[4], WritePage->Records, W25Q_PAGE_SIZE);

        if (W25Q_PageProgramDMA(W25Q_HANDLE, Address, W25Q_DMABuffer, W25Q_PAGE_SIZE) == HAL_OK) {
            xSemaphoreTake(FlashSPISemaphore, pdMS_TO_TICKS(10));
            W25Q_AdvanceWritePointer(W25Q_PAGE_SIZE);
        }
    }
}
