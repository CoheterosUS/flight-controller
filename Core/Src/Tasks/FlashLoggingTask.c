#include "Tasks/FlashLoggingTask.h"
#include "Sensors/W25Q32JV.h"
#include "Sensors/Sensors.h"
#include "semphr.h"
#include <string.h>

TaskHandle_t FlashProducerTaskHandle;
TaskHandle_t FlashWriterTaskHandle;

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t W25Q_DMABuffer[4 + W25Q_PAGE_SIZE];

static FlashPage_t PageA;
static uint8_t ActiveCount = 0;

static QueueHandle_t PageReadyQueue;
static SemaphoreHandle_t FlushCompleteSemaphore;
static SemaphoreHandle_t WriterIdleSemaphore;
static volatile bool FlushRequested;
static volatile bool WriterBusy;
static bool ProducerAccepting;

volatile uint32_t dbg_flash_pages_written = 0;
volatile uint32_t dbg_flash_write_failures = 0;

SemaphoreHandle_t FlashSPISemaphore;

void CreateFlashLoggingTask(SystemContext_t *SystemContext, const UBaseType_t Priority, const uint16_t StackSize) {
    W25Q_CreateLock();
    PageReadyQueue = xQueueCreate(2, sizeof(FlashPage_t));
    FlushCompleteSemaphore = xSemaphoreCreateBinary();
    WriterIdleSemaphore = xSemaphoreCreateBinary();
    FlashSPISemaphore = xSemaphoreCreateBinary();
    memset(&PageA, 0xFF, sizeof(PageA));
    ActiveCount = 0;
    FlushRequested = false;
    WriterBusy = false;
    ProducerAccepting = false;

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

static bool QueueActivePage(TickType_t WaitTicks) {
    FlashPage_t Page = PageA;

    if (xQueueSend(PageReadyQueue, &Page, WaitTicks) != pdPASS) return false;

    memset(&PageA, 0xFF, sizeof(PageA));
    ActiveCount = 0;
    return true;
}

static void AddRecord(const FlashLogRecord_t *Record) {
    PageA.Records[ActiveCount++] = *Record;
    if (ActiveCount >= FLASH_PAGE_RECORDS) {
        (void)QueueActivePage(portMAX_DELAY);
    }
}

static void HandleFlushRequest(void) {
    FlashLogRecord_t Record;

    while (xQueueReceive(FlashLoggingQueue, &Record, 0) == pdPASS) {
        AddRecord(&Record);
    }

    ProducerAccepting = false;
    if (ActiveCount != 0) {
        (void)QueueActivePage(portMAX_DELAY);
    }

    FlushRequested = false;
    xSemaphoreGive(FlushCompleteSemaphore);
}

bool FlashLogging_FlushAndWait(const uint32_t TimeoutMs) {
    TickType_t Start;
    TickType_t Timeout;
    TickType_t Remaining;
    TickType_t Elapsed;

    if (FlashProducerTaskHandle == NULL || FlashWriterTaskHandle == NULL
        || PageReadyQueue == NULL) {
        return true;
    }

    (void)xSemaphoreTake(FlushCompleteSemaphore, 0);
    FlushRequested = true;
    Start = xTaskGetTickCount();
    Timeout = pdMS_TO_TICKS(TimeoutMs);

    Elapsed = xTaskGetTickCount() - Start;
    if (Elapsed >= Timeout) return false;
    Remaining = Timeout - Elapsed;
    if (xSemaphoreTake(FlushCompleteSemaphore, Remaining) != pdPASS) return false;

    for (;;) {
        if (!WriterBusy && uxQueueMessagesWaiting(PageReadyQueue) == 0) return true;

        Elapsed = xTaskGetTickCount() - Start;
        if (Elapsed >= Timeout) return false;
        Remaining = Timeout - Elapsed;
        if (xSemaphoreTake(WriterIdleSemaphore, Remaining) != pdPASS) return false;
    }
}

void CreateFlashMaintenanceTask(const UBaseType_t Priority, const uint16_t StackSize) {
    xTaskCreate(
        FlashMaintenanceTask,
        "FLASH_MAINT",
        StackSize,
        NULL,
        Priority,
        NULL
    );
}

void FlashProducerTask(void *pvParameters) {
    SystemContext_t *SystemContext = pvParameters;

    for (;;) {
        FlashLogRecord_t Record;

        if (SystemContext->FlashLoggingEnabled && !FlushRequested) {
            ProducerAccepting = true;
        } else if (!SystemContext->FlashLoggingEnabled && !FlushRequested) {
            ProducerAccepting = false;
        }

        if (xQueueReceive(FlashLoggingQueue, &Record, pdMS_TO_TICKS(10)) == pdPASS) {
            if (ProducerAccepting || FlushRequested) AddRecord(&Record);
        }

        if (FlushRequested) {
            HandleFlushRequest();
        }
    }
}

void FlashWriterTask(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        FlashPage_t Page;

        if (xQueueReceive(PageReadyQueue, &Page, portMAX_DELAY) != pdPASS) continue;

        WriterBusy = true;
        W25Q_Lock();

        uint32_t Address = W25Q_GetWritePointer();
        bool Success = W25Q_HasSpace(W25Q_PAGE_SIZE);
        if (!Success) {
            SystemFaultSet(W25Q_LOG_FULL);
        } else {
            memcpy(&W25Q_DMABuffer[4], Page.Records, W25Q_PAGE_SIZE);
            while (xSemaphoreTake(FlashSPISemaphore, 0) == pdPASS) {
            }

            Success = W25Q_PageProgramDMA(W25Q_HANDLE, Address, W25Q_DMABuffer, W25Q_PAGE_SIZE) == HAL_OK;
            if (Success) {
                Success = xSemaphoreTake(FlashSPISemaphore, pdMS_TO_TICKS(10)) == pdPASS;
            }
            if (Success) {
                // DMA completion only means the bytes left the MCU; the chip is still programming.
                Success = W25Q_WaitBusy(W25Q_HANDLE, 5) == HAL_OK;
            }

            if (!Success) {
                (void)HAL_SPI_Abort(W25Q_HANDLE);
                W25Q_DeselectCS();
                while (xSemaphoreTake(FlashSPISemaphore, 0) == pdPASS) {
                }
                SystemFaultSet(W25Q_WRITE_FAILED);
                dbg_flash_write_failures++;
            }

            W25Q_AdvanceWritePointer(W25Q_PAGE_SIZE);
            if (Success) dbg_flash_pages_written++;
        }

        W25Q_Unlock();
        WriterBusy = false;
        xSemaphoreGive(WriterIdleSemaphore);
    }
}

void FlashMaintenanceTask(void *pvParameters) {
    (void)pvParameters;

    const bool Success = W25Q_MaintenanceMode();

#if FLASH_ERASE_ALL
    Buzzer_Beep_Counter(100, Success ? 3 : 5, 200, false);
#else
    Buzzer_Beep_Counter(100, Success ? 2 : 5, 200, false);
#endif

    for (;;) vTaskDelay(portMAX_DELAY);
}
