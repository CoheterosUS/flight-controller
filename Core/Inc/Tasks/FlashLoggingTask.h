#ifndef FLASHLOGGINGTASK_H
#define FLASHLOGGINGTASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "Utils/shared.h"

extern TaskHandle_t FlashProducerTaskHandle;
extern TaskHandle_t FlashWriterTaskHandle;

void CreateFlashLoggingTask(SystemContext_t *SystemContext, UBaseType_t Priority, uint16_t StackSize);
void FlashProducerTask(void *pvParameters);
void FlashWriterTask(void *pvParameters);

#endif //FLASHLOGGINGTASK_H
