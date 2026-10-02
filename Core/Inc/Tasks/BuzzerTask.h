#ifndef BUZZERTASK_H
#define BUZZERTASK_H

#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t BuzzerTaskHandle;

void CreateBuzzerTask(UBaseType_t Priority, uint16_t StackSize);
void BuzzerTask(void *pvParameters);

#endif //BUZZERTASK_H
