#ifndef BUZZERTASK_H
#define BUZZERTASK_H

#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t BuzzerTaskHandle;

void CreateBuzzerTask(UBaseType_t Priority, uint16_t StackSize);
void BuzzerTask(void *pvParameters);
void Buzzer_Notify(uint16_t Count, uint16_t DurationMs);

#endif //BUZZERTASK_H
