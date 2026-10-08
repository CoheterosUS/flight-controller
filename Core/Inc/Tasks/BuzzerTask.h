#ifndef BUZZERTASK_H
#define BUZZERTASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

typedef struct {
    uint16_t Count;
    uint16_t DurationMs;
    uint16_t GapMs;
} BuzzerCommand_t;

extern TaskHandle_t BuzzerTaskHandle;
void CreateBuzzerTask(UBaseType_t Priority, uint16_t StackSize);
void BuzzerTask(void *pvParameters);
void Buzzer_Notify(uint16_t Count, uint16_t DurationMs, uint16_t GapMs);

#endif //BUZZERTASK_H
