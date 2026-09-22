#ifndef PYROTASK_H
#define PYROTASK_H

#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t PyroTaskHandle;

void CreatePyroTask(UBaseType_t Priority, uint16_t StackSize);
void PyroTask(void *pvParameters);

#endif //PYROTASK_H
