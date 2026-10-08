#ifndef TELEMETRYTASK_H
#define TELEMETRYTASK_H

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define TELEMETRY_RX_BUFFER_SIZE 128

extern uint8_t TELEMETRY_RX_BUFFER[TELEMETRY_RX_BUFFER_SIZE];
extern TaskHandle_t TelemetryReceiveTaskHandle;
extern TaskHandle_t TelemetrySendTaskHandle;
void CreateTelemetryTask(UART_HandleTypeDef *huart, UBaseType_t Priority, uint16_t StackSize);
void TelemetryReceiveTask(void *pvParameters);
void TelemetrySendTask(void *pvParameters);

#endif //TELEMETRYTASK_H
