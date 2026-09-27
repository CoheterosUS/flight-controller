#ifndef TELEMETRYTASK_H
#define TELEMETRYTASK_H

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "Protocol/Protocol.h"
#include "task.h"

#define TELEMETRY_RX_BUFFER_SIZE 128

extern uint8_t TELEMETRY_RX_BUFFER[TELEMETRY_RX_BUFFER_SIZE];
extern TaskHandle_t TelemetryTaskHandle;

void CreateTelemetryTask(UART_HandleTypeDef *huart, UBaseType_t Priority, uint16_t StackSize);
void TelemetryTask(void *pvParameters);

static inline bool Telemetry_CommandAllowed(uint8_t Command) {
#if EXTERNAL_COMMANDS
    if (Command == COMMAND_RESET || Command == COMMAND_GROUND_ABORT ||
        Command == COMMAND_CALIBRATION) {
        return true;
    }
#if HIL_MODE
    if (Command == COMMAND_DROGUE || Command == COMMAND_LANDED) {
        return true;
    }
#endif
#else
    (void)Command;
#endif
    return false;
}

#endif //TELEMETRYTASK_H
