#include "Utils/shared.h"
#include <stdint.h>
#include <Tasks/TelemetryTask.h>
#include "Protocol/Protocol.h"
#include "HIL/HIL.h"
#include "Sensors/Sensors.h"

__attribute__((section(".dma_buffer")))
uint8_t TELEMETRY_RX_BUFFER[TELEMETRY_RX_BUFFER_SIZE];

TaskHandle_t TelemetryTaskHandle;

static ProtocolParser_t Parser;

volatile CommandType_t dbg_last_command = 0;
volatile uint8_t dbg_last_command_counter = 0;

void CreateTelemetryTask(UART_HandleTypeDef *huart, const UBaseType_t Priority, const uint16_t StackSize) {
    xTaskCreate(
        TelemetryTask,
        "TELEMETRY_TASK",
        StackSize,
        huart,
        Priority,
        &TelemetryTaskHandle
    );
}

void TelemetryTask(void *pvParameters) {
    UART_HandleTypeDef *huart = pvParameters;

    ProtocolInitParser(&Parser);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, TELEMETRY_RX_BUFFER, TELEMETRY_RX_BUFFER_SIZE);

    for (;;) {
        uint32_t Size = 0;
        xTaskNotifyWait(0, UINT32_MAX, &Size, portMAX_DELAY);

        for (uint16_t i = 0; i < (uint16_t)Size; i++) {
            uint8_t RawCommand = 0;
            if (ProtocolFeed(&Parser, TELEMETRY_RX_BUFFER[i], &RawCommand)) {
                CommandType_t Command = (CommandType_t)RawCommand;
                dbg_last_command = Command;
                dbg_last_command_counter++;
#if HIL_MODE
                if (Command == COMMAND_HIL_DATA) {
                    HandleHILPacket(Parser.Payload);
                }
#endif
                if (Command == COMMAND_GPS_DATA && Parser.Length == ZOEM8Q_PAYLOAD_SIZE) {
                    ZOEM8Q_SensorData_t GPSData;
                    ZOEM8Q_ParsePayload(Parser.Payload, &GPSData);
                    ZOEM8Q_Mailbox_Inject(&GPSData);
                }

                if (Command >= COMMAND_RESET && Command < COMMAND_HIL_DATA) {
                    xQueueSend(CommandQueue, &Command, 0);
                }
            }
        }

        HAL_UARTEx_ReceiveToIdle_DMA(huart, TELEMETRY_RX_BUFFER, TELEMETRY_RX_BUFFER_SIZE);
    }
}
