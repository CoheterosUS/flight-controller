#include "Utils/shared.h"
#include <stdint.h>
#include <Tasks/TelemetryTask.h>
#include "Protocol/Protocol.h"
#include "HIL/HIL.h"
#include "Sensors/Sensors.h"
#include "Managers/Managers.h"
#include "Managers/StructManager.h"

__attribute__((section(".dma_buffer"), aligned(32)))
uint8_t TELEMETRY_RX_BUFFER[TELEMETRY_RX_BUFFER_SIZE];

TaskHandle_t TelemetryReceiveTaskHandle;
TaskHandle_t TelemetrySendTaskHandle;

static ProtocolParser_t Parser;

volatile CommandType_t dbg_last_command = 0;
volatile uint8_t dbg_last_command_counter = 0;
volatile uint8_t dbg_gps_command_count = 0;

void CreateTelemetryTask(UART_HandleTypeDef *huart, const UBaseType_t Priority, const uint16_t StackSize) {
    xTaskCreate(
        TelemetryReceiveTask,
        "TELEM_RX_TASK",
        StackSize,
        huart,
        Priority + 1,
        &TelemetryReceiveTaskHandle
    );

    xTaskCreate(
        TelemetrySendTask,
        "TELEM_TX_TASK",
        StackSize,
        NULL,
        Priority,
        &TelemetrySendTaskHandle
    );
}

void TelemetryReceiveTask(void *pvParameters) {
    UART_HandleTypeDef *huart = pvParameters;

    ProtocolInitParser(&Parser);
    HAL_UARTEx_ReceiveToIdle_DMA(huart, TELEMETRY_RX_BUFFER, TELEMETRY_RX_BUFFER_SIZE);

    for (;;) {
        uint32_t Size = 0;
        xTaskNotifyWait(0, UINT32_MAX, &Size, portMAX_DELAY);

        for (uint16_t i = 0; i < (uint16_t)Size; i++) {
            uint8_t RawCommand = 0;
            uint8_t PayloadLength = 0;
            if (ProtocolFeed(&Parser, TELEMETRY_RX_BUFFER[i], &RawCommand, &PayloadLength)) {
                CommandType_t Command = (CommandType_t)RawCommand;
                dbg_last_command = Command;
                dbg_last_command_counter++;
#if HIL_MODE
                if (Command == COMMAND_HIL_DATA) {
                    HandleHILPacket(Parser.Payload);
                }
#endif
                if (Command == COMMAND_GPS_DATA && PayloadLength == ZOEM8Q_PAYLOAD_SIZE) {
                	dbg_gps_command_count++;
                    ZOEM8Q_SensorData_t GPSData;
                    ZOEM8Q_ParsePayload(Parser.Payload, &GPSData);
                    ZOEM8Q_Mailbox_Inject(&GPSData);
                }

                if (Command == COMMAND_REQUEST_TELEM) {
                    xTaskNotifyGive(TelemetrySendTaskHandle);
                } else if (Command >= COMMAND_RESET && Command < COMMAND_HIL_DATA) {
                    xQueueSend(CommandQueue, &Command, 0);
                }
            }
        }

        HAL_UARTEx_ReceiveToIdle_DMA(huart, TELEMETRY_RX_BUFFER, TELEMETRY_RX_BUFFER_SIZE);
    }
}

void TelemetrySendTask(void *pvParameters) {
    for (;;) {
        xTaskNotifyWait(0, UINT32_MAX, NULL, portMAX_DELAY);

        FlightData_t FlightData;
        if (xQueuePeek(FlightDataQueue, &FlightData, 0) == pdTRUE) {
            TelemetryPacket_t Packet = BuildTelemetryPacket(&FlightData);
            SerialSendFlightData(&Packet);
        }
    }
}
