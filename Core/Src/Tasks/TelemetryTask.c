#include "Utils/shared.h"
#include <stdint.h>
#include <string.h>
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

void CreateTelemetryTask(SystemContext_t *SystemContext, const UBaseType_t Priority, const uint16_t StackSize) {
    xTaskCreate(
        TelemetryReceiveTask,
        "TELEM_RX_TASK",
        StackSize,
        SystemContext,
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
    SystemContext_t *SystemContext = pvParameters;

    ProtocolInitParser(&Parser);
    HAL_UARTEx_ReceiveToIdle_DMA(USART1_HANDLE, TELEMETRY_RX_BUFFER, TELEMETRY_RX_BUFFER_SIZE);

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

                switch (Command) {
#if HIL_MODE
                case COMMAND_HIL_DATA:
                    HandleHILPacket(Parser.Payload);
                    break;
#endif
                case COMMAND_GPS_DATA:
                    if (PayloadLength == ZOEM8Q_PAYLOAD_SIZE) {
                        dbg_gps_command_count++;
                        ZOEM8Q_SensorData_t GPSData;
                        ZOEM8Q_ParsePayload(Parser.Payload, &GPSData);
                        ZOEM8Q_Mailbox_Inject(&GPSData);
                    }
                    break;
                case COMMAND_CALIBRATION:
                    if (PayloadLength >= sizeof(float)) {
                        float Pitch;
                        memcpy(&Pitch, Parser.Payload, sizeof(float));
                        SystemContext->PitchAngleRad = Pitch;
                        SystemContext->PitchReceived = true;
                    }
                    xQueueSend(CommandQueue, &Command, 0);
                    break;
                case COMMAND_REQUEST_TELEM:
                    xTaskNotifyGive(TelemetrySendTaskHandle);
                    break;
                default:
                    if (Command >= COMMAND_RESET && Command < COMMAND_HIL_DATA) {
                        xQueueSend(CommandQueue, &Command, 0);
                    }
                    break;
                }
            }
        }

        HAL_UARTEx_ReceiveToIdle_DMA(USART1_HANDLE, TELEMETRY_RX_BUFFER, TELEMETRY_RX_BUFFER_SIZE);
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
