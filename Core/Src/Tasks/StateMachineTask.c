#include "stm32h7xx_hal.h"
#include <Sensors/Sensors.h>
#include <States/StateHandlers.h>
#include <Tasks/StateMachineTask.h>
#include <Utils/FlightData.h>
#include <Utils/Battery.h>
#include "Managers/Managers.h"
#include "Managers/StructManager.h"
#include "queue.h"

TaskHandle_t StateMachineTaskHandle;

volatile SystemState_t dbg_current_state = STATE_IDLE;
volatile uint32_t dbg_system_faults = {0};
volatile FlightData_t dbg_flight_data;

void CreateStateMachineTask(SystemContext_t *xSystemContext, const UBaseType_t Priority, const uint16_t StackSize) {
    xTaskCreate(
        StateMachineTask,
        "STATE_MACHINE_TASK",
        StackSize,
        xSystemContext,
        Priority,
        &StateMachineTaskHandle
    );
}

void StateMachineTask(void *pvParameters) {
    SystemContext_t *SystemContext = pvParameters;

    // TODO: Refine
    FlightData_t FlightData = {0};
    SystemState_t CurrentSystemState = STATE_IDLE;

    OnStateEntry(CurrentSystemState, SystemContext);

    BatteryInit();
    SerialInit();
    Buzzer_Beep_Counter(100, 2, 500, false);

    for (;;) {
        // TODO: Revise timeout
        BMP581_SensorData_t BMP581_SensorData;
        IIM42653_SensorData_t IIM42653_SensorData;
        IIS2MDCTR_SensorData_t IIS2MDCTR_SensorData;
        ZOEM8Q_SensorData_t ZOEM8Q_SensorData;

        BMP581_Mailbox_Read(&BMP581_SensorData);
        IIM42653_Mailbox_Read(&IIM42653_SensorData);
        IIS2MDCTR_Mailbox_Read(&IIS2MDCTR_SensorData);
        ZOEM8Q_Mailbox_Read(&ZOEM8Q_SensorData);

        FlightData = GetFlightData(CurrentSystemState, SystemContext, IIM42653_SensorData, BMP581_SensorData, IIS2MDCTR_SensorData, ZOEM8Q_SensorData);
        dbg_flight_data = FlightData;

        CommandType_t Command;
        BaseType_t CommandReceived = xQueueReceive(CommandQueue, &Command, 0);
        SystemState_t NextSystemState = HandleCommand(CurrentSystemState, Command, CommandReceived);

        if (NextSystemState == CurrentSystemState) {
            NextSystemState = HandleState(CurrentSystemState, SystemContext, FlightData);
        }

        bool StateChanged = NextSystemState != CurrentSystemState;
        if (StateChanged) {
            OnStateEntry(NextSystemState, SystemContext);
            CurrentSystemState = NextSystemState;

            // TODO: Remove debug code
            Buzzer_Beep(5);
        }

        if (SDLoggingQueue != NULL && !StateChanged) {
            SDLogRecord_t Record = BuildSDLogRecord(&FlightData);
			xQueueSend(SDLoggingQueue, &Record, 0);
		}

        TelemetryPacket_t Packet = BuildTelemetryPacket(&FlightData);
        SerialSendFlightData(&Packet);

        dbg_current_state = CurrentSystemState;
        dbg_system_faults = SystemFaultFlags;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
