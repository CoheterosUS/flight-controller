#include "Tasks/BuzzerTask.h"
#include "Sensors/Sensors.h"

TaskHandle_t BuzzerTaskHandle;

void CreateBuzzerTask(const UBaseType_t Priority, const uint16_t StackSize) {
    xTaskCreate(
        BuzzerTask,
        "BUZZER_TASK",
        StackSize,
        NULL,
        Priority,
        &BuzzerTaskHandle
    );
}

void BuzzerTask(void *pvParameters) {
    BuzzerCommand_t Cmd;
    for (;;) {
        if (xQueueReceive(BuzzerQueue, &Cmd, portMAX_DELAY) == pdPASS) {
            Buzzer_Beep_Counter(Cmd.DurationMs, Cmd.Count, Cmd.GapMs, false);
        }
    }
}
