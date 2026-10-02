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
    for (;;) {
        uint32_t BeepCount = 0;
        xTaskNotifyWait(0, UINT32_MAX, &BeepCount, portMAX_DELAY);
        if (BeepCount == 0) {
            Buzzer_Beep(500);
        } else {
            Buzzer_Beep_Counter(80, BeepCount, 150, false);
        }
    }
}
