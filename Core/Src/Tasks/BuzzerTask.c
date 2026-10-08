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
        uint32_t Notify = 0;
        xTaskNotifyWait(0, UINT32_MAX, &Notify, portMAX_DELAY);
        uint16_t Count = Notify & 0xFFFF;
        uint16_t Duration = (Notify >> 16) ? (Notify >> 16) : 80;

        Buzzer_Beep_Counter(Duration, Count, Duration * 2, false);
    }
}
