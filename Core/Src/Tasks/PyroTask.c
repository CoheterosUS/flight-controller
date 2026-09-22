#include "Tasks/PyroTask.h"
#include "Utils/Pyro.h"
#include "Utils/configuration.h"
#include "main.h"

TaskHandle_t PyroTaskHandle;

void CreatePyroTask(const UBaseType_t Priority, const uint16_t StackSize) {
    xTaskCreate(
        PyroTask,
        "PYRO_TASK",
        StackSize,
        NULL,
        Priority,
        &PyroTaskHandle
    );
}

void PyroTask(void *pvParameters) {
    for (;;) {
        uint32_t ChannelMask = 0;
        xTaskNotifyWait(0, UINT32_MAX, &ChannelMask, portMAX_DELAY);

        for (uint32_t Ch = 0; Ch < 2; Ch++) {
            if (ChannelMask & (1u << Ch)) {
                PyroSetPin(Ch);
                vTaskDelay(pdMS_TO_TICKS(PYRO_PULSE_MS));
                PyroResetPin(Ch);
            }
        }
    }
}
