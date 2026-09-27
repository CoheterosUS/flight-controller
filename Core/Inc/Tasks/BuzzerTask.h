#ifndef BUZZERTASK_H
#define BUZZERTASK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "Sensors/Sensors.h"
#include "Utils/configuration.h"
#include "task.h"

extern TaskHandle_t BuzzerTaskHandle;

void CreateBuzzerTask(UBaseType_t Priority, uint16_t StackSize);
void BuzzerTask(void *pvParameters);

static inline bool Buzzer_PatternStep(BuzzerPattern_t Pattern, uint8_t Index, uint32_t *OnMs, uint32_t *OffMs) {
    uint8_t Count = 0;
    uint32_t OnDuration = 0;
    uint32_t OffDuration = 0;

    if (OnMs == NULL || OffMs == NULL) {
        return false;
    }

    switch (Pattern) {
    case BUZZ_DEEPCAL_ENTERED:
        Count = 1;
        OnDuration = BUZZER_DEEPCAL_ENTERED_MS;
        break;
    case BUZZ_POSE_1:
    case BUZZ_POSE_2:
    case BUZZ_POSE_3:
    case BUZZ_POSE_4:
    case BUZZ_POSE_5:
    case BUZZ_POSE_6:
        Count = (uint8_t)(Pattern - BUZZ_POSE_1 + 1);
        OnDuration = BUZZER_POSE_ON_MS;
        OffDuration = BUZZER_POSE_OFF_MS;
        break;
    case BUZZ_DEEPCAL_OK:
        Count = BUZZER_DEEPCAL_OK_COUNT;
        OnDuration = BUZZER_DEEPCAL_OK_ON_MS;
        OffDuration = BUZZER_DEEPCAL_OK_OFF_MS;
        break;
    case BUZZ_DEEPCAL_FAIL:
        Count = 1;
        OnDuration = BUZZER_DEEPCAL_FAIL_MS;
        break;
    case BUZZ_NONE:
    case BUZZ_STOP:
    default:
        return false;
    }

    if (Index >= Count) {
        return false;
    }

    *OnMs = OnDuration;
    *OffMs = OffDuration;
    return true;
}

#endif // BUZZERTASK_H
