#include "Tasks/BuzzerTask.h"

#include "stm32h7xx_hal.h"

TaskHandle_t BuzzerTaskHandle;
static volatile bool BuzzerPatternActiveFlag;

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

void Buzzer_Play(BuzzerPattern_t Pattern) {
    if (Buzzer_PatternRequestsOwnership(Pattern)) {
        BuzzerPatternActiveFlag = true;
    }
#if BUZZER_ENABLED
    if (BuzzerTaskHandle != NULL) {
        xTaskNotify(BuzzerTaskHandle, (uint32_t)Pattern, eSetValueWithOverwrite);
    }
#else
    (void)Pattern;
#endif
}

bool Buzzer_PatternActive(void) {
    return BuzzerPatternActiveFlag;
}

static bool Buzzer_WaitForStep(uint32_t DurationMs, BuzzerPattern_t *NextPattern) {
    uint32_t Notification = 0;
    TickType_t Timeout = pdMS_TO_TICKS(DurationMs);

    if (DurationMs > 0 && Timeout == 0) {
        Timeout = 1;
    }

    if (xTaskNotifyWait(0, UINT32_MAX, &Notification, Timeout) == pdTRUE) {
        *NextPattern = (BuzzerPattern_t)Notification;
        return true;
    }

    return false;
}

static void Buzzer_SetOff(void) {
#if BUZZER_ENABLED
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
#endif
}

static void Buzzer_SetOn(void) {
#if BUZZER_ENABLED
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
#endif
}

void BuzzerTask(void *pvParameters) {
    (void)pvParameters;
    BuzzerPattern_t Pattern;

    Buzzer_SetOff();
    BuzzerPatternActiveFlag = false;

    for (;;) {
        uint32_t Notification = 0;
        xTaskNotifyWait(0, UINT32_MAX, &Notification, portMAX_DELAY);
        Pattern = (BuzzerPattern_t)Notification;

        for (;;) {
            bool RestartPattern = false;

            if (Pattern == BUZZ_NONE || Pattern == BUZZ_STOP) {
                Buzzer_SetOff();
                BuzzerPatternActiveFlag = false;
                break;
            }

            for (uint8_t Index = 0;; Index++) {
                uint32_t OnMs;
                uint32_t OffMs;

                if (!Buzzer_PatternStep(Pattern, Index, &OnMs, &OffMs)) {
                    Buzzer_SetOff();
                    break;
                }

                Buzzer_SetOn();
                if (Buzzer_WaitForStep(OnMs, &Pattern)) {
                    Buzzer_SetOff();
                    RestartPattern = true;
                    break;
                }

                Buzzer_SetOff();
                if (Buzzer_WaitForStep(OffMs, &Pattern)) {
                    RestartPattern = true;
                    break;
                }
            }

            if (!RestartPattern) {
                break;
            }
        }

        Buzzer_SetOff();
        BuzzerPatternActiveFlag = false;
    }
}
