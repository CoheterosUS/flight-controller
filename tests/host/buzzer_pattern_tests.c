#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "Tasks/BuzzerTask.h"

static int CheckPattern(BuzzerPattern_t Pattern, uint8_t Count, uint32_t ExpectedOn, uint32_t ExpectedOff) {
    for (uint8_t Index = 0; Index < Count; Index++) {
        uint32_t OnMs = 0;
        uint32_t OffMs = 0;

        if (!Buzzer_PatternStep(Pattern, Index, &OnMs, &OffMs)) {
            return 1;
        }
        if (OnMs != ExpectedOn || OffMs != ExpectedOff) {
            return 1;
        }
    }

    uint32_t OnMs = 0;
    uint32_t OffMs = 0;
    if (Buzzer_PatternStep(Pattern, Count, &OnMs, &OffMs)) {
        return 1;
    }

    return 0;
}

int main(void) {
    if (!Buzzer_PatternRequestsOwnership(BUZZ_DEEPCAL_ENTERED) ||
        Buzzer_PatternRequestsOwnership(BUZZ_NONE) ||
        Buzzer_PatternRequestsOwnership(BUZZ_STOP)) {
        fprintf(stderr, "FAIL: buzzer ownership flag logic\n");
        return 1;
    }
    if (CheckPattern(BUZZ_DEEPCAL_ENTERED, 1, BUZZER_DEEPCAL_ENTERED_MS, 0) != 0) {
        return 1;
    }
    if (CheckPattern(BUZZ_POSE_1, 1, BUZZER_POSE_ON_MS, BUZZER_POSE_OFF_MS) != 0) {
        return 1;
    }
    if (CheckPattern(BUZZ_POSE_2, 2, BUZZER_POSE_ON_MS, BUZZER_POSE_OFF_MS) != 0) {
        return 1;
    }
    if (CheckPattern(BUZZ_POSE_3, 3, BUZZER_POSE_ON_MS, BUZZER_POSE_OFF_MS) != 0) {
        return 1;
    }
    if (CheckPattern(BUZZ_POSE_4, 4, BUZZER_POSE_ON_MS, BUZZER_POSE_OFF_MS) != 0) {
        return 1;
    }
    if (CheckPattern(BUZZ_POSE_5, 5, BUZZER_POSE_ON_MS, BUZZER_POSE_OFF_MS) != 0) {
        return 1;
    }
    if (CheckPattern(BUZZ_POSE_6, 6, BUZZER_POSE_ON_MS, BUZZER_POSE_OFF_MS) != 0) {
        return 1;
    }
    if (CheckPattern(BUZZ_DEEPCAL_OK, BUZZER_DEEPCAL_OK_COUNT, BUZZER_DEEPCAL_OK_ON_MS, BUZZER_DEEPCAL_OK_OFF_MS) != 0) {
        return 1;
    }
    if (CheckPattern(BUZZ_DEEPCAL_FAIL, 1, BUZZER_DEEPCAL_FAIL_MS, 0) != 0) {
        return 1;
    }

    uint32_t OnMs = 0;
    uint32_t OffMs = 0;
    if (Buzzer_PatternStep(BUZZ_NONE, 0, &OnMs, &OffMs) ||
        Buzzer_PatternStep(BUZZ_STOP, 0, &OnMs, &OffMs) ||
        Buzzer_PatternStep(BUZZ_POSE_1, 0, NULL, &OffMs) ||
        Buzzer_PatternStep(BUZZ_POSE_1, 0, &OnMs, NULL)) {
        return 1;
    }

    puts("buzzer pattern tests: PASS");
    return 0;
}
