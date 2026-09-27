#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BUZZ_NONE = 0,
    BUZZ_DEEPCAL_ENTERED,
    BUZZ_POSE_1,
    BUZZ_POSE_2,
    BUZZ_POSE_3,
    BUZZ_POSE_4,
    BUZZ_POSE_5,
    BUZZ_POSE_6,
    BUZZ_DEEPCAL_OK,
    BUZZ_DEEPCAL_FAIL,
    BUZZ_STOP
} BuzzerPattern_t;

#endif
