#ifndef PYRO_H
#define PYRO_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define PYRO_FLAG_DROGUE_FIRED     (1u << 0)
#define PYRO_FLAG_PARACHUTE_FIRED  (1u << 1)

typedef enum {
    PYRO_CHANNEL_DROGUE,
    PYRO_CHANNEL_PARACHUTE,
} PyroChannel_t;

void PyroFire(PyroChannel_t Channel);
void PyroSetPin(uint32_t Channel);
void PyroResetPin(uint32_t Channel);
void PyroSafe(PyroChannel_t Channel);
void PyroSafeAll(void);
uint8_t PyroGetState(void);

#endif //PYRO_H
