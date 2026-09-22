#include "Utils/Pyro.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "main.h"

#include "Utils/configuration.h"

typedef struct {
    GPIO_TypeDef *SRPort;
    GPIO_TypeDef *RSPort;
    uint16_t SRPin;
    uint16_t RSPin;
} PyroChannelConfig_t;

static uint8_t RelayState;
static TimerHandle_t PyroTimers[2];

static const PyroChannelConfig_t Channels[] = {
    [PYRO_CHANNEL_DROGUE]    = { DROGUE_SR_3V3_GPIO_Port, DROGUE_RS_3V3_GPIO_Port, DROGUE_SR_3V3_Pin, DROGUE_RS_3V3_Pin },
    [PYRO_CHANNEL_PARACHUTE] = { PCHUTE_SR_3V3_GPIO_Port, PCHUTE_RS_3V3_GPIO_Port, PCHUTE_SR_3V3_Pin, PCHUTE_RS_3V3_Pin },
};

static void PyroTimerCallback(TimerHandle_t xTimer) {
    uint32_t Channel = (uint32_t)pvTimerGetTimerID(xTimer);
    const PyroChannelConfig_t *Config = &Channels[Channel];
    HAL_GPIO_WritePin(Config->RSPort, Config->RSPin, GPIO_PIN_RESET);
}

void PyroFire(PyroChannel_t Channel) {
    const PyroChannelConfig_t *Config = &Channels[Channel];
    HAL_GPIO_WritePin(Config->RSPort, Config->RSPin, GPIO_PIN_SET);
    RelayState |= (1u << Channel);

    if (PyroTimers[Channel] == NULL) {
        PyroTimers[Channel] = xTimerCreate("PYRO", pdMS_TO_TICKS(PYRO_PULSE_MS), pdFALSE, (void *)(uint32_t)Channel, PyroTimerCallback);
    }
    xTimerStart(PyroTimers[Channel], 0);
}

void PyroSafe(PyroChannel_t Channel) {
    const PyroChannelConfig_t *Config = &Channels[Channel];
//    HAL_GPIO_WritePin(Config->SRPort, Config->SRPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Config->RSPort, Config->RSPin, GPIO_PIN_RESET);
//    vTaskDelay(pdMS_TO_TICKS(PYRO_PULSE_MS));
//    HAL_GPIO_WritePin(Config->RSPort, Config->RSPin, GPIO_PIN_RESET);
    RelayState &= ~(1u << Channel);
}

void PyroSafeAll(void) {
    PyroSafe(PYRO_CHANNEL_DROGUE);
    PyroSafe(PYRO_CHANNEL_PARACHUTE);
}

uint8_t PyroGetState(void) {
    return RelayState;
}
