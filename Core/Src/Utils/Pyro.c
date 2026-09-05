#include "Utils/Pyro.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#define PYRO_PULSE_MS 50

typedef struct {
    GPIO_TypeDef *SRPort;
    GPIO_TypeDef *RSPort;
    uint16_t SRPin;
    uint16_t RSPin;
} PyroChannelConfig_t;

static uint8_t RelayState;

static const PyroChannelConfig_t Channels[] = {
    [PYRO_CHANNEL_DROGUE]    = { DROGUE_SR_3V3_GPIO_Port, DROGUE_RS_3V3_GPIO_Port, DROGUE_SR_3V3_Pin, DROGUE_RS_3V3_Pin },
    [PYRO_CHANNEL_PARACHUTE] = { PCHUTE_SR_3V3_GPIO_Port, PCHUTE_RS_3V3_GPIO_Port, PCHUTE_SR_3V3_Pin, PCHUTE_RS_3V3_Pin },
};

void PyroFire(PyroChannel_t Channel) {
    const PyroChannelConfig_t *Config = &Channels[Channel];
    HAL_GPIO_WritePin(Config->RSPort, Config->RSPin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(Config->SRPort, Config->SRPin, GPIO_PIN_SET);
//    vTaskDelay(pdMS_TO_TICKS(PYRO_PULSE_MS));
//    HAL_GPIO_WritePin(Config->SRPort, Config->SRPin, GPIO_PIN_RESET);
    RelayState |= (1u << Channel);
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
