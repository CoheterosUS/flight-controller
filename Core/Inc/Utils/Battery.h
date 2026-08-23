#ifndef BATTERY_H
#define BATTERY_H

#include "shared.h"

#define BATTERY_VREF              3.3f
#define BATTERY_ADC_RESOLUTION    4095.0f
#define BATTERY_DIVIDER_RATIO     4.485f

void BatteryInit(void);
void Battery_Timer_Callback(TimerHandle_t xTimer);
float BatteryGetVoltage(void);

#endif //BATTERY_H
