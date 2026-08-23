#include "Utils/Battery.h"

__attribute__((section(".dma_buffer")))
static uint16_t BatteryADCRaw;

static volatile float BatteryVoltage;

void BatteryInit(void) {
	HAL_ADCEx_Calibration_Start(ADC1_HANDLE, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	xTimerStart(TimerBattery, 0);
}

void Battery_Timer_Callback(TimerHandle_t xTimer) {
	HAL_ADC_Start_DMA(ADC1_HANDLE, (uint32_t *)&BatteryADCRaw, 1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		BatteryVoltage = ((float)BatteryADCRaw / BATTERY_ADC_RESOLUTION) * BATTERY_VREF * BATTERY_DIVIDER_RATIO;
	}
}

float BatteryGetVoltage(void) {
	return BatteryVoltage;
}
